/**
 * @file lora_manager.c
 * @brief Driver LoRa Ra-02 (SX1278) — Carte RÉCEPTRICE RoboCare
 *
 * Corrections v2.1 :
 *  1. SPI3_HOST utilisé au lieu de SPI2_HOST (LoRa sur bus séparé de la SD)
 *  2. parse_lora_message() corrigé : accepte 8 tokens (format émetteur)
 *     Format émetteur : "NODE_ID;humidity;temperature;ph;ec;N;P;K"
 *     Exemple         : "1;32.6;20.5;7.22;75;8;8;21"
 *     L'ancienne version attendait 10 tokens avec "N" prefix et date/heure
 *     → parsing toujours échoué → aucune donnée reçue
 */

#include "lora_manager.h"

#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

static const char *TAG = "LORA_RX";

static spi_device_handle_t  s_spi_lora  = NULL;
static lora_data_callback_t s_callback  = NULL;

/* =========================================================================
 * Registres SX1278
 * ========================================================================= */
#define REG_FIFO                 0x00
#define REG_OP_MODE              0x01
#define REG_FR_MSB               0x06
#define REG_FR_MID               0x07
#define REG_FR_LSB               0x08
#define REG_PA_CONFIG            0x09
#define REG_FIFO_ADDR_PTR        0x0D
#define REG_FIFO_RX_BASE         0x0F
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_IRQ_FLAGS            0x12
#define REG_RX_NB_BYTES          0x13
#define REG_PKT_SNR              0x19
#define REG_PKT_RSSI             0x1A
#define REG_MODEM_CONFIG1        0x1D
#define REG_MODEM_CONFIG2        0x1E
#define REG_VERSION              0x42

/* Modes LoRa */
#define LORA_MODE_SLEEP          0x80
#define LORA_MODE_STDBY          0x81
#define LORA_MODE_RXCONT         0x85

#define SX1278_VERSION           0x12

/* =========================================================================
 * Helpers SPI full-duplex
 * ========================================================================= */
static uint8_t lora_read_byte(uint8_t addr)
{
    uint8_t tx[2] = { addr & 0x7F, 0x00 };
    uint8_t rx[2] = { 0x00, 0x00 };

    spi_transaction_t t = {
        .length    = 16,
        .rxlength  = 16,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    spi_device_transmit(s_spi_lora, &t);
    return rx[1];
}

static void lora_write_byte(uint8_t addr, uint8_t value)
{
    uint8_t tx[2] = { addr | 0x80, value };

    spi_transaction_t t = {
        .length    = 16,
        .tx_buffer = tx,
        .rx_buffer = NULL,
    };
    spi_device_transmit(s_spi_lora, &t);
}

/* =========================================================================
 * parse_lora_message
 *
 * CORRECTION v2.1 — format aligné sur l'émetteur :
 *
 * L'émetteur (main_emettrice.c) envoie :
 *   "%d;%.1f;%.1f;%.2f;%.0f;%.0f;%.0f;%.0f"
 *   NODE_ID ; humidity ; temperature ; ph ; ec ; N ; P ; K
 *   Exemple : "1;32.6;20.5;7.22;75;8;8;21"   → 8 tokens
 *
 * L'ancienne version attendait 10 tokens avec préfixe "N" et date/heure
 * → n < 10 toujours vrai → return false → aucune donnée jamais reçue.
 *
 * Nouvelle version : 8 tokens minimum, pas de préfixe "N".
 * date et time_str remplis avec "N/A" (pas disponibles dans ce format).
 * ========================================================================= */
static bool parse_lora_message(const uint8_t *raw, int len,
                                lora_sensor_data_t *out)
{
    if (!raw || len <= 0 || !out) return false;

    char msg[128];
    if (len > 127) len = 127;
    memcpy(msg, raw, len);
    msg[len] = '\0';

    ESP_LOGI(TAG, "Paquet brut : %s", msg);

    /* Tokenisation sur ';' */
    char buf[128];
    strncpy(buf, msg, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    char *tokens[10];
    int   n   = 0;
    char *tok = strtok(buf, ";");
    while (tok && n < 10) {
        tokens[n++] = tok;
        tok = strtok(NULL, ";");
    }

    /* Format émetteur : minimum 8 tokens */
    if (n < 8) {
        ESP_LOGW(TAG, "Message incomplet : %d tokens (min 8 attendus)", n);
        ESP_LOGW(TAG, "  → Contenu reçu : %s", msg);
        return false;
    }

    /*
     * Ordre des tokens (format émetteur main_emettrice.c) :
     *   [0] = NODE_ID      (entier, ex: "1")
     *   [1] = humidity     (float, ex: "32.6")
     *   [2] = temperature  (float, ex: "20.5")
     *   [3] = ph           (float, ex: "7.22" ou "-1.00")
     *   [4] = ec           (float, ex: "75")
     *   [5] = nitrogen     (float, ex: "8")
     *   [6] = phosphorus   (float, ex: "8")
     *   [7] = potassium    (float, ex: "21")
     */
    out->node_id = atoi(tokens[0]);
    if (out->node_id < 1 || out->node_id > 254) {
        ESP_LOGW(TAG, "node_id invalide : %d", out->node_id);
        return false;
    }

    out->humidity    = strtof(tokens[1], NULL);
    out->temperature = strtof(tokens[2], NULL);
    out->ph          = strtof(tokens[3], NULL);  /* -1.0 si invalide */
    out->ec          = strtof(tokens[4], NULL);
    out->nitrogen    = strtof(tokens[5], NULL);
    out->phosphorus  = strtof(tokens[6], NULL);
    out->potassium   = strtof(tokens[7], NULL);

    /* Date/heure non transmises dans ce format → N/A */
    strncpy(out->date,     "N/A", sizeof(out->date)     - 1);
    strncpy(out->time_str, "N/A", sizeof(out->time_str) - 1);
    out->date[sizeof(out->date) - 1]         = '\0';
    out->time_str[sizeof(out->time_str) - 1] = '\0';

    return true;
}

/* =========================================================================
 * lora_manager_init
 *
 * CORRECTION v2.1 : utilise SPI3_HOST (bus LoRa séparé de la SD SPI2_HOST)
 * ========================================================================= */
bool lora_manager_init(int cs_pin, int rst_pin, int dio0_pin)
{
    /* Reset matériel */
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << rst_pin),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    gpio_set_level(rst_pin, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(rst_pin, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(rst_pin, 1);
    vTaskDelay(pdMS_TO_TICKS(100));

    /*
     * CORRECTION v2.1 : SPI3_HOST au lieu de SPI2_HOST.
     * La carte réceptrice utilise deux bus SPI distincts :
     *   SPI2_HOST → SD Card  (IO10 CS, IO11 MOSI, IO12 SCK, IO13 MISO)
     *   SPI3_HOST → LoRa     (IO34 CS, IO35 MOSI, IO36 SCK, IO37 MISO)
     * Utiliser SPI2_HOST pour LoRa causait un conflit avec la SD.
     */
    spi_device_interface_config_t devcfg = {
        .mode           = 0,
        .clock_speed_hz = 1 * 1000 * 1000,
        .spics_io_num   = cs_pin,
        .queue_size     = 7,
        .flags          = 0,   /* full-duplex */
        .pre_cb         = NULL,
        .post_cb        = NULL,
    };

    esp_err_t ret = spi_bus_add_device(SPI3_HOST, &devcfg, &s_spi_lora);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_add_device (SPI3) échoué : %s",
                 esp_err_to_name(ret));
        return false;
    }
    ESP_LOGI(TAG, "SX1278 ajouté sur SPI3_HOST  CS=IO%d  RST=IO%d  DIO0=IO%d",
             cs_pin, rst_pin, dio0_pin);

    /* Vérification version chip */
    uint8_t version = lora_read_byte(REG_VERSION);
    ESP_LOGI(TAG, "Registre VERSION = 0x%02X (attendu 0x%02X)",
             version, SX1278_VERSION);

    if (version != SX1278_VERSION) {
        ESP_LOGE(TAG, "SX1278 non détecté !");
        ESP_LOGE(TAG, "  → Vérifier câblage IO33-IO38 et alimentation 3.3V");
        spi_bus_remove_device(s_spi_lora);
        s_spi_lora = NULL;
        return false;
    }

    /* Configuration SX1278 */
    lora_write_byte(REG_OP_MODE, LORA_MODE_SLEEP);
    vTaskDelay(pdMS_TO_TICKS(15));

    /* Fréquence 433 MHz — identique à l'émetteur */
    lora_write_byte(REG_FR_MSB, 0x6C);
    lora_write_byte(REG_FR_MID, 0x80);
    lora_write_byte(REG_FR_LSB, 0x00);

    lora_write_byte(REG_PA_CONFIG, 0x8F);
    lora_write_byte(REG_FIFO_RX_BASE, 0x00);

    /* ModemConfig1 : BW=125kHz, CR=4/5 — identique à l'émetteur */
    lora_write_byte(REG_MODEM_CONFIG1, 0x72);

    /* ModemConfig2 : SF12, CRC ON — identique à l'émetteur (0xC4) */
    lora_write_byte(REG_MODEM_CONFIG2, 0xC4);

    /* Mode réception continue */
    lora_write_byte(REG_OP_MODE, LORA_MODE_RXCONT);
    vTaskDelay(pdMS_TO_TICKS(10));

    ESP_LOGI(TAG, "LoRa RX prêt : 433 MHz | SF12 | BW125 kHz | CR4/5");
    return true;
}

/* =========================================================================
 * lora_manager_deinit
 * ========================================================================= */
void lora_manager_deinit(void)
{
    if (s_spi_lora != NULL) {
        spi_bus_remove_device(s_spi_lora);
        s_spi_lora = NULL;
        ESP_LOGI(TAG, "LoRa RX dés-initialisé");
    }
}

/* =========================================================================
 * lora_manager_set_callback
 * ========================================================================= */
void lora_manager_set_callback(lora_data_callback_t cb)
{
    s_callback = cb;
}

/* =========================================================================
 * lora_manager_process
 * ========================================================================= */
bool lora_manager_process(void)
{
    if (s_spi_lora == NULL) return false;

    uint8_t irq = lora_read_byte(REG_IRQ_FLAGS);

    /* Bit 6 = RxDone */
    if (!(irq & 0x40)) return false;

    /* Bit 5 = PayloadCrcError */
    if (irq & 0x20) {
        ESP_LOGW(TAG, "CRC erreur paquet LoRa — ignoré");
        lora_write_byte(REG_IRQ_FLAGS, 0xFF);
        return false;
    }

    uint8_t nb_bytes  = lora_read_byte(REG_RX_NB_BYTES);
    uint8_t fifo_addr = lora_read_byte(REG_FIFO_RX_CURRENT_ADDR);
    lora_write_byte(REG_FIFO_ADDR_PTR, fifo_addr);

    int    rssi    = (int)lora_read_byte(REG_PKT_RSSI) - 157;
    int8_t snr_raw = (int8_t)lora_read_byte(REG_PKT_SNR);
    float  snr     = snr_raw / 4.0f;

    uint8_t buffer[128];
    uint8_t len = nb_bytes;
    if (len > sizeof(buffer) - 1) len = sizeof(buffer) - 1;

    for (int i = 0; i < len; i++) {
        buffer[i] = lora_read_byte(REG_FIFO);
    }
    buffer[len] = '\0';

    lora_write_byte(REG_IRQ_FLAGS, 0xFF);

    lora_sensor_data_t data;
    memset(&data, 0, sizeof(data));
    data.rssi = rssi;
    data.snr  = snr;

    if (parse_lora_message(buffer, len, &data)) {
        ESP_LOGI(TAG, "RX OK — Nœud %d | T=%.1f°C H=%.1f%% pH=%.2f "
                      "EC=%.0f N=%.0f P=%.0f K=%.0f | RSSI=%ddBm SNR=%.1fdB",
                 data.node_id,
                 data.temperature, data.humidity, data.ph,
                 data.ec, data.nitrogen, data.phosphorus, data.potassium,
                 data.rssi, data.snr);

        if (s_callback) {
            s_callback(&data);
        }
        return true;
    }

    ESP_LOGW(TAG, "Paquet reçu mais parsing échoué (%d octets)", len);
    return false;
}