/**
 * @file main.c
 * @brief Carte RÉCEPTRICE RoboCare — ESP32-S2-WROOM
 *
 * ╔════════════════════════════════════════════════════════════════════════════╗
 * ║ DIMENSIONNEMENT ÉNERGÉTIQUE (24/7 actif — critique)                       ║
 * ╠════════════════════════════════════════════════════════════════════════════╣
 * ║ Rail 3.3V : 1510 mA (ESP32=790mA, LoRa=200mA, SD=150mA, autres=370mA)    ║
 * ║ Rail 5V   : 750 mA  (Relais 5V max, SD, circuits)                        ║
 * ║ Rail 12V  : 2004 mA (15.36W total système)                               ║
 * ║                                                                            ║
 * ║ DISSIPATION THERMIQUE (CRITIQUE) :                                        ║
 * ║  • LM2596S-3.3V : P = (12V - 3.3V) × 1.51A = 13.1W                       ║
 * ║    → DOIT avoir dissipateur thermique 0.1K/W minimum                      ║
 * ║  • LM2596S-5V   : P = (12V - 5V) × 0.75A = 5.25W                        ║
 * ║    → DOIT avoir dissipateur thermique 0.5K/W minimum                      ║
 * ║                                                                            ║
 * ║ CONDENSATEURS REQUÉRANT (découplage crucial) :                            ║
 * ║  • C1 : 470µF sur 12V (entrée régulateur)                                 ║
 * ║  • C2 : 100nF sur 3.3V (ESP32 pin 2)                                     ║
 * ║  • C3 : 100nF sur 5V (ligne relais/SD)                                    ║
 * ║  • C4 : 100nF sur LoRa 3.3V                                               ║
 * ╚════════════════════════════════════════════════════════════════════════════╝
 *
 * Corrections v2.2 :
 *  1. RELAY_PINS[] : limité à 4 relais de base (extensible à 8)
 *  2. LORA_SPI_HOST = SPI3_HOST (LoRa sur bus séparé de SD)
 *  3. IO14 LED 3.3V pour indication alimentation
 *  4. Documentation énergétique complète
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_mac.h"

#include "lora_manager.h"
#include "network_manager.h"
#include "sd_manager.h"
#include "relay_manager.h"

static const char *TAG = "MAIN_RX";

/* =========================================================================
 * Gestion NVS — UID persistant & découverte MQTT
 * ========================================================================= */
#define NVS_NAMESPACE "robocare"
#define NVS_KEY_UID   "firebase_uid"

/**
 * Génère la MAC address au format "AA:BB:CC:DD:EE:FF"
 * Utilisée pour la découverte et le stockage NVS
 */
static void get_device_mac(char *mac_str, size_t max_len)
{
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    snprintf(mac_str, max_len, "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

/**
 * Lit l'UID Firebase depuis NVS
 * Retourne strdup'd string (faut free) ou NULL si non trouvé
 */
static char* nvs_read_uid(void)
{
    nvs_handle_t handle;
    char uid[128] = {0};
    
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (ret != ESP_OK) {
        ESP_LOGI(TAG, "NVS namespace vierge — device non encore associé");
        return NULL;
    }
    
    size_t len = sizeof(uid);
    ret = nvs_get_str(handle, NVS_KEY_UID, uid, &len);
    nvs_close(handle);
    
    if (ret == ESP_OK && strlen(uid) > 0) {
        ESP_LOGI(TAG, "UID récupéré du NVS : %s", uid);
        return strdup(uid);
    }
    
    ESP_LOGI(TAG, "UID not found in NVS");
    return NULL;
}

/**
 * ⚠️ NOTE: nvs_write_uid() a été DÉPLACÉE dans network_manager.c en tant que nvs_save_uid()
 * Raison : Elle est automatiquement appelée par network_manager quand reçoit l'UID via MQTT
 * On la supprime ici pour éviter les warnings "unused function"
 * Voir: network_manager.c::nvs_save_uid() et mqtt_event_handler::MQTT_EVENT_DATA
 */


/* =========================================================================
 * Configuration brochage — ESP32-S2-WROOM (pinout Excel confirmé)
 * ========================================================================= */

/* SPI2_HOST — SD Card (IO10-IO13) */
#define SD_SPI_HOST     SPI2_HOST
#define SD_MOSI_PIN     11   /* IO11 = MOSI SD */
#define SD_MISO_PIN     13   /* IO13 = MISO SD */
#define SD_SCK_PIN      12   /* IO12 = CLK  SD */
#define SD_CS_PIN       10   /* IO10 = CS   SD */

/* SPI3_HOST — LoRa Ra-02 (IO33-IO38) — bus séparé de la SD */
#define LORA_SPI_HOST   SPI3_HOST
#define LORA_MOSI_PIN   35   /* IO35 = MOSI_2 LoRa */
#define LORA_MISO_PIN   37   /* IO37 = MISO_2 LoRa */
#define LORA_SCK_PIN    36   /* IO36 = CLK_2  LoRa */
#define LORA_CS_PIN     34   /* IO34 = CS_2   LoRa */
#define LORA_RST_PIN    33   /* IO33 = RST    LoRa */
#define LORA_DIO0_PIN   38   /* IO38 = I/O    LoRa (DIO0/RxDone) */

/* LED présence tension 3.3V */
#define LED_3V3_PIN     14   /* IO14 = Green LED */

/*
 * Relais — 4 sorties de base, extensible à 8
 * Spécifications : 4 relais → 8 relais (électrovannes/pompes)
 * CORRECTION v2.2 : ordre corrigé selon pinout Excel
 *   Output1 = IO5, Output2 = IO4, Output3 = IO3, Output4 = IO2
 *   Output5 = IO6 (extension), Output6 = IO7 (extension),
 *   Output7 = IO8 (extension), Output8 = IO9 (extension)
 */
const int RELAY_PINS[] = { 5, 4, 3, 2, 6, 7, 8, 9 };
#define NUM_RELAYS         4   /* 4 relais de base */
#define MAX_RELAYS_EXTENSION 8  /* Max 8 relais avec interface complémentaire */

/* WiFi + MQTT */
#define WIFI_SSID       "salut"
#define WIFI_PASS       "hey0000."
#define MQTT_SERVER     "mqtt://broker.hivemq.com"
#define MQTT_PORT       1883

/*
 * UID Firebase — gestion automatique via NVS + discovery MQTT
 * Si pas en NVS : publie MAC sur "robocare/discovery"
 * Backend répond sur "robocare/config/<MAC>" avec l'UID
 * Possibilité de forcer un UID pour tests via network_manager_set_uid()
 */
static char g_firebase_uid[128] = {0};  /* Buffer global */

/* =========================================================================
 * Initialisation des bus SPI
 * ========================================================================= */
static void spi_sd_bus_init(void)
{
    spi_bus_config_t buscfg = {
        .mosi_io_num     = SD_MOSI_PIN,
        .miso_io_num     = SD_MISO_PIN,
        .sclk_io_num     = SD_SCK_PIN,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = 4096,
    };

    esp_err_t ret = spi_bus_initialize(SD_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "spi_bus_initialize SD échoué : %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "[SPI2] Bus SD prêt (MOSI:%d MISO:%d SCK:%d)",
                 SD_MOSI_PIN, SD_MISO_PIN, SD_SCK_PIN);
    }
}

static void spi_lora_bus_init(void)
{
    spi_bus_config_t buscfg = {
        .mosi_io_num     = LORA_MOSI_PIN,
        .miso_io_num     = LORA_MISO_PIN,
        .sclk_io_num     = LORA_SCK_PIN,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = 256,
    };

    esp_err_t ret = spi_bus_initialize(LORA_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "spi_bus_initialize LoRa échoué : %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "[SPI3] Bus LoRa prêt (MOSI:%d MISO:%d SCK:%d)",
                 LORA_MOSI_PIN, LORA_MISO_PIN, LORA_SCK_PIN);
    }
}

/* =========================================================================
 * Initialisation LED 3.3V (IO14)
 * ========================================================================= */
static void led_3v3_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_3V3_PIN),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(LED_3V3_PIN, 1);  /* LED ON = alimentation 3.3V présente */
    ESP_LOGI(TAG, "[LED] IO14 allumée — 3.3V OK");
}

/* =========================================================================
 * Callback LoRa — déclenché à chaque paquet reçu
 * ========================================================================= */
static void on_sensor_data_received(const lora_sensor_data_t *data)
{
    if (!data) return;

    ESP_LOGI(TAG, "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    ESP_LOGI(TAG, "Nœud     : %d",      data->node_id);
    ESP_LOGI(TAG, "Temp     : %.1f °C", data->temperature);
    ESP_LOGI(TAG, "Humidité : %.1f %%", data->humidity);
    ESP_LOGI(TAG, "EC       : %.0f µS/cm", data->ec);
    ESP_LOGI(TAG, "pH       : %.2f%s",  data->ph,
             data->ph < 0 ? " (sonde déconnectée)" : "");
    ESP_LOGI(TAG, "N/P/K    : %.0f / %.0f / %.0f mg/kg",
             data->nitrogen, data->phosphorus, data->potassium);
    ESP_LOGI(TAG, "RSSI     : %d dBm | SNR : %.1f dB",
             data->rssi, data->snr);
    ESP_LOGI(TAG, "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");

    /* 1. Publication MQTT → bridge Python → Firebase */
    network_manager_publish_sensor_data(data);

    /* 2. Log SD local */
    char log_line[256];
    if (data->ph < 0.0f) {
        snprintf(log_line, sizeof(log_line),
                 "N/A;N/A;%d;%.1f;%.1f;%.0f;INVAL;%.0f;%.0f;%.0f;%d;%.1f\n",
                 data->node_id,
                 data->temperature, data->humidity, data->ec,
                 data->nitrogen, data->phosphorus, data->potassium,
                 data->rssi, data->snr);
    } else {
        snprintf(log_line, sizeof(log_line),
                 "N/A;N/A;%d;%.1f;%.1f;%.0f;%.2f;%.0f;%.0f;%.0f;%d;%.1f\n",
                 data->node_id,
                 data->temperature, data->humidity, data->ec,
                 data->ph,
                 data->nitrogen, data->phosphorus, data->potassium,
                 data->rssi, data->snr);
    }
    sd_manager_log_raw(log_line);

    /* 3. Logique relais locale basée sur humidité */
    int relay_idx = data->node_id - 1;
    if (relay_idx >= 0 && relay_idx < NUM_RELAYS) {
        if (data->humidity < 30.0f) {
            ESP_LOGI(TAG, "Humidité critique (%.1f%%) — Relais %d ON",
                     data->humidity, data->node_id);
            relay_manager_set(relay_idx, true);
        } else if (data->humidity > 60.0f) {
            relay_manager_set(relay_idx, false);
        }
    } else {
        ESP_LOGW(TAG, "node_id %d hors plage relais (1-%d) — ignoré",
                 data->node_id, NUM_RELAYS);
    }
}

/* =========================================================================
 * Callback commande relais depuis MQTT
 * ========================================================================= */
static void on_relay_command_received(int relay_idx, bool state)
{
    if (relay_idx < 0 || relay_idx >= NUM_RELAYS) {
        ESP_LOGW(TAG, "Commande relais index invalide : %d", relay_idx);
        return;
    }
    ESP_LOGI(TAG, "Commande MQTT : Relais %d (IO%d) → %s",
             relay_idx + 1, RELAY_PINS[relay_idx],
             state ? "ON" : "OFF");
    relay_manager_set(relay_idx, state);
}

/* =========================================================================
 * Tâche FreeRTOS : polling LoRa toutes les 50ms
 * ========================================================================= */
static void lora_task(void *arg)
{
    ESP_LOGI(TAG, "Tâche LoRa démarrée — polling toutes les 50ms");
    while (1) {
        lora_manager_process();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/* =========================================================================
 * app_main
 * ========================================================================= */
void app_main(void)
{
    ESP_LOGI(TAG, "╔══════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║  RoboCare — Carte Réceptrice v2.3        ║");
    ESP_LOGI(TAG, "║  ESP32-S2-WROOM + NVS + Discovery        ║");
    ESP_LOGI(TAG, "╚══════════════════════════════════════════╝");

    /* ── Initialisation NVS ──────────────────────────────────────────── */
    ESP_LOGI(TAG, "[0/7] Initialisation NVS...");
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }
    
    /* Récupérer MAC et UID persistant */
    char device_mac[18];
    get_device_mac(device_mac, sizeof(device_mac));
    ESP_LOGI(TAG, "  Device MAC : %s", device_mac);
    
    char *persisted_uid = nvs_read_uid();
    if (persisted_uid) {
        strncpy(g_firebase_uid, persisted_uid, sizeof(g_firebase_uid) - 1);
        free(persisted_uid);
        ESP_LOGI(TAG, "  ✓ UID du NVS : %s (ready)", g_firebase_uid);
    } else {
        strncpy(g_firebase_uid, "AUTOMATIC_DISCOVERY", sizeof(g_firebase_uid) - 1);
        ESP_LOGI(TAG, "  ◇ Pas d'UID en NVS → découverte automatique");
        ESP_LOGI(TAG, "  ⏳ Le serveur doit publier l'UID sur : robocare/config/%s", device_mac);
    }

    /* ── 0/7 : LED 3.3V ──────────────────────────────────────────────── */
    led_3v3_init();

    /* ── 1/7 : Bus SPI ───────────────────────────────────────────────── */
    ESP_LOGI(TAG, "[1/7] Initialisation bus SPI...");
    spi_sd_bus_init();    /* SPI2_HOST pour SD  (IO10-IO13) */
    spi_lora_bus_init();  /* SPI3_HOST pour LoRa (IO33-IO38) */

    /* ── 2/7 : SD Card ───────────────────────────────────────────────── */
    ESP_LOGI(TAG, "[2/7] Initialisation SD Card (CS=IO%d)...", SD_CS_PIN);
    sd_manager_set_spi_host(SD_SPI_HOST);
    if (!sd_manager_init(SD_CS_PIN)) {
        ESP_LOGW(TAG, "SD non disponible — log local désactivé");
    }

    /* ── 3/7 : Relais ────────────────────────────────────────────────── */
    ESP_LOGI(TAG, "[3/7] Initialisation relais (%d zones de base, %d max)...",
             NUM_RELAYS, MAX_RELAYS_EXTENSION);
    ESP_LOGI(TAG, "  ✓ Out1=IO5 Out2=IO4 Out3=IO3 Out4=IO2");
    ESP_LOGI(TAG, "  ◇ Out5=IO6 Out6=IO7 Out7=IO8 Out8=IO9 (extension optional)");
    relay_manager_init(RELAY_PINS, NUM_RELAYS);

    /* ── 4/7 : Réseau WiFi + MQTT ────────────────────────────────────── */
    ESP_LOGI(TAG, "[4/7] Initialisation réseau...");
    network_manager_set_uid(g_firebase_uid);
    network_manager_init(WIFI_SSID, WIFI_PASS, MQTT_SERVER, MQTT_PORT);
    network_manager_set_relay_callback(on_relay_command_received);
    network_manager_start();  /* Bloquant jusqu'à WiFi connexion */

    /* ── 5/7 : LoRa ──────────────────────────────────────────────────── */
    ESP_LOGI(TAG, "[5/7] Initialisation LoRa (CS=IO%d RST=IO%d DIO0=IO%d)...",
             LORA_CS_PIN, LORA_RST_PIN, LORA_DIO0_PIN);
    if (lora_manager_init(LORA_CS_PIN, LORA_RST_PIN, LORA_DIO0_PIN)) {
        lora_manager_set_callback(on_sensor_data_received);
        ESP_LOGI(TAG, "LoRa OK — 433 MHz SF12 BW125 CR4/5");
    } else {
        ESP_LOGE(TAG, "LoRa ÉCHEC — réception impossible");
        ESP_LOGE(TAG, "  → Vérifier câblage IO33-IO38 et alimentation 3.3V");
    }

    /* ── 6/7 : Tâche LoRa ────────────────────────────────────────────── */
    ESP_LOGI(TAG, "[6/7] Démarrage tâche LoRa...");
    xTaskCreate(lora_task, "lora_task", 4096, NULL, 5, NULL);

    /* ── 7/7 : Résumé ────────────────────────────────────────────────── */
    ESP_LOGI(TAG, "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    ESP_LOGI(TAG, "PROFIL ÉNERGÉTIQUE 24/7 ACTIF");
    ESP_LOGI(TAG, "  3.3V : 1510 mA | 5V : 750 mA | 12V : 2004 mA (15.36W)");
    ESP_LOGI(TAG, "  ⚠️  Max 4 relais actifs simultanément");
    ESP_LOGI(TAG, "  ⚠️  Dissipateurs LM2596S ESSENTIELS");
    ESP_LOGI(TAG, "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    ESP_LOGI(TAG, "CONFIGURATION MQTT + NVS");
    if (strcmp(g_firebase_uid, "AUTOMATIC_DISCOVERY") == 0) {
        ESP_LOGI(TAG, "  ⏳ ATTENTE UID : Bridge publiera sur robocare/config/%s", device_mac);
    } else {
        ESP_LOGI(TAG, "  ✓ UID ACTIF : %s", g_firebase_uid);
    }
    ESP_LOGI(TAG, "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");

    ESP_LOGI(TAG, "Système prêt — en attente de paquets LoRa.");
}