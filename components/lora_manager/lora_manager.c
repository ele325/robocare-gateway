#include "lora_manager.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "LORA";
static spi_device_handle_t spi_lora;
static lora_data_callback_t user_callback = NULL;

// Registres LoRa (SX1276/SX1278)
#define REG_FIFO         0x00
#define REG_OP_MODE      0x01
#define REG_FR_MSB       0x06
#define REG_FR_MID       0x07
#define REG_FR_LSB       0x08
#define REG_PA_CONFIG    0x09
#define REG_FIFO_ADDR_PTR 0x0D
#define REG_FIFO_TX_BASE 0x0E
#define REG_FIFO_RX_BASE 0x0F
#define REG_RX_NB_BYTES  0x13
#define REG_IRQ_FLAGS    0x12
#define REG_PKT_SNR      0x19
#define REG_PKT_RSSI     0x1A
#define REG_MODEM_CONFIG1 0x1D
#define REG_MODEM_CONFIG2 0x1E
#define REG_PAYLOAD_LENGTH 0x22
#define REG_VERSION      0x42

#define LORA_MODE_SLEEP  0x80
#define LORA_MODE_STDBY  0x81
#define LORA_MODE_RXCONT 0x85

// Lecture/écriture SPI
static uint8_t lora_read_byte(uint8_t addr)
{
    spi_transaction_t t = {
        .flags = 0,
        .length = 16,
        .rxlength = 8,
        .tx_buffer = NULL,
        .rx_buffer = NULL,
    };
    uint8_t tx_data[2] = { addr & 0x7F, 0x00 };
    uint8_t rx_data[2];
    t.tx_buffer = tx_data;
    t.rx_buffer = rx_data;
    spi_device_transmit(spi_lora, &t);
    return rx_data[1];
}

static void lora_write_byte(uint8_t addr, uint8_t value)
{
    spi_transaction_t t = {
        .flags = 0,
        .length = 16,
        .tx_buffer = NULL,
    };
    uint8_t tx_data[2] = { addr | 0x80, value };
    t.tx_buffer = tx_data;
    spi_device_transmit(spi_lora, &t);
}

bool lora_manager_init(int mosi_pin, int miso_pin, int sck_pin, 
                      int cs_pin, int rst_pin, int dio0_pin)
{
    esp_err_t ret;
    
    // Reset du module LoRa
    gpio_set_direction(rst_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(rst_pin, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(rst_pin, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(rst_pin, 1);
    vTaskDelay(pdMS_TO_TICKS(50));
    
    // Configuration du bus SPI
    spi_bus_config_t buscfg = {
        .mosi_io_num = mosi_pin,
        .miso_io_num = miso_pin,
        .sclk_io_num = sck_pin,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0
    };
    
    ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_DISABLED);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Erreur initialisation bus SPI");
        return false;
    }
    
    // Configuration du périphérique LoRa
    spi_device_interface_config_t devcfg = {
        .mode = 0,
        .clock_speed_hz = 5 * 1000 * 1000,
        .spics_io_num = cs_pin,
        .queue_size = 7,
        .flags = SPI_DEVICE_HALFDUPLEX,
    };
    
    ret = spi_bus_add_device(SPI2_HOST, &devcfg, &spi_lora);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Erreur ajout périphérique SPI");
        return false;
    }
    
    // Vérifier la version du module
    uint8_t version = lora_read_byte(REG_VERSION);
    ESP_LOGI(TAG, "Version LoRa: 0x%02X", version);
    
    // Configuration LoRa
    lora_write_byte(REG_OP_MODE, LORA_MODE_SLEEP);
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // Fréquence 433 MHz
    lora_write_byte(REG_FR_MSB, 0x6C);  // 433 MHz
    lora_write_byte(REG_FR_MID, 0x80);
    lora_write_byte(REG_FR_LSB, 0x00);
    
    lora_write_byte(REG_PA_CONFIG, 0x8F);  // Puissance max
    
    // Configuration modulation (BW=125kHz, CR=4/5, SF=12)
    lora_write_byte(REG_MODEM_CONFIG1, 0x72);
    lora_write_byte(REG_MODEM_CONFIG2, 0x74);
    
    // Mode réception continue
    lora_write_byte(REG_OP_MODE, LORA_MODE_RXCONT);
    
    ESP_LOGI(TAG, "LoRa initialisé sur 433MHz");
    return true;
}

void lora_manager_set_callback(lora_data_callback_t cb)
{
    user_callback = cb;
}

static void parse_lora_message(const uint8_t *data, int len)
{
    // Format attendu: "ID:xx,H:xx.xx"
    char msg[64];
    if (len > 63) len = 63;
    memcpy(msg, data, len);
    msg[len] = '\0';
    
    ESP_LOGD(TAG, "Message reçu: %s", msg);
    
    // Extraire zone ID
    char *id_ptr = strstr(msg, "ID:");
    if (!id_ptr) return;
    int zoneID = atoi(id_ptr + 3);
    
    // Extraire humidité
    char *h_ptr = strstr(msg, "H:");
    if (!h_ptr) return;
    float humidite = atof(h_ptr + 2);
    
    if (zoneID >= 1 && zoneID <= 12 && user_callback) {
        user_callback(zoneID, humidite);
    }
}

void lora_manager_process(void)
{
    uint8_t irq_flags = lora_read_byte(REG_IRQ_FLAGS);
    
    if (irq_flags & 0x40) {  // RxDone
        uint8_t len = lora_read_byte(REG_RX_NB_BYTES);
        uint8_t fifo_addr = lora_read_byte(REG_FIFO_RX_BASE);
        lora_write_byte(REG_FIFO_ADDR_PTR, fifo_addr);
        
        uint8_t buffer[64];
        for (int i = 0; i < len && i < 64; i++) {
            buffer[i] = lora_read_byte(REG_FIFO);
        }
        
        parse_lora_message(buffer, len);
        
        // Effacer IRQ
        lora_write_byte(REG_IRQ_FLAGS, 0xFF);
    }
}