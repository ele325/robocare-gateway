#include "modbus_manager.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"


static const char *TAG = "MODBUS";
static int rs485_en_pin = -1;

void modbus_manager_init(int rx_pin, int tx_pin, int en_pin)
{
    rs485_en_pin = en_pin;
    
    // Configuration UART
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    
ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, 256, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    
    // Configuration broche de contrôle RS485
    if (en_pin >= 0) {
        gpio_set_direction(en_pin, GPIO_MODE_OUTPUT);
        gpio_set_level(en_pin, 0);
    }
    
    ESP_LOGI(TAG, "Modbus initialisé");
}

// Calcul CRC16 Modbus
static uint16_t crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1) crc = (crc >> 1) ^ 0xA001;
            else crc >>= 1;
        }
    }
    return crc;
}

void modbus_manager_set_frequency(int active_zones)
{
    uint16_t freq = (active_zones == 0) ? 0 : (2000 + (active_zones * 375));
    
    ESP_LOGI(TAG, "Réglage fréquence: %d Hz (zones actives: %d)", freq, active_zones);
    
    // Activer RS485 (émission)
    if (rs485_en_pin >= 0) {
        gpio_set_level(rs485_en_pin, 1);
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    
    // Trame: Ecriture registre 0x1000
    uint8_t frame[8] = {
        0x01,                   // Adresse esclave
        0x06,                   // Fonction: Write Single Register
        0x10, 0x00,             // Registre 0x1000 (fréquence)
        (freq >> 8) & 0xFF,     // Valeur haut
        freq & 0xFF,            // Valeur bas
        0x00, 0x00              // CRC (calculé après)
    };
    
    uint16_t crc = crc16(frame, 6);
    frame[6] = crc & 0xFF;
    frame[7] = (crc >> 8) & 0xFF;
    
uart_write_bytes(UART_NUM_1, (const char*)frame, 8);    vTaskDelay(pdMS_TO_TICKS(10));
    
    // Commande RUN/STOP (registre 0x2000)
    uint16_t cmd = (active_zones > 0) ? 0x0001 : 0x0005;
    
    frame[2] = 0x20;  // Registre 0x2000
    frame[3] = 0x00;
    frame[4] = (cmd >> 8) & 0xFF;
    frame[5] = cmd & 0xFF;
    
    crc = crc16(frame, 6);
    frame[6] = crc & 0xFF;
    frame[7] = (crc >> 8) & 0xFF;
    
    uart_write_bytes(UART_NUM_1, (const char*)frame, 8);
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // Désactiver RS485
    if (rs485_en_pin >= 0) {
        gpio_set_level(rs485_en_pin, 0);
    }
}