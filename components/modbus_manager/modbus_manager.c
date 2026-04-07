#include "modbus_manager.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <string.h> // Nécessaire pour memcmp

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
        gpio_set_level(en_pin, 0); // Mode lecture par défaut
    }
    
    ESP_LOGI(TAG, "Modbus initialisé avec vérification de réponse");
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

// --- TA FONCTION AJOUTÉE ET CORRIGÉE ---
static bool wait_for_response(uint8_t *expected_frame, size_t len) 
{
    uint8_t response[8]; 
    
    // Lecture sur le port UART (Timeout 100ms)
    int rxBytes = uart_read_bytes(UART_NUM_1, response, len, pdMS_TO_TICKS(100));
    
    if (rxBytes == len) {
        if (memcmp(response, expected_frame, len) == 0) {
            ESP_LOGI(TAG, "Confirmation reçue du variateur : OK");
            return true;
        } else {
            ESP_LOGE(TAG, "Erreur : Réponse reçue mais invalide (Données corrompues)");
        }
    } else {
        ESP_LOGE(TAG, "Erreur : Aucune réponse du variateur (Timeout ou déconnexion)");
    }
    return false;
}

void modbus_manager_set_frequency(int active_zones)
{
    uint16_t freq = (active_zones == 0) ? 0 : (2000 + (active_zones * 375));
    ESP_LOGI(TAG, "Action : Réglage fréquence à %d (Zones : %d)", freq, active_zones);
    
    // --- 1. ENVOI DE LA FRÉQUENCE (Registre 0x1000) ---
    uint8_t frame[8] = {
        0x01, 0x06, 0x10, 0x00, 
        (freq >> 8) & 0xFF, freq & 0xFF, 
        0x00, 0x00
    };
    uint16_t crc = crc16(frame, 6);
    frame[6] = crc & 0xFF;
    frame[7] = (crc >> 8) & 0xFF;

    if (rs485_en_pin >= 0) gpio_set_level(rs485_en_pin, 1); // Mode Émission
    uart_write_bytes(UART_NUM_1, (const char*)frame, 8);
    uart_wait_tx_done(UART_NUM_1, pdMS_TO_TICKS(10));      // Attendre la fin réelle de l'envoi
    if (rs485_en_pin >= 0) gpio_set_level(rs485_en_pin, 0); // Mode Réception
    
    wait_for_response(frame, 8); // On vérifie si le variateur a compris
    vTaskDelay(pdMS_TO_TICKS(50));

    // --- 2. ENVOI DE LA COMMANDE RUN/STOP (Registre 0x2000) ---
    uint16_t cmd = (active_zones > 0) ? 0x0001 : 0x0005;
    frame[2] = 0x20; // Adresse registre 0x2000
    frame[3] = 0x00;
    frame[4] = (cmd >> 8) & 0xFF;
    frame[5] = cmd & 0xFF;
    
    crc = crc16(frame, 6);
    frame[6] = crc & 0xFF;
    frame[7] = (crc >> 8) & 0xFF;

    if (rs485_en_pin >= 0) gpio_set_level(rs485_en_pin, 1); // Mode Émission
    uart_write_bytes(UART_NUM_1, (const char*)frame, 8);
    uart_wait_tx_done(UART_NUM_1, pdMS_TO_TICKS(10));
    if (rs485_en_pin >= 0) gpio_set_level(rs485_en_pin, 0); // Mode Réception
    
    wait_for_response(frame, 8); // On vérifie si l'ordre de marche est accepté
}