#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "time.h"

#include "network_manager.h"
#include "lora_manager.h"
#include "sd_manager.h"
#include "modbus_manager.h"
#include "relay_manager.h"

static const char *TAG = "MAIN";

// Pin Configuration from Receiver Schematic (Verified)
#define LORA_MOSI_PIN 29
#define LORA_MISO_PIN 31
#define LORA_SCK_PIN  30
#define LORA_CS_PIN   28
#define LORA_RST_PIN  27
#define LORA_DIO0_PIN 32

#define SD_CS_PIN     10

#define MODBUS_RX_PIN 18
#define MODBUS_TX_PIN 17
#define MODBUS_EN_PIN 4

const int RELAY_PINS[] = {5, 4, 3, 2, 6, 7, 8, 9, 39, 40, 41, 42};
#define NUM_RELAYS 12

// MQTT Configuration
#define WIFI_SSID      "salut"
#define WIFI_PASS      "hey0000."
#define MQTT_SERVER    "broker.hivemq.com"
#define MQTT_PORT      1883

// Callback when data is received via LoRa from sensors
void on_sensor_data_received(int zone_id, float humidity) {
    ESP_LOGI(TAG, "Capteur: Zone %d, Humidité: %.2f%%", zone_id, humidity);

    // 1. Publier sur MQTT
    network_manager_publish_humidity(zone_id, humidity);

    // 2. Enregistrer sur SD
    sd_manager_log_data((uint32_t)time(NULL), zone_id, humidity);

    // 3. Logique de contrôle (Optionnel: Modbus ou Relais)
    // Exemple: Si humidité trop basse, on pourrait activer un relais
    if (humidity < 30.0) {
        relay_manager_set(zone_id - 1, true);
    } else if (humidity > 60.0) {
        relay_manager_set(zone_id - 1, false);
    }

    // Mettre à jour la fréquence VFD via Modbus en fonction des zones actives
    int active_zones = relay_manager_active_count();
    modbus_manager_set_frequency(active_zones);
}

// Callback when a command is received from MQTT to control relays
void on_relay_command_received(int relay_idx, bool state) {
    ESP_LOGI(TAG, "Commande Relais: %d -> %s", relay_idx, state ? "ON" : "OFF");
    relay_manager_set(relay_idx, state);
    
    // Mettre à jour VFD
    int active_zones = relay_manager_active_count();
    modbus_manager_set_frequency(active_zones);
}

void app_main(void) {
    ESP_LOGI(TAG, "Démarrage Robocare Gateway...");

    // 1. Initialiser le stockage SD
    if (!sd_manager_init(SD_CS_PIN)) {
        ESP_LOGE(TAG, "Impossible d'initialiser la carte SD");
    }

    // 2. Initialiser les relais
    relay_manager_init(RELAY_PINS, NUM_RELAYS);

    // 3. Initialiser Modbus
    modbus_manager_init(MODBUS_RX_PIN, MODBUS_TX_PIN, MODBUS_EN_PIN);

    // 4. Initialiser le Réseau (WiFi + MQTT)
    network_manager_init(WIFI_SSID, WIFI_PASS, MQTT_SERVER, MQTT_PORT);
    network_manager_set_relay_callback(on_relay_command_received);
    network_manager_start();

    // 5. Initialiser LoRa
    if (lora_manager_init(LORA_MOSI_PIN, LORA_MISO_PIN, LORA_SCK_PIN, LORA_CS_PIN, LORA_RST_PIN, LORA_DIO0_PIN)) {
        lora_manager_set_callback(on_sensor_data_received);
    } else {
        ESP_LOGE(TAG, "Échec initialisation LoRa");
    }

    ESP_LOGI(TAG, "Système prêt.");
    
    while(1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
