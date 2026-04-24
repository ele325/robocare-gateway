#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "nvs_flash.h"

#include "lora_manager.h"
#include "network_manager.h"
#include "sd_manager.h"
#include "relay_manager.h"

static const char *TAG = "MAIN_RX";

/* =========================================================================
 * Configuration brochage - ESP32-S2-WROOM
 * ========================================================================= */

#define SD_SPI_HOST     SPI2_HOST
#define SD_MOSI_PIN     11
#define SD_MISO_PIN     13
#define SD_SCK_PIN      12
#define SD_CS_PIN       10

#define LORA_SPI_HOST   SPI3_HOST
#define LORA_MOSI_PIN   35
#define LORA_MISO_PIN   37
#define LORA_SCK_PIN    36
#define LORA_CS_PIN     34
#define LORA_RST_PIN    33
#define LORA_DIO0_PIN   38

#define LED_3V3_PIN     14

const int RELAY_PINS[] = { 5, 4, 3, 2, 6, 7, 8, 9 };
#define NUM_RELAYS            4
#define MAX_RELAYS_EXTENSION  8

/* WiFi + MQTT + UID utilisateur fixe */
#define WIFI_SSID    "salut"
#define WIFI_PASS    "hey0000."
#define MQTT_SERVER  "mqtt://80.75.212.179"
#define MQTT_PORT    1883
#define FIREBASE_UID "2SKcuqIcjSb3a2B6NWs2LebCO4g2"
static char g_firebase_uid[128] = {0};

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
        ESP_LOGE(TAG, "spi_bus_initialize SD echoue : %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "[SPI2] Bus SD pret (MOSI:%d MISO:%d SCK:%d)",
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
        ESP_LOGE(TAG, "spi_bus_initialize LoRa echoue : %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "[SPI3] Bus LoRa pret (MOSI:%d MISO:%d SCK:%d)",
                 LORA_MOSI_PIN, LORA_MISO_PIN, LORA_SCK_PIN);
    }
}

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
    gpio_set_level(LED_3V3_PIN, 1);
    ESP_LOGI(TAG, "[LED] IO14 allumee - 3.3V OK");
}

static void on_sensor_data_received(const lora_sensor_data_t *data)
{
    if (!data) return;

    ESP_LOGI(TAG, "Noeud     : %d", data->node_id);
    ESP_LOGI(TAG, "Temp      : %.1f C", data->temperature);
    ESP_LOGI(TAG, "Humidite  : %.1f %%", data->humidity);
    ESP_LOGI(TAG, "EC        : %.0f uS/cm", data->ec);
    ESP_LOGI(TAG, "pH        : %.2f%s", data->ph,
             data->ph < 0 ? " (sonde deconnectee)" : "");
    ESP_LOGI(TAG, "N/P/K     : %.0f / %.0f / %.0f mg/kg",
             data->nitrogen, data->phosphorus, data->potassium);
    ESP_LOGI(TAG, "RSSI      : %d dBm | SNR : %.1f dB",
             data->rssi, data->snr);

    network_manager_publish_sensor_data(data);

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

    int relay_idx = data->node_id - 1;
    if (relay_idx >= 0 && relay_idx < NUM_RELAYS) {
        if (data->humidity < 30.0f) {
            ESP_LOGI(TAG, "Humidite critique (%.1f%%) - Relais %d ON",
                     data->humidity, data->node_id);
            relay_manager_set(relay_idx, true);
        } else if (data->humidity > 60.0f) {
            relay_manager_set(relay_idx, false);
        }
    } else {
        ESP_LOGW(TAG, "node_id %d hors plage relais (1-%d) - ignore",
                 data->node_id, NUM_RELAYS);
    }
}

static void on_relay_command_received(int relay_idx, bool state)
{
    if (relay_idx < 0 || relay_idx >= NUM_RELAYS) {
        ESP_LOGW(TAG, "Commande relais index invalide : %d", relay_idx);
        return;
    }

    ESP_LOGI(TAG, "Commande MQTT : Relais %d (IO%d) -> %s",
             relay_idx + 1, RELAY_PINS[relay_idx], state ? "ON" : "OFF");
    relay_manager_set(relay_idx, state);
}

static void lora_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "Tache LoRa demarree - polling toutes les 50ms");
    while (1) {
        lora_manager_process();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "RoboCare - Carte receptrice");

    ESP_LOGI(TAG, "[0/7] Initialisation NVS...");
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    strncpy(g_firebase_uid, FIREBASE_UID, sizeof(g_firebase_uid) - 1);
    g_firebase_uid[sizeof(g_firebase_uid) - 1] = '\0';
    ESP_LOGI(TAG, "UID fixe configure : %s", g_firebase_uid);

    led_3v3_init();

    ESP_LOGI(TAG, "[1/7] Initialisation bus SPI...");
    spi_sd_bus_init();
    spi_lora_bus_init();

    ESP_LOGI(TAG, "[2/7] Initialisation SD Card (CS=IO%d)...", SD_CS_PIN);
    sd_manager_set_spi_host(SD_SPI_HOST);
    if (!sd_manager_init(SD_CS_PIN)) {
        ESP_LOGW(TAG, "SD non disponible - log local desactive");
    }

    ESP_LOGI(TAG, "[3/7] Initialisation relais (%d zones de base, %d max)...",
             NUM_RELAYS, MAX_RELAYS_EXTENSION);
    relay_manager_init(RELAY_PINS, NUM_RELAYS);

    ESP_LOGI(TAG, "[4/7] Initialisation reseau...");
    network_manager_init(WIFI_SSID, WIFI_PASS, MQTT_SERVER, MQTT_PORT);
    network_manager_set_uid(g_firebase_uid);
    ESP_LOGI(TAG, "UID transmis au network_manager : %s", g_firebase_uid);
    network_manager_set_relay_callback(on_relay_command_received);
    network_manager_start();

    ESP_LOGI(TAG, "[5/7] Initialisation LoRa (CS=IO%d RST=IO%d DIO0=IO%d)...",
             LORA_CS_PIN, LORA_RST_PIN, LORA_DIO0_PIN);
    if (lora_manager_init(LORA_CS_PIN, LORA_RST_PIN, LORA_DIO0_PIN)) {
        lora_manager_set_callback(on_sensor_data_received);
        ESP_LOGI(TAG, "LoRa OK - 433 MHz SF12 BW125 CR4/5");
    } else {
        ESP_LOGE(TAG, "LoRa ECHEC - reception impossible");
        ESP_LOGE(TAG, "Verifier cablage IO33-IO38 et alimentation 3.3V");
    }

    ESP_LOGI(TAG, "[6/7] Demarrage tache LoRa...");
    xTaskCreate(lora_task, "lora_task", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "[7/7] Configuration MQTT");
    ESP_LOGI(TAG, "UID actif : %s", g_firebase_uid);
    ESP_LOGI(TAG, "Systeme pret - en attente de paquets LoRa.");
}
