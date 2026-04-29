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

/* SPI SD */
#define SD_SPI_HOST     SPI2_HOST
#define SD_MOSI_PIN     11
#define SD_MISO_PIN     13
#define SD_SCK_PIN      12
#define SD_CS_PIN       10

/* SPI LoRa */
#define LORA_SPI_HOST   SPI3_HOST
#define LORA_MOSI_PIN   35
#define LORA_MISO_PIN   37
#define LORA_SCK_PIN    36
#define LORA_CS_PIN     34
#define LORA_RST_PIN    33
#define LORA_DIO0_PIN   38

#define LED_3V3_PIN     14

/*
 * Mapping carte :
 * OUTPUT1 = IO5 = index 0
 * OUTPUT2 = IO4 = index 1
 * OUTPUT3 = IO3 = index 2 = POMPE
 * OUTPUT4 = IO2 = index 3 = VANNE
 */
const int RELAY_PINS[] = { 5, 4, 3, 2 };

#define NUM_RELAYS          4
#define PUMP_RELAY_INDEX    2   // OUTPUT3 / IO3
#define VALVE_RELAY_INDEX   3   // OUTPUT4 / IO2

#define HUMIDITY_THRESHOLD_ON   30.0f
#define HUMIDITY_THRESHOLD_OFF  60.0f
#define VALVE_DELAY_MS          500

#define WIFI_SSID    "salut"
#define WIFI_PASS    "hey0000."
#define MQTT_SERVER  "mqtt://80.75.212.179"
#define MQTT_PORT    1883
#define FIREBASE_UID "2SKcuqIcjSb3a2B6NWs2LebCO4g2"

static char g_firebase_uid[128] = {0};

/* ========================================================= */

static void irrigation_start(void)
{
    if (relay_manager_get(VALVE_RELAY_INDEX) &&
        relay_manager_get(PUMP_RELAY_INDEX)) {
        ESP_LOGI(TAG, "Irrigation déjà active");
        return;
    }

    ESP_LOGI(TAG, "Irrigation START : VANNE OUTPUT4 puis POMPE OUTPUT3");

    relay_manager_set(VALVE_RELAY_INDEX, true);
    vTaskDelay(pdMS_TO_TICKS(VALVE_DELAY_MS));

    relay_manager_set(PUMP_RELAY_INDEX, true);
}

static void irrigation_stop(void)
{
    if (!relay_manager_get(VALVE_RELAY_INDEX) &&
        !relay_manager_get(PUMP_RELAY_INDEX)) {
        ESP_LOGI(TAG, "Irrigation déjà arrêtée");
        return;
    }

    ESP_LOGI(TAG, "Irrigation STOP : POMPE OUTPUT3 puis VANNE OUTPUT4");

    relay_manager_set(PUMP_RELAY_INDEX, false);
    vTaskDelay(pdMS_TO_TICKS(VALVE_DELAY_MS));

    relay_manager_set(VALVE_RELAY_INDEX, false);
}

/* ========================================================= */

static void spi_sd_bus_init(void)
{
    spi_bus_config_t buscfg = {
        .mosi_io_num = SD_MOSI_PIN,
        .miso_io_num = SD_MISO_PIN,
        .sclk_io_num = SD_SCK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
    };

    esp_err_t ret = spi_bus_initialize(SD_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "SPI SD erreur : %s", esp_err_to_name(ret));
    }
}

static void spi_lora_bus_init(void)
{
    spi_bus_config_t buscfg = {
        .mosi_io_num = LORA_MOSI_PIN,
        .miso_io_num = LORA_MISO_PIN,
        .sclk_io_num = LORA_SCK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 256,
    };

    esp_err_t ret = spi_bus_initialize(LORA_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "SPI LoRa erreur : %s", esp_err_to_name(ret));
    }
}

static void led_3v3_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_3V3_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    gpio_config(&io_conf);
    gpio_set_level(LED_3V3_PIN, 1);
}

/* ========================================================= */
/* Callback LoRa automatique */

static void on_sensor_data_received(const lora_sensor_data_t *data)
{
    if (!data) return;

    ESP_LOGI(TAG, "Noeud: %d | Humidite: %.1f%% | Temp: %.1fC",
             data->node_id, data->humidity, data->temperature);

    network_manager_publish_sensor_data(data);

    if (data->humidity < HUMIDITY_THRESHOLD_ON) {
        ESP_LOGI(TAG, "Humidité basse -> irrigation ON");
        irrigation_start();
    }
    else if (data->humidity > HUMIDITY_THRESHOLD_OFF) {
        ESP_LOGI(TAG, "Humidité haute -> irrigation OFF");
        irrigation_stop();
    }
}

/* ========================================================= */
/* Callback MQTT manuel */

static void on_relay_command_received(int relay_idx, bool state)
{
    /*
     * Avec MQTT Explorer, utilise :
     * robocare/<uid>/valve/control/1
     *
     * payload 1 -> irrigation_start()
     * payload 0 -> irrigation_stop()
     */

    ESP_LOGI(TAG, "Commande MQTT reçue : relay_idx=%d state=%s",
             relay_idx, state ? "ON" : "OFF");

    if (state) {
        irrigation_start();
    } else {
        irrigation_stop();
    }
}

/* ========================================================= */

static void lora_task(void *arg)
{
    (void)arg;

    ESP_LOGI(TAG, "Tâche LoRa démarrée");

    while (1) {
        lora_manager_process();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/* ========================================================= */

void app_main(void)
{
    ESP_LOGI(TAG, "RoboCare RX - Pompe OUTPUT3 / Vanne OUTPUT4");

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    strncpy(g_firebase_uid, FIREBASE_UID, sizeof(g_firebase_uid) - 1);

    led_3v3_init();

    spi_sd_bus_init();
    spi_lora_bus_init();

    sd_manager_set_spi_host(SD_SPI_HOST);
    if (!sd_manager_init(SD_CS_PIN)) {
        ESP_LOGW(TAG, "SD non disponible");
    }

    ESP_LOGI(TAG, "Initialisation relais...");
    ESP_LOGI(TAG, "OUTPUT3 / IO3 = POMPE");
    ESP_LOGI(TAG, "OUTPUT4 / IO2 = VANNE");

    relay_manager_init(RELAY_PINS, NUM_RELAYS);

    /* sécurité démarrage */
    relay_manager_set(PUMP_RELAY_INDEX, false);
    relay_manager_set(VALVE_RELAY_INDEX, false);

    network_manager_init(WIFI_SSID, WIFI_PASS, MQTT_SERVER, MQTT_PORT);
    network_manager_set_uid(g_firebase_uid);
    network_manager_set_relay_callback(on_relay_command_received);
    network_manager_start();

    if (lora_manager_init(LORA_CS_PIN, LORA_RST_PIN, LORA_DIO0_PIN)) {
        lora_manager_set_callback(on_sensor_data_received);
        ESP_LOGI(TAG, "LoRa OK");
        xTaskCreate(lora_task, "lora_task", 4096, NULL, 5, NULL);
    } else {
        ESP_LOGE(TAG, "LoRa échec");
    }

    ESP_LOGI(TAG, "Système prêt");
}