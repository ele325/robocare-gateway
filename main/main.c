#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "nvs_flash.h"

#include "robocare_types.h"
#include "lora_manager.h"
#include "network_manager.h"
#include "sd_manager.h"
#include "relay_manager.h"

static const char *TAG = "MAIN_RX";

/* ── SPI SD ──────────────────────────────────────────────── */
#define SD_SPI_HOST     SPI2_HOST
#define SD_MOSI_PIN     11
#define SD_MISO_PIN     13
#define SD_SCK_PIN      12
#define SD_CS_PIN       10

/* ── SPI LoRa ────────────────────────────────────────────── */
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
#define PUMP_RELAY_INDEX    2   /* OUTPUT3 / IO3 */
#define VALVE_RELAY_INDEX   3   /* OUTPUT4 / IO2 */

#define HUMIDITY_THRESHOLD_ON   30.0f
#define HUMIDITY_THRESHOLD_OFF  60.0f
#define VALVE_DELAY_MS          500

#define WIFI_SSID    "salut"
#define WIFI_PASS    "hey0000."
#define MQTT_SERVER  "mqtt://80.75.212.179"
#define MQTT_PORT    1883
#define FIREBASE_UID "2SKcuqIcjSb3a2B6NWs2LebCO4g2"

static char g_firebase_uid[128] = {0};

/* ═══════════════════════════════════════════════════════════
 * Prototypes
 * ═══════════════════════════════════════════════════════════ */
static void led_3v3_init(void);
static void spi_sd_bus_init(void);
static void spi_lora_bus_init(void);
static void irrigation_start(void);
static void irrigation_stop(void);
static void on_sensor_data_received(const lora_sensor_data_t *data);
static void on_relay_command_received(int relay_idx, bool state);
static void lora_task(void *arg);

/* ═══════════════════════════════════════════════════════════
 * Sécurité démarrage — force GPIO HIGH (relais actif LOW = OFF)
 * ═══════════════════════════════════════════════════════════ */
static void gpio_safety_init(void)
{
    gpio_config_t safe = {
        .pin_bit_mask = (1ULL << 2) | (1ULL << 3) |
                        (1ULL << 4) | (1ULL << 5),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&safe);

    /* ✅ CORRIGÉ : Relais actif LOW → HIGH = OFF */
    gpio_set_level(2, 1);  /* VANNE OFF */
    gpio_set_level(3, 1);  /* POMPE OFF */
    gpio_set_level(4, 1);  /* OUTPUT2 OFF */
    gpio_set_level(5, 1);  /* OUTPUT1 OFF */
    ESP_LOGI("SAFETY", "GPIO forcés HIGH — relais OFF au démarrage (actif LOW)");
}

/* ═══════════════════════════════════════════════════════════
 * Initialisation bus SPI
 * ═══════════════════════════════════════════════════════════ */
static void spi_sd_bus_init(void)
{
    spi_bus_config_t bus_cfg = {
        .mosi_io_num   = SD_MOSI_PIN,
        .miso_io_num   = SD_MISO_PIN,
        .sclk_io_num   = SD_SCK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SD_SPI_HOST, &bus_cfg, SPI_DMA_CH_AUTO));
    ESP_LOGI(TAG, "Bus SPI SD initialisé (SPI2)");
}

static void spi_lora_bus_init(void)
{
    spi_bus_config_t bus_cfg = {
        .mosi_io_num   = LORA_MOSI_PIN,
        .miso_io_num   = LORA_MISO_PIN,
        .sclk_io_num   = LORA_SCK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LORA_SPI_HOST, &bus_cfg, SPI_DMA_CH_AUTO));
    ESP_LOGI(TAG, "Bus SPI LoRa initialisé (SPI3)");
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
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    gpio_set_level(LED_3V3_PIN, 1);
    ESP_LOGI(TAG, "LED 3V3 ON (IO%d)", LED_3V3_PIN);
}

/* ═══════════════════════════════════════════════════════════
 * Irrigation
 * ═══════════════════════════════════════════════════════════ */
static void irrigation_start(void)
{
    if (relay_manager_get(VALVE_RELAY_INDEX) ||
        relay_manager_get(PUMP_RELAY_INDEX)) {
        ESP_LOGI(TAG, "Irrigation déjà active (ou partiellement)");
        return;
    }

    ESP_LOGI(TAG, "Irrigation START : VANNE puis POMPE");
    relay_manager_set(VALVE_RELAY_INDEX, true);
    vTaskDelay(pdMS_TO_TICKS(VALVE_DELAY_MS));  /* 500ms vanne s'ouvre */
    relay_manager_set(PUMP_RELAY_INDEX, true);
}

static void irrigation_stop(void)
{
    bool valve_on = relay_manager_get(VALVE_RELAY_INDEX);
    bool pump_on  = relay_manager_get(PUMP_RELAY_INDEX);

    if (!valve_on && !pump_on) {
        ESP_LOGI(TAG, "Irrigation déjà arrêtée");
        return;
    }

    ESP_LOGI(TAG, "Irrigation STOP : POMPE puis VANNE");

    if (pump_on) {
        relay_manager_set(PUMP_RELAY_INDEX, false);
        vTaskDelay(pdMS_TO_TICKS(VALVE_DELAY_MS));  /* 500ms pression retombe */
    }
    if (valve_on) {
        relay_manager_set(VALVE_RELAY_INDEX, false);
    }
}

/* ═══════════════════════════════════════════════════════════
 * Callback LoRa
 * ═══════════════════════════════════════════════════════════ */
static void on_sensor_data_received(const lora_sensor_data_t *data)
{
    if (!data) return;

    ESP_LOGI(TAG, "LoRa RX — node=%d hum=%.1f%% temp=%.1f°C "
                  "pH=%.2f EC=%.0f N=%.0f P=%.0f K=%.0f RSSI=%d SNR=%.1f",
             data->node_id,
             data->humidity,
             data->temperature,
             data->ph,
             data->ec,
             data->nitrogen,
             data->phosphorus,
             data->potassium,
             data->rssi,
             data->snr);

    network_manager_publish_sensor_data(data);

    if (data->humidity < HUMIDITY_THRESHOLD_ON) {
        ESP_LOGI(TAG, "Humidité faible (%.1f%%) → irrigation START",
                 data->humidity);
        irrigation_start();
    } else if (data->humidity > HUMIDITY_THRESHOLD_OFF) {
        ESP_LOGI(TAG, "Humidité suffisante (%.1f%%) → irrigation STOP",
                 data->humidity);
        irrigation_stop();
    } else {
        ESP_LOGI(TAG, "Humidité OK (%.1f%%) — aucune action", data->humidity);
    }
}

/* ═══════════════════════════════════════════════════════════
 * Callback MQTT
 * ═══════════════════════════════════════════════════════════ */
static void on_relay_command_received(int relay_idx, bool state)
{
    ESP_LOGI(TAG, "Commande MQTT — relay_idx=%d state=%s",
             relay_idx, state ? "ON" : "OFF");

    if (state) {
        irrigation_start();
    } else {
        irrigation_stop();
    }
}

/* ═══════════════════════════════════════════════════════════
 * Tâche LoRa
 * ═══════════════════════════════════════════════════════════ */
static void lora_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "Tâche LoRa démarrée");
    while (1) {
        lora_manager_process();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/* ═══════════════════════════════════════════════════════════
 * app_main
 * ═══════════════════════════════════════════════════════════ */
void app_main(void)
{
    ESP_LOGI(TAG, "RoboCare RX — Pompe OUTPUT3 / Vanne OUTPUT4");

    /* 🔒 SÉCURITÉ : GPIO HIGH avant tout le reste (actif LOW = relais OFF) */
    gpio_safety_init();
    vTaskDelay(pdMS_TO_TICKS(500));  /* stabilisation 500ms */

    /* NVS */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    strncpy(g_firebase_uid, FIREBASE_UID, sizeof(g_firebase_uid) - 1);

    /* Périphériques */
    led_3v3_init();
    spi_sd_bus_init();
    spi_lora_bus_init();

    /* Carte SD */
    sd_manager_set_spi_host(SD_SPI_HOST);
    if (!sd_manager_init(SD_CS_PIN)) {
        ESP_LOGW(TAG, "SD non disponible");
    }

    /* Relais */
    ESP_LOGI(TAG, "Init relais — OUTPUT3/IO3=POMPE  OUTPUT4/IO2=VANNE");
    relay_manager_init(RELAY_PINS, NUM_RELAYS);
    relay_manager_set(PUMP_RELAY_INDEX,  false);  /* sécurité */
    relay_manager_set(VALVE_RELAY_INDEX, false);  /* sécurité */

    /* Réseau */
    network_manager_init(WIFI_SSID, WIFI_PASS, MQTT_SERVER, MQTT_PORT);
    network_manager_set_uid(g_firebase_uid);
    network_manager_set_relay_callback(on_relay_command_received);
    network_manager_start();

    /* LoRa */
    if (lora_manager_init(LORA_CS_PIN, LORA_RST_PIN, LORA_DIO0_PIN)) {
        lora_manager_set_callback(on_sensor_data_received);
        ESP_LOGI(TAG, "LoRa OK");
        xTaskCreate(lora_task, "lora_task", 4096, NULL, 5, NULL);
    } else {
        ESP_LOGE(TAG, "LoRa ÉCHEC");
    }

    ESP_LOGI(TAG, "Système prêt ✓");
}