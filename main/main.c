/**
 * @file main.c
 * @brief Carte RÉCEPTRICE RoboCare — ESP32-S2-WROOM
 * @version 2.0 — Pompe + électrovanne corrigées
 *
 * Corrections v2.0 :
 *  - Pompe et vanne activées ensemble automatiquement
 *  - Ordre correct : vanne d'abord, pompe ensuite (activation)
 *  - Ordre correct : pompe d'abord, vanne ensuite (arrêt)
 *  - Délai 200ms entre vanne et pompe
 *  - Commande manuelle MQTT complète
 */

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
 * Basé sur pinout réel de la carte réceptrice
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

/* =========================================================================
 * Configuration relais
 * Pinout carte réceptrice :
 *   IO5 = Output1 = Relais 1 = POMPE      (index 0)
 *   IO4 = Output2 = Relais 2 = VANNE Z1   (index 1)
 *   IO3 = Output3 = Relais 3 = VANNE Z2   (index 2)
 *   IO2 = Output4 = Relais 4 = VANNE Z3   (index 3)
 * ========================================================================= */

const int RELAY_PINS[] = { 5, 4, 3, 2, 6, 7, 8, 9 };
#define NUM_RELAYS            4
#define MAX_RELAYS_EXTENSION  8

/* Index logiques */
#define PUMP_RELAY_INDEX      0    /* IO5 — pompe (commune à toutes les zones) */
#define VALVE_ZONE1_INDEX     1    /* IO4 — électrovanne zone 1 */
#define VALVE_ZONE2_INDEX     2    /* IO3 — électrovanne zone 2 */
#define VALVE_ZONE3_INDEX     3    /* IO2 — électrovanne zone 3 */

/* Seuils humidité */
#define HUMIDITY_THRESHOLD_ON   30.0f   /* En dessous → irrigation ON  */
#define HUMIDITY_THRESHOLD_OFF  60.0f   /* Au dessus  → irrigation OFF */

/* Délai mécanique ouverture électrovanne (ms) */
#define VALVE_OPEN_DELAY_MS     200

/* WiFi + MQTT */
#define WIFI_SSID    "salut"
#define WIFI_PASS    "hey0000."
#define MQTT_SERVER  "mqtt://80.75.212.179"
#define MQTT_PORT    1883
#define FIREBASE_UID "2SKcuqIcjSb3a2B6NWs2LebCO4g2"

static char g_firebase_uid[128] = {0};

/* =========================================================================
 * Fonctions utilitaires internes
 * ========================================================================= */

/**
 * @brief Retourne l'index relais de la vanne pour une zone donnée
 * @param zone_id ID de zone (1-based, venant du node_id LoRa)
 * @return index relais vanne, ou -1 si hors plage
 */
static int get_valve_relay_index(int zone_id)
{
    /* zone_id 1 → relais 1 (index 1 = IO4)
     * zone_id 2 → relais 2 (index 2 = IO3)
     * zone_id 3 → relais 3 (index 3 = IO2)
     * etc.
     * La pompe est toujours index 0, les vannes commencent à index 1
     */
    int valve_idx = zone_id; /* zone 1 → index 1, zone 2 → index 2 ... */
    if (valve_idx < 1 || valve_idx >= NUM_RELAYS) {
        return -1;
    }
    return valve_idx;
}

/**
 * @brief Active vanne puis pompe avec délai (ordre correct)
 * @param valve_idx Index relais de la vanne
 */
static void irrigation_start(int valve_idx)
{
    ESP_LOGI(TAG, "Irrigation START — vanne (relais %d) puis pompe (relais %d)",
             valve_idx + 1, PUMP_RELAY_INDEX + 1);

    /* 1. Ouvrir la vanne d'abord */
    relay_manager_set(valve_idx, true);

    /* 2. Attendre ouverture mécanique de l'électrovanne */
    vTaskDelay(pdMS_TO_TICKS(VALVE_OPEN_DELAY_MS));

    /* 3. Démarrer la pompe */
    relay_manager_set(PUMP_RELAY_INDEX, true);

    ESP_LOGI(TAG, "Irrigation active — actifs : %d relais",
             relay_manager_active_count());
}

/**
 * @brief Arrête pompe puis ferme vanne avec délai (ordre correct)
 * @param valve_idx Index relais de la vanne
 */
static void irrigation_stop(int valve_idx)
{
    ESP_LOGI(TAG, "Irrigation STOP — pompe (relais %d) puis vanne (relais %d)",
             PUMP_RELAY_INDEX + 1, valve_idx + 1);

    /* 1. Arrêter la pompe d'abord (évite surpression) */
    relay_manager_set(PUMP_RELAY_INDEX, false);

    /* 2. Attendre arrêt complet de la pompe */
    vTaskDelay(pdMS_TO_TICKS(VALVE_OPEN_DELAY_MS));

    /* 3. Fermer la vanne */
    relay_manager_set(valve_idx, false);

    ESP_LOGI(TAG, "Irrigation arrêtée — actifs : %d relais",
             relay_manager_active_count());
}

/* =========================================================================
 * Initialisations matériel
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

/* =========================================================================
 * Callbacks
 * ========================================================================= */

/**
 * @brief Callback LoRa — données capteur reçues
 * Déclenche irrigation automatique selon seuil humidité
 */
static void on_sensor_data_received(const lora_sensor_data_t *data)
{
    if (!data) return;

    ESP_LOGI(TAG, "─────────────────────────────────────");
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
    ESP_LOGI(TAG, "─────────────────────────────────────");

    /* Publication MQTT */
    network_manager_publish_sensor_data(data);

    /* Log SD card */
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

    /* ── CONTRÔLE AUTOMATIQUE IRRIGATION ── */
    int valve_idx = get_valve_relay_index(data->node_id);

    if (valve_idx < 0) {
        ESP_LOGW(TAG, "node_id %d hors plage vannes (1-%d) — ignore",
                 data->node_id, NUM_RELAYS - 1);
        return;
    }

    if (data->humidity < HUMIDITY_THRESHOLD_ON) {
        /* Humidité critique → démarrer irrigation si pas déjà active */
        if (!relay_manager_get(valve_idx) || !relay_manager_get(PUMP_RELAY_INDEX)) {
            ESP_LOGI(TAG, "Humidite %.1f%% < %.0f%% → Irrigation zone %d ON",
                     data->humidity, HUMIDITY_THRESHOLD_ON, data->node_id);
            irrigation_start(valve_idx);
        } else {
            ESP_LOGI(TAG, "Zone %d déjà en irrigation", data->node_id);
        }
    }
    else if (data->humidity > HUMIDITY_THRESHOLD_OFF) {
        /* Humidité suffisante → arrêter irrigation si active */
        if (relay_manager_get(valve_idx) || relay_manager_get(PUMP_RELAY_INDEX)) {
            ESP_LOGI(TAG, "Humidite %.1f%% > %.0f%% → Irrigation zone %d OFF",
                     data->humidity, HUMIDITY_THRESHOLD_OFF, data->node_id);
            irrigation_stop(valve_idx);
        } else {
            ESP_LOGI(TAG, "Zone %d déjà arrêtée", data->node_id);
        }
    }
    else {
        ESP_LOGI(TAG, "Humidite %.1f%% — zone entre seuils, pas de changement",
                 data->humidity);
    }
}

/**
 * @brief Callback MQTT — commande manuelle depuis app mobile
 *
 * Topics reçus :
 *   robocare/<uid>/valve/control/<zone_id>  → payload "1" ou "0"
 *   robocare/<uid>/pump/control             → payload "1" ou "0"
 *
 * L'ordre vanne/pompe est géré côté Flutter (délai 300ms entre les deux)
 * Donc ici on exécute directement la commande reçue.
 */
static void on_relay_command_received(int relay_idx, bool state)
{
    if (relay_idx < 0 || relay_idx >= NUM_RELAYS) {
        ESP_LOGW(TAG, "Commande relais invalide : %d", relay_idx);
        return;
    }

    /*
     * Pour test 1 électrovanne :
     * Relais 0 = pompe
     * Relais 1 = électrovanne zone 1
     */

    if (relay_idx == VALVE_ZONE1_INDEX) {
        if (state) {
            ESP_LOGI(TAG, "Commande MQTT : irrigation zone 1 ON");
            irrigation_start(VALVE_ZONE1_INDEX);
        } else {
            ESP_LOGI(TAG, "Commande MQTT : irrigation zone 1 OFF");
            irrigation_stop(VALVE_ZONE1_INDEX);
        }
        return;
    }

    if (relay_idx == PUMP_RELAY_INDEX) {
        ESP_LOGI(TAG, "Commande MQTT directe pompe : %s",
                 state ? "ON" : "OFF");
        relay_manager_set(PUMP_RELAY_INDEX, state);
        return;
    }
}

/* =========================================================================
 * Tâches FreeRTOS
 * ========================================================================= */

static void lora_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "Tache LoRa demarree — polling toutes les 50ms");
    while (1) {
        lora_manager_process();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/* =========================================================================
 * Point d'entrée principal
 * ========================================================================= */

void app_main(void)
{
    ESP_LOGI(TAG, "╔══════════════════════════════════════╗");
    ESP_LOGI(TAG, "║  RoboCare — Carte réceptrice v2.0    ║");
    ESP_LOGI(TAG, "║  Pompe + Électrovanne corrigées      ║");
    ESP_LOGI(TAG, "╚══════════════════════════════════════╝");

    /* [0/7] NVS */
    ESP_LOGI(TAG, "[0/7] Initialisation NVS...");
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    strncpy(g_firebase_uid, FIREBASE_UID, sizeof(g_firebase_uid) - 1);
    g_firebase_uid[sizeof(g_firebase_uid) - 1] = '\0';
    ESP_LOGI(TAG, "UID configure : %s", g_firebase_uid);

    /* LED témoin 3.3V */
    led_3v3_init();

    /* [1/7] SPI */
    ESP_LOGI(TAG, "[1/7] Initialisation bus SPI...");
    spi_sd_bus_init();
    spi_lora_bus_init();

    /* [2/7] SD Card */
    ESP_LOGI(TAG, "[2/7] Initialisation SD Card (CS=IO%d)...", SD_CS_PIN);
    sd_manager_set_spi_host(SD_SPI_HOST);
    if (!sd_manager_init(SD_CS_PIN)) {
        ESP_LOGW(TAG, "SD non disponible — log local désactivé");
    }

    /* [3/7] Relais */
    ESP_LOGI(TAG, "[3/7] Initialisation relais...");
    ESP_LOGI(TAG, "  Relais 0 (IO%d) = POMPE", RELAY_PINS[PUMP_RELAY_INDEX]);
    ESP_LOGI(TAG, "  Relais 1 (IO%d) = VANNE zone 1", RELAY_PINS[VALVE_ZONE1_INDEX]);
    ESP_LOGI(TAG, "  Relais 2 (IO%d) = VANNE zone 2", RELAY_PINS[VALVE_ZONE2_INDEX]);
    ESP_LOGI(TAG, "  Relais 3 (IO%d) = VANNE zone 3", RELAY_PINS[VALVE_ZONE3_INDEX]);
    relay_manager_init(RELAY_PINS, NUM_RELAYS);

    /* [4/7] Réseau */
    ESP_LOGI(TAG, "[4/7] Initialisation réseau...");
    network_manager_init(WIFI_SSID, WIFI_PASS, MQTT_SERVER, MQTT_PORT);
    network_manager_set_uid(g_firebase_uid);
    network_manager_set_relay_callback(on_relay_command_received);
    network_manager_start();

    /* [5/7] LoRa */
    ESP_LOGI(TAG, "[5/7] Initialisation LoRa (CS=IO%d RST=IO%d DIO0=IO%d)...",
             LORA_CS_PIN, LORA_RST_PIN, LORA_DIO0_PIN);
    if (lora_manager_init(LORA_CS_PIN, LORA_RST_PIN, LORA_DIO0_PIN)) {
        lora_manager_set_callback(on_sensor_data_received);
        ESP_LOGI(TAG, "LoRa OK — 433 MHz SF12 BW125 CR4/5");
    } else {
        ESP_LOGE(TAG, "LoRa ÉCHEC — vérifier câblage IO33-IO38");
    }

    /* [6/7] Tâche LoRa */
    ESP_LOGI(TAG, "[6/7] Démarrage tâche LoRa...");
    xTaskCreate(lora_task, "lora_task", 4096, NULL, 5, NULL);

    /* [7/7] Prêt */
    ESP_LOGI(TAG, "[7/7] Système prêt");
    ESP_LOGI(TAG, "  UID actif   : %s", g_firebase_uid);
    ESP_LOGI(TAG, "  Seuil ON    : < %.0f%%", HUMIDITY_THRESHOLD_ON);
    ESP_LOGI(TAG, "  Seuil OFF   : > %.0f%%", HUMIDITY_THRESHOLD_OFF);
    ESP_LOGI(TAG, "  Délai vanne : %d ms", VALVE_OPEN_DELAY_MS);
    ESP_LOGI(TAG, "En attente de paquets LoRa...");
}