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
 * Mapping relais.
 *
 * IMPORTANT: selon la version de ta carte, "OUTPUT7" peut être câblé sur IO8
 * ou IO2. Pour trancher, change uniquement VALVE_GPIO_PIN (8 ↔ 2) et reflashe.
 */
#ifndef PUMP_GPIO_PIN
#define PUMP_GPIO_PIN  3   /* Correspond à OUTPUT3 */
#endif
#ifndef VALVE_GPIO_PIN
#define VALVE_GPIO_PIN 8   /* Correspond à OUTPUT7 */
#endif

const int RELAY_PINS[] = { 5, 4, PUMP_GPIO_PIN, VALVE_GPIO_PIN };

#define NUM_RELAYS          4
#define PUMP_RELAY_INDEX    2   /* OUTPUT3 / IO3 = POMPE */
#define VALVE_RELAY_INDEX   3   /* OUTPUT7 / IOx = VANNE (voir VALVE_GPIO_PIN) */

#define HUMIDITY_THRESHOLD_ON   30.0f
#define HUMIDITY_THRESHOLD_OFF  60.0f
#define VALVE_DELAY_MS          500

#define WIFI_SSID    "salut"
#define WIFI_PASS    "hey0000."
#define MQTT_BROKER  "mqtt://broker.hivemq.com"
#define MQTT_SERVER  MQTT_BROKER   /* alias pour compatibilité */
#define MQTT_PORT    1883
#define FIREBASE_UID "2SKcuqIcjSb3a2B6NWs2LebCO4g2"

static char g_firebase_uid[128] = {0};

/* ═══════════════════════════════════════════════════════════
 * Prototypes
 * ═══════════════════════════════════════════════════════════ */
static bool irrigation_active = false;
static bool manual_override   = false;
static void led_3v3_init(void);
static void spi_sd_bus_init(void);
static void spi_lora_bus_init(void);
static void irrigation_start(void);
static void irrigation_stop(void);
static void on_sensor_data_received(const lora_sensor_data_t *data);
static void on_relay_command_received(int relay_idx, bool state);
static void on_mode_changed(bool auto_mode);
static void lora_task(void *arg);

/* ═══════════════════════════════════════════════════════════
 * Sécurité démarrage — force GPIO HIGH (relais actif LOW = OFF)
 * ✅ CORRIGÉ : Ajout de IO8 pour la vanne
 * ═══════════════════════════════════════════════════════════ */
static void gpio_safety_init(void)
{
    /* Configuration des pins POMPE (IO3) et VANNE (IO8) + autres */
    gpio_config_t safe = {
        .pin_bit_mask = (1ULL << VALVE_GPIO_PIN) | (1ULL << PUMP_GPIO_PIN) |
                        (1ULL << 4) | (1ULL << 5),    /* IO4 et IO5 (libres) */
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,           /* ← Activer pull-up ! */
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&safe);

    /* Forcer niveau INACTIF (OFF) selon le câblage actif LOW/HIGH */
    const int off_level = relay_manager_inactive_level();
    gpio_set_level(VALVE_GPIO_PIN, off_level);  /* VANNE OFF */
    gpio_set_level(PUMP_GPIO_PIN, off_level);   /* POMPE OFF */
    gpio_set_level(4, off_level);  /* OUTPUT2 OFF */
    gpio_set_level(5, off_level);  /* OUTPUT1 OFF */
    
    ESP_LOGI("SAFETY", "GPIO forcés OFF(level=%d) — POMPE IO%d, VANNE IO%d",
             off_level, PUMP_GPIO_PIN, VALVE_GPIO_PIN);
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
    bool valve_on = relay_manager_get(VALVE_RELAY_INDEX);
    bool pump_on  = relay_manager_get(PUMP_RELAY_INDEX);

    if (valve_on && pump_on) {
        ESP_LOGI(TAG, "Irrigation déjà active");
        return;
    }

    ESP_LOGI(TAG, "🌱 Irrigation START : VANNE (IO8) puis POMPE (IO3)");

    if (!valve_on) {
        relay_manager_set(VALVE_RELAY_INDEX, true);   /* ouvre vanne */
        vTaskDelay(pdMS_TO_TICKS(VALVE_DELAY_MS));
    }
    if (!pump_on) {
        relay_manager_set(PUMP_RELAY_INDEX, true);    /* démarre pompe */
    }
}

static void irrigation_stop(void)
{
    bool valve_on = relay_manager_get(VALVE_RELAY_INDEX);
    bool pump_on  = relay_manager_get(PUMP_RELAY_INDEX);

    if (!valve_on && !pump_on) {
        ESP_LOGI(TAG, "Irrigation déjà arrêtée");
        return;
    }

    ESP_LOGI(TAG, "🛑 Irrigation STOP : POMPE (IO3) puis VANNE (IO8)");

    if (pump_on) {
        relay_manager_set(PUMP_RELAY_INDEX, false);   // Arrête pompe
        vTaskDelay(pdMS_TO_TICKS(VALVE_DELAY_MS));    // Attente 500ms
    }
    if (valve_on) {
        relay_manager_set(VALVE_RELAY_INDEX, false);  // Ferme vanne
    }
}

/* ═══════════════════════════════════════════════════════════
 * Callback LoRa
 * ═══════════════════════════════════════════════════════════ */
static void on_sensor_data_received(const lora_sensor_data_t *data)
{
    if (!data) return;

    ESP_LOGI(TAG, "📡 LoRa RX — node=%d hum=%.1f%%", 
             data->node_id, data->humidity);

    /* Envoi MQTT */
    network_manager_publish_sensor_data(data);

    /* Log SD (si utilisé) */
    sd_manager_log_data(data);

    /* Ignorer AUTO si manuel actif */
    if (manual_override) {
        ESP_LOGW(TAG, "Mode MANUEL actif → AUTO ignoré");
        return;
    }

    /* Logique AUTO */
    if (data->humidity < HUMIDITY_THRESHOLD_ON && !irrigation_active)
    {
        ESP_LOGI(TAG, "🌱 AUTO → START irrigation (hum=%.1f%% < %d%%)", 
                 data->humidity, (int)HUMIDITY_THRESHOLD_ON);
        irrigation_start();
        irrigation_active = true;
    }
    else if (data->humidity > HUMIDITY_THRESHOLD_OFF && irrigation_active)
    {
        ESP_LOGI(TAG, "🛑 AUTO → STOP irrigation (hum=%.1f%% > %d%%)", 
                 data->humidity, (int)HUMIDITY_THRESHOLD_OFF);
        irrigation_stop();
        irrigation_active = false;
    }
    else
    {
        ESP_LOGI(TAG, "AUTO → aucune action (hum=%.1f%%)", data->humidity);
    }
}

/* ═══════════════════════════════════════════════════════════
 * Callback MQTT
 * ═══════════════════════════════════════════════════════════ */
static void on_relay_command_received(int relay_idx, bool state)
{
    ESP_LOGI(TAG, "📱 Commande MQTT reçue — relay_idx=%d state=%s",
             relay_idx, state ? "ON" : "OFF");

    /* Activer mode manuel (désactive la logique AUTO) */
    manual_override = true;

    if (relay_idx == PUMP_RELAY_INDEX) {
        if (relay_manager_get(PUMP_RELAY_INDEX) == state) {
            ESP_LOGI(TAG, "Pompe déjà à l'état demandé → ignoré");
            goto done;
        }
        if (state) {
            /* Sécurité: ouvrir la vanne avant d'allumer la pompe */
            if (!relay_manager_get(VALVE_RELAY_INDEX)) {
                relay_manager_set(VALVE_RELAY_INDEX, true);
                vTaskDelay(pdMS_TO_TICKS(VALVE_DELAY_MS));
            }
            relay_manager_set(PUMP_RELAY_INDEX, true);
        } else {
            relay_manager_set(PUMP_RELAY_INDEX, false);
        }
    } else if (relay_idx == VALVE_RELAY_INDEX) {
        if (relay_manager_get(VALVE_RELAY_INDEX) == state) {
            ESP_LOGI(TAG, "Vanne déjà à l'état demandé → ignoré");
            goto done;
        }
        if (state) {
            relay_manager_set(VALVE_RELAY_INDEX, true);
        } else {
            /* Sécurité: couper la pompe avant de fermer la vanne */
            if (relay_manager_get(PUMP_RELAY_INDEX)) {
                relay_manager_set(PUMP_RELAY_INDEX, false);
                vTaskDelay(pdMS_TO_TICKS(VALVE_DELAY_MS));
            }
            relay_manager_set(VALVE_RELAY_INDEX, false);
        }
    } else {
        ESP_LOGW(TAG, "Commande relais ignorée (index non géré): %d", relay_idx);
    }

done:
    /* État global irrigation = (pompe ON et vanne ON) */
    irrigation_active =
        relay_manager_get(PUMP_RELAY_INDEX) && relay_manager_get(VALVE_RELAY_INDEX);
}

/* ═══════════════════════════════════════════════════════════
 * Callback changement de mode AUTO / MANUEL
 * Topic MQTT : robocare/{uid}/mode/control
 * Payload    : "auto"   → retour en mode automatique
 *              "manual" → forçage mode manuel
 * ═══════════════════════════════════════════════════════════ */
static void on_mode_changed(bool auto_mode)
{
    ESP_LOGI(TAG, "🔄 Mode changé via MQTT : %s", auto_mode ? "AUTO" : "MANUEL");
    manual_override = !auto_mode;
}

/* ── Irrigation temporisée ── */
typedef struct {
    int zone;
    int seconds;
} timed_irrig_params_t;

static void timed_irrigation_task(void *pvParameters)
{
    timed_irrig_params_t *params = (timed_irrig_params_t *)pvParameters;
    int zone = params->zone;
    int seconds = params->seconds;
    
    ESP_LOGI(TAG, "🕒 Lancement irrigation temporisée : %d secondes (Zone %d)", seconds, zone);

    manual_override = true; 
    irrigation_start();
    network_manager_publish_irrigation_status(zone, "STARTED");

    vTaskDelay(pdMS_TO_TICKS(seconds * 1000));

    ESP_LOGI(TAG, "🕒 Temps écoulé, arrêt de l'irrigation...");
    irrigation_stop();

    /* Notification MQTT pour l'application mobile (status + mise à jour des switches) */
    network_manager_publish_irrigation_status(zone, "FINISHED");
    network_manager_publish_relay_state(false, false);

    free(params);
    vTaskDelete(NULL);
}

static void on_timed_irrigation_received(int zone, int seconds)
{
    if (seconds <= 0) {
        ESP_LOGW(TAG, "Durée invalide reçue: %d", seconds);
        irrigation_stop();
        return;
    }

    timed_irrig_params_t *params = malloc(sizeof(timed_irrig_params_t));
    if (params) {
        params->zone = zone;
        params->seconds = seconds;
        xTaskCreate(timed_irrigation_task, "timed_irrig", 4096, (void*)params, 5, NULL);
    }
}

/* ═══════════════════════════════════════════════════════════
 * Tâche LoRa
 * ═══════════════════════════════════════════════════════════ */
static void lora_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "🔄 Tâche LoRa démarrée");
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
    ESP_LOGI(TAG, "╔══════════════════════════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║     ROBOCARE - CARTE RÉCEPTRICE v2.1                         ║");
    ESP_LOGI(TAG, "║     POMPE: IO%d | VANNE: IO%d                              ║", PUMP_GPIO_PIN, VALVE_GPIO_PIN);
    ESP_LOGI(TAG, "╚══════════════════════════════════════════════════════════════╝");

    /* Sécurité : GPIO HIGH avant tout */
    gpio_safety_init();
    vTaskDelay(pdMS_TO_TICKS(500));

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
        ESP_LOGW(TAG, "⚠️ SD non disponible (optionnel)");
    }

    /* Relais - NOUVEAU MAPPING avec VANNE sur IO8 */
    ESP_LOGI(TAG, " Init relais — POMPE:IO3 | VANNE:IO8");
    relay_manager_init(RELAY_PINS, NUM_RELAYS);
    
    /* Forcer OFF au démarrage */
    relay_manager_set(PUMP_RELAY_INDEX, false);
    relay_manager_set(VALVE_RELAY_INDEX, false);
    
    /* Diagnostic : vérifier l'état des GPIO */
    ESP_LOGI(TAG, "=== DIAGNOSTIC GPIO ===");
    int on_level = relay_manager_active_level();
    ESP_LOGI(TAG, "IO%d (POMPE) = %d (%s)",
             PUMP_GPIO_PIN, gpio_get_level(PUMP_GPIO_PIN),
             gpio_get_level(PUMP_GPIO_PIN) == on_level ? "ON" : "OFF");
    ESP_LOGI(TAG, "IO%d (VANNE) = %d (%s)", VALVE_GPIO_PIN, gpio_get_level(VALVE_GPIO_PIN),
             gpio_get_level(VALVE_GPIO_PIN) == on_level ? "ON" : "OFF");

    /* Réseau */
    ESP_LOGI(TAG, "📡 Connexion WiFi/MQTT...");
    network_manager_init(WIFI_SSID, WIFI_PASS, MQTT_BROKER, MQTT_PORT);
    /* Enregistrement des callbacks MQTT */
    network_manager_set_relay_callback(on_relay_command_received);
    network_manager_set_mode_callback(on_mode_changed);
    network_manager_set_timed_irrigation_callback(on_timed_irrigation_received);   /* ← NOUVEAU */
    network_manager_start();

    /* LoRa */
    ESP_LOGI(TAG, "📻 Initialisation LoRa...");
    if (lora_manager_init(LORA_CS_PIN, LORA_RST_PIN, LORA_DIO0_PIN)) {
        lora_manager_set_callback(on_sensor_data_received);
        ESP_LOGI(TAG, "✅ LoRa OK");
        xTaskCreate(lora_task, "lora_task", 4096, NULL, 5, NULL);
    } else {
        ESP_LOGE(TAG, "❌ LoRa ÉCHEC");
    }

    ESP_LOGI(TAG, "══════════════════════════════════════════════");
    ESP_LOGI(TAG, "  SYSTÈME PRÊT  v2.1");
    ESP_LOGI(TAG, "  POMPE IO%d | VANNE IO%d", PUMP_GPIO_PIN, VALVE_GPIO_PIN);
    ESP_LOGI(TAG, "  AUTO  : hum < 30%% → START | hum > 60%% → STOP");
    ESP_LOGI(TAG, "  MANUEL: topic mode/control payload=manual");
    ESP_LOGI(TAG, "  RETOUR AUTO : topic mode/control payload=auto");
    ESP_LOGI(TAG, "══════════════════════════════════════════════");
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}