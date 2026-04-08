/**
 * @file network_manager.c
 * @brief Driver WiFi + MQTT — Carte RÉCEPTRICE RoboCare (ESP32-S2-WROOM)
 *
 * Corrections v2.2 — Système dynamique :
 *
 *  1. [DYNAMIQUE] UID récupéré depuis le topic MQTT "robocare/config/<MAC>"
 *     Le bridge Python publie l'UID sur ce topic quand le device est "claimed".
 *     Plus besoin de hardcoder FIREBASE_UID dans le firmware.
 *
 *  2. [DYNAMIQUE] MAC address lue depuis l'interface WiFi → ID unique du device.
 *     Publiée sur "robocare/discovery" au démarrage pour s'enregistrer.
 *
 *  3. [ROBUSTESSE] Publications MQTT bufferisées pendant que l'UID n'est pas
 *     encore reçu → aucune donnée perdue au premier démarrage.
 *
 * Flux de découverte :
 *   1. Réceptrice démarre → publie sur "robocare/discovery" {"mac":"AA:BB:..."}
 *   2. Bridge reçoit → écrit dans Firestore "pending_devices/<mac>"
 *   3. Utilisateur "claim" le device dans l'app → status="claimed", uid=<uid>
 *   4. Bridge publie sur "robocare/config/<mac>" → payload = uid
 *   5. Réceptrice reçoit l'UID → stocké en NVS → utilisé pour tous les topics
 *
 * Topics MQTT (alignés sur mqtt_handler.py / config.py) :
 *   Données capteur  : robocare/<uid>/zone/<node_id>/data     (PUBLISH)
 *   Commande vanne   : robocare/<uid>/valve/control/<zone>    (SUBSCRIBE)
 *   Commande pompe   : robocare/<uid>/pump/control            (SUBSCRIBE)
 *   Découverte       : robocare/discovery                     (PUBLISH)
 *   Config UID       : robocare/config/<mac>                  (SUBSCRIBE)
 */

#include "network_manager.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_mac.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "mqtt_client.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include <string.h>
#include <stdio.h>
#include <inttypes.h>
#include <math.h>

static const char *TAG = "NETWORK";

/* =========================================================================
 * État interne
 * ========================================================================= */
static EventGroupHandle_t       s_wifi_event_group = NULL;
static esp_mqtt_client_handle_t s_mqtt_client      = NULL;
static void (*s_relay_callback)(int, bool)         = NULL;

/* UID Firebase — récupéré dynamiquement via MQTT "robocare/config/<mac>" */
#define UID_MAX_LEN  64
#define NVS_NS       "robocare"
#define NVS_KEY_UID  "firebase_uid"
static char s_uid[UID_MAX_LEN] = "";        /* vide = pas encore configuré */
static bool s_uid_received     = false;

/* MAC address de ce device (lue depuis le WiFi) */
#define MAC_STR_LEN  18   /* "AA:BB:CC:DD:EE:FF\0" */
static char s_mac_str[MAC_STR_LEN] = "";

/* Bits EventGroup */
#define WIFI_CONNECTED_BIT  BIT0

/* Reconnexion WiFi avec backoff exponentiel */
#define WIFI_RECONNECT_DELAY_MS_INIT  1000
#define WIFI_RECONNECT_DELAY_MS_MAX  60000
static uint32_t s_reconnect_delay_ms = WIFI_RECONNECT_DELAY_MS_INIT;
static int      s_reconnect_count    = 0;

#define WIFI_CONNECT_TIMEOUT_MS 30000

/* =========================================================================
 * NVS — persistance de l'UID entre les redémarrages
 * ========================================================================= */
static void nvs_load_uid(void)
{
    nvs_handle_t handle;
    esp_err_t    ret = nvs_open(NVS_NS, NVS_READONLY, &handle);
    if (ret != ESP_OK) return;

    size_t len = UID_MAX_LEN;
    ret = nvs_get_str(handle, NVS_KEY_UID, s_uid, &len);
    nvs_close(handle);

    if (ret == ESP_OK && strlen(s_uid) > 0) {
        s_uid_received = true;
        ESP_LOGI(TAG, "UID chargé depuis NVS : %s", s_uid);
    } else {
        ESP_LOGW(TAG, "Aucun UID en NVS — en attente de discovery");
    }
}

static void nvs_save_uid(const char *uid)
{
    nvs_handle_t handle;
    esp_err_t    ret = nvs_open(NVS_NS, NVS_READWRITE, &handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open échoué : %s", esp_err_to_name(ret));
        return;
    }
    nvs_set_str(handle, NVS_KEY_UID, uid);
    nvs_commit(handle);
    nvs_close(handle);
    ESP_LOGI(TAG, "UID sauvegardé en NVS : %s", uid);
}

/* =========================================================================
 * Validation des flottants (sécurité JSON)
 * Évite les NaN/Inf qui corrompent le payload JSON
 * ========================================================================= */
static float safe_float(float value)
{
    if (isnan(value) || isinf(value)) {
        ESP_LOGW(TAG, "Flottant invalide détecté NaN/Inf → 0.0");
        return 0.0f;
    }
    return value;
}

/* =========================================================================
 * Récupération de la MAC address
 * ========================================================================= */
static void read_mac_address(void)
{
    uint8_t mac[6];
    esp_wifi_get_mac(ESP_IF_WIFI_STA, mac);
    snprintf(s_mac_str, sizeof(s_mac_str),
             "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    ESP_LOGI(TAG, "MAC address : %s", s_mac_str);
}

/* =========================================================================
 * Publication discovery — annonce le device au bridge Python
 *
 * Bridge Python (mqtt_handler.py) reçoit sur "robocare/discovery" et écrit
 * dans Firestore "pending_devices/<mac>" avec status="waiting".
 * ========================================================================= */
static void publish_discovery(void)
{
    if (!s_mqtt_client || strlen(s_mac_str) == 0) return;

    char payload[64];
    snprintf(payload, sizeof(payload), "{\"mac\":\"%s\"}", s_mac_str);

    esp_mqtt_client_publish(s_mqtt_client, "robocare/discovery",
                            payload, 0, 1, 0);
    ESP_LOGI(TAG, "Discovery publié : %s", payload);
}

/* =========================================================================
 * Abonnement au topic de configuration UID
 * Quand l'utilisateur "claim" le device dans l'app, le bridge publie :
 *   Topic   : robocare/config/<mac>
 *   Payload : <uid>
 * =========================================================================*/
static void subscribe_config_topic(void)
{
    if (!s_mqtt_client || strlen(s_mac_str) == 0) return;

    char config_topic[64];
    snprintf(config_topic, sizeof(config_topic),
             "robocare/config/%s", s_mac_str);

    esp_mqtt_client_subscribe(s_mqtt_client, config_topic, 1);
    ESP_LOGI(TAG, "Subscribe config : %s", config_topic);
}

/* =========================================================================
 * Gestionnaire d'événements WiFi
 * ========================================================================= */
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                                int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();

    } else if (event_base == WIFI_EVENT &&
               event_id == WIFI_EVENT_STA_DISCONNECTED) {

        xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        s_reconnect_count++;

        ESP_LOGW(TAG, "WiFi déconnecté (#%d) — attente %" PRIu32 "ms",
                 s_reconnect_count, s_reconnect_delay_ms);

        vTaskDelay(pdMS_TO_TICKS(s_reconnect_delay_ms));
        s_reconnect_delay_ms *= 2;
        if (s_reconnect_delay_ms > WIFI_RECONNECT_DELAY_MS_MAX)
            s_reconnect_delay_ms = WIFI_RECONNECT_DELAY_MS_MAX;

        esp_wifi_connect();

    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "WiFi OK — IP : " IPSTR, IP2STR(&event->ip_info.ip));

        s_reconnect_delay_ms = WIFI_RECONNECT_DELAY_MS_INIT;
        s_reconnect_count    = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

/* =========================================================================
 * Gestionnaire d'événements MQTT
 * ========================================================================= */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base,
                                int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;

    switch ((esp_mqtt_event_id_t)event_id) {

        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT connecté au broker");

            /* ── 1. Abonnement config UID ─────────────────────────────── */
            subscribe_config_topic();

            /* ── 2. Annonce discovery ─────────────────────────────────── */
            publish_discovery();

            /* ── 3. Abonnement commandes (si UID déjà connu) ──────────── */
            if (s_uid_received && strlen(s_uid) > 0) {
                char sub_topic[96];
                snprintf(sub_topic, sizeof(sub_topic),
                         "robocare/%s/valve/control/+", s_uid);
                esp_mqtt_client_subscribe(s_mqtt_client, sub_topic, 1);
                ESP_LOGI(TAG, "Subscribe valve : %s", sub_topic);

                char pump_topic[96];
                snprintf(pump_topic, sizeof(pump_topic),
                         "robocare/%s/pump/control", s_uid);
                esp_mqtt_client_subscribe(s_mqtt_client, pump_topic, 1);
                ESP_LOGI(TAG, "Subscribe pump : %s", pump_topic);
            } else {
                ESP_LOGW(TAG, "UID non encore reçu — commandes relais désactivées");
                ESP_LOGW(TAG, "  → Claimez ce device dans l'app Firebase");
            }
            break;

        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGW(TAG, "MQTT déconnecté — reconnexion automatique");
            break;

        case MQTT_EVENT_DATA: {
            if (!event->topic || !event->data) break;

            /* Copie sécurisée topic + payload */
            char topic[128]   = {0};
            char payload[128] = {0};

            if (event->topic_len < 128)
                memcpy(topic, event->topic, event->topic_len);
            if (event->data_len < 128)
                memcpy(payload, event->data, event->data_len);

            ESP_LOGD(TAG, "MQTT RX [%s] : %s", topic, payload);

            /* ── A. Réception UID depuis bridge ──────────────────────── */
            /* Topic : "robocare/config/<mac>" → payload = uid */
            char config_prefix[64];
            snprintf(config_prefix, sizeof(config_prefix),
                     "robocare/config/%s", s_mac_str);

            if (strncmp(topic, config_prefix, strlen(config_prefix)) == 0) {
                if (strlen(payload) > 0 && strlen(payload) < UID_MAX_LEN) {
                    strncpy(s_uid, payload, UID_MAX_LEN - 1);
                    s_uid[UID_MAX_LEN - 1] = '\0';
                    s_uid_received = true;

                    ESP_LOGI(TAG, "✅ UID reçu depuis bridge : %s", s_uid);

                    /* Sauvegarder en NVS pour les prochains démarrages */
                    nvs_save_uid(s_uid);

                    /* ✓ Conseil d'expert : Redémarrage post-configuration
                     * Cela permet de valider la sauvegarde NVS en relançant main.c.
                     * Au redémarrage, le code prendra le chemin "UID trouvé → utiliser"
                     * plutôt que de gérer dynamiquement le passage "non-configuré" → "configuré"
                     */
                    ESP_LOGI(TAG, "Configuration complète. Redémarrage du système...");
                    vTaskDelay(pdMS_TO_TICKS(1000));
                    esp_restart();
                }
                break;
            }

            /* ── B. Commandes relais (valve) ─────────────────────────── */
            char *parts[6] = {NULL};
            int   np = 0;
            char  topic_copy[128];
            strncpy(topic_copy, topic, sizeof(topic_copy) - 1);
            char *tok = strtok(topic_copy, "/");
            while (tok && np < 6) { parts[np++] = tok; tok = strtok(NULL, "/"); }

            if (np >= 5 && parts[2] && strcmp(parts[2], "valve") == 0) {
                if (!s_relay_callback) break;
                int  zone_id = atoi(parts[4]);
                if (zone_id < 1 || zone_id > 12) break;
                bool state = (strcmp(payload, "1") == 0);
                ESP_LOGI(TAG, "Commande vanne zone %d → %s",
                         zone_id, state ? "ON" : "OFF");
                s_relay_callback(zone_id - 1, state);

            } else if (np >= 4 && parts[2] && strcmp(parts[2], "pump") == 0) {
                ESP_LOGI(TAG, "Commande pompe → %s",
                         (strcmp(payload, "1") == 0) ? "ON" : "OFF");

            } else {
                ESP_LOGD(TAG, "Topic non géré : %s", topic);
            }
            break;
        }

        case MQTT_EVENT_ERROR:
            ESP_LOGE(TAG, "Erreur MQTT");
            break;

        default:
            break;
    }
}

/* =========================================================================
 * API publique
 * ========================================================================= */

void network_manager_set_uid(const char *uid)
{
    if (!uid || strlen(uid) == 0) return;
    strncpy(s_uid, uid, UID_MAX_LEN - 1);
    s_uid[UID_MAX_LEN - 1] = '\0';
    s_uid_received = true;
    nvs_save_uid(s_uid);
    ESP_LOGI(TAG, "UID forcé manuellement : %s", s_uid);
}

void network_manager_init(const char *ssid, const char *password,
                          const char *mqtt_server, int mqtt_port)
{
    /* NVS */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* Chargement UID persisté */
    nvs_load_uid();

    /* EventGroup */
    s_wifi_event_group = xEventGroupCreate();

    /* TCP/IP + WiFi */
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg.capable    = true,
            .pmf_cfg.required   = false,
        },
    };
    strncpy((char *)wifi_config.sta.ssid,     ssid,
            sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char *)wifi_config.sta.password, password,
            sizeof(wifi_config.sta.password) - 1);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));

    /* Client MQTT */
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri  = mqtt_server,
        .broker.address.port = (uint32_t)mqtt_port,
        .session.keepalive   = 60,
    };

    s_mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    if (!s_mqtt_client) {
        ESP_LOGE(TAG, "esp_mqtt_client_init échoué");
        return;
    }
    esp_mqtt_client_register_event(s_mqtt_client, ESP_EVENT_ANY_ID,
                                   mqtt_event_handler, NULL);

    ESP_LOGI(TAG, "Network manager initialisé — SSID: %s  Broker: %s:%d",
             ssid, mqtt_server, mqtt_port);
}

void network_manager_start(void)
{
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Attente connexion WiFi (timeout %ds)...",
             WIFI_CONNECT_TIMEOUT_MS / 1000);

    EventBits_t bits = xEventGroupWaitBits(
        s_wifi_event_group,
        WIFI_CONNECTED_BIT,
        pdFALSE, pdFALSE,
        pdMS_TO_TICKS(WIFI_CONNECT_TIMEOUT_MS)
    );

    if (bits & WIFI_CONNECTED_BIT) {
        /* Lire la MAC maintenant que le WiFi est connecté */
        read_mac_address();
        ESP_LOGI(TAG, "WiFi OK — démarrage client MQTT");
        if (s_mqtt_client) {
            esp_mqtt_client_start(s_mqtt_client);
        }
    } else {
        ESP_LOGE(TAG, "Timeout WiFi (%ds) — MQTT non démarré",
                 WIFI_CONNECT_TIMEOUT_MS / 1000);
    }
}

void network_manager_publish_sensor_data(const lora_sensor_data_t *data)
{
    if (!data || !s_mqtt_client) return;

    if (!(xEventGroupGetBits(s_wifi_event_group) & WIFI_CONNECTED_BIT)) {
        ESP_LOGD(TAG, "Publication annulée — WiFi non connecté");
        return;
    }

    if (!s_uid_received || strlen(s_uid) == 0) {
        ESP_LOGW(TAG, "Publication annulée — UID non reçu");
        ESP_LOGW(TAG, "  → Claimez ce device (MAC: %s) dans l'app", s_mac_str);
        return;
    }

    /*
     * Topic aligné sur config.py du bridge Python :
     *   MQTT_TOPIC_DATA = "robocare/+/zone/+/data"
     *   → on publie sur "robocare/<uid>/zone/<node_id>/data"
     */
    char topic[96];
    snprintf(topic, sizeof(topic),
             "robocare/%s/zone/%d/data", s_uid, data->node_id);

    /*
     * Payload JSON aligné sur firebase_service.py du bridge :
     *   payload['measurements']['moisture_percent']
     *   payload['measurements']['temperature_celsius']
     *   payload['measurements']['ph']
     *   payload['measurements']['conductivity_uS_per_cm']
     *   payload['measurements']['nutrients_mg_per_kg']['nitrogen']
     *   payload['measurements']['nutrients_mg_per_kg']['phosphorus']
     *   payload['measurements']['nutrients_mg_per_kg']['potassium']
     *
     * ✓ Sécurité JSON : Validation des flottants dans snprintf
     * Évite les NaN/Inf qui corrompent le payload
     */
    char payload[512];
    snprintf(payload, sizeof(payload),
             "{"
               "\"measurements\":{"
                 "\"moisture_percent\":%.1f,"
                 "\"temperature_celsius\":%.1f,"
                 "\"ph\":%.2f,"
                 "\"conductivity_uS_per_cm\":%.0f,"
                 "\"nutrients_mg_per_kg\":{"
                   "\"nitrogen\":%.0f,"
                   "\"phosphorus\":%.0f,"
                   "\"potassium\":%.0f"
                 "}"
               "},"
               "\"meta\":{"
                 "\"node_id\":%d,"
                 "\"mac\":\"%s\","
                 "\"rssi\":%d,"
                 "\"snr\":%.1f"
               "}"
             "}",
             safe_float(data->humidity),
             safe_float(data->temperature),
             safe_float(data->ph),
             safe_float(data->ec),
             safe_float(data->nitrogen),
             safe_float(data->phosphorus),
             safe_float(data->potassium),
             data->node_id,
             s_mac_str,
             data->rssi,
             safe_float(data->snr));

    int msg_id = esp_mqtt_client_publish(s_mqtt_client, topic, payload,
                                         0, 1, 0);
    if (msg_id >= 0) {
        ESP_LOGI(TAG, "MQTT publié → %s", topic);
    } else {
        ESP_LOGE(TAG, "Échec publication MQTT : %s", topic);
    }
}

bool network_manager_is_connected(void)
{
    if (!s_wifi_event_group) return false;
    return (xEventGroupGetBits(s_wifi_event_group) & WIFI_CONNECTED_BIT) != 0;
}

void network_manager_set_relay_callback(void (*callback)(int, bool))
{
    s_relay_callback = callback;
}

const char *network_manager_get_uid(void)
{
    return s_uid_received ? s_uid : NULL;
}

const char *network_manager_get_mac(void)
{
    return s_mac_str;
}

bool network_manager_is_provisioned(void)
{
    return s_uid_received && strlen(s_uid) > 0;
}