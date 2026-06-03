#include "network_manager.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_mac.h"
#include "esp_timer.h"
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

static EventGroupHandle_t       s_wifi_event_group = NULL;
static esp_mqtt_client_handle_t s_mqtt_client      = NULL;

/* Callbacks */
static network_manager_relay_cb_t            s_relay_callback            = NULL;
static network_manager_mode_cb_t             s_mode_callback             = NULL;
static network_manager_timed_irrigation_cb_t s_timed_irrigation_callback = NULL;

#define UID_MAX_LEN  64
#define NVS_NS       "robocare"
#define NVS_KEY_UID  "firebase_uid"
static char s_uid[UID_MAX_LEN] = "";
static bool s_uid_received     = false;
static bool s_uid_fixed        = false;

#define MAC_STR_LEN  18
static char s_mac_str[MAC_STR_LEN] = "";

#define WIFI_CONNECTED_BIT  BIT0

#define WIFI_RECONNECT_DELAY_MS_INIT  1000
#define WIFI_RECONNECT_DELAY_MS_MAX  60000
static uint32_t s_reconnect_delay_ms = WIFI_RECONNECT_DELAY_MS_INIT;
static int      s_reconnect_count    = 0;
static esp_timer_handle_t s_wifi_reconnect_timer = NULL;

#define WIFI_CONNECT_TIMEOUT_MS 30000

/* ── Helpers ─────────────────────────────────────────────────────────── */

static void wifi_reconnect_timer_cb(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "Tentative de reconnexion WiFi...");
    esp_wifi_connect();
}

static void schedule_wifi_reconnect(void)
{
    if (!s_wifi_reconnect_timer) {
        const esp_timer_create_args_t timer_args = {
            .callback              = wifi_reconnect_timer_cb,
            .arg                   = NULL,
            .dispatch_method       = ESP_TIMER_TASK,
            .name                  = "wifi_reconnect",
            .skip_unhandled_events = true,
        };
        ESP_ERROR_CHECK(esp_timer_create(&timer_args, &s_wifi_reconnect_timer));
    }

    if (esp_timer_is_active(s_wifi_reconnect_timer)) {
        ESP_ERROR_CHECK(esp_timer_stop(s_wifi_reconnect_timer));
    }

    ESP_ERROR_CHECK(esp_timer_start_once(
        s_wifi_reconnect_timer,
        (uint64_t)s_reconnect_delay_ms * 1000ULL));
}

static void nvs_load_uid(void)
{
    if (s_uid_fixed && strlen(s_uid) > 0) {
        s_uid_received = true;
        ESP_LOGI(TAG, "UID fixe deja configure : %s", s_uid);
        return;
    }

    nvs_handle_t handle;
    esp_err_t ret = nvs_open(NVS_NS, NVS_READONLY, &handle);
    if (ret != ESP_OK) return;

    size_t len = UID_MAX_LEN;
    ret = nvs_get_str(handle, NVS_KEY_UID, s_uid, &len);
    nvs_close(handle);

    if (ret == ESP_OK && strlen(s_uid) > 0) {
        s_uid_received = true;
        ESP_LOGI(TAG, "UID charge depuis NVS : %s", s_uid);
    } else {
        ESP_LOGW(TAG, "Aucun UID en NVS");
    }
}

static void nvs_save_uid(const char *uid)
{
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(NVS_NS, NVS_READWRITE, &handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open echoue : %s", esp_err_to_name(ret));
        return;
    }

    nvs_set_str(handle, NVS_KEY_UID, uid);
    nvs_commit(handle);
    nvs_close(handle);
    ESP_LOGI(TAG, "UID sauvegarde en NVS : %s", uid);
}

static float safe_float(float value)
{
    if (isnan(value) || isinf(value)) {
        return 0.0f;
    }
    return value;
}

static void read_mac_address(void)
{
    uint8_t mac[6];
    esp_wifi_get_mac(ESP_IF_WIFI_STA, mac);
    snprintf(s_mac_str, sizeof(s_mac_str),
             "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

/* ── MQTT Actions ────────────────────────────────────────────────────── */

void network_manager_publish_irrigation_status(int zone, const char *status)
{
    if (!s_mqtt_client || !s_uid_received) return;

    char topic[128];
    snprintf(topic, sizeof(topic), "robocare/%s/zone/%d/irrigation/status", s_uid, zone);

    esp_mqtt_client_publish(s_mqtt_client, topic, status, 0, 1, 0);
    ESP_LOGI(TAG, "Statut irrigation publié [%s]: %s", topic, status);
}

void network_manager_publish_relay_state(bool pump, bool valve)
{
    if (!s_mqtt_client || !s_uid_received) return;

    char topic_pump[128];
    char topic_valve[128];
    
    /* Publier l'état sur des topics STATE pour éviter de réinjecter des commandes. */
    snprintf(topic_pump, sizeof(topic_pump), "robocare/%s/pump/state", s_uid);
    snprintf(topic_valve, sizeof(topic_valve), "robocare/%s/valve/state/1", s_uid);

    esp_mqtt_client_publish(s_mqtt_client, topic_pump, pump ? "1" : "0", 0, 1, 0);
    esp_mqtt_client_publish(s_mqtt_client, topic_valve, valve ? "1" : "0", 0, 1, 0);
    
    ESP_LOGI(TAG, "Etats relais publies - Pompe: %d, Vanne: %d", pump, valve);
}

static void publish_discovery(void)
{
    if (!s_mqtt_client || strlen(s_mac_str) == 0) return;

    char payload[64];
    snprintf(payload, sizeof(payload), "{\"mac\":\"%s\"}", s_mac_str);

    esp_mqtt_client_publish(s_mqtt_client, "robocare/discovery",
                            payload, 0, 1, 0);
    ESP_LOGI(TAG, "Discovery publie : %s", payload);
}

static void subscribe_config_topic(void)
{
    if (!s_mqtt_client || strlen(s_mac_str) == 0) return;

    char config_topic[64];
    snprintf(config_topic, sizeof(config_topic),
             "robocare/config/%s", s_mac_str);

    esp_mqtt_client_subscribe(s_mqtt_client, config_topic, 1);
    ESP_LOGI(TAG, "Subscribe config : %s", config_topic);
}

/* ── Handlers ────────────────────────────────────────────────────────── */

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                                int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        s_reconnect_count++;
        schedule_wifi_reconnect();
        s_reconnect_delay_ms = (s_reconnect_delay_ms * 2 < WIFI_RECONNECT_DELAY_MS_MAX) ? 
                                s_reconnect_delay_ms * 2 : WIFI_RECONNECT_DELAY_MS_MAX;
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        s_reconnect_delay_ms = WIFI_RECONNECT_DELAY_MS_INIT;
        s_reconnect_count    = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base,
                               int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT connecte");
            if (s_uid_received && strlen(s_uid) > 0) {
                /* UID déjà connu (NVS ou set_uid) → s'abonner directement aux topics */
                char sub_topic[128];
                snprintf(sub_topic, sizeof(sub_topic), "robocare/%s/valve/control/+", s_uid);
                esp_mqtt_client_subscribe(s_mqtt_client, sub_topic, 1);
                ESP_LOGI(TAG, "Subscribe : %s", sub_topic);

                snprintf(sub_topic, sizeof(sub_topic), "robocare/%s/mode/control", s_uid);
                esp_mqtt_client_subscribe(s_mqtt_client, sub_topic, 1);
                ESP_LOGI(TAG, "Subscribe : %s", sub_topic);

                snprintf(sub_topic, sizeof(sub_topic), "robocare/%s/irrigation/timed/+", s_uid);
                esp_mqtt_client_subscribe(s_mqtt_client, sub_topic, 1);
                ESP_LOGI(TAG, "Subscribe : %s", sub_topic);
            } else {
                /* UID pas encore connu → discovery pour le recevoir du bridge */
                subscribe_config_topic();
                publish_discovery();
                ESP_LOGW(TAG, "UID inconnu → discovery lancé");
            }
            break;

        case MQTT_EVENT_DATA: {
            if (!event->topic || !event->data) break;
            char topic[128] = {0};
            char payload[128] = {0};
            memcpy(topic, event->topic, event->topic_len < 127 ? event->topic_len : 127);
            memcpy(payload, event->data, event->data_len < 127 ? event->data_len : 127);

            ESP_LOGI(TAG, "MQTT RX [%s] : %s", topic, payload);

            /* Parse topic */
            char *parts[6] = {NULL};
            int np = 0;
            char topic_copy[128] = {0};
            strncpy(topic_copy, topic, 127);
            char *tok = strtok(topic_copy, "/");
            while (tok && np < 6) { parts[np++] = tok; tok = strtok(NULL, "/"); }

            /* UID config */
            if (!s_uid_fixed && np >= 3 && strcmp(parts[1], "config") == 0) {
                if (strcmp(parts[2], s_mac_str) == 0) {
                    strncpy(s_uid, payload, UID_MAX_LEN - 1);
                    s_uid_received = true;
                    nvs_save_uid(s_uid);
                    esp_restart(); // Restart to subscribe to new topics
                }
            }

            /* Commands */
            if (!s_uid_received) break;

            if (np >= 4 && strcmp(parts[2], "mode") == 0) {
                if (s_mode_callback) s_mode_callback(strcmp(payload, "auto") == 0);
            } 
            else if (np >= 4 && strcmp(parts[2], "pump") == 0) {
                ESP_LOGW(TAG, "Commande POMPE ignoree via MQTT (pompe reservee au remplissage auto)");
            }
            else if (np >= 5 && strcmp(parts[2], "valve") == 0) {
                if (s_relay_callback) s_relay_callback(3, strcmp(payload, "1") == 0); // VALVE_RELAY_INDEX = 3
            }
            else if (np >= 5 && strcmp(parts[2], "irrigation") == 0 && strcmp(parts[3], "timed") == 0) {
                if (s_timed_irrigation_callback) {
                    int zone = atoi(parts[4]);
                    s_timed_irrigation_callback(zone, atoi(payload));
                }
            }
            break;
        }
        default: break;
    }
}

/* ── Public API ──────────────────────────────────────────────────────── */

void network_manager_init(const char *ssid, const char *password,
                          const char *mqtt_server, int mqtt_port)
{
    s_wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_config = { .sta = { .threshold.authmode = WIFI_AUTH_WPA2_PSK } };
    strncpy((char *)wifi_config.sta.ssid, ssid, 31);
    strncpy((char *)wifi_config.sta.password, password, 63);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));

    /* Connexion directe par hostname + port + transport.
     * On N'utilise PAS .broker.address.uri car http_parser_parse_url() échoue
     * sur les adresses IPv4 brutes dans certaines versions d'ESP-IDF 5.x,
     * retournant NULL et empêchant tout démarrage MQTT. */
    ESP_LOGI(TAG, "Config MQTT : host=%s port=%d (TCP)", mqtt_server, mqtt_port);

    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.hostname  = mqtt_server,
        .broker.address.port      = (uint32_t)mqtt_port,
        .broker.address.transport = MQTT_TRANSPORT_OVER_TCP,
    };
    s_mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    if (!s_mqtt_client) {
        ESP_LOGE(TAG, "ERREUR : esp_mqtt_client_init() a retourné NULL !");
        return;
    }
    esp_mqtt_client_register_event(s_mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    
    nvs_load_uid();
}

void network_manager_start(void)
{
    esp_wifi_start();
    xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdFALSE, pdMS_TO_TICKS(WIFI_CONNECT_TIMEOUT_MS));
    read_mac_address();
    if (s_mqtt_client) esp_mqtt_client_start(s_mqtt_client);
}

void network_manager_publish_sensor_data(const lora_sensor_data_t *data)
{
    if (!s_mqtt_client || !s_uid_received) return;

    char topic[128];
    snprintf(topic, sizeof(topic), "robocare/%s/zone/%d/sensor/%d/data", s_uid, data->node_id, data->node_id);

    char payload[512];
    snprintf(payload, sizeof(payload),
             "{\"measurements\":{\"moisture_percent\":%.1f,\"temperature_celsius\":%.1f,\"ph\":%.2f,\"conductivity_uS_per_cm\":%.0f},"
             "\"meta\":{\"mac\":\"%s\",\"node_id\":%d,\"rssi\":%d,\"snr\":%.1f}}",
             safe_float(data->humidity), safe_float(data->temperature), safe_float(data->ph), safe_float(data->ec),
             s_mac_str, data->node_id, data->rssi, data->snr);

    esp_mqtt_client_publish(s_mqtt_client, topic, payload, 0, 1, 0);
}

void network_manager_set_relay_callback(network_manager_relay_cb_t cb) { s_relay_callback = cb; }
void network_manager_set_mode_callback(network_manager_mode_cb_t cb) { s_mode_callback = cb; }
void network_manager_set_timed_irrigation_callback(network_manager_timed_irrigation_cb_t cb) { s_timed_irrigation_callback = cb; }

void network_manager_set_uid(const char *uid)
{
    if (!uid || strlen(uid) == 0) return;
    strncpy(s_uid, uid, UID_MAX_LEN - 1);
    s_uid[UID_MAX_LEN - 1] = '\0';
    s_uid_received = true;
    s_uid_fixed    = true;
    nvs_save_uid(s_uid);
    ESP_LOGI(TAG, "UID forcé manuellement : %s", s_uid);
}

bool network_manager_is_connected(void) 
{ 
    return s_wifi_event_group && (xEventGroupGetBits(s_wifi_event_group) & WIFI_CONNECTED_BIT); 
}

bool network_manager_is_provisioned(void) { return s_uid_received; }
const char* network_manager_get_uid(void) { return s_uid_received ? s_uid : NULL; }
const char* network_manager_get_mac(void) { return s_mac_str; }
