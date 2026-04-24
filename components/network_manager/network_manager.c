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
static void (*s_relay_callback)(int, bool)         = NULL;

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
            .callback = wifi_reconnect_timer_cb,
            .arg = NULL,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "wifi_reconnect",
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
        ESP_LOGW(TAG, "Flottant invalide detecte NaN/Inf -> 0.0");
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
    ESP_LOGI(TAG, "MAC address : %s", s_mac_str);
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
    snprintf(config_topic, sizeof(config_topic), "robocare/config/%s", s_mac_str);

    esp_mqtt_client_subscribe(s_mqtt_client, config_topic, 1);
    ESP_LOGI(TAG, "Subscribe config : %s", config_topic);
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    (void)arg;
    (void)event_data;

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        s_reconnect_count++;

        ESP_LOGW(TAG, "WiFi deconnecte (#%d) - attente %" PRIu32 "ms",
                 s_reconnect_count, s_reconnect_delay_ms);

        schedule_wifi_reconnect();
        s_reconnect_delay_ms *= 2;
        if (s_reconnect_delay_ms > WIFI_RECONNECT_DELAY_MS_MAX) {
            s_reconnect_delay_ms = WIFI_RECONNECT_DELAY_MS_MAX;
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "WiFi OK - IP : " IPSTR, IP2STR(&event->ip_info.ip));

        if (s_wifi_reconnect_timer && esp_timer_is_active(s_wifi_reconnect_timer)) {
            ESP_ERROR_CHECK(esp_timer_stop(s_wifi_reconnect_timer));
        }
        s_reconnect_delay_ms = WIFI_RECONNECT_DELAY_MS_INIT;
        s_reconnect_count    = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base,
                               int32_t event_id, void *event_data)
{
    (void)handler_args;
    (void)base;
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;

    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED: {
            ESP_LOGI(TAG, "MQTT connecte au broker");

            if (!s_uid_fixed) {
                subscribe_config_topic();
                publish_discovery();
            }

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
            } else if (!s_uid_fixed) {
                ESP_LOGW(TAG, "UID non recu - commandes relais desactivees");
            }
            break;
        }

        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGW(TAG, "MQTT deconnecte - reconnexion automatique");
            break;

        case MQTT_EVENT_DATA: {
            if (!event->topic || !event->data) break;

            char topic[128] = {0};
            char payload[128] = {0};

            if (event->topic_len < (int)sizeof(topic)) {
                memcpy(topic, event->topic, event->topic_len);
            }
            if (event->data_len < (int)sizeof(payload)) {
                memcpy(payload, event->data, event->data_len);
            }

            ESP_LOGD(TAG, "MQTT RX [%s] : %s", topic, payload);

            if (!s_uid_fixed) {
                char config_prefix[64];
                snprintf(config_prefix, sizeof(config_prefix),
                         "robocare/config/%s", s_mac_str);

                if (strncmp(topic, config_prefix, strlen(config_prefix)) == 0) {
                    if (strlen(payload) > 0 && strlen(payload) < UID_MAX_LEN) {
                        strncpy(s_uid, payload, UID_MAX_LEN - 1);
                        s_uid[UID_MAX_LEN - 1] = '\0';
                        s_uid_received = true;

                        ESP_LOGI(TAG, "UID recu depuis bridge : %s", s_uid);
                        nvs_save_uid(s_uid);
                    }
                    break;
                }
            }

            char *parts[6] = {NULL};
            int np = 0;
            char topic_copy[128] = {0};
            strncpy(topic_copy, topic, sizeof(topic_copy) - 1);
            char *tok = strtok(topic_copy, "/");
            while (tok && np < 6) {
                parts[np++] = tok;
                tok = strtok(NULL, "/");
            }

            if (np >= 5 && parts[2] && strcmp(parts[2], "valve") == 0) {
                if (!s_relay_callback) break;
                int zone_id = atoi(parts[4]);
                if (zone_id < 1 || zone_id > 12) break;
                bool state = (strcmp(payload, "1") == 0);
                ESP_LOGI(TAG, "Commande vanne zone %d -> %s",
                         zone_id, state ? "ON" : "OFF");
                s_relay_callback(zone_id - 1, state);
            } else if (np >= 4 && parts[2] && strcmp(parts[2], "pump") == 0) {
                ESP_LOGI(TAG, "Commande pompe -> %s",
                         (strcmp(payload, "1") == 0) ? "ON" : "OFF");
            } else {
                ESP_LOGD(TAG, "Topic non gere : %s", topic);
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

void network_manager_set_uid(const char *uid)
{
    if (!uid || strlen(uid) == 0) return;

    strncpy(s_uid, uid, UID_MAX_LEN - 1);
    s_uid[UID_MAX_LEN - 1] = '\0';
    s_uid_received = true;
    s_uid_fixed = true;
    nvs_save_uid(s_uid);
    ESP_LOGI(TAG, "UID fixe configure : %s", s_uid);
}

void network_manager_init(const char *ssid, const char *password,
                          const char *mqtt_server, int mqtt_port)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    nvs_load_uid();

    s_wifi_event_group = xEventGroupCreate();

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
    strncpy((char *)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char *)wifi_config.sta.password, password,
            sizeof(wifi_config.sta.password) - 1);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));

    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri  = mqtt_server,
        .broker.address.port = (uint32_t)mqtt_port,
        .session.keepalive   = 60,
    };

    s_mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    if (!s_mqtt_client) {
        ESP_LOGE(TAG, "esp_mqtt_client_init echoue");
        return;
    }

    esp_mqtt_client_register_event(s_mqtt_client, ESP_EVENT_ANY_ID,
                                   mqtt_event_handler, NULL);

    ESP_LOGI(TAG, "Network manager initialise - SSID: %s Broker: %s:%d",
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
        pdMS_TO_TICKS(WIFI_CONNECT_TIMEOUT_MS));

    if (bits & WIFI_CONNECTED_BIT) {
        read_mac_address();
        ESP_LOGI(TAG, "WiFi OK - demarrage client MQTT");
        if (s_mqtt_client) {
            esp_mqtt_client_start(s_mqtt_client);
        }
    } else {
        ESP_LOGE(TAG, "Timeout WiFi (%ds) - MQTT non demarre",
                 WIFI_CONNECT_TIMEOUT_MS / 1000);
    }
}

void network_manager_publish_sensor_data(const lora_sensor_data_t *data)
{
    if (!data) {
        ESP_LOGW(TAG, "network_manager_publish_sensor_data: data=NULL");
        return;
    }

    if (!s_mqtt_client) {
        ESP_LOGW(TAG, "MQTT client non initialise");
        return;
    }

    if (!s_uid_received || strlen(s_uid) == 0) {
        ESP_LOGW(TAG, "UID non recu - publication annulee");
        return;
    }

    int zone_num  = data->node_id;
    int sensor_id = data->node_id;

    char topic[128];
    snprintf(topic, sizeof(topic),
             "robocare/%s/zone/%d/sensor/%d/data",
             s_uid, zone_num, sensor_id);

    char payload[768];
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
                 "\"mac\":\"%s\","
                 "\"node_id\":%d,"
                 "\"rssi\":%d,"
                 "\"snr\":%.1f,"
                 "\"date\":\"%s\","
                 "\"time\":\"%s\""
               "}"
             "}",
             safe_float(data->humidity),
             safe_float(data->temperature),
             safe_float(data->ph),
             safe_float(data->ec),
             safe_float(data->nitrogen),
             safe_float(data->phosphorus),
             safe_float(data->potassium),
             (strlen(s_mac_str) > 0) ? s_mac_str : "UNKNOWN",
             data->node_id,
             data->rssi,
             data->snr,
             data->date,
             data->time_str);

    int msg_id = esp_mqtt_client_publish(s_mqtt_client, topic, payload, 0, 1, 0);

    if (msg_id >= 0) {
        ESP_LOGI(TAG, "MQTT PUBLISH OK");
        ESP_LOGI(TAG, "  Topic   : %s", topic);
        ESP_LOGI(TAG, "  Payload : %s", payload);
    } else {
        ESP_LOGE(TAG, "MQTT publish echoue");
    }
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

bool network_manager_is_connected(void)
{
    if (!s_wifi_event_group) {
        return false;
    }

    EventBits_t bits = xEventGroupGetBits(s_wifi_event_group);
    return (bits & WIFI_CONNECTED_BIT) != 0;
}
