#include "network_manager.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "mqtt_client.h"
#include "freertos/event_groups.h"

static const char *TAG = "NETWORK";
static EventGroupHandle_t wifi_event_group;
const int WIFI_CONNECTED_BIT = BIT0;

static esp_mqtt_client_handle_t mqtt_client = NULL;
static void (*relay_callback)(int, bool) = NULL;
static char mqtt_topic_commands[] = "robocare/commands";

// Gestionnaire d'événements WiFi
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                              int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG, "WiFi déconnecté, tentative de reconnexion...");
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "IP obtenue: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

// Gestionnaire d'événements MQTT
static void mqtt_event_handler(void *handler_args, esp_event_base_t base,
                              int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;
    
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT connecté");
            esp_mqtt_client_subscribe(mqtt_client, mqtt_topic_commands, 1);
            break;
            
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT déconnecté");
            break;
            
        case MQTT_EVENT_DATA:
            if (relay_callback) {
                char command[32];
                memcpy(command, event->data, event->data_len);
                command[event->data_len] = '\0';
                
                for (int i = 1; i <= 12; i++) {
                    char on_cmd[16], off_cmd[16];
                    sprintf(on_cmd, "ZONE%d_ON", i);
                    sprintf(off_cmd, "ZONE%d_OFF", i);
                    
                    if (strcmp(command, on_cmd) == 0) {
                        relay_callback(i-1, true);
                    } else if (strcmp(command, off_cmd) == 0) {
                        relay_callback(i-1, false);
                    }
                }
            }
            break;
            
        default:
            break;
    }
}

void network_manager_init(const char *ssid, const char *password, 
                         const char *mqtt_server, int mqtt_port)
{
    // Initialiser NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Créer groupe d'événements
    wifi_event_group = xEventGroupCreate();
    
    // Initialiser TCP/IP
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    
    // Initialiser WiFi
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    
    // Enregistrer les handlers
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                         ESP_EVENT_ANY_ID,
                                                         &wifi_event_handler,
                                                         NULL,
                                                         NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                         IP_EVENT_STA_GOT_IP,
                                                         &wifi_event_handler,
                                                         NULL,
                                                         NULL));
    
    // Configurer WiFi
    wifi_config_t wifi_config = {
        .sta = {
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    strcpy((char*)wifi_config.sta.ssid, ssid);
    strcpy((char*)wifi_config.sta.password, password);
    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    
    // Configurer MQTT
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = mqtt_server,
        .broker.address.port = mqtt_port,
    };
    
    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    
    ESP_LOGI(TAG, "Network manager initialisé");
}

void network_manager_start(void)
{
    ESP_ERROR_CHECK(esp_wifi_start());
    esp_mqtt_client_start(mqtt_client);
}

void network_manager_publish_humidity(int zone, float humidity)
{
    if (mqtt_client && xEventGroupGetBits(wifi_event_group) & WIFI_CONNECTED_BIT) {
        char topic[64];
        char payload[32];
        sprintf(topic, "robocare/zone/%d/humidity", zone);
        sprintf(payload, "%.2f", humidity);
        esp_mqtt_client_publish(mqtt_client, topic, payload, 0, 1, 0);
        ESP_LOGI(TAG, "Publié: %s -> %s", topic, payload);
    }
}

bool network_manager_is_connected(void)
{
    return (xEventGroupGetBits(wifi_event_group) & WIFI_CONNECTED_BIT) != 0;
}

void network_manager_set_relay_callback(void (*callback)(int, bool))
{
    relay_callback = callback;
}