/**
 * @file relay_manager.c
 * @brief Gestionnaire relais — Carte RÉCEPTRICE RoboCare
 */

#include "relay_manager.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <stdlib.h>
#include <string.h>

static const char *TAG = "RELAY";

static int  *s_relay_pins   = NULL;
static bool *s_relay_states = NULL;
static int   s_num_relays   = 0;
static SemaphoreHandle_t s_mutex = NULL;

#define RELAY_MAX_ACTIVE  4
/*
 * Certains modules relais sont "actif LOW" (niveau 0 = ON),
 * d'autres "actif HIGH" (niveau 1 = ON).
 *
 * Par défaut on garde l'ancien comportement: actif LOW.
 * Pour basculer en actif HIGH, définir RELAY_ACTIVE_LOW=0 au build.
 */
#ifndef RELAY_ACTIVE_LOW
#define RELAY_ACTIVE_LOW 1
#endif

#if RELAY_ACTIVE_LOW
#define RELAY_ACTIVE_LEVEL    0   /* GPIO LOW  = relais ON */
#define RELAY_INACTIVE_LEVEL  1   /* GPIO HIGH = relais OFF */
#else
#define RELAY_ACTIVE_LEVEL    1   /* GPIO HIGH =  relais ON */
#define RELAY_INACTIVE_LEVEL  0   /* GPIO LOW  = relais OFF */
#endif

int relay_manager_active_level(void)   { return RELAY_ACTIVE_LEVEL; }
int relay_manager_inactive_level(void) { return RELAY_INACTIVE_LEVEL; }

void relay_manager_init(const int *pins, int num_pins)
{
    if (!pins || num_pins <= 0) {
        ESP_LOGE(TAG, "Paramètres invalides");
        return;
    }

    if (!s_mutex) {
        s_mutex = xSemaphoreCreateMutex();
        if (!s_mutex) {
            ESP_LOGE(TAG, "Création mutex échouée");
            return;
        }
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);

    if (s_relay_states) {
        free(s_relay_states);
        s_relay_states = NULL;
    }
    if (s_relay_pins) {
        free(s_relay_pins);
        s_relay_pins = NULL;
    }

    s_num_relays = num_pins;

    s_relay_pins = malloc(s_num_relays * sizeof(int));
    if (!s_relay_pins) {
        ESP_LOGE(TAG, "malloc relay_pins échoué");
        s_num_relays = 0;
        xSemaphoreGive(s_mutex);
        return;
    }
    memcpy(s_relay_pins, pins, s_num_relays * sizeof(int));

    s_relay_states = malloc(s_num_relays * sizeof(bool));
    if (!s_relay_states) {
        ESP_LOGE(TAG, "malloc relay_states échoué");
        free(s_relay_pins);
        s_relay_pins = NULL;
        s_num_relays = 0;
        xSemaphoreGive(s_mutex);
        return;
    }
    memset(s_relay_states, 0, s_num_relays * sizeof(bool));

    /* ✅ Configuration avec PULL-UP activé */
    for (int i = 0; i < s_num_relays; i++) {
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << s_relay_pins[i]),
            .mode         = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type    = GPIO_INTR_DISABLE,
        };
        gpio_config(&io_conf);
        gpio_set_level(s_relay_pins[i], RELAY_INACTIVE_LEVEL);
        
        /* Forcer HIGH une seconde fois pour être sûr */
        vTaskDelay(pdMS_TO_TICKS(10));
        gpio_set_level(s_relay_pins[i], RELAY_INACTIVE_LEVEL);
    }

    xSemaphoreGive(s_mutex);

    ESP_LOGI(TAG, "Initialisé : %d relais (actif LOW, pull-up activé)", s_num_relays);
    for (int i = 0; i < s_num_relays; i++) {
        ESP_LOGI(TAG, "  Relais %d → IO%d", i + 1, s_relay_pins[i]);
    }
}

void relay_manager_set(int index, bool state)
{
    if (!s_mutex || !s_relay_states) {
        ESP_LOGW(TAG, "relay_manager_set : non initialisé");
        return;
    }

    if (index < 0 || index >= s_num_relays) {
        ESP_LOGW(TAG, "relay_manager_set : index %d invalide", index);
        return;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);

    gpio_set_level(s_relay_pins[index],
                   state ? RELAY_ACTIVE_LEVEL : RELAY_INACTIVE_LEVEL);
    s_relay_states[index] = state;

    int active_count = 0;
    for (int i = 0; i < s_num_relays; i++) {
        if (s_relay_states[i]) active_count++;
    }

    xSemaphoreGive(s_mutex);

    ESP_LOGI(TAG, "Relais %d (IO%d) : %s  [actifs: %d/%d]",
             index + 1, s_relay_pins[index],
             state ? "ON ✓" : "OFF",
             active_count, RELAY_MAX_ACTIVE);
}

bool relay_manager_get(int index)
{
    if (!s_mutex || !s_relay_states || index < 0 || index >= s_num_relays) {
        return false;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    bool val = s_relay_states[index];
    xSemaphoreGive(s_mutex);
    return val;
}

int relay_manager_active_count(void)
{
    if (!s_mutex || !s_relay_states) return 0;

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    int count = 0;
    for (int i = 0; i < s_num_relays; i++) {
        if (s_relay_states[i]) count++;
    }
    xSemaphoreGive(s_mutex);
    return count;
}

void relay_manager_deinit(void)
{
    if (s_mutex) {
        xSemaphoreTake(s_mutex, portMAX_DELAY);
    }

    if (s_relay_pins && s_relay_states) {
        for (int i = 0; i < s_num_relays; i++) {
            gpio_set_level(s_relay_pins[i], RELAY_INACTIVE_LEVEL);
        }
    }

    free(s_relay_pins);
    free(s_relay_states);
    s_relay_pins   = NULL;
    s_relay_states = NULL;
    s_num_relays   = 0;

    if (s_mutex) {
        xSemaphoreGive(s_mutex);
        vSemaphoreDelete(s_mutex);
        s_mutex = NULL;
    }

    ESP_LOGI(TAG, "Deinit : tous les relais éteints");
}