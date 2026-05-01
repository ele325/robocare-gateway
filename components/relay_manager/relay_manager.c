/**
 * @file relay_manager.c
 * @brief Gestionnaire relais — Carte RÉCEPTRICE RoboCare (ESP32-S2-WROOM)
 *
 * Corrections appliquées (v1.0 → v1.4) :
 *
 *  v1.1 [CRITIQUE]   malloc() vérifié — return si heap insuffisante.
 *  v1.1 [CRITIQUE]   Guard double-init : free() avant réallocation.
 *  v1.1 [ROBUSTESSE] gpio_config() remplace gpio_set_direction().
 *  v1.2 [ROBUSTESSE] Protection courant max LM2596S-5V.
 *  v1.3 [ROBUSTESSE] Mutex FreeRTOS pour thread-safety (tâche LoRa + MQTT).
 *  v1.3 [ROBUSTESSE] Copie interne des pins (évite dangling pointer).
 *  v1.3 [AJOUT]      relay_manager_deinit() pour nettoyage propre.
 *  v1.4 [CRITIQUE]   Logique inversée : relais ACTIF LOW
 *                    → ON  = gpio_set_level(pin, 0)
 *                    → OFF = gpio_set_level(pin, 1)
 *                    → Init GPIO à HIGH (relais éteint au démarrage)
 */

#include "relay_manager.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <stdlib.h>
#include <string.h>

static const char *TAG = "RELAY";

/* Copie interne des pins (propriétaire) */
static int  *s_relay_pins   = NULL;
static bool *s_relay_states = NULL;
static int   s_num_relays   = 0;

/* Mutex pour thread-safety (tâche LoRa + callback MQTT simultanés) */
static SemaphoreHandle_t s_mutex = NULL;

/*
 * ╔════════════════════════════════════════════════════════════════════╗
 * ║ Protection courant LM2596S-5V (Power Tree carte réceptrice)       ║
 * ╠════════════════════════════════════════════════════════════════════╣
 * ║ Rail 12V  : 3A total (Power Tree)                                 ║
 * ║ Rail 5V   : 750 mA total                                          ║
 * ║ Rail 3.3V : 1510 mA total                                         ║
 * ║                                                                   ║
 * ║ Relais SRD-12VDC : 40-70 mA par bobine                           ║
 * ║ Pompe 12V         : ~400 mA                                       ║
 * ║ Électrovanne 12V  : ~200 mA                                       ║
 * ║ → Pompe + 1 vanne = ~600 mA < 3A OK                              ║
 * ║                                                                   ║
 * ║ RELAY_MAX_ACTIVE = 4 (protection logicielle conservative)        ║
 * ╚════════════════════════════════════════════════════════════════════╝
 */
#define RELAY_MAX_ACTIVE  4

/*
 * ╔════════════════════════════════════════════════════════════════════╗
 * ║ LOGIQUE ACTIF LOW                                                 ║
 * ╠════════════════════════════════════════════════════════════════════╣
 * ║ Circuit : ESP32-S2 GPIO → PC817 optocoupleur → BC547 → relais    ║
 * ║                                                                   ║
 * ║ GPIO LOW  (0) → courant dans PC817 → transistor saturé           ║
 * ║               → bobine alimentée → relais ON  ✓                  ║
 * ║                                                                   ║
 * ║ GPIO HIGH (1) → pas de courant → transistor bloqué               ║
 * ║               → bobine coupée  → relais OFF ✓                    ║
 * ║                                                                   ║
 * ║ RELAY_ACTIVE_LEVEL  = 0  (LOW  = ON)                             ║
 * ║ RELAY_INACTIVE_LEVEL= 1  (HIGH = OFF)                            ║
 * ╚════════════════════════════════════════════════════════════════════╝
 */
#define RELAY_ACTIVE_LEVEL    0   /* GPIO LOW  = relais ON  */
#define RELAY_INACTIVE_LEVEL  1   /* GPIO HIGH = relais OFF */

/* =========================================================================
 * API publique
 * ========================================================================= */

void relay_manager_init(const int *pins, int num_pins)
{
    if (!pins || num_pins <= 0) {
        ESP_LOGE(TAG, "Paramètres invalides");
        return;
    }

    /* Créer mutex si pas encore fait */
    if (!s_mutex) {
        s_mutex = xSemaphoreCreateMutex();
        if (!s_mutex) {
            ESP_LOGE(TAG, "Création mutex échouée");
            return;
        }
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);

    /* Libérer allocations précédentes (guard double-init) */
    if (s_relay_states) {
        free(s_relay_states);
        s_relay_states = NULL;
    }
    if (s_relay_pins) {
        free(s_relay_pins);
        s_relay_pins = NULL;
    }

    s_num_relays = num_pins;

    /* Copie propriétaire des pins (évite dangling pointer) */
    s_relay_pins = malloc(s_num_relays * sizeof(int));
    if (!s_relay_pins) {
        ESP_LOGE(TAG, "malloc relay_pins échoué (%d octets)",
                 (int)(s_num_relays * sizeof(int)));
        s_num_relays = 0;
        xSemaphoreGive(s_mutex);
        return;
    }
    memcpy(s_relay_pins, pins, s_num_relays * sizeof(int));

    /* Allocation états */
    s_relay_states = malloc(s_num_relays * sizeof(bool));
    if (!s_relay_states) {
        ESP_LOGE(TAG, "malloc relay_states échoué (%d octets)",
                 (int)(s_num_relays * sizeof(bool)));
        free(s_relay_pins);
        s_relay_pins = NULL;
        s_num_relays = 0;
        xSemaphoreGive(s_mutex);
        return;
    }
    memset(s_relay_states, 0, s_num_relays * sizeof(bool));

    /* Configuration GPIO — HIGH au démarrage = relais OFF (actif LOW) */
    for (int i = 0; i < s_num_relays; i++) {
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << s_relay_pins[i]),
            .mode         = GPIO_MODE_OUTPUT,
            .pull_up_en   = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type    = GPIO_INTR_DISABLE,
        };
        gpio_config(&io_conf);
        /* ✅ CORRIGÉ : HIGH = relais OFF (logique actif LOW) */
        gpio_set_level(s_relay_pins[i], RELAY_INACTIVE_LEVEL);
    }

    xSemaphoreGive(s_mutex);

    ESP_LOGI(TAG, "Initialisé : %d relais (logique actif LOW)", s_num_relays);
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
        ESP_LOGW(TAG, "relay_manager_set : index %d invalide (max %d)",
                 index, s_num_relays - 1);
        return;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);

    /* Protection courant max */
    if (state && !s_relay_states[index]) {
        int active = 0;
        for (int i = 0; i < s_num_relays; i++) {
            if (s_relay_states[i]) active++;
        }
        if (active >= RELAY_MAX_ACTIVE) {
            ESP_LOGW(TAG, "⚠ %d relais actifs — limite LM2596S-5V approchée "
                          "(max recommandé : %d). Relais %d activé malgré tout.",
                     active, RELAY_MAX_ACTIVE, index + 1);
        }
    }

    /*
     * ✅ CORRIGÉ : Logique actif LOW
     * state=true  (ON)  → GPIO LOW  (0) → relais s'active
     * state=false (OFF) → GPIO HIGH (1) → relais se désactive
     */
    gpio_set_level(s_relay_pins[index],
                   state ? RELAY_ACTIVE_LEVEL : RELAY_INACTIVE_LEVEL);
    s_relay_states[index] = state;

    /* Compter actifs pour le log */
    int active_count = 0;
    for (int i = 0; i < s_num_relays; i++) {
        if (s_relay_states[i]) active_count++;
    }

    xSemaphoreGive(s_mutex);

    ESP_LOGI(TAG, "Relais %d (IO%d) : %s  [actifs : %d/%d]",
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

    /* Éteindre tous les relais (HIGH = OFF en actif LOW) */
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