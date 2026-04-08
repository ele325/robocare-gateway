/**
 * @file relay_manager.c
 * @brief Gestionnaire relais — Carte RÉCEPTRICE RoboCare (ESP32-S2-WROOM)
 *
 * Corrections appliquées (v1.0 → v1.2) :
 *
 *  1. [CRITIQUE]  malloc() vérifié — return si heap insuffisante.
 *
 *  2. [CRITIQUE]  Guard double-init : free() avant réallocation.
 *
 *  3. [ROBUSTESSE] gpio_config() remplace gpio_set_direction().
 *
 *  4. [ROBUSTESSE] Protection courant max LM2596S-5V (v1.2) :
 *                  Plan énergétique réceptrice : relais = 40/70mA sur 5V.
 *                  LM2596S-5V nominale = 1A → max 12 relais × 70mA = 840mA.
 *                  Protection logicielle : si l'activation d'un relais
 *                  dépasse RELAY_MAX_ACTIVE simultanément, log warning.
 *                  Valeur conservative : 10 relais max = 700mA < 1A LM2596.
 *
 *  5. [INFO]      Logique de niveau : gpio HIGH = relais ON.
 *                 Circuit : GPIO 3.3V → PC817 → BC547 → bobine 12V.
 *                 À inverser (state ? 0 : 1) si logique optocoupleur inverse.
 */

#include "relay_manager.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <stdlib.h>
#include <string.h>

static const char *TAG = "RELAY";

static int  *s_relay_pins   = NULL;
static bool *s_relay_states = NULL;
static int   s_num_relays   = 0;

/*
 * ╔════════════════════════════════════════════════════════════════════╗
 * ║ Protection courant LM2596S-5V (dimensionnement système 24/7)       ║
 * ╠════════════════════════════════════════════════════════════════════╣
 * ║ Specs réelles :                                                    ║
 * ║ • Rail 3.3V : 1510 mA total (critique)                           ║
 * ║ • Rail 5V   : 750 mA total                                       ║
 * ║ • Rail 12V  : 2004 mA (15.36W — limite absolue)                 ║
 * ║                                                                   ║
 * ║ Relais SRD-12VDC : 40-70 mA par bobine (12V ou 5V) selon design  ║
 * ║ → 4 relais max actifs × 70 mA = 280 mA                           ║
 * ║ → Reste pour ESP32/LoRa : 2004 - 280 = 1724 mA OK               ║
 * ║                                                                   ║
 * ║ ATTENTION : Si plus de 4 relais actifs → thermal shutdown        ║
 * ║ des régulateurs (dissipateurs essentiels !)                      ║
 * ╚════════════════════════════════════════════════════════════════════╝
 */
#define RELAY_MAX_ACTIVE  4   /* Limité à 4 de base selon dimensionnement */

void relay_manager_init(const int *pins, int num_pins)
{
    if (!pins || num_pins <= 0) {
        ESP_LOGE(TAG, "Paramètres invalides");
        return;
    }

    /* CORRECTION v1.1 : libérer l'ancienne allocation si double-init */
    if (s_relay_states) {
        free(s_relay_states);
        s_relay_states = NULL;
    }

    s_relay_pins  = (int *)pins;
    s_num_relays  = num_pins;

    /* CORRECTION v1.1 : vérifier malloc */
    s_relay_states = malloc(s_num_relays * sizeof(bool));
    if (!s_relay_states) {
        ESP_LOGE(TAG, "malloc relay_states échoué (%d octets)",
                 (int)(s_num_relays * sizeof(bool)));
        s_num_relays = 0;
        return;
    }
    memset(s_relay_states, 0, s_num_relays * sizeof(bool));

    /* CORRECTION v1.1 : gpio_config() pour configuration complète */
    for (int i = 0; i < s_num_relays; i++) {
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << pins[i]),
            .mode         = GPIO_MODE_OUTPUT,
            .pull_up_en   = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type    = GPIO_INTR_DISABLE,
        };
        gpio_config(&io_conf);
        gpio_set_level(pins[i], 0);  /* relais OFF au démarrage */
    }

    ESP_LOGI(TAG, "Initialisé : %d relais", s_num_relays);
}

void relay_manager_set(int index, bool state)
{
    if (!s_relay_states || index < 0 || index >= s_num_relays) {
        ESP_LOGW(TAG, "relay_manager_set : index %d invalide (max %d)",
                 index, s_num_relays - 1);
        return;
    }

    /*
     * Protection courant max LM2596S-5V :
     * Si on veut activer un relais alors que RELAY_MAX_ACTIVE sont déjà ON,
     * on log un warning mais on laisse passer (décision applicative).
     * Changer le comportement en "return" si protection stricte souhaitée.
     */
    if (state && !s_relay_states[index]) {
        int active = relay_manager_active_count();
        if (active >= RELAY_MAX_ACTIVE) {
            ESP_LOGW(TAG, "⚠️  %d relais actifs — limite LM2596S-5V approchée "
                          "(max recommandé : %d). Relais %d activé malgré tout.",
                     active, RELAY_MAX_ACTIVE, index + 1);
        }
    }

    /*
     * Logique de niveau GPIO → relais :
     * Circuit : ESP32-S2 GPIO 3.3V → R220Ω → PC817 LED → GND
     *           PC817 phototransistor → BC547 base → bobine relais 12V
     * gpio HIGH (1) = courant dans LED PC817 = transistor saturé = relais ON
     * Si comportement inverse observé → remplacer par : state ? 0 : 1
     */
    gpio_set_level(s_relay_pins[index], state ? 1 : 0);
    s_relay_states[index] = state;

    ESP_LOGI(TAG, "Relais %d (IO%d) : %s  [actifs : %d/%d]",
             index + 1, s_relay_pins[index],
             state ? "ON" : "OFF",
             relay_manager_active_count(),
             RELAY_MAX_ACTIVE);
}

bool relay_manager_get(int index)
{
    if (!s_relay_states || index < 0 || index >= s_num_relays) return false;
    return s_relay_states[index];
}

int relay_manager_active_count(void)
{
    int count = 0;
    for (int i = 0; i < s_num_relays; i++) {
        if (s_relay_states && s_relay_states[i]) count++;
    }
    return count;
}