/**
 * @file network_manager.h
 * @brief Driver WiFi + MQTT — Carte RÉCEPTRICE RoboCare (ESP32-S2-WROOM)
 *
 * v2.2 — Système dynamique :
 *   - UID récupéré automatiquement via "robocare/config/<mac>"
 *   - MAC address = identifiant unique du device
 *   - UID persisté en NVS entre les redémarrages
 *   - Discovery automatique au démarrage
 */

#ifndef NETWORK_MANAGER_H
#define NETWORK_MANAGER_H

#include <stdbool.h>
#include "robocare_types.h"

/**
 * @brief Force un UID Firebase manuellement (optionnel).
 * Si non appelé, l'UID est récupéré automatiquement via discovery MQTT.
 * Utile pour les tests ou si le device est déjà connu.
 */
void network_manager_set_uid(const char *uid);

/**
 * @brief Retourne l'UID Firebase actuel (NULL si pas encore reçu).
 */
const char *network_manager_get_uid(void);

/**
 * @brief Retourne la MAC address du device (ex: "AA:BB:CC:DD:EE:FF").
 * Disponible après network_manager_start().
 */
const char *network_manager_get_mac(void);

/**
 * @brief Initialise WiFi STA + client MQTT.
 * @param ssid        SSID WiFi
 * @param password    Mot de passe WiFi
 * @param mqtt_server URI broker (ex: "mqtt://broker.hivemq.com")
 * @param mqtt_port   Port MQTT (1883)
 */
void network_manager_init(const char *ssid, const char *password,
                          const char *mqtt_server, int mqtt_port);

/**
 * @brief Démarre WiFi + MQTT. Bloquant jusqu'à connexion WiFi (30s timeout).
 * Après connexion : publie discovery, s'abonne à config/<mac>.
 */
void network_manager_start(void);

/**
 * @brief Publie les données capteur en JSON sur MQTT.
 * Topic : robocare/<uid>/zone/<node_id>/data
 * Publication annulée si UID pas encore reçu.
 */
void network_manager_publish_sensor_data(const lora_sensor_data_t *data);

/**
 * @brief Enregistre le callback pour les commandes relais MQTT.
 * Appelé quand bridge publie : robocare/<uid>/valve/control/<zone>
 * @param callback fonction(relay_index 0-11, state ON/OFF)
 */
void network_manager_set_relay_callback(void (*callback)(int, bool));

/**
 * @brief Retourne true si WiFi connecté et IP obtenue.
 */
bool network_manager_is_connected(void);

/**
 * @brief Retourne true si le device est configuré (UID reçu et stocké en NVS).
 * Utilisé par main.c pour clignoter une LED d'état (ex: Orange tant que non-configuré).
 */
bool network_manager_is_provisioned(void);

#endif /* NETWORK_MANAGER_H */