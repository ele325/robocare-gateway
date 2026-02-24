#ifndef NETWORK_MANAGER_H
#define NETWORK_MANAGER_H

#include <stdbool.h>

/**
 * @brief Initialise le gestionnaire réseau
 * @param ssid Nom du WiFi
 * @param password Mot de passe WiFi
 * @param mqtt_server Serveur MQTT
 * @param mqtt_port Port MQTT
 */
void network_manager_init(const char *ssid, const char *password, 
                         const char *mqtt_server, int mqtt_port);

/**
 * @brief Démarre la connexion WiFi et MQTT
 */
void network_manager_start(void);

/**
 * @brief Publie une valeur d'humidité sur MQTT
 * @param zone Numéro de zone
 * @param humidity Valeur d'humidité
 */
void network_manager_publish_humidity(int zone, float humidity);

/**
 * @brief Vérifie si le réseau est connecté
 * @return true si connecté
 */
bool network_manager_is_connected(void);

/**
 * @brief Enregistre le callback pour les commandes MQTT
 * @param callback Fonction appelée sur commande relais
 */
void network_manager_set_relay_callback(void (*callback)(int, bool));

#endif