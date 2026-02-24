#ifndef SD_MANAGER_H
#define SD_MANAGER_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Initialise la carte SD
 * @param cs_pin Broche Chip Select
 * @return true si succès
 */
bool sd_manager_init(int cs_pin);

/**
 * @brief Enregistre les données d'humidité
 * @param timestamp Timestamp Unix
 * @param zone Numéro de zone
 * @param humidity Valeur d'humidité
 */
void sd_manager_log_data(uint32_t timestamp, int zone, float humidity);

#endif