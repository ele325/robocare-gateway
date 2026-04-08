/**
 * @file sd_manager.h
 * @brief Gestionnaire carte SD — Carte RÉCEPTRICE RoboCare (ESP32-S2-WROOM)
 *
 * Corrections appliquées (v1.0 → v2.0) :
 *
 *  1. [CRITIQUE]  sd_manager_log_raw() ajoutée — appelée par main_receptrice.c.
 *                 L'original ne l'avait pas → erreur de compilation.
 *
 *  2. [CRITIQUE]  sd_manager_log_data() signature étendue : reçoit
 *                 lora_sensor_data_t complète au lieu de (timestamp, zone, hum).
 *                 Timestamp Unix retiré (invalide sans NTP) — date/heure RTC
 *                 émettrice utilisée depuis data->date et data->time_str.
 *
 *  3. [ROBUSTESSE] sd_manager_set_spi_host() pour configurer le bon bus SPI
 *                  (SPI2_HOST ou SPI3_HOST selon le PCB).
 */

#ifndef SD_MANAGER_H
#define SD_MANAGER_H

#include <stdint.h>
#include <stdbool.h>
#include "robocare_types.h"   /* lora_sensor_data_t */

/**
 * @brief Configure le bus SPI à utiliser pour la carte SD.
 *
 * DOIT être appelé AVANT sd_manager_init() si le bus n'est pas SPI2_HOST.
 * Par défaut : SPI2_HOST (valeur de SDSPI_HOST_DEFAULT).
 *
 * @param spi_host SPI2_HOST (1) ou SPI3_HOST (2)
 */
void sd_manager_set_spi_host(int spi_host);

/**
 * @brief Initialise la carte SD sur le bus SPI déjà initialisé.
 *
 * PRÉREQUIS : spi_bus_initialize(host, ...) appelé avant dans main.c.
 *
 * @param cs_pin Broche Chip Select → IO10
 * @return true si montage réussi
 */
bool sd_manager_init(int cs_pin);

/**
 * @brief Enregistre toutes les données d'un paquet capteur dans le CSV.
 *
 * Format CSV :
 *   date;heure;node_id;temp;humidity;ec;ph;nitrogen;phosphorus;potassium;rssi;snr
 *
 * @param data Pointeur sur la structure capteur reçue via LoRa
 */
void sd_manager_log_data(const lora_sensor_data_t *data);

/**
 * @brief Écrit une ligne brute dans le fichier de log.
 *
 * Utilisé par main_receptrice.c pour les lignes pré-formatées.
 * La ligne doit inclure le '\n' final.
 *
 * @param line Chaîne à écrire (terminée par '\n')
 */
void sd_manager_log_raw(const char *line);

#endif /* SD_MANAGER_H */