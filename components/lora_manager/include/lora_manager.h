/**
 * @file lora_manager.h
 * @brief Driver LoRa Ra-02 (SX1278) — Carte RÉCEPTRICE RoboCare
 *
 * Corrections appliquées (v1.0 → v2.0) :
 *
 *  1. [CRITIQUE]  Signature corrigée : lora_manager_init(cs, rst, dio0)
 *                 Plus de mosi/miso/sck — le bus SPI2 est initialisé
 *                 une seule fois dans main.c (spi2_bus_init).
 *
 *  2. [CRITIQUE]  Callback étendu : lora_data_callback_t reçoit désormais
 *                 la structure complète lora_sensor_data_t au lieu de
 *                 (zone_id, humidity) uniquement.
 *                 L'émetteur envoie : N1;T;H;EC;pH;N;P;K;date;heure
 *                 — toutes les données doivent être transmises au callback.
 *
 *  3. [CRITIQUE]  Ajout lora_manager_deinit() pour libérer le device SPI
 *                 proprement si nécessaire.
 *
 *  4. [ROBUSTESSE] lora_manager_process() retourne bool pour signaler
 *                  si un paquet a été traité (utile pour la task FreeRTOS).
 */

#ifndef LORA_MANAGER_H
#define LORA_MANAGER_H

#include <stdbool.h>
#include <stdint.h>
#include "robocare_types.h"   /* lora_sensor_data_t — source unique, pas de doublon */

/*
 * Callback déclenché à chaque paquet LoRa valide reçu.
 * Le pointeur data est valide uniquement pendant la durée du callback.
 */
typedef void (*lora_data_callback_t)(const lora_sensor_data_t *data);

/* =========================================================================
 * API publique
 * ========================================================================= */

/**
 * @brief Initialise le module LoRa Ra-02 (SX1278) sur le bus SPI2.
 *
 * PRÉREQUIS : spi_bus_initialize(SPI2_HOST, ...) doit avoir été appelé
 * AVANT cette fonction dans main.c (spi2_bus_init()).
 * Cette fonction appelle uniquement spi_bus_add_device() en interne.
 *
 * @param cs_pin   Broche Chip Select (NSS) → IO34
 * @param rst_pin  Broche Reset             → IO33
 * @param dio0_pin Broche DIO0 (RxDone IRQ) → IO38
 *
 * @return true si le SX1278 est détecté (VERSION=0x12) et configuré.
 */
bool lora_manager_init(int cs_pin, int rst_pin, int dio0_pin);

/**
 * @brief Libère le périphérique SPI LoRa du bus SPI2.
 */
void lora_manager_deinit(void);

/**
 * @brief Enregistre le callback appelé à chaque paquet reçu.
 * @param cb Fonction de callback (NULL pour désactiver)
 */
void lora_manager_set_callback(lora_data_callback_t cb);

/**
 * @brief Interroge les flags IRQ et traite le paquet si RxDone.
 *
 * À appeler périodiquement depuis une tâche FreeRTOS (polling),
 * ou depuis l'ISR DIO0 si l'interruption matérielle est utilisée.
 *
 * @return true si un paquet valide a été traité, false sinon.
 */
bool lora_manager_process(void);

#endif /* LORA_MANAGER_H */