#ifndef LORA_MANAGER_H
#define LORA_MANAGER_H

#include <stdbool.h>
#include <stdint.h>

// Callback pour les données reçues (zone, humidité)
typedef void (*lora_data_callback_t)(int zone, float humidity);

/**
 * @brief Initialise le module LoRa
 * @param mosi_pin Broche MOSI
 * @param miso_pin Broche MISO
 * @param sck_pin Broche SCK
 * @param cs_pin Broche Chip Select
 * @param rst_pin Broche Reset
 * @param dio0_pin Broche DIO0 (IRQ)
 * @return true si succès
 */
bool lora_manager_init(int mosi_pin, int miso_pin, int sck_pin, 
                      int cs_pin, int rst_pin, int dio0_pin);

/**
 * @brief Enregistre le callback pour les données reçues
 * @param cb Fonction de callback
 */
void lora_manager_set_callback(lora_data_callback_t cb);

/**
 * @brief Traite les paquets LoRa reçus (à appeler périodiquement)
 */
void lora_manager_process(void);

#endif