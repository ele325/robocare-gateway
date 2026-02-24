#ifndef RELAY_MANAGER_H
#define RELAY_MANAGER_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Initialise les broches des relais
 * @param pins Tableau des numéros de broches GPIO
 * @param num_pins Nombre de relais
 */
void relay_manager_init(const int *pins, int num_pins);

/**
 * @brief Active ou désactive un relais
 * @param index Index du relais (0-11)
 * @param state true = ON, false = OFF
 */
void relay_manager_set(int index, bool state);

/**
 * @brief Lit l'état d'un relais
 * @param index Index du relais
 * @return true si ON, false si OFF
 */
bool relay_manager_get(int index);

/**
 * @brief Compte le nombre de relais actifs
 * @return Nombre de relais allumés
 */
int relay_manager_active_count(void);

#endif // RELAY_MANAGER_H