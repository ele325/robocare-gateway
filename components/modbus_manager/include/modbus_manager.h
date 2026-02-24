#ifndef MODBUS_MANAGER_H
#define MODBUS_MANAGER_H

#include <stdint.h>

/**
 * @brief Initialise la communication Modbus
 * @param rx_pin Broche RX
 * @param tx_pin Broche TX
 * @param en_pin Broche de contrôle RS485
 */
void modbus_manager_init(int rx_pin, int tx_pin, int en_pin);

/**
 * @brief Règle la fréquence du variateur
 * @param active_zones Nombre de zones actives
 */
void modbus_manager_set_frequency(int active_zones);

#endif