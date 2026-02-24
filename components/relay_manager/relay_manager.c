#include "relay_manager.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <stdlib.h>

static const char *TAG = "RELAY";
static int *relay_pins = NULL;
static int num_relays = 0;
static bool *relay_states = NULL;

void relay_manager_init(const int *pins, int num_pins)
{
    relay_pins = (int *)pins;
    num_relays = num_pins;
    relay_states = malloc(num_relays * sizeof(bool));
    
    for (int i = 0; i < num_relays; i++) {
        gpio_set_direction(pins[i], GPIO_MODE_OUTPUT);
        gpio_set_level(pins[i], 0);
        relay_states[i] = false;
    }
    ESP_LOGI(TAG, "Initialisé avec %d relais", num_relays);
}

void relay_manager_set(int index, bool state)
{
    if (index >= 0 && index < num_relays) {
        gpio_set_level(relay_pins[index], state ? 1 : 0);
        relay_states[index] = state;
        ESP_LOGI(TAG, "Relais %d %s", index, state ? "ON" : "OFF");
    }
}

bool relay_manager_get(int index)
{
    return (index >= 0 && index < num_relays) ? relay_states[index] : false;
}

int relay_manager_active_count(void)
{
    int count = 0;
    for (int i = 0; i < num_relays; i++) {
        if (relay_states[i]) count++;
    }
    return count;
} 