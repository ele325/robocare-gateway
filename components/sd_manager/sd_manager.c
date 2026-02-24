#include "sd_manager.h"
#include "esp_vfs_fat.h"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "sdmmc_cmd.h"
#include "esp_log.h"
#include <stdio.h>

static const char *TAG = "SD";
static sdmmc_card_t *card = NULL;
static bool sd_initialized = false;

bool sd_manager_init(int cs_pin)
{
    esp_err_t ret;
    
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };
    
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    
    // Utiliser le bus SPI déjà initialisé
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = cs_pin;
    slot_config.host_id = host.slot;
    
    ret = esp_vfs_fat_sdspi_mount("/sdcard", &host, &slot_config, &mount_config, &card);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Échec montage SD: %s", esp_err_to_name(ret));
        return false;
    }
    
    sd_initialized = true;
    ESP_LOGI(TAG, "Carte SD initialisée");
    
    // Créer l'en-tête du fichier CSV si nécessaire
    FILE *f = fopen("/sdcard/humidity_log.csv", "r");
    if (!f) {
        f = fopen("/sdcard/humidity_log.csv", "w");
        if (f) {
            fprintf(f, "Timestamp;Zone;Humidite\n");
            fclose(f);
        }
    } else {
        fclose(f);
    }
    
    return true;
}

void sd_manager_log_data(uint32_t timestamp, int zone, float humidity)
{
    if (!sd_initialized) return;
    
    FILE *f = fopen("/sdcard/humidity_log.csv", "a");
    if (f) {
        fprintf(f, "%lu;%d;%.2f\n", timestamp, zone, humidity);
        fclose(f);
        ESP_LOGD(TAG, "Donnée enregistrée: Zone %d, %.2f%%", zone, humidity);
    } else {
        ESP_LOGE(TAG, "Erreur ouverture fichier SD");
    }
}