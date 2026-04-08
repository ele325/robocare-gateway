
#include "sd_manager.h"

#include "esp_vfs_fat.h"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "sdmmc_cmd.h"
#include "esp_log.h"

#include <stdio.h>
#include <inttypes.h>   /* PRIu32 */
#include <string.h>

static const char *TAG = "SD_RX";

static sdmmc_card_t *s_card          = NULL;
static bool          s_initialized   = false;

/*
 * Bus SPI par défaut : SPI2_HOST (= valeur SDSPI_HOST_DEFAULT).
 * Surcharger avec sd_manager_set_spi_host() si la SD est sur SPI3_HOST.
 */
static int s_spi_host = SPI2_HOST;

#define SD_MOUNT_POINT  "/sdcard"
#define SD_LOG_FILE     SD_MOUNT_POINT "/capteur_log.csv"

/* =========================================================================
 * sd_manager_set_spi_host
 * ========================================================================= */
void sd_manager_set_spi_host(int spi_host)
{
    s_spi_host = spi_host;
    ESP_LOGI(TAG, "Bus SPI configuré : SPI%d_HOST", spi_host + 1);
}

/* =========================================================================
 * sd_manager_init
 * ========================================================================= */
bool sd_manager_init(int cs_pin)
{
    if (s_initialized) {
        ESP_LOGW(TAG, "SD déjà initialisée");
        return true;
    }

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files              = 5,
        .allocation_unit_size   = 16 * 1024,
    };

    /*
     * CORRECTION v2.0 : utiliser s_spi_host au lieu de host.slot fixe.
     * SDSPI_HOST_DEFAULT() initialise host.slot = SPI2_HOST.
     * On surcharge ensuite avec le bus configuré.
     *
     * RÈGLE : ne pas modifier host.flags après SDSPI_HOST_DEFAULT()
     * (les flags internes seraient corrompus).
     */
    sdmmc_host_t host   = SDSPI_HOST_DEFAULT();
    host.slot           = s_spi_host;   /* surcharge du bus SPI */

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs  = cs_pin;
    slot_config.host_id  = s_spi_host;

    esp_err_t ret = esp_vfs_fat_sdspi_mount(SD_MOUNT_POINT, &host,
                                             &slot_config, &mount_config,
                                             &s_card);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Montage SD échoué : %s", esp_err_to_name(ret));
        ESP_LOGE(TAG, "  → Vérifier CS=IO%d, bus SPI%d, carte présente",
                 cs_pin, s_spi_host + 1);
        return false;
    }

    s_initialized = true;
    sdmmc_card_print_info(stdout, s_card);
    ESP_LOGI(TAG, "Carte SD montée sur %s", SD_MOUNT_POINT);

    /*
     * CORRECTION v2.0 : header CSV aligné sur les 12 champs réels.
     * L'original avait "Timestamp;Zone;Humidite" — 3 colonnes seulement.
     */
    FILE *f = fopen(SD_LOG_FILE, "r");
    if (!f) {
        /* Fichier inexistant → créer avec header */
        f = fopen(SD_LOG_FILE, "w");
        if (f) {
            fprintf(f, "Date;Heure;NodeID;Temperature;Humidite;EC;"
                       "pH;Azote;Phosphore;Potassium;RSSI;SNR\n");
            fclose(f);
            ESP_LOGI(TAG, "Fichier CSV créé : %s", SD_LOG_FILE);
        } else {
            ESP_LOGE(TAG, "Impossible de créer %s", SD_LOG_FILE);
        }
    } else {
        fclose(f);
        ESP_LOGI(TAG, "Fichier CSV existant — reprise des données");
    }

    return true;
}

/* =========================================================================
 * sd_manager_log_data
 *
 * CORRECTION v2.0 — signature étendue : lora_sensor_data_t*
 *
 * L'ancienne version écrivait : "%lu;%d;%.2f\n" (timestamp, zone, humidity)
 * Problèmes :
 *   - timestamp = time(NULL) = 0 sans NTP → inutile
 *   - seulement humidité loguée → T, EC, pH, NPK, RSSI perdus
 *   - %lu non portable pour uint32_t sur ESP-IDF
 *
 * Nouvelle version :
 *   - Utilise data->date et data->time_str (horodatage RTC émettrice)
 *   - Logue les 12 grandeurs mesurées
 * ========================================================================= */
void sd_manager_log_data(const lora_sensor_data_t *data)
{
    if (!s_initialized || !data) return;

    FILE *f = fopen(SD_LOG_FILE, "a");
    if (!f) {
        ESP_LOGE(TAG, "Impossible d'ouvrir %s en écriture", SD_LOG_FILE);
        return;
    }

    /*
     * Format : Date;Heure;NodeID;T;H;EC;pH;N;P;K;RSSI;SNR
     * pH = -1.0 si sonde déconnectée (affiché "INVAL" dans le CSV)
     */
    if (data->ph < 0.0f) {
        fprintf(f, "%s;%s;%d;%.1f;%.1f;%.0f;INVAL;%.0f;%.0f;%.0f;%d;%.1f\n",
                data->date, data->time_str,
                data->node_id,
                data->temperature, data->humidity, data->ec,
                data->nitrogen, data->phosphorus, data->potassium,
                data->rssi, data->snr);
    } else {
        fprintf(f, "%s;%s;%d;%.1f;%.1f;%.0f;%.2f;%.0f;%.0f;%.0f;%d;%.1f\n",
                data->date, data->time_str,
                data->node_id,
                data->temperature, data->humidity, data->ec,
                data->ph,
                data->nitrogen, data->phosphorus, data->potassium,
                data->rssi, data->snr);
    }

    fclose(f);
    ESP_LOGD(TAG, "Log SD : Nœud %d | %s %s",
             data->node_id, data->date, data->time_str);
}

/* =========================================================================
 * sd_manager_log_raw
 *
 * AJOUT v2.0 : fonction manquante appelée par main_receptrice.c.
 * Écrit une ligne brute pré-formatée dans le fichier de log.
 * ========================================================================= */
void sd_manager_log_raw(const char *line)
{
    if (!s_initialized || !line) return;

    FILE *f = fopen(SD_LOG_FILE, "a");
    if (!f) {
        ESP_LOGE(TAG, "sd_manager_log_raw : impossible d'ouvrir %s", SD_LOG_FILE);
        return;
    }

    fputs(line, f);
    fclose(f);
}