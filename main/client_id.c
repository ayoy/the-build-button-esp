#include "client_id.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "esp_log.h"

#define CLIENT_ID_TAG "CLIENT_ID"
#define STORAGE_NAMESPACE "build_btn_nvs"
#define CLIENT_ID_KEY "client_id"

int client_id_compare(uint8_t *one, uint8_t *other, uint16_t length)
{
    ESP_LOGI(CLIENT_ID_TAG, "compare (%d bytes)", length);

    esp_log_buffer_hex(CLIENT_ID_TAG, other, length);
    esp_log_buffer_hex(CLIENT_ID_TAG, one, length);

    for (uint16_t i = 0; i < length; i++) {
        if (one[i] != other[i]) return 0;
    }
    return 1;
}

esp_err_t fetch_client_id_from_nvs(uint8_t *client_id, size_t *length)
{
    ESP_LOGI(CLIENT_ID_TAG, "%s", __func__);
    ESP_LOGI(CLIENT_ID_TAG, "Opening Non-Volatile Storage (NVS) handle... ");

    nvs_handle my_handle;
    esp_err_t err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle);

    if (err != ESP_OK) {
        ESP_LOGE(CLIENT_ID_TAG, "Error (%d) opening NVS handle", err);
    } else {
        ESP_LOGI(CLIENT_ID_TAG, "Done");

        // Read
        ESP_LOGI(CLIENT_ID_TAG, "Reading client id from NVS ... ");

        *length = 0;
        // obtain required memory space to store the string being read from NVS
        err = nvs_get_blob(my_handle, CLIENT_ID_KEY, NULL, length);

        if (err == ESP_OK) {
            if (length == 0) {
                ESP_LOGI(CLIENT_ID_TAG, "Client ID not saved yet");
            } else {
                err = nvs_get_blob(my_handle, CLIENT_ID_KEY, client_id, length);
                if (err == ESP_OK) {
                    ESP_LOGI(CLIENT_ID_TAG, "Client ID (%d bytes): ", *length);
                    esp_log_buffer_hex(CLIENT_ID_TAG, client_id, *length);
                }
            }    
        }
        if (err != ESP_OK) {
            ESP_LOGE(CLIENT_ID_TAG, "Error fetching client id from NVS (%d)", err);
        }
        nvs_close(my_handle);
    }

    return err;
}

esp_err_t write_client_id_to_nvs(uint8_t *client_id, size_t length)
{
    ESP_LOGI(CLIENT_ID_TAG, "%s", __func__);
    ESP_LOGI(CLIENT_ID_TAG, "Opening Non-Volatile Storage (NVS) handle... ");

    nvs_handle my_handle;

    esp_err_t err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        ESP_LOGI(CLIENT_ID_TAG, "Error (%d) opening NVS handle", err);
    } else {
        ESP_LOGI(CLIENT_ID_TAG, "Done");

        // Write
        ESP_LOGI(CLIENT_ID_TAG, "Writing client id (%d bytes) to NVS ... ", length);
        esp_log_buffer_hex(CLIENT_ID_TAG, client_id, length);

        // obtain required memory space to store the string being read from NVS
        err = nvs_set_blob(my_handle, CLIENT_ID_KEY, client_id, length);
        if (err != ESP_OK) {
            ESP_LOGE(CLIENT_ID_TAG, "Error storing client id to NVS (%d)", err);
        }
        err = nvs_commit(my_handle);
        if (err != ESP_OK) {
            ESP_LOGE(CLIENT_ID_TAG, "Error committing client id value to NVS (%d)", err);
        }
        nvs_close(my_handle);
    }
    return err;
}