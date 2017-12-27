#ifndef CLIENT_ID_H
#define CLIENT_ID_H

#include "esp_system.h"

int client_id_compare(uint8_t *one, uint8_t *other, uint16_t length);

esp_err_t fetch_client_id_from_nvs(uint8_t *client_id, size_t *length);
esp_err_t write_client_id_to_nvs(uint8_t *client_id, size_t length);

#endif // CLIENT_ID_H