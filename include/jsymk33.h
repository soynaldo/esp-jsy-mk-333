#ifndef __JSY_MK_33_LIB
#define __JSY_MK_33_LIB

#include "esp_err_t.h"
#include <stdint.h>
#include "freeRTOS/queue.h"

typedef struct
{
    uint32_t uart_num;
    uint32_t baud;
    uint16_t rx_pin;
    uint16_t tx_pin;
    uint32_t uart_buffer_size;
    QueueHandle_t uart_queue;
} jsymk33_config_t;

typedef void *jsymk33_handle_t;

esp_err_t jsymk33_init(jsymk33_handle_t handle, jsymk33_config_t *conf);
esp_err_t jsymk33_deinit(jsymk33_handle_t handle);

esp_err_t jsymk33_read_voltage_A(jsymk33_handle_t handle, float& value);
esp_err_t jsymk33_read_voltage_B(jsymk33_handle_t handle, float& value);
esp_err_t jsymk33_read_voltage_C(jsymk33_handle_t handle, float& value);

esp_err_t jsymk33_read_current_A(jsymk33_handle_t handle, float& value);
esp_err_t jsymk33_read_current_B(jsymk33_handle_t handle, float& value);
esp_err_t jsymk33_read_current_C(jsymk33_handle_t handle, float& value);

esp_err_t jsymk33_read_active_power_A(jsymk33_handle_t handle, float& value);
esp_err_t jsymk33_read_active_power_B(jsymk33_handle_t handle, float& value);
esp_err_t jsymk33_read_active_power_C(jsymk33_handle_t handle, float& value);
esp_err_t jsymk33_read_total_active_power(jsymk33_handle_t handle, float& value);

esp_err_t jsymk33_read_reactive_power_A(jsymk33_handle_t handle, float& value);
esp_err_t jsymk33_read_reactive_power_B(jsymk33_handle_t handle, float& value);
esp_err_t jsymk33_read_reactive_power_C(jsymk33_handle_t handle, float& value);
esp_err_t jsymk33_read_total_reactive_power(jsymk33_handle_t handle, float& value);

esp_err_t jsymk33_read_apparent_power_A(jsymk33_handle_t handle, float& value);
esp_err_t jsymk33_read_apparent_power_B(jsymk33_handle_t handle, float& value);
esp_err_t jsymk33_read_apparent_power_C(jsymk33_handle_t handle, float& value);
esp_err_t jsymk33_read_total_apparent_power(jsymk33_handle_t handle, float& value);

esp_err_t jsymk33_read_frequency(jsymk33_handle_t handle, float& value);

esp_err_t jsymk33_read_power_factor_A(jsymk33_handle_t handle, float& value);
esp_err_t jsymk33_read_power_factor_B(jsymk33_handle_t handle, float& value);
esp_err_t jsymk33_read_power_factor_C(jsymk33_handle_t handle, float& value);
esp_err_t jsymk33_read_total_power_factor(jsymk33_handle_t handle, float& value);

esp_err_t jsymk33_read_active_energy_A(jsymk33_handle_t handle, float& value);
esp_err_t jsymk33_read_active_energy_B(jsymk33_handle_t handle, float& value);
esp_err_t jsymk33_read_active_energy_C(jsymk33_handle_t handle, float& value);
esp_err_t jsymk33_read_total_active_energy(jsymk33_handle_t handle, float& value);

esp_err_t jsymk33_read_reactive_energy_A(jsymk33_handle_t handle, float& value);
esp_err_t jsymk33_read_reactive_energy_B(jsymk33_handle_t handle, float& value);
esp_err_t jsymk33_read_reactive_energy_C(jsymk33_handle_t handle, float& value);
esp_err_t jsymk33_read_total_reactive_energy(jsymk33_handle_t handle, float& value);

esp_err_t jsymk33_read_apparent_energy_A(jsymk33_handle_t handle, float& value);
esp_err_t jsymk33_read_apparent_energy_B(jsymk33_handle_t handle, float& value);
esp_err_t jsymk33_read_apparent_energy_C(jsymk33_handle_t handle, float& value);
esp_err_t jsymk33_read_total_apparent_energy(jsymk33_handle_t handle, float& value);

esp_err_t jsymk33_read_power_direction(jsymk33_handle_t handle, uint16_t& value);
esp_err_t jsymk33_read_alarm_status(jsymk33_handle_t handle, uint16_t& value );


#endif // __JSY_MK_33_LIB