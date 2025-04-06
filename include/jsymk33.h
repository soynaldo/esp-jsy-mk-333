#ifndef __JSY_MK_33_LIB
#define __JSY_MK_33_LIB

#include <stdint.h>
#include <string.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

/**
 * @brief Configuration structure for UART communication.
 *
 * This structure holds all necessary parameters for configuring UART communication,
 * including the UART number, baud rate, pin assignments, buffer size, and queue handle.
 */
typedef struct
{
    uint32_t uart_num;             /**< UART port number to be used. */
    uint32_t baud;                 /**< Baud rate for UART communication. */
    uint16_t rx_pin;               /**< GPIO pin number for UART RX. */
    uint16_t tx_pin;               /**< GPIO pin number for UART TX. */
    uint32_t uart_buffer_size;     /**< Size of the buffer for UART communication. */
    QueueHandle_t uart_queue;      /**< Queue handle for receiving UART events. */
} jsymk33_config_t;

/**
 * @brief Handle type for the jsymk33 library.
 *
 * This handle is used as an opaque reference to an instance of the library.
 */
typedef void *jsymk33_handle_t;


/**
 * @brief Initializes the JSYM K33 communication module.
 *
 * This function configures the UART peripheral according to the provided configuration structure.
 *
 * @param handle Pointer to the handle where the configuration will be stored.
 * @param conf Pointer to the configuration structure.
 * @return ESP_OK on success, or an appropriate error code on failure.
 */
esp_err_t jsymk33_init(jsymk33_handle_t handle, jsymk33_config_t *conf);

/**
 * @brief Deinitializes the JSYM K33 communication module.
 *
 * This function releases the resources associated with the UART communication.
 *
 * @param handle Pointer to the handle representing the configuration.
 * @return ESP_OK on successful deinitialization, or an error code if the driver was not installed.
 */
esp_err_t jsymk33_deinit(jsymk33_handle_t handle);


/**
 * @brief Reads the phase A voltage.
 *
 * This function reads the voltage of phase A from the device.
 *
 * @param handle Pointer to the communication handle.
 * @param value Reference to a float where the voltage will be stored.
 * @return ESP_OK on successful read, ESP_FAIL otherwise.
 */
esp_err_t jsymk33_read_voltage_A(jsymk33_handle_t handle, float *value);

/**
 * @brief Reads the phase B voltage.
 *
 * @param handle Pointer to the communication handle.
 * @param value Reference to a float where the voltage will be stored.
 * @return ESP_OK on successful read, ESP_FAIL otherwise.
 */
esp_err_t jsymk33_read_voltage_B(jsymk33_handle_t handle, float *value);

/**
 * @brief Reads the phase C voltage.
 *
 * @param handle Pointer to the communication handle.
 * @param value Reference to a float where the voltage will be stored.
 * @return ESP_OK on successful read, ESP_FAIL otherwise.
 */
esp_err_t jsymk33_read_voltage_C(jsymk33_handle_t handle, float *value);

/**
 * @brief Reads the phase A current.
 *
 * @param handle Pointer to the communication handle.
 * @param value Reference to a float where the current will be stored.
 * @return ESP_OK on successful read, ESP_FAIL otherwise.
 */
esp_err_t jsymk33_read_current_A(jsymk33_handle_t handle, float *value);

/**
 * @brief Reads the phase B current.
 *
 * @param handle Pointer to the communication handle.
 * @param value Reference to a float where the current will be stored.
 * @return ESP_OK on successful read, ESP_FAIL otherwise.
 */
esp_err_t jsymk33_read_current_B(jsymk33_handle_t handle, float *value);

/**
 * @brief Reads the phase C current.
 *
 * @param handle Pointer to the communication handle.
 * @param value Reference to a float where the current will be stored.
 * @return ESP_OK on successful read, ESP_FAIL otherwise.
 */
esp_err_t jsymk33_read_current_C(jsymk33_handle_t handle, float *value);

/**
 * @brief Reads the active power for phase A.
 *
 * @param handle Pointer to the communication handle.
 * @param value Reference to a float where the power will be stored.
 * @return ESP_OK on successful read, ESP_FAIL otherwise.
 */
esp_err_t jsymk33_read_active_power_A(jsymk33_handle_t handle, float *value);

/**
 * @brief Reads the active power for phase B.
 *
 * @param handle Pointer to the communication handle.
 * @param value Reference to a float where the power will be stored.
 * @return ESP_OK on successful read, ESP_FAIL otherwise.
 */
esp_err_t jsymk33_read_active_power_B(jsymk33_handle_t handle, float *value);

/**
 * @brief Reads the active power for phase C.
 *
 * @param handle Pointer to the communication handle.
 * @param value Reference to a float where the power will be stored.
 * @return ESP_OK on successful read, ESP_FAIL otherwise.
 */
esp_err_t jsymk33_read_active_power_C(jsymk33_handle_t handle, float *value);

/**
 * @brief Reads the total active power.
 *
 * @param handle Pointer to the communication handle.
 * @param value Reference to a float where the power will be stored.
 * @return ESP_OK on successful read, ESP_FAIL otherwise.
 */
esp_err_t jsymk33_read_total_active_power(jsymk33_handle_t handle, float *value);

/**
 * @brief Reads the reactive power for phase A.
 *
 * @param handle Pointer to the communication handle.
 * @param value Reference to a float where the power will be stored.
 * @return ESP_OK on successful read, ESP_FAIL otherwise.
 */
esp_err_t jsymk33_read_reactive_power_A(jsymk33_handle_t handle, float *value);

/**
 * @brief Reads the reactive power for phase B.
 *
 * @param handle Pointer to the communication handle.
 * @param value Reference to a float where the power will be stored.
 * @return ESP_OK on successful read, ESP_FAIL otherwise.
 */
esp_err_t jsymk33_read_reactive_power_B(jsymk33_handle_t handle, float *value);

/**
 * @brief Reads the reactive power for phase C.
 *
 * @param handle Pointer to the communication handle.
 * @param value Reference to a float where the power will be stored.
 * @return ESP_OK on successful read, ESP_FAIL otherwise.
 */
esp_err_t jsymk33_read_reactive_power_C(jsymk33_handle_t handle, float *value);

/**
 * @brief Reads the total reactive power.
 *
 * @param handle Pointer to the communication handle.
 * @param value Reference to a float where the power will be stored.
 * @return ESP_OK on successful read, ESP_FAIL otherwise.
 */
esp_err_t jsymk33_read_total_reactive_power(jsymk33_handle_t handle, float *value);

/**
 * @brief Reads the apparent power for phase A.
 *
 * @param handle Pointer to the communication handle.
 * @param value Reference to a float where the power will be stored.
 * @return ESP_OK on successful read, ESP_FAIL otherwise.
 */
esp_err_t jsymk33_read_apparent_power_A(jsymk33_handle_t handle, float *value);

/**
 * @brief Reads the apparent power for phase B.
 *
 * @param handle Pointer to the communication handle.
 * @param value Reference to a float where the power will be stored.
 * @return ESP_OK on successful read, ESP_FAIL otherwise.
 */
esp_err_t jsymk33_read_apparent_power_B(jsymk33_handle_t handle, float *value);

/**
 * @brief Reads the apparent power for phase C.
 *
 * @param handle Pointer to the communication handle.
 * @param value Reference to a float where the power will be stored.
 * @return ESP_OK on successful read, ESP_FAIL otherwise.
 */
esp_err_t jsymk33_read_apparent_power_C(jsymk33_handle_t handle, float *value);

/**
 * @brief Reads the total apparent power.
 *
 * @param handle Pointer to the communication handle.
 * @param value Reference to a float where the power will be stored.
 * @return ESP_OK on successful read, ESP_FAIL otherwise.
 */
esp_err_t jsymk33_read_total_apparent_power(jsymk33_handle_t handle, float *value);

/**
 * @brief Reads the frequency.
 *
 * @param handle Pointer to the communication handle.
 * @param value Reference to a float where the frequency will be stored.
 * @return ESP_OK on successful read, ESP_FAIL otherwise.
 */
esp_err_t jsymk33_read_frequency(jsymk33_handle_t handle, float *value);

/**
 * @brief Reads the power factor for phase A.
 *
 * @param handle Pointer to the communication handle.
 * @param value Reference to a float where the power factor will be stored.
 * @return ESP_OK on successful read, ESP_FAIL otherwise.
 */
esp_err_t jsymk33_read_power_factor_A(jsymk33_handle_t handle, float *value);

/**
 * @brief Reads the power factor for phase B.
 *
 * @param handle Pointer to the communication handle.
 * @param value Reference to a float where the power factor will be stored.
 * @return ESP_OK on successful read, ESP_FAIL otherwise.
 */
esp_err_t jsymk33_read_power_factor_B(jsymk33_handle_t handle, float *value);

/**
 * @brief Reads the power factor for phase C.
 *
 * @param handle Pointer to the communication handle.
 * @param value Reference to a float where the power factor will be stored.
 * @return ESP_OK on successful read, ESP_FAIL otherwise.
 */
esp_err_t jsymk33_read_power_factor_C(jsymk33_handle_t handle, float *value);

/**
 * @brief Reads the total power factor.
 *
 * @param handle Pointer to the communication handle.
 * @param value Reference to a float where the power factor will be stored.
 * @return ESP_OK on successful read, ESP_FAIL otherwise.
 */
esp_err_t jsymk33_read_total_power_factor(jsymk33_handle_t handle, float *value);

/**
 * @brief Reads the active energy for phase A.
 *
 * @param handle Pointer to the communication handle.
 * @param value Reference to a float where the energy will be stored.
 * @return ESP_OK on successful read, ESP_FAIL otherwise.
 */
esp_err_t jsymk33_read_active_energy_A(jsymk33_handle_t handle, float *value);

/**
 * @brief Reads the active energy for phase B.
 *
 * @param handle Pointer to the communication handle.
 * @param value Reference to a float where the energy will be stored.
 * @return ESP_OK on successful read, ESP_FAIL otherwise.
 */
esp_err_t jsymk33_read_active_energy_B(jsymk33_handle_t handle, float *value);

/**
 * @brief Reads the active energy for phase C.
 *
 * @param handle Pointer to the communication handle.
 * @param value Reference to a float where the energy will be stored.
 * @return ESP_OK on successful read, ESP_FAIL otherwise.
 */
esp_err_t jsymk33_read_active_energy_C(jsymk33_handle_t handle, float *value);

/**
 * @brief Reads the total active energy.
 *
 * @param handle Pointer to the communication handle.
 * @param value Reference to a float where the energy will be stored.
 * @return ESP_OK on successful read, ESP_FAIL otherwise.
 */
esp_err_t jsymk33_read_total_active_energy(jsymk33_handle_t handle, float *value);

/**
 * @brief Reads the reactive energy for phase A.
 *
 * @param handle Pointer to the communication handle.
 * @param value Reference to a float where the energy will be stored.
 * @return ESP_OK on successful read, ESP_FAIL otherwise.
 */
esp_err_t jsymk33_read_reactive_energy_A(jsymk33_handle_t handle, float *value);

/**
 * @brief Reads the reactive energy for phase B.
 *
 * @param handle Pointer to the communication handle.
 * @param value Reference to a float where the energy will be stored.
 * @return ESP_OK on successful read, ESP_FAIL otherwise.
 */
esp_err_t jsymk33_read_reactive_energy_B(jsymk33_handle_t handle, float *value);

/**
 * @brief Reads the reactive energy for phase C.
 *
 * @param handle Pointer to the communication handle.
 * @param value Reference to a float where the energy will be stored.
 * @return ESP_OK on successful read, ESP_FAIL otherwise.
 */
esp_err_t jsymk33_read_reactive_energy_C(jsymk33_handle_t handle, float *value);

/**
 * @brief Reads the total reactive energy.
 *
 * @param handle Pointer to the communication handle.
 * @param value Reference to a float where the energy will be stored.
 * @return ESP_OK on successful read, ESP_FAIL otherwise.
 */
esp_err_t jsymk33_read_total_reactive_energy(jsymk33_handle_t handle, float *value);

/**
 * @brief Reads the apparent energy for phase A.
 *
 * @param handle Pointer to the communication handle.
 * @param value Reference to a float where the energy will be stored.
 * @return ESP_OK on successful read, ESP_FAIL otherwise.
 */
esp_err_t jsymk33_read_apparent_energy_A(jsymk33_handle_t handle, float *value);

/**
 * @brief Reads the apparent energy for phase B.
 *
 * @param handle Pointer to the communication handle.
 * @param value Reference to a float where the energy will be stored.
 * @return ESP_OK on successful read, ESP_FAIL otherwise.
 */
esp_err_t jsymk33_read_apparent_energy_B(jsymk33_handle_t handle, float *value);

/**
 * @brief Reads the apparent energy for phase C.
 *
 * @param handle Pointer to the communication handle.
 * @param value Reference to a float where the energy will be stored.
 * @return ESP_OK on successful read, ESP_FAIL otherwise.
 */
esp_err_t jsymk33_read_apparent_energy_C(jsymk33_handle_t handle, float *value);

/**
 * @brief Reads the total apparent energy.
 *
 * @param handle Pointer to the communication handle.
 * @param value Reference to a float where the energy will be stored.
 * @return ESP_OK on successful read, ESP_FAIL otherwise.
 */
esp_err_t jsymk33_read_total_apparent_energy(jsymk33_handle_t handle, float *value);

/**
 * @brief Reads the power direction.
 *
 * @param handle Pointer to the communication handle.
 * @param value Reference to a uint16_t where the power direction will be stored.
 * @return ESP_OK on successful read, ESP_FAIL otherwise.
 */
esp_err_t jsymk33_read_power_direction(jsymk33_handle_t handle, uint16_t *value);

/**
 * @brief Reads the alarm status.
 *
 * @param handle Pointer to the communication handle.
 * @param value Reference to a uint16_t where the alarm status will be stored.
 * @return ESP_OK on successful read, ESP_FAIL otherwise.
 */
esp_err_t jsymk33_read_alarm_status(jsymk33_handle_t handle, uint16_t *value);


#endif // __JSY_MK_33_LIB