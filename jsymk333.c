#include "jsymk333.h"

#include "jsy_crc.h"

#include "esp_check.h"
#include "esp_timer.h"
#include "driver/uart.h"
#include "jsymk333_regs.h"


/**
 * @brief Receives data from the UART port.
 *
 * This function reads data from the UART buffer until the desired length is reached or the timeout expires.
 *
 * @param handle Pointer to the communication handle.
 * @param resp Buffer to store the received data.
 * @param len Number of bytes expected to be read.
 * @param readed_bytes Reference to a variable where the number of bytes read will be stored.
 * @param timeout_ms Timeout in milliseconds for receiving data.
 * @return ESP_OK if data is successfully received, ESP_FAIL otherwise.
 */
esp_err_t jsymk333_receive(jsymk333_handle_t handle, uint8_t *resp, uint16_t len, uint16_t* readed_bytes, uint32_t timeout_ms) {
    if (!handle) {
        ESP_LOGE("JSYMK333", "Invalid handle");
        return ESP_FAIL;
    }
    jsymk333_config_t* conf = (jsymk333_config_t*)handle;
    uint32_t time_cmp = esp_timer_get_time() / 1000L;
    *readed_bytes = 0;

    while ((*readed_bytes < len) && ((esp_timer_get_time() / 1000L) - time_cmp) < timeout_ms) {
        size_t available = 0;
        if (uart_get_buffered_data_len(conf->uart_num, &available) == ESP_OK) {
            if (available) {
                available = uart_read_bytes(conf->uart_num, resp + *readed_bytes, available, 0);
                *readed_bytes += available;
            }
        }
    }

#if CONFIG_JSY_MK_333_PRINT_BUFFER
    ESP_LOG_BUFFER_HEXDUMP("JSYMK333-RX", resp, *readed_bytes, ESP_LOG_INFO);
#endif

    return ESP_OK;
}

/**
 * @brief Sends a command via UART and optionally checks the response.
 *
 * @param handle Pointer to the communication handle.
 * @param cmd Command code to send.
 * @param reg_addr Register address to communicate with.
 * @param regs_num Value to be sent to the register.
 * @param check Boolean flag indicating whether to check the response.
 * @param slave_addr Address of the slave device (1 - 247).
 * @return ESP_OK if the command is successfully sent and validated (if applicable).
 */
esp_err_t jsymk333_send_cmd_8(jsymk333_handle_t handle, uint8_t cmd, uint16_t reg_addr, uint16_t regs_num, uint16_t slave_addr) {
    if (!handle) {
        ESP_LOGE("JSYMK333", "Invalid handle");
        return ESP_FAIL;
    }
    jsymk333_config_t* conf = (jsymk333_config_t*)handle;
    uint8_t send_buffer[8] = {0};

    if ((slave_addr == 0xFFFF) || (slave_addr < 0x01) || (slave_addr > 0xF7)) {
        slave_addr = 1; // Default address
    }

    send_buffer[0] = slave_addr;
    send_buffer[1] = cmd;
    send_buffer[2] = (reg_addr >> 8) & 0xFF;
    send_buffer[3] = (reg_addr) & 0xFF;
    send_buffer[4] = (regs_num >> 8) & 0xFF;
    send_buffer[5] = (regs_num) & 0xFF;
    jsy_set_crc(send_buffer, 8);
    uart_write_bytes(conf->uart_num, send_buffer, 8);
#if CONFIG_JSY_MK_333_PRINT_BUFFER
    ESP_LOG_BUFFER_HEXDUMP("JSYMK333-TX", send_buffer, 8, ESP_LOG_INFO);
#endif
    return ESP_OK;
}

/**
 * @brief Reads a single 16-bit register from the device and converts it to a float.
 *
 * @param handle Pointer to the communication handle.
 * @param address Register address to read from.
 * @param reg_value Reference to the float where the result will be stored.
 * @param factor Conversion factor to apply to the read value.
 * @return ESP_OK on successful read, ESP_FAIL otherwise.
 */
esp_err_t jsymk333_read_single_register(jsymk333_handle_t handle, uint16_t address, float* reg_value, float factor) {
    uint16_t reg = 0;
    if (jsymk333_send_cmd_8(handle, 0x03, address, 1, UINT16_MAX) == ESP_OK) {
        uint16_t readed_bytes = 0;
        if (jsymk333_receive(handle, (uint8_t*)&reg, 2, &readed_bytes, 500) == ESP_OK) {
            if (readed_bytes == 2) {
                *reg_value = reg / factor;
                return ESP_OK;
            }
        }
    }
    
    return ESP_FAIL;
}

/**
 * @brief Reads a double register (32 bits) from the device and converts it to a float.
 *
 * @param handle Pointer to the communication handle.
 * @param address Register address to read from.
 * @param reg_value Reference to the float where the result will be stored.
 * @param factor Conversion factor to apply to the read value.
 * @return ESP_OK on successful read, ESP_FAIL otherwise.
 */
esp_err_t jsymk333_read_double_register(jsymk333_handle_t handle, uint16_t address, float* reg_value, float factor) {
    uint16_t regs[2];
    if (jsymk333_send_cmd_8(handle, 0x03, address, 2, UINT16_MAX) == ESP_OK) {
        uint16_t readed_bytes = 0;
        if (jsymk333_receive(handle, (uint8_t*)regs, 4, &readed_bytes, 500) == ESP_OK) {
            if (readed_bytes == 4) {
                uint32_t value = (regs[0] << 16) | regs[1];
                *reg_value = value / factor;
                return ESP_OK;
            }
        }
    }
    
    return ESP_FAIL;
}

esp_err_t jsymk333_read_registers(jsymk333_handle_t handle, uint16_t address, uint16_t num, uint8_t* value) {
    if (jsymk333_send_cmd_8(handle, 0x03, address, num, UINT16_MAX) == ESP_OK) {
        uint16_t readed_bytes = 0;
        uint16_t expected_bytes = JSY_MK_RESPONSE_SIZE_READ + JSY_MK_333_REGISTER_LEN * num;
        if (jsymk333_receive(handle, value, expected_bytes, &readed_bytes, 500) == ESP_OK) {
            if (readed_bytes == expected_bytes) {
                return ESP_OK;
            }
        }
    }
    
    return ESP_FAIL;
}

esp_err_t jsymk333_read_all_registers(jsymk333_handle_t handle, uint8_t* value) {
    return jsymk333_read_registers(handle, JSY_MK_333_REGISTER_START, JSY_MK_333_REGISTER_COUNT, value);
}

esp_err_t jsymk333_init(jsymk333_handle_t *handle, jsymk333_config_t *conf) {
    uart_config_t uart_config;
    esp_err_t ret = ESP_OK;
    memset(&uart_config, 0, sizeof(uart_config_t));
    uart_config.baud_rate = conf->baud;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_EVEN;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_config.rx_flow_ctrl_thresh = 122;

    // Configure UART parameters
    ESP_GOTO_ON_ERROR(uart_param_config(conf->uart_num, &uart_config), err, "JSYMK333", "Failed to configure UART parameters");
    ESP_GOTO_ON_ERROR(uart_set_pin(conf->uart_num, conf->tx_pin, conf->rx_pin, -1, -1), err, "JSYMK333", "Failed to set UART pins");

    // Install UART driver using an event queue here
    ESP_GOTO_ON_ERROR(uart_driver_install(conf->uart_num, conf->uart_buffer_size, conf->uart_buffer_size, 10, &conf->uart_queue, 0), err, "JSYMK333", "Failed to install UART driver");
  
    // Flush the input buffer
    uart_flush_input(conf->uart_num);

    *handle = (jsymk333_handle_t)malloc(sizeof(jsymk333_config_t));
    ESP_GOTO_ON_FALSE(*handle, ESP_ERR_NO_MEM, err, "JSYMK333", "Failed to allocate memory for JSYMK333 handle");
    memcpy(*handle, conf, sizeof(jsymk333_config_t));

    return ESP_OK;
err:
    if (uart_is_driver_installed(conf->uart_num)) {
        uart_driver_delete(conf->uart_num);
    }

    if (*handle) {
        free(*handle);
        *handle = NULL;
    }
    ESP_LOGE("JSYMK333", "Failed to initialize JSYMK333 device (%s)", esp_err_to_name(ret));
    return ret;
}

esp_err_t jsymk333_deinit(jsymk333_handle_t handle) {
    if (!handle) {
        return ESP_FAIL;
    }

    esp_err_t err = ESP_OK;
    jsymk333_config_t* conf = (jsymk333_config_t*)handle;

    // Delete the UART driver
    if (uart_is_driver_installed(conf->uart_num)) {
        err = uart_driver_delete(conf->uart_num);
    }

    free(handle);
    handle = NULL;
    return err;
}

esp_err_t jsymk333_read_voltage_A(jsymk333_handle_t handle, float *value) { 
    return jsymk333_read_single_register(handle, 0x0100, value, 100.0); 
}

esp_err_t jsymk333_read_voltage_B(jsymk333_handle_t handle, float *value) { 
    return jsymk333_read_single_register(handle, 0x0101, value, 100.0); 
}

esp_err_t jsymk333_read_voltage_C(jsymk333_handle_t handle, float *value) { 
    return jsymk333_read_single_register(handle, 0x0102, value, 100.0); 
}

esp_err_t jsymk333_read_current_A(jsymk333_handle_t handle, float *value) { 
    return jsymk333_read_single_register(handle, 0x0103, value, 100.0); 
}

esp_err_t jsymk333_read_current_B(jsymk333_handle_t handle, float *value) { 
    return jsymk333_read_single_register(handle, 0x0104, value, 100.0); 
}

esp_err_t jsymk333_read_current_C(jsymk333_handle_t handle, float *value) { 
    return jsymk333_read_single_register(handle, 0x0105, value, 100.0); 
}

esp_err_t jsymk333_read_active_power_A(jsymk333_handle_t handle, float *value) { 
    return jsymk333_read_single_register(handle, 0x0106, value, 1.0); 
}

esp_err_t jsymk333_read_active_power_B(jsymk333_handle_t handle, float *value) { 
    return jsymk333_read_single_register(handle, 0x0107, value, 1.0); 
}

esp_err_t jsymk333_read_active_power_C(jsymk333_handle_t handle, float *value) { 
    return jsymk333_read_single_register(handle, 0x0108, value, 1.0); 
}

esp_err_t jsymk333_read_total_active_power(jsymk333_handle_t handle, float *value) { 
    return jsymk333_read_double_register(handle, 0x0109, value, 1.0); 
}

esp_err_t jsymk333_read_reactive_power_A(jsymk333_handle_t handle, float *value) { 
    return jsymk333_read_single_register(handle, 0x010B, value, 1.0); 
}

esp_err_t jsymk333_read_reactive_power_B(jsymk333_handle_t handle, float *value) { 
    return jsymk333_read_single_register(handle, 0x010C, value, 1.0); 
}

esp_err_t jsymk333_read_reactive_power_C(jsymk333_handle_t handle, float *value) { 
    return jsymk333_read_single_register(handle, 0x010D, value, 1.0); 
}

esp_err_t jsymk333_read_total_reactive_power(jsymk333_handle_t handle, float *value) { 
    return jsymk333_read_double_register(handle, 0x010E, value, 1.0); 
}

esp_err_t jsymk333_read_apparent_power_A(jsymk333_handle_t handle, float *value) { 
    return jsymk333_read_single_register(handle, 0x0110, value, 1.0); 
}

esp_err_t jsymk333_read_apparent_power_B(jsymk333_handle_t handle, float *value) { 
    return jsymk333_read_single_register(handle, 0x0111, value, 1.0); 
}

esp_err_t jsymk333_read_apparent_power_C(jsymk333_handle_t handle, float *value) { 
    return jsymk333_read_single_register(handle, 0x0112, value, 1.0); 
}

esp_err_t jsymk333_read_total_apparent_power(jsymk333_handle_t handle, float *value) { 
    return jsymk333_read_double_register(handle, 0x0113, value, 1.0);
}

esp_err_t jsymk333_read_frequency(jsymk333_handle_t handle, float *value) { 
    return jsymk333_read_single_register(handle, 0x0115, value, 100.0); 
}

esp_err_t jsymk333_read_power_factor_A(jsymk333_handle_t handle, float *value) { 
    return jsymk333_read_single_register(handle, 0x0116, value, 1000.0); 
}

esp_err_t jsymk333_read_power_factor_B(jsymk333_handle_t handle, float *value) { 
    return jsymk333_read_single_register(handle, 0x0117, value, 1000.0); 
}

esp_err_t jsymk333_read_power_factor_C(jsymk333_handle_t handle, float *value) { 
    return jsymk333_read_single_register(handle, 0x0118, value, 1000.0); 
}

esp_err_t jsymk333_read_total_power_factor(jsymk333_handle_t handle, float *value) { 
    return jsymk333_read_single_register(handle, 0x0119, value, 1000.0); 
}

esp_err_t jsymk333_read_active_energy_A(jsymk333_handle_t handle, float *value) { 
    return jsymk333_read_double_register(handle, 0x011A, value, 100.0); 
}

esp_err_t jsymk333_read_active_energy_B(jsymk333_handle_t handle, float *value) { 
    return jsymk333_read_double_register(handle, 0x011C, value, 100.0); 
}

esp_err_t jsymk333_read_active_energy_C(jsymk333_handle_t handle, float *value) { 
    return jsymk333_read_double_register(handle, 0x011E, value, 100.0); 
}

esp_err_t jsymk333_read_total_active_energy(jsymk333_handle_t handle, float *value) { 
    return jsymk333_read_double_register(handle, 0x0120, value, 100.0); 
}

esp_err_t jsymk333_read_reactive_energy_A(jsymk333_handle_t handle, float *value) { 
    return jsymk333_read_double_register(handle, 0x0122, value, 100.0); 
}

esp_err_t jsymk333_read_reactive_energy_B(jsymk333_handle_t handle, float *value) { 
    return jsymk333_read_double_register(handle, 0x0124, value, 100.0); 
}

esp_err_t jsymk333_read_reactive_energy_C(jsymk333_handle_t handle, float *value) { 
    return jsymk333_read_double_register(handle, 0x0126, value, 100.0); 
}

esp_err_t jsymk333_read_total_reactive_energy(jsymk333_handle_t handle, float *value) { 
    return jsymk333_read_double_register(handle, 0x0128, value, 100.0); 
}

esp_err_t jsymk333_read_apparent_energy_A(jsymk333_handle_t handle, float *value) { 
    return jsymk333_read_double_register(handle, 0x012A, value, 100.0); 
}

esp_err_t jsymk333_read_apparent_energy_B(jsymk333_handle_t handle, float *value) { 
    return jsymk333_read_double_register(handle, 0x012C, value, 100.0); 
}

esp_err_t jsymk333_read_apparent_energy_C(jsymk333_handle_t handle, float *value) { 
    return jsymk333_read_double_register(handle, 0x012E, value, 100.0); 
}

esp_err_t jsymk333_read_total_apparent_energy(jsymk333_handle_t handle, float *value) { 
    return jsymk333_read_double_register(handle, 0x0130, value, 100.0); 
}

esp_err_t jsymk333_read_power_direction(jsymk333_handle_t handle, uint16_t *value) {
    float v_value = 0;
    if (jsymk333_read_single_register(handle, 0x0132, &v_value, 1.0) == ESP_OK) {
        *value = (uint16_t)v_value;
        return ESP_OK;
    }
    return ESP_FAIL;
}

esp_err_t jsymk333_read_alarm_status(jsymk333_handle_t handle, uint16_t *value) {
    float v_value = 0;
    if (jsymk333_read_single_register(handle, 0x0133, &v_value, 1.0) == ESP_OK) {
        *value = (uint16_t)v_value;
        return ESP_OK;
    }
    return ESP_FAIL;
}
