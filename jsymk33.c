#include "jsymk33.h"

#include "jsy_crc.h"

#include "esp_check.h"
#include "esp_timer.h"
#include "driver/uart.h"


/**
 * @brief Reads a single 16-bit register from the device and converts it to a float.
 *
 * @param handle Pointer to the communication handle.
 * @param address Register address to read from.
 * @param reg_value Reference to the float where the result will be stored.
 * @param factor Conversion factor to apply to the read value.
 * @return ESP_OK on successful read, ESP_FAIL otherwise.
 */
esp_err_t jsymk33_read_single_register(jsymk33_handle_t handle, uint16_t address, float& reg_value, float factor) {
    uint16_t reg;
    if (jsymk33_send_cmd_8(handle, 0x03, address, 1, true, OxFFFF) == ESP_OK) {
        uint16_t readed_bytes = 0;
        if (jsymk33_receive(handle, (uint8_t*)regs, 2, readed_bytes, 500)) {
            if (readed_bytes == 2) {
                reg_value =  reg / factor;
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
esp_err_t jsymk33_read_double_register(jsymk33_handle_t handle, uint16_t address, float& reg_value, float factor) {
    uint16_t regs[2];
    if (jsymk33_send_cmd_8(handle, 0x03, address, 2, true, OxFFFF) == ESP_OK) {
        uint16_t readed_bytes = 0;
        if (jsymk33_receive(handle, (uint8_t*)regs, 4, readed_bytes, 500)) {
            if (readed_bytes == 4) {
                uint32_t value = (regs[0] << 16) | regs[1];
                reg_value =  value / factor;
                return ESP_OK;
            }
        }
    }
    
    return ESP_FAIL;
}

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
esp_err_t jsymk33_receive(jsymk33_handle_t handle, uint8_t *resp, uint16_t len, uint16_t& readed_bytes, uint32_t timeout_ms) {
    if (!handle) {
        return ESP_FAIL;
    }
    jsymk33_config_t* conf = (jsymk33_config_t)handle;
    uint16_t pos = 0;
    uint32_t time_cmp = esp_timer_get_time() / 1000L;

    while ((pos < len) && ((esp_timer_get_time() / 1000L) - time_cmp) < timeout_ms) {
        size_t available;
        uart_get_buffered_data_len(conf->uart_num, &available);
        if (available) {
            available = uart_read_bytes(conf->uart_num, resp + pos, available, 0);
            pos += available;
        }
    }

    return ESP_OK;
}

/**
 * @brief Sends a command via UART and optionally checks the response.
 *
 * @param handle Pointer to the communication handle.
 * @param cmd Command code to send.
 * @param r_addr Register address to communicate with.
 * @param val Value to be sent to the register.
 * @param check Boolean flag indicating whether to check the response.
 * @param slave_addr Address of the slave device (1 - 247).
 * @return ESP_OK if the command is successfully sent and validated (if applicable).
 */
esp_err_t jsymk33_send_cmd_8(jsymk33_handle_t handle, uint8_t cmd, uint16_t r_addr, uint16_t val, bool check, uint16_t slave_addr) {
    if (!handle) {
        return ESP_FAIL;
    }
    jsymk33_config_t* conf = (jsymk33_config_t)handle;
    uint8_t send_buffer[8] = {0};
    uint8_t resp_buffer[8] = {0};

    if ((slave_addr == 0xFFFF) || (slave_addr < 0x01) || (slave_addr > 0xF7)) {
        slave_addr = 1; // Default address
    }

    send_buffer[0] = slave_addr;
    send_buffer[1] = cmd;
    memcpy(send_buffer, &r_addr, sizeof(r_addr));
    memcpy(send_buffer, &val, sizeof(val));
    jsy_set_crc(send_buffer, 8);
    uart_write_bytes(conf->uart_num, send_buffer, 8);

    if (check) {
        uint16_t readed_bytes
        if (jsymk33_receive(handle, resp_buffer, 8, readed_bytes, 500) != ESP_OK) {
            return ESP_FAIL;
        }

        if (readed_bytes != 8) {
            return ESP_FAIL;
        }

        if (memcmp(send_buffer, resp_buffer, 8)) {
            return ESP_FAIL;
        }
    }
    return ESP_OK;
}


esp_err_t jsymk33_init(jsymk33_handle_t handle, jsymk33_config_t *conf) {
    uart_config_t uart_config;
    memset(&uart_config, 0, sizeof(uart_config_t));
    uart_config.baud_rate = conf->baud;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_EVEN;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_config.rx_flow_ctrl_thresh = 122;

    // Configure UART parameters
    ESP_ERROR_CHECK(uart_param_config(conf->uart_num, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(conf->uart_num, conf->tx_pin, conf->rx_pin, -1, -1));

    // Install UART driver using an event queue here
    ESP_ERROR_CHECK(uart_driver_install(conf->uart_num, conf->uart_buffer_size, conf->uart_buffer_size, 10, &conf->uart_queue, 0));
  
    // Flush the input buffer
    uart_flush_input(conf->uart_num);

    handle = malloc(1, sizeof(jsymk33_config_t));
    memcpy(handle, conf, sizeof(jsymk33_config_t));

    return ESP_OK;
}

esp_err_t jsymk33_deinit(jsymk33_handle_t handle) {
    if (!handle) {
        return ESP_FAIL;
    }
    if (uart_is_driver_installed(handle->uart_num)) {
        return uart_driver_delete(handle->uart_num);
    }
    return ESP_OK;
}

esp_err_t jsymk33_read_voltage_A(jsymk33_handle_t handle, float& value) { 
    return jsymk33_read_single_register(handle, 0x0100, value, 100.0); 
}

esp_err_t jsymk33_read_voltage_B(jsymk33_handle_t handle, float& value) { 
    return jsymk33_read_single_register(handle, 0x0101, value, 100.0); 
}

esp_err_t jsymk33_read_voltage_C(jsymk33_handle_t handle, float& value) { 
    return jsymk33_read_single_register(handle, 0x0102, value, 100.0); 
}

esp_err_t jsymk33_read_current_A(jsymk33_handle_t handle, float& value) { 
    return jsymk33_read_single_register(handle, 0x0103, value, 100.0); 
}

esp_err_t jsymk33_read_current_B(jsymk33_handle_t handle, float& value) { 
    return jsymk33_read_single_register(handle, 0x0104, value, 100.0); 
}

esp_err_t jsymk33_read_current_C(jsymk33_handle_t handle, float& value) { 
    return jsymk33_read_single_register(handle, 0x0105, value, 100.0); 
}

esp_err_t jsymk33_read_active_power_A(jsymk33_handle_t handle, float& value) { 
    return jsymk33_read_single_register(handle, 0x0106, value, 1.0); 
}

esp_err_t jsymk33_read_active_power_B(jsymk33_handle_t handle, float& value) { 
    return jsymk33_read_single_register(handle, 0x0107, value, 1.0); 
}

esp_err_t jsymk33_read_active_power_C(jsymk33_handle_t handle, float& value) { 
    return jsymk33_read_single_register(handle, 0x0108, value, 1.0); 
}

esp_err_t jsymk33_read_total_active_power(jsymk33_handle_t handle, float& value) { 
    return jsymk33_read_double_register(handle, 0x0109, value, 1.0); 
}

esp_err_t jsymk33_read_reactive_power_A(jsymk33_handle_t handle, float& value) { 
    return jsymk33_read_single_register(handle, 0x010B, value, 1.0); 
}

esp_err_t jsymk33_read_reactive_power_B(jsymk33_handle_t handle, float& value) { 
    return jsymk33_read_single_register(handle, 0x010C, value, 1.0); 
}

esp_err_t jsymk33_read_reactive_power_C(jsymk33_handle_t handle, float& value) { 
    return jsymk33_read_single_register(handle, 0x010D, value, 1.0); 
}

esp_err_t jsymk33_read_total_reactive_power(jsymk33_handle_t handle, float& value) { 
    return jsymk33_read_double_register(handle, 0x010E, value, 1.0); 
}

esp_err_t jsymk33_read_apparent_power_A(jsymk33_handle_t handle, float& value) { 
    return jsymk33_read_single_register(handle, 0x0110, value, 1.0); 
}

esp_err_t jsymk33_read_apparent_power_B(jsymk33_handle_t handle, float& value) { 
    return jsymk33_read_single_register(handle, 0x0111, value, 1.0); 
}

esp_err_t jsymk33_read_apparent_power_C(jsymk33_handle_t handle, float& value) { 
    return jsymk33_read_single_register(handle, 0x0112, value, 1.0); 
}

esp_err_t jsymk33_read_total_apparent_power(jsymk33_handle_t handle, float& value) { 
    return jsymk33_read_double_register(handle, 0x0113, value, 1.0);
}

esp_err_t jsymk33_read_frequency(jsymk33_handle_t handle, float& value) { 
    return jsymk33_read_single_register(handle, 0x0115, value, 100.0); 
}

esp_err_t jsymk33_read_power_factor_A(jsymk33_handle_t handle, float& value) { 
    return jsymk33_read_single_register(handle, 0x0116, value, 1000.0); 
}

esp_err_t jsymk33_read_power_factor_B(jsymk33_handle_t handle, float& value) { 
    return jsymk33_read_single_register(handle, 0x0117, value, 1000.0); 
}

esp_err_t jsymk33_read_power_factor_C(jsymk33_handle_t handle, float& value) { 
    return jsymk33_read_single_register(handle, 0x0118, value, 1000.0); 
}

esp_err_t jsymk33_read_total_power_factor(jsymk33_handle_t handle, float& value) { 
    return jsymk33_read_single_register(handle, 0x0119, value, 1000.0); 
}

esp_err_t jsymk33_read_active_energy_A(jsymk33_handle_t handle, float& value) { 
    return jsymk33_read_double_register(handle, 0x011A, value, 100.0); 
}

esp_err_t jsymk33_read_active_energy_B(jsymk33_handle_t handle, float& value) { 
    return jsymk33_read_double_register(handle, 0x011C, value, 100.0); 
}

esp_err_t jsymk33_read_active_energy_C(jsymk33_handle_t handle, float& value) { 
    return jsymk33_read_double_register(handle, 0x011E, value, 100.0); 
}

esp_err_t jsymk33_read_total_active_energy(jsymk33_handle_t handle, float& value) { 
    return jsymk33_read_double_register(handle, 0x0120, value, 100.0); 
}

esp_err_t jsymk33_read_reactive_energy_A(jsymk33_handle_t handle, float& value) { 
    return jsymk33_read_double_register(handle, 0x0122, value, 100.0); 
}

esp_err_t jsymk33_read_reactive_energy_B(jsymk33_handle_t handle, float& value) { 
    return jsymk33_read_double_register(handle, 0x0124, value, 100.0); 
}

esp_err_t jsymk33_read_reactive_energy_C(jsymk33_handle_t handle, float& value) { 
    return jsymk33_read_double_register(handle, 0x0126, value, 100.0); 
}

esp_err_t jsymk33_read_total_reactive_energy(jsymk33_handle_t handle, float& value) { 
    return jsymk33_read_double_register(handle, 0x0128, value, 100.0); 
}

esp_err_t jsymk33_read_apparent_energy_A(jsymk33_handle_t handle, float& value) { 
    return jsymk33_read_double_register(handle, 0x012A, value, 100.0); 
}

esp_err_t jsymk33_read_apparent_energy_B(jsymk33_handle_t handle, float& value) { 
    return jsymk33_read_double_register(handle, 0x012C, value, 100.0); 
}

esp_err_t jsymk33_read_apparent_energy_C(jsymk33_handle_t handle, float& value) { 
    return jsymk33_read_double_register(handle, 0x012E, value, 100.0); 
}

esp_err_t jsymk33_read_total_apparent_energy(jsymk33_handle_t handle, float& value) { 
    return jsymk33_read_double_register(handle, 0x0130, value, 100.0); 
}

esp_err_t jsymk33_read_power_direction(jsymk33_handle_t handle, uint16_t& value) {
    float v_value = 0;
    if (jsymk33_read_single_register(handle, 0x0132, v_value, 1.0) == ESP_OK) {
        value = v_value;
        return ESP_OK;
    }
    return ESP_FAIL;
}

esp_err_t jsymk33_read_alarm_status(jsymk33_handle_t handle, uint16_t& value ) {
    float v_value = 0;
    if (jsymk33_read_single_register(handle, 0x0133, v_value, 1.0) == ESP_OK) {
        value = v_value;
        return ESP_OK;
    }
    return ESP_FAIL;
}
