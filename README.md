# JSY-MK-333 Component for ESP-IDF

## Description

This is a component/library for interfacing with the JSY-MK-333 energy monitoring device using ESP-IDF. It provides functions to read various electrical parameters such as voltage, current, power, energy, power factor, and frequency from the device through a communication protocol.

This component is based on the [https://github.com/saniyo/JSY-MK-333](https://github.com/saniyo/JSY-MK-333) repo.

## Features
- Read voltage, current, power, energy, power factor, and frequency for all three phases (A, B, C).
- Support for total power and energy measurements.
- Provides functions for reading alarm status and power direction.
- Designed to work seamlessly with ESP-IDF projects.

## Requirements
- ESP-IDF (version 4.4 or higher recommended).
- JSY-MK-333 device connected via a compatible communication interface (e.g., UART).

## Installation
1. Clone or copy this repository into the `components/` directory of your ESP-IDF project.
2. Include the component in your `CMakeLists.txt` file.
3. Build your project with `idf.py build`.

## Initialization and Configuration

To use this component in your project, you first need to include it and configure communication with the device. Below is an example of how to initialize the component and read voltage and current values:

### Step 1: Include Necessary Headers

```cpp
#include "jsymk333.h"
```

### Step 2: Configuration and Initialization

Before using the component, you need to create a communication handler and configure the device. Here’s an example of how to do it:

```c
// Define the device configuration
jsymk33_config_t config = {
    .uart_num = 2, 
    .baud = 115200,
    .rx_pin = 16,
    .tx_pin = 17,
    .uart_buffer_size = 512,
};

// Define the device handle
jsymk33_handle_t device_handle = NULL;

// Initialize the device with the configuration
esp_err_t ret = jsymk333_init(device_handle, &config);
if (ret != ESP_OK) {
    ESP_LOGE("JSYM", "Failed to initialize the device");
    return;
}
```

### Step 3: Read Data from the Device

Once the device is initialized, you can start reading the electrical parameters. Here’s an example of how to read phase A voltage and phase A current:

```c
float voltage_A = 0.0;
float current_A = 0.0;

// Read the voltage for phase A
ret = jsymk333_read_voltage_A(device_handle, &voltage_A);
if (ret == ESP_OK) {
    ESP_LOGI("JSYM", "Voltage A: %.2f V", voltage_A);
} else {
    ESP_LOGE("JSYM", "Failed to read voltage A");
}

// Read the current for phase A
ret = jsymk333_read_current_A(device_handle, &current_A);
if (ret == ESP_OK) {
    ESP_LOGI("JSYM", "Current A: %.2f A", current_A);
} else {
    ESP_LOGE("JSYM", "Failed to read current A");
}
```

### Step 4: Deinitialize the Device

When you're done reading data, you can deinitialize the device communication:

```c
jsymk333_deinit(device_handle);
```

## API Documentation

The following functions are available:

- `jsymk333_read_voltage_A()`
- `jsymk333_read_voltage_B()` 
- `jsymk333_read_voltage_C()`
- `jsymk333_read_current_A()`
- `jsymk333_read_current_B()`
- `jsymk333_read_current_C()`
- `jsymk333_read_active_power_A()`
- `jsymk333_read_active_power_B()`
- `jsymk333_read_active_power_C()`
- `jsymk333_read_total_active_power()`
- `jsymk333_read_reactive_power_A()`
- `jsymk333_read_reactive_power_B()`
- `jsymk333_read_reactive_power_C()`
- `jsymk333_read_total_reactive_power()`
- `jsymk333_read_apparent_power_A()`
- `jsymk333_read_apparent_power_B()`
- `jsymk333_read_apparent_power_C()`
- `jsymk333_read_total_apparent_power()`
- `jsymk333_read_frequency()`
- `jsymk333_read_power_factor_A()`
- `jsymk333_read_power_factor_B()`
- `jsymk333_read_power_factor_C()`
- `jsymk333_read_total_power_factor()`
- `jsymk333_read_active_energy_A()`
- `jsymk333_read_active_energy_B()`
- `jsymk333_read_active_energy_C()`
- `jsymk333_read_total_active_energy()`
- `jsymk333_read_reactive_energy_A()`
- `jsymk333_read_reactive_energy_B()`
- `jsymk333_read_reactive_energy_C()`
- `jsymk333_read_total_reactive_energy()`
- `jsymk333_read_apparent_energy_A()`
- `jsymk333_read_apparent_energy_B()`
- `jsymk333_read_apparent_energy_C()`
- `jsymk333_read_total_apparent_energy()`
- `jsymk333_read_power_direction()`
- `jsymk333_read_alarm_status()`

## License

This project is licensed under the MIT License.
