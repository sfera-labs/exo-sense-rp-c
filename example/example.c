/*
  example.c - Exo Sense RP usage example

    Copyright (C) 2022 Sfera Labs S.r.l. - All rights reserved.

    For information, see:
    http://www.sferalabs.cc/

  This code is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.
  See file LICENSE.txt for further informations on licensing terms.
*/

#include <stdio.h>
#include "exo_sense_rp.h"
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/i2c.h"

#include "sensirion_common.h"
#include "sht4x.h"

#define RS485_UART_ID uart0
#define RS485_BAUD_RATE 115200
#define RS485_DATA_BITS 8
#define RS485_STOP_BITS 1
#define RS485_PARITY    UART_PARITY_NONE

#define I2C_BAUD_RATE 100000

// Sensirion library I2C functions stubs implementation

int8_t sensirion_i2c_read(uint8_t address, uint8_t* data, uint16_t count) {
    if (i2c_read_blocking(i2c_default, address, data, count, false) == count) {
        return STATUS_OK;
    }
    return STATUS_FAIL;
}

int8_t sensirion_i2c_write(uint8_t address, const uint8_t* data,
                           uint16_t count) {
    if (i2c_write_blocking(i2c_default, address, data, count, false) == count) {
        return STATUS_OK;
    }
    return STATUS_FAIL;
}

void sensirion_sleep_usec(uint32_t useconds) {
    sleep_us(useconds);
}

// Main loop

int main() {
    // Init pins
    gpio_init(EXOS_PIN_DO1);
    gpio_set_dir(EXOS_PIN_DO1, GPIO_OUT);

    gpio_init(EXOS_PIN_DI1);
    gpio_init(EXOS_PIN_DI2);
    gpio_set_dir(EXOS_PIN_DI1, GPIO_IN);
    gpio_set_dir(EXOS_PIN_DI2, GPIO_IN);
    gpio_disable_pulls(EXOS_PIN_DI1);
    gpio_disable_pulls(EXOS_PIN_DI2);

    gpio_init(EXOS_PIN_LED);
    gpio_set_dir(EXOS_PIN_LED, GPIO_OUT);

    gpio_init(EXOS_PIN_BUZZER);
    gpio_set_dir(EXOS_PIN_BUZZER, GPIO_OUT);

    gpio_init(EXOS_PIN_PIR);
    gpio_set_dir(EXOS_PIN_PIR, GPIO_IN);

    // Init RS-485 interface
    uart_init(RS485_UART_ID, RS485_BAUD_RATE);
    gpio_set_function(EXOS_PIN_RS485_TX, GPIO_FUNC_UART);
    gpio_set_function(EXOS_PIN_RS485_RX, GPIO_FUNC_UART);
    uart_set_hw_flow(RS485_UART_ID, false, false);
    uart_set_format(RS485_UART_ID, RS485_DATA_BITS, RS485_STOP_BITS, RS485_PARITY);

    // Init RS-485 TX-enable pin (inverted)
    gpio_init(EXOS_PIN_RS485_TXEN_N);
    gpio_set_dir(EXOS_PIN_RS485_TXEN_N, GPIO_OUT);
    gpio_put(EXOS_PIN_RS485_TXEN_N, 1); // 0 = transmit

    // Init I2C
    i2c_init(i2c_default, I2C_BAUD_RATE);
    gpio_set_function(EXOS_PIN_I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(EXOS_PIN_I2C_SCL, GPIO_FUNC_I2C);

    // Init USB for logging/debugging (see CMakeLists.txt)
    stdio_usb_init();

    sleep_ms(3000);

    // Useless: just to show the EXOS_I2C_ADDR_* constants
    if (sht4x_get_configured_address() != EXOS_I2C_ADDR_SENS_TEMP_RH) {
        printf("SHT4x lib error\n");
    }

    printf("Ready!\n");

    while (true) {
        // Blink LED
        gpio_put(EXOS_PIN_LED, 1);
        sleep_ms(250);
        gpio_put(EXOS_PIN_LED, 0);

        // Write DI1's state to USB
        printf("DI1: %d\n", gpio_get(EXOS_PIN_DI1));

        // Check RS-485 for incoming data
        while (uart_is_readable(RS485_UART_ID)) {
            uint8_t ch = uart_getc(RS485_UART_ID);
            if (ch == '\n') {
                // On a new line send OK back, 
                // driving the TX-enble line
                gpio_put(EXOS_PIN_RS485_TXEN_N, 0);
                uart_puts(RS485_UART_ID, "OK\n");
                gpio_put(EXOS_PIN_RS485_TXEN_N, 1);
            }
        }

        // Read temperature and humidity
        if (sht4x_probe() == STATUS_OK) {
            int32_t temperature, humidity;
            int8_t ret = sht4x_measure_blocking_read(&temperature, &humidity);
            if (ret == STATUS_OK) {
                printf("Temperature: %0.2f C\n", temperature / 1000.0f);
                printf("Humidity: %0.2f %%\n", humidity / 1000.0f);
            } else {
                printf("error reading SHT sensor\n");
            }
        } else {
            printf("error probing SHT sensor\n");
        }

        sleep_ms(2000);
    }

    return 0;
}
