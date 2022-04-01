/*
 * Copyright (c) 2021 Valentin Milea <valentin.milea@gmail.com>
 *
 * SPDX-License-Identifier: MIT
 */

#include <i2c_fifo.h>
#include <i2c_slave.h>
#include <pico/stdlib.h>
#include <stdio.h>
#include <string.h>
#include "hardware/pwm.h"

static const uint I2C_SLAVE_ADDRESS = 0x17;
static const uint I2C_BAUDRATE = 100000; // 100 kHz

// For this example, we run both the master and slave from the same board.
// You'll need to wire pin GP4 to GP6 (SDA), and pin GP5 to GP7 (SCL).
static const uint I2C_SLAVE_SDA_PIN = PICO_DEFAULT_I2C_SDA_PIN; // 4
static const uint I2C_SLAVE_SCL_PIN = PICO_DEFAULT_I2C_SCL_PIN; // 5

// The slave implements a 256 byte memory. To write a series of bytes, the master first
// writes the memory address, followed by the data. The address is automatically incremented
// for each byte transferred, looping back to 0 upon reaching the end. Reading is done
// sequentially from the current memory address.

enum i2c_register {
    INVALID = -1,
    HAT_ID_LSB = 0x00,
    HAT_ID_MSB = 0x01,
    HARD_ID = 0x02,
    SOFT_ID_LSB = 0x03,
    SOFT_ID_MSB = 0x04,
    HAT_POWER = 0x10,
    IR_MODE = 0x11,
    RING_MODE = 0x12,
    IR_BRIGHT = 0x20,
    RING_BRIGHT = 0x21,
    ROTATION = 0x30,
    TEMP = 0x31,
    P_STATUS = 0x40,
    H_STATUS = 0x41
};

static struct
{
    enum i2c_register address;
    bool write_mode;
} context;

struct state
{
    uint8_t ir_led_brightness;
    uint8_t ring_led_brightness;
} ;

struct state current_state;
struct state future_state;

#define IR_LED_PIN 18

static void i2c_slave_handler(i2c_inst_t *i2c, i2c_slave_event_t event) {
    uint8_t test;

    switch (event) {
        case I2C_SLAVE_RECEIVE:
            if (!context.write_mode) {
                context.address = i2c_register(i2c_read_byte(i2c));
                context.write_mode = true;

            } else {
                
                switch(context.address) {
                    case HAT_POWER:
                        printf("set power \n");
                        break;
                    case IR_BRIGHT:
                        test = i2c_read_byte(i2c);
                        future_state.ir_led_brightness = test;
                        break;
                    case RING_BRIGHT:
                        break;
                }
            }

            break;
        case I2C_SLAVE_REQUEST:

            switch(context.address) {
                case HARD_ID:
                    i2c_write_byte(i2c, 0x02);
                    break;
                case HAT_POWER:
                    i2c_write_byte(i2c, 0xEE);
                    break;
            }

            break;
        case I2C_SLAVE_FINISH: 
            context.write_mode = false;

            break;
        default:
            break;
    }
}

static void setup_slave() {
    gpio_init(I2C_SLAVE_SDA_PIN);
    gpio_set_function(I2C_SLAVE_SDA_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SLAVE_SDA_PIN);

    gpio_init(I2C_SLAVE_SCL_PIN);
    gpio_set_function(I2C_SLAVE_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SLAVE_SCL_PIN);

    i2c_init(i2c0, I2C_BAUDRATE);
    // configure I2C0 for slave mode
    i2c_slave_init(i2c0, I2C_SLAVE_ADDRESS, &i2c_slave_handler);
}


int main() {
    // ----------------- setup system vars -----------------
    current_state.ir_led_brightness = 0;
    current_state.ring_led_brightness = 0;

    // ----------------- setup all the io -----------------

    // CONSOLE
    stdio_init_all();
    puts("\n I2C slave example \n");

    // I2C SLAVE
    setup_slave();

    // // 0. Initialize LED strip
    // printf("0. Initialize LED strip\n");
    // auto ledStrip = PicoLed::addLeds<PicoLed::WS2812B>(pio1, 2, 15, 10, PicoLed::FORMAT_GRB);
    // ledStrip.setBrightness(64);
    // printf("1. Clear the strip!\n");
    
    // // 1. Set all LEDs to red!
    // printf("1. Set all LEDs to red!\n");
    // ledStrip.fill( PicoLed::RGB(255, 0, 0) );
    // ledStrip.show();
    // sleep_ms(500);

    // IR LED
    gpio_set_function(IR_LED_PIN, GPIO_FUNC_PWM);
    uint ir_led_slice = pwm_gpio_to_slice_num(IR_LED_PIN);
    uint ir_led_channel = pwm_gpio_to_channel(IR_LED_PIN);
    pwm_set_wrap(ir_led_slice, 255);
    pwm_set_chan_level(ir_led_slice, ir_led_channel, 0);
    pwm_set_enabled(ir_led_slice, true);
    

    while(true) {

        if (future_state.ir_led_brightness != current_state.ir_led_brightness) {

            printf("CHANGED \n");
            current_state.ir_led_brightness = future_state.ir_led_brightness;
            pwm_set_chan_level(ir_led_slice, ir_led_channel, current_state.ir_led_brightness);

        }
    }
}
