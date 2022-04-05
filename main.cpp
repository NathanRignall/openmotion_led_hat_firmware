#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/unique_id.h"

#include "hardware/pwm.h"
#include "hardware/adc.h"

#include <i2c_fifo.h>
#include <i2c_slave.h>

#include <PicoLed.hpp>
#include <Effects/Marquee.hpp>
#include <Effects/Stars.hpp>
#include <Effects/Comet.hpp>
#include <Effects/Bounce.hpp>
#include <Effects/Particles.hpp>

#define HARDWARE_ID 0xFFFF;
#define SOFTWARE_ID 0xFFFE;

static const uint I2C_SLAVE_ADDRESS = 0x17;
static const uint I2C_BAUDRATE = 100000;
static const uint I2C_SLAVE_SDA_PIN = 4;
static const uint I2C_SLAVE_SCL_PIN = 5;

enum i2c_register {
    HAT_ID_B0 = 0x00,
    HAT_ID_B1 = 0x01,
    HAT_ID_B2 = 0x02,
    HAT_ID_B3 = 0x03,
    HAT_ID_B4 = 0x04,
    HAT_ID_B5 = 0x05,
    HAT_ID_B6 = 0x06,
    HAT_ID_B7 = 0x07,
    HARD_ID_B0 = 0x08,
    HARD_ID_B1 = 0x09,
    SOFT_ID_B0 = 0x0A,
    SOFT_ID_B1 = 0x0B,
    HAT_POWER = 0x10,
    IR_MODE = 0x11,
    RING_MODE = 0x12,
    IR_BRIGHT = 0x20,
    RING_BRIGHT = 0x21,
    ROTATION = 0x30,
    TEMP_B0 = 0x31,
    TEMP_B1 = 0x32,
    P_STATUS = 0x40,
    H_STATUS = 0x41,
    ALIVE = 0xA0,
};

enum mode {
    SIMPLE,
    ACTIVE
};

enum led_effect {
    BLANK = 0x00,
    BOOTING = 0x01,
    LOADING = 0x02,
    OKAY = 0x03,
    DISCONNECTED = 0x04,
    ERROR = 0x05,
};

struct hardware_state
{
    enum mode mode;
    uint8_t ir_led_brightness;
    uint8_t ring_led_brightness;
    enum led_effect ring_led_effect;
};

struct register_state
{
    uint8_t address_raw;
    enum i2c_register address;
    bool write_mode;
};

struct program_state
{
    uint64_t system_clock_start;
    uint64_t system_clock_end;
    uint64_t system_clock_dt;
    uint64_t system_clock_alive_start;
    uint64_t system_clock_alive_end;
    uint64_t system_clock_alive_dt;
    bool connected;
    bool connected_once;
    pico_unique_board_id_t board_id;
    uint16_t hardware_id;
    uint16_t software_id;  
};

struct hardware_state current_hardware_state;
struct hardware_state future_hardware_state;
struct register_state register_state;
struct program_state program_state;

#define IR_LED_PIN 18
#define RING_LED_PIN 23

static void i2c_slave_handler(i2c_inst_t *i2c, i2c_slave_event_t event) {
    uint8_t temp;

    switch (event) {
        case I2C_SLAVE_RECEIVE:

            // check if writing to slave first
            if (!register_state.write_mode) {
                register_state.address_raw = i2c_read_byte(i2c);
                register_state.write_mode = true;

            } else {
                // update the i2c register enum address from raw number
                register_state.address = i2c_register(register_state.address_raw);

                // slave write actions
                switch(register_state.address) {
                    case IR_BRIGHT:
                        future_hardware_state.ir_led_brightness = i2c_read_byte(i2c);
                        break;
                    case RING_BRIGHT:
                        future_hardware_state.ring_led_brightness = i2c_read_byte(i2c);
                        break;

                    case RING_MODE:
                        future_hardware_state.ring_led_effect = led_effect(i2c_read_byte(i2c));
                        break;   

                    case ALIVE:
                        program_state.connected_once = true;
                        program_state.system_clock_alive_start = time_us_64();
                        break;

                    default:
                        i2c_read_byte(i2c);
                        break;

                }
            }

            break;

        case I2C_SLAVE_REQUEST:
            // update the i2c register enum address from raw int
            register_state.address = i2c_register(register_state.address_raw);

            // slave read actions
            switch(register_state.address) {
                case HAT_ID_B0:
                    i2c_write_byte(i2c, program_state.board_id.id[0]);
                    break;
                case HAT_ID_B1:
                    i2c_write_byte(i2c, program_state.board_id.id[1]);
                    break;
                case HAT_ID_B2:
                    i2c_write_byte(i2c, program_state.board_id.id[2]);
                    break;
                case HAT_ID_B3:
                    i2c_write_byte(i2c, program_state.board_id.id[3]);
                    break;
                case HAT_ID_B4:
                    i2c_write_byte(i2c, program_state.board_id.id[4]);
                    break;
                case HAT_ID_B5:
                    i2c_write_byte(i2c, program_state.board_id.id[5]);
                    break;
                case HAT_ID_B6:
                    i2c_write_byte(i2c, program_state.board_id.id[6]);
                    break;
                case HAT_ID_B7:
                    i2c_write_byte(i2c, program_state.board_id.id[7]);
                    break;   

                case HARD_ID_B0:
                    i2c_write_byte(i2c, program_state.hardware_id >> 8);
                    break;  
                case HARD_ID_B1:
                    i2c_write_byte(i2c, uint8_t(program_state.hardware_id));
                    break;

                case SOFT_ID_B0:
                    i2c_write_byte(i2c, program_state.software_id >> 8);
                    break;  
                case SOFT_ID_B1:
                    i2c_write_byte(i2c, uint8_t(program_state.software_id));
                    break;

                case RING_MODE:
                    i2c_write_byte(i2c, current_hardware_state.ring_led_effect);
                    break;

                case IR_BRIGHT:
                    i2c_write_byte(i2c, current_hardware_state.ir_led_brightness);
                    break;
                case RING_BRIGHT:
                    i2c_write_byte(i2c, current_hardware_state.ring_led_brightness);
                    break;

                case TEMP_B0:
                    i2c_write_byte(i2c, adc_read() >> 8);
                    break;  
                case TEMP_B1:
                    i2c_write_byte(i2c, uint8_t(adc_read()));
                    break;

                default:
                    i2c_write_byte(i2c, 0x00);
                    break;
            }

            // incremenet the raw int for sequential read
            register_state.address_raw++;

            break;

        case I2C_SLAVE_FINISH: 
            register_state.write_mode = false;
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
    // ----------------- setup system vars (from storage) -----------------
    future_hardware_state.mode = SIMPLE;
    future_hardware_state.ir_led_brightness = 0;
    future_hardware_state.ring_led_brightness = 5;
    future_hardware_state.ring_led_effect = BOOTING;

    pico_get_unique_board_id(&program_state.board_id);
    program_state.hardware_id = HARDWARE_ID;
    program_state.software_id = SOFTWARE_ID;

    // ----------------- setup all the io -----------------

    // CONSOLE
    stdio_init_all();
    printf( "\n I2C HAT - Harware Version[%4x] - Software Version[%4x] \n",  program_state.hardware_id,  program_state.software_id );

    // I2C SLAVE
    setup_slave();

    // Initialize LED strip and associated effects
    auto ledStrip = PicoLed::addLeds<PicoLed::WS2812B>(pio0, 2, RING_LED_PIN, 11, PicoLed::FORMAT_GRB);
    ledStrip.setBrightness(0);
    ledStrip.fill( PicoLed::RGB(0, 0, 0) );
    ledStrip.show();

    std::vector<PicoLed::Color> bootingPallet;
    bootingPallet.push_back( PicoLed::RGB(0, 0, 0) );
    bootingPallet.push_back( PicoLed::RGB(0, 100, 255) );
    PicoLed::Marquee effectBooting(ledStrip, bootingPallet, 4.0, 6.0);

    std::vector<PicoLed::Color> loadingPallet;
    loadingPallet.push_back( PicoLed::RGB(0, 0, 0) );
    loadingPallet.push_back( PicoLed::RGB(255, 255, 255) );
    PicoLed::Marquee effectLoading(ledStrip, loadingPallet, 5.0, -2.0, 1.0);

    bool effectReset = false;

    // IR LED
    gpio_set_function(21, GPIO_FUNC_PWM);
    uint ir_led_slice = pwm_gpio_to_slice_num(21);
    uint ir_led_channel = pwm_gpio_to_channel(21);
    pwm_set_wrap(ir_led_slice, 255);
    pwm_set_chan_level(ir_led_slice, ir_led_channel, 0);
    pwm_set_enabled(ir_led_slice, true);

    gpio_init(IR_LED_PIN);
    gpio_set_dir(IR_LED_PIN, GPIO_OUT);
    gpio_put(IR_LED_PIN, 0);

    // temperature sensor
    adc_init();
    adc_set_temp_sensor_enabled(true);
    adc_select_input(4);

    uint16_t voltage = adc_read() * 3.3 / 65535;
    float T = 27 - ( voltage - 0.706) / 0.001721;
    printf("\n temperature - [%9.6f] \n", T); // currently broken and not sure why

    // main program loop
    while(true) {
        // get the current time i
        program_state.system_clock_start = time_us_64();

        // check if still conencted to host
        program_state.system_clock_alive_end = time_us_64();
        program_state.system_clock_alive_dt = program_state.system_clock_alive_end - program_state.system_clock_alive_start;

        // check for connected and disconnected events
        if (program_state.system_clock_alive_dt < 1000000 && program_state.connected == false && program_state.connected_once == true) {
            program_state.connected = true;
            future_hardware_state.ring_led_effect = OKAY;
            printf("connected! \n");
        } else if (program_state.system_clock_alive_dt > 1000000 && program_state.connected == true) {
            program_state.connected = false;
            future_hardware_state.ring_led_effect = DISCONNECTED;
            printf("disconnected! \n");
        }

        // check if the hardware mode has changed
        if (future_hardware_state.mode != current_hardware_state.mode) {
            current_hardware_state.mode = future_hardware_state.mode;
            printf("HARDWARE MODE - [%i] \n", current_hardware_state.mode);
        }

        // state control
        switch (current_hardware_state.mode)
        {
            case SIMPLE:

                // check if the ir led has changed brightness
                if (future_hardware_state.ir_led_brightness != current_hardware_state.ir_led_brightness) {
                    current_hardware_state.ir_led_brightness = future_hardware_state.ir_led_brightness;
                    pwm_set_chan_level(ir_led_slice, ir_led_channel, current_hardware_state.ir_led_brightness);
                    printf("IR LED BRIGHTNESS - [%i] \n", current_hardware_state.ir_led_brightness);
                }

                // check if the led ring has changed brightness
                if (future_hardware_state.ring_led_brightness != current_hardware_state.ring_led_brightness) {
                    current_hardware_state.ring_led_brightness = future_hardware_state.ring_led_brightness;
                    ledStrip.setBrightness(current_hardware_state.ring_led_brightness);
                    ledStrip.show();
                    printf("RING LED BRIGHTNESS - [%i] \n", current_hardware_state.ring_led_brightness);
                }

                // check if the led ring changed effect
                if (future_hardware_state.ring_led_effect != current_hardware_state.ring_led_effect) {
                    current_hardware_state.ring_led_effect = future_hardware_state.ring_led_effect;
                    effectReset = true;
                    printf("RING LED EFFECT - [%i] \n", current_hardware_state.ring_led_effect);
                }
            
                break;

            case ACTIVE:
                /* code */
                break;
            
            default:
                break;
        }

        // led animations run
        switch (current_hardware_state.ring_led_effect) {
            case BLANK:
                if (effectReset) {
                    effectReset = false;
                    ledStrip.fill( PicoLed::RGB(0, 0, 0) );
                    ledStrip.show();
                }
                break;
            case BOOTING:
                if (effectReset) {
                    effectReset = false;
                    effectBooting.reset();
                }
                if (effectBooting.animate()) {
                    ledStrip.show();
                }
                break;
            case LOADING:
                if (effectReset) {
                    effectReset = false;
                    effectLoading.reset();
                }
                if (effectLoading.animate()) {
                    ledStrip.show();
                }
                break;
            case OKAY:
                if (effectReset) {
                    effectReset = false;
                    ledStrip.fill( PicoLed::RGB(0, 255, 0) );
                    ledStrip.show();
                }
                break;
            case DISCONNECTED:
                if (effectReset) {
                    effectReset = false;
                    ledStrip.fill( PicoLed::RGB(255, 0, 0) );
                    ledStrip.show();
                }
                break;
            case ERROR:
                if (effectReset) {
                    effectReset = false;
                    ledStrip.fill( PicoLed::RGB(255, 100, 0) );
                    ledStrip.show();
                }
                break;
        }

        // get the change in time since last loop
        program_state.system_clock_end= time_us_64();
        program_state.system_clock_dt = program_state.system_clock_end - program_state.system_clock_start;

        // sleep if needed
        if (program_state.system_clock_dt < 10000)
        {
            sleep_us(10000 - program_state.system_clock_dt);
        }

    }
}
