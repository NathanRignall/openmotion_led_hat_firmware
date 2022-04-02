#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/unique_id.h"

#include "hardware/pwm.h"

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
    INVALID = -1,
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
    TEMP = 0x31,
    P_STATUS = 0x40,
    H_STATUS = 0x41
};

enum program {
    SIMPLE,
    ACTIVE
};

static struct
{
    enum i2c_register address;
    bool write_mode;
} context;

struct state
{
    enum program program;
    uint8_t ir_led_brightness;
    uint8_t ring_led_brightness;
    pico_unique_board_id_t board_id;
    uint16_t hardware_id;
    uint16_t software_id;
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
                        future_state.ir_led_brightness = i2c_read_byte(i2c);
                        break;
                    case RING_BRIGHT:
                        future_state.ring_led_brightness = i2c_read_byte(i2c);
                        break;

                    default:
                        break;
                }
            }

            printf(" write %i\n", context.address);

            break;

        case I2C_SLAVE_REQUEST:
            
            switch(context.address) {
                case HAT_ID_B0:
                    i2c_write_byte(i2c, current_state.board_id.id[0]);
                    break;
                case HAT_ID_B1:
                    i2c_write_byte(i2c, current_state.board_id.id[1]);
                    break;
                case HAT_ID_B2:
                    i2c_write_byte(i2c, current_state.board_id.id[2]);
                    break;
                case HAT_ID_B3:
                    i2c_write_byte(i2c, current_state.board_id.id[3]);
                    break;
                case HAT_ID_B4:
                    i2c_write_byte(i2c, current_state.board_id.id[4]);
                    break;
                case HAT_ID_B5:
                    i2c_write_byte(i2c, current_state.board_id.id[5]);
                    break;
                case HAT_ID_B6:
                    i2c_write_byte(i2c, current_state.board_id.id[6]);
                    break;
                case HAT_ID_B7:
                    i2c_write_byte(i2c, current_state.board_id.id[7]);
                    break;   

                case HARD_ID_B0:
                    i2c_write_byte(i2c, current_state.hardware_id >> 8);
                    break;  
                case HARD_ID_B1:
                    i2c_write_byte(i2c, uint8_t(current_state.hardware_id));
                    break;

                case SOFT_ID_B0:
                    i2c_write_byte(i2c, current_state.software_id >> 8);
                    break;  
                case SOFT_ID_B1:
                    i2c_write_byte(i2c, uint8_t(current_state.software_id));
                    break;

                case IR_BRIGHT:
                    i2c_write_byte(i2c, current_state.ir_led_brightness);
                    break;
                case RING_BRIGHT:
                    i2c_write_byte(i2c, current_state.ring_led_brightness);
                    break;

                default:
                    i2c_write_byte(i2c, 0x00);
                    break;
            }

            printf(" request %i\n", context.address);

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
    // ----------------- setup system vars (from storage) -----------------
    current_state.program = SIMPLE;
    current_state.ir_led_brightness = 0;
    current_state.ring_led_brightness = 0;
    pico_get_unique_board_id(&current_state.board_id);
    current_state.hardware_id = HARDWARE_ID;
    current_state.software_id = SOFTWARE_ID;

    // ----------------- setup all the io -----------------

    // CONSOLE
    stdio_init_all();
    puts("\n I2C slave example \n");

    // I2C SLAVE
    setup_slave();

    // Initialize LED strip
    printf("0. Initialize LED strip\n");
    auto ledStrip = PicoLed::addLeds<PicoLed::WS2812B>(pio1, 2, 15, 10, PicoLed::FORMAT_GRB);
    ledStrip.setBrightness(0);
    ledStrip.fill( PicoLed::RGB(255, 0, 0) );
    ledStrip.show();

    // IR LED
    gpio_set_function(IR_LED_PIN, GPIO_FUNC_PWM);
    uint ir_led_slice = pwm_gpio_to_slice_num(IR_LED_PIN);
    uint ir_led_channel = pwm_gpio_to_channel(IR_LED_PIN);
    pwm_set_wrap(ir_led_slice, 255);
    pwm_set_chan_level(ir_led_slice, ir_led_channel, 0);
    pwm_set_enabled(ir_led_slice, true);

    while(true) {

        switch (current_state.program)
        {
            case SIMPLE:

                // check if the ir led has changed brightness
                if (future_state.ir_led_brightness != current_state.ir_led_brightness) {
                    current_state.ir_led_brightness = future_state.ir_led_brightness;
                    pwm_set_chan_level(ir_led_slice, ir_led_channel, current_state.ir_led_brightness);
                }

                // check if the led ring has changed brightness
                if (future_state.ring_led_brightness != current_state.ring_led_brightness) {
                    current_state.ring_led_brightness = future_state.ring_led_brightness;
                    ledStrip.setBrightness(current_state.ring_led_brightness);
                }
            
                break;

            case ACTIVE:
                /* code */
                break;
            
            default:
                break;
        }
    }
}
