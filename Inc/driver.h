/*

  driver.h - driver code for STM32F3xx ARM processors

  Part of grblHAL

  Copyright (c) 2021 Terje Io

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.

*/

//
// NOTE: do NOT change configuration here - edit my_machine.h instead!
//

#ifndef __DRIVER_H__
#define __DRIVER_H__

#include <stdbool.h>
#include <stdint.h>

#ifndef OVERRIDE_MY_MACHINE
#include "my_machine.h"
#endif

#include "main.h"
#include "grbl/hal.h"
#include "grbl/grbl.h"
#include "grbl/nuts_bolts.h"
#include "grbl/driver_opts.h"

#define DIGITAL_OUT(port, bit, on) { if(on) port->BSRR = bit; else port->BRR = bit; }

#define timer(p) timerN(p)
#define timerN(p) TIM ## p
#define timerINT(p) timeri(p)
#define timeri(p) TIM ## p ## _IRQn
#define timerHANDLER(p) timerh(p)
#define timerh(p) TIM ## p ## _IRQHandler

// Define GPIO output mode options

#define GPIO_SHIFT0   0
#define GPIO_SHIFT1   1
#define GPIO_SHIFT2   2
#define GPIO_SHIFT3   3
#define GPIO_SHIFT4   4
#define GPIO_SHIFT5   5
#define GPIO_SHIFT6   6
#define GPIO_SHIFT7   7
#define GPIO_SHIFT8   8
#define GPIO_SHIFT9   9
#define GPIO_SHIFT10 10
#define GPIO_SHIFT11 11
#define GPIO_SHIFT12 12
#define GPIO_SHIFT13 13
#define GPIO_MAP     14

// Define timer allocations.

#define STEPPER_TIMER_N             2
#define STEPPER_TIMER               timer(STEPPER_TIMER_N)
#define STEPPER_TIMER_IRQn          timerINT(STEPPER_TIMER_N)
#define STEPPER_TIMER_IRQHandler    timerHANDLER(STEPPER_TIMER_N)

#define PULSE_TIMER_N               3
#define PULSE_TIMER                 timer(PULSE_TIMER_N)
#define PULSE_TIMER_IRQn            timerINT(PULSE_TIMER_N)
#define PULSE_TIMER_IRQHandler      timerHANDLER(PULSE_TIMER_N)

#define SPINDLE_PWM_TIMER_N         1
#define SPINDLE_PWM_TIMER           timer(SPINDLE_PWM_TIMER_N)

#define DEBOUNCE_TIMER_N            4
#define DEBOUNCE_TIMER              timer(DEBOUNCE_TIMER_N)
#define DEBOUNCE_TIMER_IRQn         timerINT(DEBOUNCE_TIMER_N)
#define DEBOUNCE_TIMER_IRQHandler   timerHANDLER(DEBOUNCE_TIMER_N)

#ifdef BOARD_CNC_BOOSTERPACK
  #include "cnc_boosterpack_map.h"
#elif defined(BOARD_CNC3040)
  #include "cnc3040_map.h"
#elif defined(BOARD_MY_MACHINE)
#include "my_machine_map.h"
#else // default board
  #include "generic_map.h"
#endif

// Adjust STEP_PULSE_LATENCY to get accurate step pulse length when required, e.g if using high step rates.
// The default value is calibrated for 10 microseconds length.
// NOTE: step output mode, number of axes and compiler optimization settings may all affect this value.
#ifndef STEP_PULSE_LATENCY
#define STEP_PULSE_LATENCY 1.1f // microseconds
#endif

// End configuration

#if EEPROM_ENABLE == 0
#define FLASH_ENABLE 1
#else
#define FLASH_ENABLE 0
#endif

#if EEPROM_ENABLE|| KEYPAD_ENABLE
//    #define I2C_PORT 1 // GPIOB, SCL_PIN = 8, SDA_PIN = 9
    #define I2C_PORT 2 // GPIOA, SCL_PIN = 9, SDA_PIN = 10
#endif

// End configuration

#if KEYPAD_ENABLE == 1 && !defined(I2C_STROBE_PORT)
#error Keypad plugin not supported!
#elif I2C_STROBE_ENABLE && !defined(I2C_STROBE_PORT)
#error I2C strobe not supported!
#endif

#if SDCARD_ENABLE && !defined(SD_CS_PORT)
#error SD card plugin not supported!
#endif

typedef struct {
    pin_function_t id;
    GPIO_TypeDef *port;
    uint8_t pin;
    uint32_t bit;
    pin_group_t group;
    volatile bool active;
    volatile bool debounce;
    pin_irq_mode_t irq_mode;
    pin_mode_t cap;
    ioport_interrupt_callback_ptr interrupt_callback;
    const char *description;
} input_signal_t;

typedef struct {
    pin_function_t id;
    GPIO_TypeDef *port;
    uint8_t pin;
    pin_group_t group;
    pin_mode_t mode;
    const char *description;
} output_signal_t;

typedef struct {
    uint8_t n_pins;
    union {
        input_signal_t *inputs;
        output_signal_t *outputs;
    } pins;
} pin_group_pins_t;

#ifdef HAS_BOARD_INIT
void board_init (void);
#endif

bool driver_init (void);
void gpio_irq_enable (const input_signal_t *input, pin_irq_mode_t irq_mode);

#endif // __DRIVER_H__
