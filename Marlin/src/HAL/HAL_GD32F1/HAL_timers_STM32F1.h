/**
 * Marlin 3D Printer Firmware
 *
 * Copyright (C) 2019 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 * Copyright (c) 2016 Bob Cousins bobcousins42@googlemail.com
 * Copyright (c) 2017 Victor Perez
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
#pragma once

/**
 * HAL for stm32duino.com based on Libmaple and compatible (STM32F1)
 */

// --------------------------------------------------------------------------
// Includes
// --------------------------------------------------------------------------

#include <stdint.h>
#include <libmaple/timer.h>

// --------------------------------------------------------------------------
// Defines
// --------------------------------------------------------------------------

/**
 * TODO: Check and confirm what timer we will use for each Temps and stepper driving.
 * We should probable drive temps with PWM.
 */
#define FORCE_INLINE __attribute__((always_inline)) inline

typedef uint16_t hal_timer_t;
#define HAL_TIMER_TYPE_MAX 0xFFFF

#define HAL_TIMER_RATE         uint32_t(F_CPU)  // frequency of timers peripherals

#define STEP_TIMER_CHAN 1 // Channel of the timer to use for compare and interrupts
#define TEMP_TIMER_CHAN 1 // Channel of the timer to use for compare and interrupts

#if defined(MCU_STM32F103CB) || defined(MCU_STM32F103C8) || defined(MCU_GD32F105VE)
  #define STEP_TIMER_NUM 4 // For C8/CB boards, use timer 4
#else
  #define STEP_TIMER_NUM 5 // for other boards, five is fine.
#endif
#define TEMP_TIMER_NUM 2  // index of timer to use for temperature
#define PULSE_TIMER_NUM STEP_TIMER_NUM

#define TEMP_TIMER_PRESCALE     1000 // prescaler for setting Temp timer, 72Khz
#define TEMP_TIMER_FREQUENCY    1000 // temperature interrupt frequency

#define STEPPER_TIMER_PRESCALE 24             // prescaler for setting stepper timer, 4Mhz
#define STEPPER_TIMER_RATE                  (HAL_TIMER_RATE / STEPPER_TIMER_PRESCALE)   // frequency of stepper timer
#define STEPPER_TIMER_TICKS_PER_US          ((STEPPER_TIMER_RATE) / 1000000) // stepper timer ticks per µs
#define STEPPER_TIMER_TICKS_PER_MS          ((STEPPER_TIMER_RATE) / 1000) // stepper timer ticks per ms

#define PULSE_TIMER_RATE       STEPPER_TIMER_RATE   // frequency of pulse timer
#define PULSE_TIMER_PRESCALE   STEPPER_TIMER_PRESCALE
#define PULSE_TIMER_TICKS_PER_US STEPPER_TIMER_TICKS_PER_US

timer_dev* get_timer_dev(int number);
#define TIMER_DEV(num) get_timer_dev(num)
#define STEP_TIMER_DEV TIMER_DEV(STEP_TIMER_NUM)
#define TEMP_TIMER_DEV TIMER_DEV(TEMP_TIMER_NUM)

#define ENABLE_STEPPER_DRIVER_INTERRUPT() timer_enable_irq(STEP_TIMER_DEV, STEP_TIMER_CHAN)
#define DISABLE_STEPPER_DRIVER_INTERRUPT() timer_disable_irq(STEP_TIMER_DEV, STEP_TIMER_CHAN)
#define STEPPER_ISR_ENABLED() HAL_timer_interrupt_enabled(STEP_TIMER_NUM)

#define ENABLE_TEMPERATURE_INTERRUPT() timer_enable_irq(TEMP_TIMER_DEV, TEMP_TIMER_CHAN)
#define DISABLE_TEMPERATURE_INTERRUPT() timer_disable_irq(TEMP_TIMER_DEV, TEMP_TIMER_CHAN)

#define HAL_timer_get_count(timer_num) timer_get_count(TIMER_DEV(timer_num))

// TODO change this

#define HAL_TEMP_TIMER_ISR() extern "C" void tempTC_Handler(void)
#define HAL_STEP_TIMER_ISR() extern "C" void stepTC_Handler(void)

extern "C" void tempTC_Handler(void);
extern "C" void stepTC_Handler(void);

// --------------------------------------------------------------------------
// Types
// --------------------------------------------------------------------------


// --------------------------------------------------------------------------
// Public Variables
// --------------------------------------------------------------------------
/*
static HardwareTimer StepperTimer(STEP_TIMER_NUM);
static HardwareTimer TempTimer(TEMP_TIMER_NUM);
*/
// --------------------------------------------------------------------------
// Public functions
// --------------------------------------------------------------------------

void HAL_timer_start(const uint8_t timer_num, const uint32_t frequency);
void HAL_timer_enable_interrupt(const uint8_t timer_num);
void HAL_timer_disable_interrupt(const uint8_t timer_num);
bool HAL_timer_interrupt_enabled(const uint8_t timer_num);

/**
 * NOTE: By default libmaple sets ARPE = 1, which means the Auto reload register is preloaded (will only update with an update event)
 * Thus we have to pause the timer, update the value, refresh, resume the timer.
 * That seems like a big waste of time and may be better to change the timer config to ARPE = 0, so ARR can be updated any time.
 * We are using a Channel in each timer in Capture/Compare mode. We could also instead use the Time Update Event Interrupt, but need to disable ARPE
 * so we can change the ARR value on the fly (without calling refresh), and not get an interrupt right there because we caused an UEV.
 * This mode pretty much makes 2 timers unusable for PWM since they have their counts updated all the time on ISRs.
 * The way Marlin manages timer interrupts doesn't make for an efficient usage in STM32F1
 * Todo: Look at that possibility later.
 */

FORCE_INLINE static void HAL_timer_set_compare(const uint8_t timer_num, const hal_timer_t compare) {
  //compare = MIN(compare, HAL_TIMER_TYPE_MAX);
  switch (timer_num) {
  case STEP_TIMER_NUM:
//    timer_set_compare(STEP_TIMER_DEV, STEP_TIMER_CHAN, compare);
    timer_set_reload(STEP_TIMER_DEV, compare);
    return;
  case TEMP_TIMER_NUM:
    timer_set_compare(TEMP_TIMER_DEV, TEMP_TIMER_CHAN, compare);
    return;
  default:
    return;
  }
}

FORCE_INLINE static hal_timer_t HAL_timer_get_compare(const uint8_t timer_num) {
  switch (timer_num) {
  case STEP_TIMER_NUM:
    return timer_get_compare(STEP_TIMER_DEV, STEP_TIMER_CHAN);
  case TEMP_TIMER_NUM:
    return timer_get_compare(TEMP_TIMER_DEV, TEMP_TIMER_CHAN);
  default:
    return 0;
  }
}

FORCE_INLINE static void HAL_timer_isr_prologue(const uint8_t timer_num) {
  switch (timer_num) {
  case STEP_TIMER_NUM:
//    timer_set_count(STEP_TIMER_DEV, 0);
    timer_generate_update(STEP_TIMER_DEV);
    return;
  case TEMP_TIMER_NUM:
    timer_set_count(TEMP_TIMER_DEV, 0);
    timer_generate_update(TEMP_TIMER_DEV);
    return;
  default:
    return;
  }
}

#define HAL_timer_isr_epilogue(TIMER_NUM)

FORCE_INLINE static void timer_no_ARR_preload_ARPE(timer_dev *dev) {
  bb_peri_set_bit(&(dev->regs).gen->CR1, TIMER_CR1_ARPE_BIT, 0);
}

#define TIMER_OC_NO_PRELOAD 0