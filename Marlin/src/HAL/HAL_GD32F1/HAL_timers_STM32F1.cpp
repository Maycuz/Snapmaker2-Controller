/**
 * Marlin 3D Printer Firmware
 *
 * Copyright (C) 2019 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 * Copyright (c) 2016 Bob Cousins bobcousins42@googlemail.com
 * Copyright (c) 2015-2016 Nico Tonnhofer wurstnase.reprap@gmail.com
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

/**
 * HAL for stm32duino.com based on Libmaple and compatible (STM32F1)
 */

#ifdef __GD32F1__

// --------------------------------------------------------------------------
// Includes
// --------------------------------------------------------------------------

#include "HAL.h"

#include "HAL_timers_STM32F1.h"

// --------------------------------------------------------------------------
// Externals
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Local defines
// --------------------------------------------------------------------------

#define NUM_HARDWARE_TIMERS 7

//#define PRESCALER 1
// --------------------------------------------------------------------------
// Types
// --------------------------------------------------------------------------


// --------------------------------------------------------------------------
// Public Variables
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Private Variables
// --------------------------------------------------------------------------
/* VGPV
const tTimerConfig TimerConfig [NUM_HARDWARE_TIMERS] = {
  { TC0, 0, TC0_IRQn, 0},  // 0 - [servo timer5]
  { TC0, 1, TC1_IRQn, 0},  // 1
  { TC0, 2, TC2_IRQn, 0},  // 2
  { TC1, 0, TC3_IRQn, 2},  // 3 - stepper
  { TC1, 1, TC4_IRQn, 15}, // 4 - temperature
  { TC1, 2, TC5_IRQn, 0},  // 5 - [servo timer3]
  { TC2, 0, TC6_IRQn, 0},  // 6
  { TC2, 1, TC7_IRQn, 0},  // 7
  { TC2, 2, TC8_IRQn, 0},  // 8
};
*/
// --------------------------------------------------------------------------
// Function prototypes
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Private functions
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Public functions
// --------------------------------------------------------------------------

/**
 * Timer_clock1: Prescaler   2 ->  36    MHz
 * Timer_clock2: Prescaler   8 ->   9    MHz
 * Timer_clock3: Prescaler  32 ->   2.25 MHz
 * Timer_clock4: Prescaler 128 -> 562.5  kHz
 */

/**
 * TODO: Calculate Timer prescale value, so we get the 32bit to adjust
 */

void HAL_timer_start(const uint8_t timer_num, const uint32_t frequency) {
  nvic_irq_num irq_num;
  switch (timer_num) {
    case 1: irq_num = NVIC_TIMER1_CC; break;
    case 2: irq_num = NVIC_TIMER2; break;
    case 3: irq_num = NVIC_TIMER3; break;
    case 4: irq_num = NVIC_TIMER4; break;
    case 5: irq_num = NVIC_TIMER5; break;
    case 6: irq_num = NVIC_TIMER6; break;
    case 7: irq_num = NVIC_TIMER7; break;
    default:
      /**
       *  We should not get here, add Sanitycheck for timer number. Should be a general timer
       *  since basic timers do not have CC channels.
       *  Advanced timers should be skipped if possible too, and are not listed above.
       */
      break;
  }

  /**
   * Give the Stepper ISR a higher priority (lower number)
   * so it automatically preempts the Temperature ISR.
   */

  switch (timer_num) {
    case STEP_TIMER_NUM:
      timer_pause(STEP_TIMER_DEV);
      timer_set_mode(STEP_TIMER_DEV, STEP_TIMER_CHAN, TIMER_OUTPUT_COMPARE); // counter
      timer_set_count(STEP_TIMER_DEV, 0);
      timer_set_prescaler(STEP_TIMER_DEV, (uint16_t)(STEPPER_TIMER_PRESCALE - 1));
      timer_set_reload(STEP_TIMER_DEV, MIN((uint32_t)HAL_TIMER_TYPE_MAX, (STEPPER_TIMER_RATE / frequency)));
      timer_oc_set_mode(STEP_TIMER_DEV, STEP_TIMER_CHAN, TIMER_OC_MODE_FROZEN, TIMER_OC_NO_PRELOAD);
      timer_set_compare(STEP_TIMER_DEV, STEP_TIMER_CHAN, (uint32_t)HAL_TIMER_TYPE_MAX);
      timer_no_ARR_preload_ARPE(STEP_TIMER_DEV); // Need to be sure no preload on ARR register
      timer_attach_interrupt(STEP_TIMER_DEV, STEP_TIMER_CHAN, stepTC_Handler);
      nvic_irq_set_priority(irq_num, 0);
      timer_generate_update(STEP_TIMER_DEV);
      timer_resume(STEP_TIMER_DEV);
      break;
    case TEMP_TIMER_NUM:
      timer_pause(TEMP_TIMER_DEV);
      timer_set_count(TEMP_TIMER_DEV, 0);
      timer_set_prescaler(TEMP_TIMER_DEV, (uint16_t)(TEMP_TIMER_PRESCALE - 1));
      timer_set_reload(TEMP_TIMER_DEV, 0xFFFF);
      timer_set_compare(TEMP_TIMER_DEV, TEMP_TIMER_CHAN, MIN((uint32_t)HAL_TIMER_TYPE_MAX, ((F_CPU / TEMP_TIMER_PRESCALE) / frequency)));
      timer_attach_interrupt(TEMP_TIMER_DEV, TEMP_TIMER_CHAN, tempTC_Handler);
      nvic_irq_set_priority(irq_num, 4);
      timer_generate_update(TEMP_TIMER_DEV);
      timer_resume(TEMP_TIMER_DEV);
      break;
  }
}

void HAL_timer_enable_interrupt(const uint8_t timer_num) {
  switch (timer_num) {
    case STEP_TIMER_NUM: ENABLE_STEPPER_DRIVER_INTERRUPT(); break;
    case TEMP_TIMER_NUM: ENABLE_TEMPERATURE_INTERRUPT(); break;
    default: break;
  }
}

void HAL_timer_disable_interrupt(const uint8_t timer_num) {
  switch (timer_num) {
    case STEP_TIMER_NUM: DISABLE_STEPPER_DRIVER_INTERRUPT(); break;
    case TEMP_TIMER_NUM: DISABLE_TEMPERATURE_INTERRUPT(); break;
    default: break;
  }
}

static inline bool timer_irq_enabled(const timer_dev * const dev, const uint8_t interrupt) {
  return bool(*bb_perip(&(dev->regs).gen->DIER, interrupt));
}

bool HAL_timer_interrupt_enabled(const uint8_t timer_num) {
  switch (timer_num) {
    case STEP_TIMER_NUM: return timer_irq_enabled(STEP_TIMER_DEV, STEP_TIMER_CHAN);
    case TEMP_TIMER_NUM: return timer_irq_enabled(TEMP_TIMER_DEV, TEMP_TIMER_CHAN);
  }
  return false;
}

static __attribute__((section(".data"))) timer_dev *timer_devs[] = {
    NULL,

    #if STM32_HAVE_TIMER(1)
      &timer1,
    #endif
    #if STM32_HAVE_TIMER(2)
      &timer2,
    #endif
    #if STM32_HAVE_TIMER(3)
      &timer3,
    #endif
    #if STM32_HAVE_TIMER(4)
      &timer4,
    #endif
    #if STM32_HAVE_TIMER(5)
      &timer5,
    #endif
    #if STM32_HAVE_TIMER(6)
      &timer6,
    #endif
    #if STM32_HAVE_TIMER(7)
      &timer7,
    #endif
    #if STM32_HAVE_TIMER(8)
      &timer8,
    #endif
    #if STM32_HAVE_TIMER(9)
      &timer9,
    #endif
    #if STM32_HAVE_TIMER(10)
      &timer10,
    #endif
    #if STM32_HAVE_TIMER(11)
      &timer11,
    #endif
    #if STM32_HAVE_TIMER(12)
      &timer12,
    #endif
    #if STM32_HAVE_TIMER(13)
      &timer13,
    #endif
    #if STM32_HAVE_TIMER(14)
      &timer14,
    #endif
};

timer_dev* get_timer_dev(int number) {
  if (number < (int)sizeof(timer_devs))
    return timer_devs[number];

  return NULL;
}

#endif // __STM32F1__
