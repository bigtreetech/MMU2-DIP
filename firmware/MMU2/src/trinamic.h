

/**
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
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
 * stepper/trinamic.h
 * Stepper driver indirection for Trinamic
 */
// #include "..\HAL\SoftwareSerial.h>"
#include <TMCStepper.h>
#include "config.h"

#define CHOPPER_DEFAULT_12V  { 3, 1, -1 }
#define CHOPPER_DEFAULT_19V  { 4, 1,  1 }
#define CHOPPER_DEFAULT_24V  { 4, 1,  2 }
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))


//1 - PULLEY  
//2 - SELECTOR
//3 - IDLER
typedef enum {
  AX_PUL = 0,
  AX_SEL = 1,
  AX_IDL = 2,
  ALL_AX = 7,
  AX_DISABLE=0,
} AXIS;

enum TMC_MODE
{
  //mode
  HOMING_MODE = 0,
  NORMAL_MODE,
  STEALTH_MODE,
};


inline void selector_step_pin_set()    {GPIOA->ODR|=1<<11;}
inline void selector_step_pin_reset()  {GPIOA->ODR&=~(1<<11);}
inline void idler_step_pin_set()       {GPIOA->ODR|=1<<8;}
inline void idler_step_pin_reset()     {GPIOA->ODR&=~(1<<8);}
inline void pulley_step_pin_set()      {GPIOB->ODR|=1<<4;}
inline void pulley_step_pin_reset()    {GPIOB->ODR&=~(1<<4);}

#if defined(TMC2208) || defined(TMC2209)
  #define HAS_TMC220x 1
#endif

#if defined(TMC5160) || defined(TMC5161) || defined(TMC2130)
  #define HAS_TMC_SPI 1
#endif

#if defined(HAS_TMC220x) || defined(HAS_TMC_SPI)
  #define TMC_DRIVE
#endif

extern uint8_t idler_diag;
extern uint8_t sel_diag;

#ifdef TMC2208
extern TMC2208Stepper pulley;
extern TMC2208Stepper selector;
extern TMC2208Stepper idler;
#endif

#ifdef TMC2209
extern TMC2209Stepper pulley;
extern TMC2209Stepper selector;
extern TMC2209Stepper idler;
#endif

#ifdef TMC2130 // Hardware SPI
extern TMC2130Stepper pulley;
extern TMC2130Stepper selector;
extern TMC2130Stepper idler;  
#endif

#ifdef TMC5160 // Hardware SPI
extern TMC5160Stepper pulley;
extern TMC5160Stepper selector;
extern TMC5160Stepper idler; 
#endif

uint16_t tmc_read_sg(TMC2209Stepper &st);
void test_tmc_connection();

#ifdef __cplusplus
extern "C" {
#endif

void tmc_begin(void);
void tmc_init(TMC_MODE mode);
void tmc_pin_init(void);
uint8_t tmc_read_gstat();

#ifdef TMC2208
void tmc_init_axis(TMC2208Stepper &st, AXIS axis, TMC_MODE mode);
#endif
#ifdef TMC2209
void tmc_init_axis(TMC2209Stepper &st, AXIS axis, TMC_MODE mode);
#endif
#ifdef TMC2130
void tmc_init_axis(TMC2130Stepper &st, AXIS axis, TMC_MODE mode);
#endif 
#ifdef TMC5160
void tmc_init_axis(TMC5160Stepper &st, AXIS axis, TMC_MODE mode);
#endif

void tmc_disable_axis(AXIS axis);

#if defined(__cplusplus)
}
#endif //defined(__cplusplus)

