// @author Marek Bel
#ifndef PINS_H_
#define PINS_H_
#include "trinamic.h"

// SPI
#define AX_PUL_CS         PB1
#define AX_SET_CS         PB2
#define AX_IDL_CS         PB10
#define SPI_PORT          GPIOA
#define MOSI              PA7
#define MISO              PA6
#define SCK               PA5

// STEEP
#define AX_PUL_STEP       PB4
#define AX_SET_STEP       PA11
#define AX_IDL_STEP       PA8

// LED
#define LED1  PB8
#define LED2  PB9

// 74hc595
#define HC595_PORT        GPIOB
#define HC595_DS          PB15
#define HC595_SCK         PB13
#define HC595_STCP        PB12

#define FIL_RUNOUT        PA1    //filament_withSensor
#define FIL_RUNOUT_PORT   GPIOA     //filament_withSensor
#define FIL_RUNOUT_PIN    GPIO_PIN_1

#define BUTTON_PIN        PA2    //3 buttons on analog channel

#ifdef HAS_TMC220x     // Software serial
  #define AX_PUL_TX       PB1
  #define AX_PUL_RX       PB1

  #define AX_SEL_TX       PB2
  #define AX_SEL_RX       PB2

  #define AX_IDL_TX       PB10
  #define AX_IDL_RX       PB10
#endif  //HAS_TMC220x

//diag
#define PUL_DIAG_PIN      GPIO_PIN_7
#define SEL_DIAG_PIN      GPIO_PIN_6
#define IDL_DIAG_PIN      GPIO_PIN_5
#define DIAG_PORT         GPIOB

#endif /* PINS_H_ */
