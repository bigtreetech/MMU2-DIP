//! @file
#include <Arduino.h>
#include <stm32_eeprom.h>
#include "main.h"
#include <stdio.h>
#include <string.h>
#include "shr16.h"
#include "adc.h"
#include "uart.h"
#include "pins.h"
#include "mmctl.h"
#include "stepper.h"
#include "Buttons.h"
#include "permanent_storage.h"
#include "version.h"
#include "config.h"
#include "motion.h"
#include "IWatchdog.h"
#include "trinamic.h"

#if (UART_COM == 1)
FILE* uart_com = uart1io;
#endif //(UART_COM == 1)

uint8_t tmc_mode = STEALTH_MODE;

namespace
{
  //! @brief State
  enum class S
  {
  Idle,
  Setup,
  Printing,
  SignalFilament,
  Wait,
  WaitOk,
  };
}


//! @brief Main MMU state
//!
//! @startuml
//!
//! title MMU Main State Diagram
//!
//! state Any {
//!   state Idle : Manual extruder selector
//!   state Setup
//!   state Printing
//!   state SignalFilament
//!
//!   [*] --> Idle : !MiddleButton
//!   [*] --> Setup : MiddleButton
//!   Any --> Printing : T<nr> || Eject
//!   Any --> Idle : Unload || RecoverEject
//!   Any --> SignalFilament : Load && filamentLoaded
//!   Any --> Wait : W0
//!   Setup --> Idle
//!   Wait --> Idle : RightButton
//!   WaitOk --> Idle : RightButton
//!   Wait --> WaitOk : MiddleButton && mmctl_IsOk
//!   WaitOk --> Wait : MiddleButton && !mmctl_IsOk
//! }
//! @enduml
static S state;

static void process_commands();

static void led_blink(int _no)
{
  shr16_set_led(1 << 2 * _no);
  delay(40);
  shr16_set_led(0x000);
  delay(20);
  shr16_set_led(1 << 2 * _no);
  delay(40);
  shr16_set_led(0x000);
  delay(10);
}

//! @brief signal filament presence
//!
//! non-blocking
//! LED indication of states
//!
//! RG | RG | RG | RG | RG | meaning
//! -- | -- | -- | -- | -- | ------------------------
//! b0 | b0 | b0 | b0 | b0 | Error, filament detected, still present
//!
//! @n R - Red LED
//! @n G - Green LED
//! @n 1 - active
//! @n 0 - inactive
//! @n b - blinking
static void signal_filament_present()
{
  shr16_set_led(0x2aa);
  delay(300);
  shr16_set_led(0x000);
  delay(300);
}

void signal_load_failure()
{
  shr16_set_led(0x000);
  delay(800);
  shr16_set_led(2 << 2 * (4 - active_extruder));
  delay(800);
}

void signal_ok_after_load_failure()
{
  shr16_set_led(0x000);
  delay(800);
  shr16_set_led(1 << 2 * (4 - active_extruder));
  delay(100);
  shr16_set_led(2 << 2 * (4 - active_extruder));
  delay(100);
  delay(800);
}

//! @brief Signal filament presence
//!
//! @retval true still present
//! @retval false not present any more
bool filament_presence_signaler()
{
  if (digitalRead(FIL_RUNOUT) == FILAMENT_SENSOR_INVERTING)
  {  
    signal_filament_present();
    return true;
  }
  else
  {
    isFilamentLoaded = false;
    return false;
  }
}

//! @brief Check, if filament is not present in FINDA
//!
//! blocks, until filament is not removed and button pushed
//!
//! button | action
//! ------ | ------
//! right  | continue after error
//!
//! LED indication of states
//!
//! RG | RG | RG | RG | RG | meaning
//! -- | -- | -- | -- | -- | ------------------------
//! b0 | b0 | b0 | b0 | b0 | Error, filament detected, still present
//! 0b | 0b | 0b | 0b | 0b | Error, filament detected, no longer present, continue by right button click
//!
//! @n R - Red LED
//! @n G - Green LED
//! @n 1 - active
//! @n 0 - inactive
//! @n b - blinking
void check_filament_not_present()
{
  while (digitalRead(FIL_RUNOUT) == FILAMENT_SENSOR_INVERTING)
  {
    while (Btn::right != buttonPressed())
    {
      if (digitalRead(FIL_RUNOUT) == FILAMENT_SENSOR_INVERTING)
      {
        signal_filament_present();
      }
      else
      {
        shr16_set_led(0x155);
        delay(300);
        shr16_set_led(0x000);
        delay(300);
      }
    }
  }
}

static void signal_drive_error()
{
  shr16_set_led(0x3ff);
  delay(300);
  shr16_set_led(0x000);
  delay(300);
}

void drive_error()
{
  for (uint8_t i = 0; i < 3; ++i)
  {
    signal_drive_error();
  }
  // DriveError::increment();
}

//! @brief Unrecoverable hardware fault
//!
//! Stay in infinite loop and blink.
//!
//! LED indication of states
//!
//! RG | RG | RG | RG | RG
//! -- | -- | -- | -- | --
//! bb | bb | bb | bb | bb
//!
//! @n R - Red LED
//! @n G - Green LED
//! @n 1 - active
//! @n 0 - inactive
//! @n b - blinking
void unrecoverable_error()
{
  while (1)
  {
    signal_drive_error();
  }
}

//! @brief Initialization after reset
//!
//! button | action
//! ------ | ------
//! middle | enter setup
//! right  | continue after error
//!
//! LED indication of states
//!
//! RG | RG | RG | RG | RG | meaning
//! -- | -- | -- | -- | -- | ------------------------
//! 00 | 00 | 00 | 00 | 0b | Shift register initialized
//! 00 | 00 | 00 | 0b | 00 | uart initialized
//! 00 | 00 | 0b | 00 | 00 | spi initialized
//! 00 | 0b | 00 | 00 | 00 | tmc initialized
//! 0b | 00 | 00 | 00 | 00 | A/D converter initialized
//!
//! @n R - Red LED
//! @n G - Green LED
//! @n 1 - active
//! @n 0 - inactive
//! @n b - blinking

void setup(void)
{
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);

  permanentStorageInit();
  shr16_init(); // shift register
  led_blink(0);
  uart1_init(); //uart1
  led_blink(1);

  tmc_begin();
  led_blink(2);
  led_blink(3);

  adc_init(); // ADC get button logic signal
  led_blink(4);
  shr16_set_ena(~ALL_AX);
  shr16_set_led(0x000);

  // check if to goto the settings menu
  if (buttonPressed() == Btn::middle)
  {
    state = S::Setup;
  }

  tmc_init(HOMING_MODE);
  
  uint8_t filament;
  if (FilamentLoaded::get(filament))
  {
    motion_set_idler(filament);
  }

  if (digitalRead(FIL_RUNOUT) == FILAMENT_SENSOR_INVERTING) isFilamentLoaded = true;

  #ifdef TMC_DEBUG
  test_tmc_connection();
  #endif
  printf_P("start\r\n"); //startup message
}

//! @brief Select filament menu
//!
//! Select filament by pushing left and right button, park position can be also selected.
//!
//! button | action
//! ------ | ------
//! left   | select previous filament
//! right  | select next filament
//!
//! LED indication of states
//!
//! RG | RG | RG | RG | RG | meaning
//! -- | -- | -- | -- | -- | ------------------------
//! 01 | 00 | 00 | 00 | 00 | filament 1
//! 00 | 01 | 00 | 00 | 00 | filament 2
//! 00 | 00 | 01 | 00 | 00 | filament 3
//! 00 | 00 | 00 | 01 | 00 | filament 4
//! 00 | 00 | 00 | 00 | 01 | filament 5
//! 00 | 00 | 00 | 00 | bb | park position
//!
//! @n R - Red LED
//! @n G - Green LED
//! @n 1 - active
//! @n 0 - inactive
//! @n b - blinking
void manual_extruder_selector()
{
  shr16_set_led(1 << 2 * (4 - active_extruder));

  if ((Btn::left|Btn::right) & buttonPressed())
  {
    delay(100);
    if (digitalRead(FIL_RUNOUT) == FILAMENT_SENSOR_INVERTING)
    {
      signal_filament_present();
      return ;
    }
    switch (buttonPressed())
    {
      case Btn::right:
        if (active_extruder < 5)
        {
          select_extruder(active_extruder + 1);
        }
        break;
      case Btn::left:
        if (active_extruder > 0) select_extruder(active_extruder - 1);
        break;

      default:
        break;
    }
    delay(100);
  }

  if (active_extruder == 5)
  {
    shr16_set_led(2 << 2 * 0);
    delay(50);
    shr16_set_led(1 << 2 * 0);
    delay(50);
  }

}

//! @brief main loop
//!
//! It is possible to manually select filament and feed it when S::Idle.
//!
//! button | action
//! ------ | ------
//! middle | feed filament
//!
//! @copydoc manual_extruder_selector()
void loop(void)
{
  process_commands();
  switch (state)
  {
    case S::Setup:
      if (!setupMenu()) state = S::Idle;
      break;
    case S::Printing:
      break;
    case S::SignalFilament:
      if (!filament_presence_signaler()) state = S::Idle;
      break;
    case S::Idle:
      manual_extruder_selector();
      if (Btn::middle == buttonPressed() && active_extruder < 5)
      {
        delay(200);
        shr16_set_led(2 << 2 * (4 - active_extruder));
        if (Btn::middle == buttonPressed())
        {
          motion_set_idler_selector(active_extruder);
          feed_filament();
        }
      }
      break;
    case S::Wait:
      signal_load_failure();
      switch (buttonClicked())
      {
      case Btn::middle:
        if (mmctl_IsOk()) state = S::WaitOk;
        break;
      case Btn::right:
        state = S::Idle;
        printf_P(PSTR("ok\n"));
        break;
      default:
        break;
      }
      break;
    case S::WaitOk:
      signal_ok_after_load_failure();
      switch (buttonClicked())
      {
      case Btn::middle:
        if (!mmctl_IsOk()) state = S::Wait;
        break;
      case Btn::right:
        state = S::Idle;
        printf_P(PSTR("ok\n"));
        break;
      default:
        break;
      }
      break;
  }
}

//! @brief receive and process commands from serial line
//! @param[in,out] inout struct connected to serial line to be used
//!
//! All commands have syntax in form of one letter integer number.
void process_commands()
{
  static char line[32] = {0};
  static int count = 0;
  int c = -1;
  while (Serial.available())
  {
    line[count] = Serial.read();
    if (line[count++] == '\n')
    {
      line[count] = '\0';
      c = 0;
    }
    if (count >= 32) count = 0;
  }
  int value = 0;
  int value0 = 0;

  if ((count > 0) && (c == 0))
  {
    //line received
    // printf_P("adc:%d,%d\r\n",adc_val[0], adc_val[1]);
    // printf_P("line received: '%s' %d\n", line, count);
    count = 0;
      //! T<nr.> change to filament <nr.>
    if (sscanf(line, PSTR("T%d"), &value) > 0)
    {
      if ((value >= 0) && (value < EXTRUDERS))
      {
        state = S::Printing;
        switch_extruder_withSensor(value);
        printf_P("ok\n");
      }
    }
        //! L<nr.> Load filament <nr.>
    else if (sscanf(line, PSTR("L%d"), &value) > 0)
    {
      if ((value >= 0) && (value < EXTRUDERS))
      {
      if (isFilamentLoaded) state = S::SignalFilament;
      else
      {
        select_extruder(value);
        feed_filament();
      }
        printf_P("ok\n");
      }
    }
    else if (sscanf(line, PSTR("M%d"), &value) > 0)
    {
      //! M0 set to normal mode
      //!@n M1 set to stealth mode
      switch (value) {
      case 0: tmc_mode = NORMAL_MODE; break;
      case 1: tmc_mode = STEALTH_MODE; break;
      default: return;
      }

      //init all axes
      tmc_init((TMC_MODE)tmc_mode);
      printf_P("ok\n");
    }
    //! U<nr.> Unload filament. <nr.> is ignored but mandatory.
    else if (sscanf(line, PSTR("U%d"), &value) > 0)
    {

      unload_filament_withSensor();
      printf_P("ok\n");

      state = S::Idle;
    }
    else if (sscanf(line, PSTR("X%d"), &value) > 0)
    {
      if (value == 0)   //! X0 MMU reset
      IWatchdog.begin(1000);
    }
    else if (sscanf(line, PSTR("P%d"), &value) > 0)
    {
      if (value == 0) //! P0 Read finda
      {
      if(digitalRead(FIL_RUNOUT) == FILAMENT_SENSOR_INVERTING)
      {
        printf_P("1ok\n");
      }
      else
      {
        printf_P("0ok\n");
      }    
      }
      
    }
    else if (sscanf(line, PSTR("S%d"), &value) > 0)
    {
      if (value == 0) //! S0 return ok
      printf_P("ok\n");
      else if (value == 1) //! S1 Read version
      printf_P("%dok\n", fw_version);
      else if (value == 2) //! S2 Read build nr.
      printf_P("%dok\n", fw_buildnr);
      else if (value == 3) //! S3 Read drive errors
        printf_P("%dok\n", DriveError::get());
    }
    //! F<nr.> \<type\> filament type. <nr.> filament number, \<type\> 0, 1 or 2. Does nothing.
    else if (sscanf(line, PSTR("F%d %d"), &value, &value0) > 0)
    {
      if (((value >= 0) && (value < EXTRUDERS)) &&
      ((value0 >= 0) && (value0 <= 2)))
      {
      filament_type[value] = value0;
      printf_P("ok\n");
      }
    }
    else if (sscanf(line, PSTR("C%d"), &value) > 0)
    {
      if (value == 0) //! C0 continue loading current filament (used after T-code).
      {
        load_filament_inPrinter();
        printf_P("ok\n");
      }
    }
    else if (sscanf(line, PSTR("E%d"), &value) > 0)
    {
      if ((value >= 0) && (value < EXTRUDERS)) //! E<nr.> eject filament
      {
        eject_filament(value);
        printf_P("ok\n");
        state = S::Printing;
      }
    }
    else if (sscanf(line, PSTR("R%d"), &value) > 0)
    {
      if (value == 0) //! R0 recover after eject filament
      {
        recover_after_eject();
        printf_P("ok\n");
        state = S::Idle;
      }
    }
    else if (sscanf(line, PSTR("W%d"), &value) > 0)
    {
      if (value == 0) //! W0 Wait for user click
      {
        state = S::Wait;
      }
    }
    else if (sscanf(line, PSTR("K%d"), &value) > 0)
    {
      if ((value >= 0) && (value < EXTRUDERS)) //! K<nr.> cut filament
      {
        mmctl_cut_filament(value);
        printf_P("ok\n");
      }
    }
  }
  else
  { //nothing received
  }
}

