/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
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

/**
 * stepper/trinamic.cpp
 * Stepper driver indirection for Trinamic
 */

#include "trinamic.h"
#include "interrupt.h"
#include "pins.h"
#include "shr16.h"


#define TMC_BAUD_RATE  19200
#define current_max    28
#define current_min    0


typedef struct {
  uint8_t toff;
  uint8_t hstrt;
  int8_t hend;
} chopper_timing_t;

static constexpr chopper_timing_t chopper_timing = CHOPPER_TIMING;

#ifdef TMC2208
  #ifndef R_SENSE
    #define R_SENSE 0.11
  #endif
  TMC2208Stepper pulley(AX_PUL_RX, AX_PUL_TX, R_SENSE);
  TMC2208Stepper selector(AX_SEL_RX, AX_SEL_TX, R_SENSE);
  TMC2208Stepper idler(AX_IDL_RX, AX_IDL_TX, R_SENSE);
  // TMC2208Stepper pulley(AX_PUL_RX, AX_PUL_TX, R_SENSE, DRIVER_ADDRESS);
  // TMC2208Stepper selector(AX_SEL_RX, AX_SEL_TX, R_SENSE, DRIVER_ADDRESS);
  // TMC2208Stepper idler(AX_IDL_RX, AX_IDL_TX, R_SENSE, DRIVER_ADDRESS);
  static uint8_t sgt_min = 0, sgt_max = 255;
#endif

#ifdef TMC2209
  #define DRIVER_ADDRESS 0b00 // TMC2209 Driver address according to MS1 and MS2
  #ifndef R_SENSE
    #define R_SENSE 0.11
  #endif
  TMC2209Stepper pulley(AX_PUL_RX, AX_PUL_TX, R_SENSE, DRIVER_ADDRESS);
  TMC2209Stepper selector(AX_SEL_RX, AX_SEL_TX, R_SENSE, DRIVER_ADDRESS);
  TMC2209Stepper idler(AX_IDL_RX, AX_IDL_TX, R_SENSE, DRIVER_ADDRESS);
  static uint8_t sgt_min = 0, sgt_max = 255;
#endif

#ifdef TMC2130
  #ifndef R_SENSE
    #define R_SENSE 0.11
  #endif
  TMC2130Stepper pulley(AX_PUL_CS_PIN, R_SENSE);
  TMC2130Stepper selector(AX_SEL_CS_PIN, R_SENSE);
  TMC2130Stepper idler(AX_IDL_CS_PIN, R_SENSE);   // Hardware SPI
  static int8_t sgt_min = -64, sgt_max = 63;
#endif

#ifdef TMC5160
  #ifndef R_SENSE
    #define R_SENSE 0.075
  #endif
  TMC5160Stepper pulley(AX_PUL_CS_PIN, R_SENSE);
  TMC5160Stepper selector(AX_SEL_CS_PIN, R_SENSE);
  TMC5160Stepper idler(AX_IDL_CS_PIN, R_SENSE); // Hardware SPI
  static int8_t sgt_min = -64, sgt_max = 63; 
#endif

int8_t __sg_thr(AXIS axis)
{
  int8_t stallguard=5;
	switch (axis)
	{
  case AX_PUL:
    break;
	case AX_SEL:
    stallguard=(int8_t)constrain(TMC_SG_THR_SEL,sgt_min, sgt_max);
    break;
	case AX_IDL:
    stallguard=(int8_t)constrain(TMC_SG_THR_IDL,sgt_min, sgt_max);
	  break;
  default:
    break;
	}
  return stallguard;
}


uint8_t tmc_usteps2mres(AXIS axis)
{
  switch(axis) 
  {
    case AX_PUL: return 7;         //2 ustep
    case AX_SEL: return 7;         //2 ustep
    case AX_IDL: return 4;         //16 ustep
    default: break;
  }
  return 0;
}

//Inverse motor direction
uint8_t tmc_direction(AXIS axis)
{
  switch(axis) 
  {
    case AX_PUL: return PULLEY_DIR_INVERTING; 
    case AX_SEL: return SELE_DIR_INVERTING; 
    case AX_IDL: return IDLER_DIR_INVERTING; 
    default: break;
  }
  return 0;
}


// void tmc_enable_stallguard(TMC5160Stepper &st) 
// {
//   st.en_pwm_mode(0);
//   st.TCOOLTHRS(0x5FFF);
//   st.diag1_stall(true);
// }


void tmc_enable_stallguard(TMC2130Stepper &st) 
{
  st.en_pwm_mode(0);
  st.TCOOLTHRS(0x5FF);
  st.diag1_stall(true);
}

void tmc_disable_stallguard(TMC2130Stepper &st) 
{
  st.TCOOLTHRS(0);
  st.en_pwm_mode(1);
  st.diag1_stall(false);
}

void tmc_enable_stallguard(TMC2209Stepper &st) 
{
  st.TCOOLTHRS(0xFFFF);
  st.en_spreadCycle(false);
}

void tmc_disable_stallguard(TMC2209Stepper &st) 
{
  st.en_spreadCycle(true);
  st.TCOOLTHRS(0);
}

uint8_t idler_diag = 0;
uint8_t sel_diag = 0;

void IDL_DIAG_Callback(void)
{
  digitalWrite(LED1, LOW);
  delay(50);
  idler_diag = 1;
  digitalWrite(LED1, HIGH);
}

void SEL_DIAG_Callback(void)
{
  digitalWrite(LED2, LOW);
  delay(50);
  sel_diag = 1;
  digitalWrite(LED2, HIGH);
}

void tmc_pin_init()
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOA_CLK_ENABLE();    
  __HAL_RCC_GPIOB_CLK_ENABLE(); 		
	GPIO_InitStruct.Pin  = GPIO_PIN_11|GPIO_PIN_8;				 //stpe pin
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  GPIO_InitStruct.Pin  = GPIO_PIN_4;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);             

  #if HAS_TMC_SPI
    GPIO_InitStruct.Pin  = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10;      //spi CS pin
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);  
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10, GPIO_PIN_SET);
  #endif

  stm32_interrupt_enable(DIAG_PORT, IDL_DIAG_PIN, IDL_DIAG_Callback, GPIO_MODE_IT_RISING);
  stm32_interrupt_enable(DIAG_PORT, SEL_DIAG_PIN, SEL_DIAG_Callback, GPIO_MODE_IT_RISING);

  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Pin = SEL_DIAG_PIN|IDL_DIAG_PIN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
}

void tmc_begin()
{
  #if HAS_TMC220x
    pulley.beginSerial(TMC_BAUD_RATE);
    selector.beginSerial(TMC_BAUD_RATE);
    idler.beginSerial(TMC_BAUD_RATE);
  #endif
  #if HAS_TMC_SPI
    SPI.begin();
  #endif
    tmc_pin_init();
}


void tmc_disable_axis(AXIS axis)
{
  uint8_t en = 0;
  en = shr16_get_ena();
  en = en & (~(axis+1));
  shr16_set_ena(en);
}

#ifdef TMC2208
  void tmc_init_axis(TMC2208Stepper &st, AXIS axis, TMC_MODE mode)
  {
    uint8_t current_holding[3] = CURRENT_HOLDING;
    uint8_t current_running[3] = CURRENT_RUNNING;

    uint8_t current_h = current_holding[axis];
    uint8_t current_r = current_running[axis];

    current_h = (int8_t)constrain(current_h,current_min, current_max);
    current_r = (int8_t)constrain(current_r,current_min, current_max);
    
    TMC2208_n::GCONF_t gconf{0};
    gconf.pdn_disable = true;         // Use UART
    gconf.mstep_reg_select = true;    // Select microsteps with UART
    gconf.i_scale_analog = false;
    gconf.en_spreadcycle = true; 
    gconf.shaft = tmc_direction(axis);
    st.GCONF(gconf.sr); 

    IHOLD_IRUN_t ihold_irun{0};
    ihold_irun.ihold = current_h;
    ihold_irun.irun = current_r;
    ihold_irun.iholddelay = 10;
    st.IHOLD_IRUN(ihold_irun.sr);

    TMC2208_n::CHOPCONF_t chopconf{0};
    chopconf.toff = 3;
    chopconf.hstrt = 5;
    chopconf.hend = 1;
    chopconf.tbl = 2;
    // chopconf.vsense = 1;
    chopconf.mres = tmc_usteps2mres(axis);
    chopconf.intpol = 1;
    st.CHOPCONF(chopconf.sr);
    st.TPOWERDOWN(TMC_TPOWERDOWN);

    TMC2208_n::PWMCONF_t pwmconf{0};
    pwmconf.pwm_lim = 12;
    pwmconf.pwm_reg = 8;
    pwmconf.pwm_autograd = true;
    pwmconf.pwm_autoscale = true;
    pwmconf.pwm_freq = 0b01;
    pwmconf.pwm_grad = 14;
    pwmconf.pwm_ofs = 36;
    st.PWMCONF(pwmconf.sr);

  }
#endif  // TMC2208

#ifdef TMC2209
  void tmc_init_axis(TMC2209Stepper &st, AXIS axis, TMC_MODE mode) 
  {
    uint8_t current_holding[3] = CURRENT_HOLDING;
    uint8_t current_running[3] = CURRENT_RUNNING;

    uint8_t current_h = current_holding[axis];
    uint8_t current_r = current_running[axis];

    current_h = (int8_t)constrain(current_h,current_min, current_max);
    current_r = (int8_t)constrain(current_r,current_min, current_max);

    TMC2208_n::GCONF_t gconf{0};
    gconf.pdn_disable = true;         // Use UART
    gconf.mstep_reg_select = true;    // Select microsteps with UART
    gconf.i_scale_analog = false;
    // gconf.en_spreadcycle = stealth;
    gconf.shaft = tmc_direction(axis);
    st.GCONF(gconf.sr);

    TMC2208_n::CHOPCONF_t chopconf{0};
    chopconf.toff = chopper_timing.toff;
    chopconf.hstrt = chopper_timing.hstrt - 1;
    chopconf.hend = chopper_timing.hend + 3;
    chopconf.tbl = 2;
    // chopconf.vsense = 1;
    chopconf.mres = tmc_usteps2mres(axis);
    chopconf.intpol = INTERPOLATE;
    st.CHOPCONF(chopconf.sr);

    IHOLD_IRUN_t ihold_irun{0};
    ihold_irun.ihold = current_h;
    ihold_irun.irun = current_r;
    ihold_irun.iholddelay = 10;
    st.IHOLD_IRUN(ihold_irun.sr);

    st.TPOWERDOWN(TMC_TPOWERDOWN);
    
    TMC2208_n::PWMCONF_t pwmconf{0};
    pwmconf.pwm_lim = 12;
    pwmconf.pwm_reg = 8;
    pwmconf.pwm_autograd = true;
    pwmconf.pwm_autoscale = true;
    pwmconf.pwm_freq = 0b01;
    pwmconf.pwm_grad = 14;
    pwmconf.pwm_ofs = 36;
    st.PWMCONF(pwmconf.sr);


    if (mode == HOMING_MODE)
    {
      st.SGTHRS(__sg_thr(axis));
      tmc_enable_stallguard(st);
    }
    else if (mode == STEALTH_MODE)
    {
      tmc_disable_stallguard(st);
      // st.TCOOLTHRS(__tcoolthrs(axis));
      // #ifdef HAS_TMC220x
      //   st.GCONF(0x000000C4);
      // #endif
    }

    // st.TPOWERDOWN(10);
    // st.TCOOLTHRS(__tcoolthrs(axis));
  }
#endif  // TMC2209

#ifdef TMC2130
  void tmc_init_axis(TMC2130Stepper &st, AXIS axis, TMC_MODE mode)
  {
    uint8_t current_holding[3] = CURRENT_HOLDING;
    uint8_t current_running[3] = CURRENT_RUNNING;

    uint8_t current_h = current_holding[axis];
    uint8_t current_r = current_running[axis];

    current_h = (int8_t)constrain(current_h,current_min, current_max);
    current_r = (int8_t)constrain(current_r,current_min, current_max);

    GCONF_t gconf{0};
    gconf.diag1_stall = 1;
    gconf.diag1_pushpull = 1;
    // gconf.en_pwm_mode = stealth;
    gconf.shaft = tmc_direction(axis);
    st.GCONF(gconf.sr);

    IHOLD_IRUN_t ihold_irun{0};
    ihold_irun.ihold = current_h;
    ihold_irun.irun = current_r;
    ihold_irun.iholddelay = 10;
    st.IHOLD_IRUN(ihold_irun.sr);

    st.TPOWERDOWN(TMC_TPOWERDOWN);
    
    PWMCONF_t pwmconf{0};
    pwmconf.pwm_freq = 0b01; // f_pwm = 2/683 f_clk
    pwmconf.pwm_autoscale = true;
    pwmconf.pwm_grad = 5;
    pwmconf.pwm_ampl = 180;
    st.PWMCONF(pwmconf.sr);

    CHOPCONF_t chopconf{0};
    chopconf.toff = chopper_timing.toff;
    chopconf.hstrt = chopper_timing.hstrt - 1;
    chopconf.hend = chopper_timing.hend + 3;
    chopconf.tbl = 2;
    chopconf.vsense = 1;
    chopconf.mres = tmc_usteps2mres(axis);
    chopconf.intpol = INTERPOLATE;
    st.CHOPCONF(chopconf.sr);


    if(mode == HOMING_MODE)
    {
      COOLCONF_t coolconf{0};
      coolconf.sgt = __sg_thr(axis);
      st.COOLCONF(coolconf.sr);
      tmc_enable_stallguard(st);
    }
    else if(mode == STEALTH_MODE)
    {
      tmc_disable_stallguard(st);
    }

  }
#endif
#ifdef TMC5160
  void tmc_init_axis(TMC5160Stepper &st, AXIS axis, TMC_MODE mode)
  {
    uint8_t current_holding[3] = CURRENT_HOLDING;
    uint8_t current_running[3] = CURRENT_RUNNING;

    uint8_t current_h = current_holding[axis];
    uint8_t current_r = current_running[axis];

    current_h = (int8_t)constrain(current_h,current_min, current_max);
    current_r = (int8_t)constrain(current_r,current_min, current_max);

    GCONF_t gconf{0};
    gconf.diag1_stall = 1;
    gconf.diag1_pushpull = 1;
    // gconf.en_pwm_mode = 1;
    gconf.multistep_filt = 1;
    gconf.shaft = tmc_direction(axis);
    st.GCONF(gconf.sr);

    CHOPCONF_t chopconf{0};
    chopconf.tbl = 2;
    chopconf.toff = 3;//hopper_timing.toff;
    chopconf.intpol = INTERPOLATE;
    chopconf.hend = 2;//chopper_timing.hend + 3;
    chopconf.hstrt = 5;//chopper_timing.hstrt - 1;
    chopconf.mres = tmc_usteps2mres(axis);
    chopconf.chm = 0;
    chopconf.tpfd = 4;
    st.CHOPCONF(chopconf.sr);

    IHOLD_IRUN_t ihold_irun{0};
    ihold_irun.ihold = current_h;
    ihold_irun.irun = current_r;
    ihold_irun.iholddelay = 10;
    st.IHOLD_IRUN(ihold_irun.sr);

    st.TPOWERDOWN(TMC_TPOWERDOWN); // ~1s until driver lowers to hold current

    st.TPWMTHRS(0);

    TMC2160_n::PWMCONF_t pwmconf{0};
    pwmconf.pwm_lim = 12;
    pwmconf.pwm_reg = 4;
    pwmconf.pwm_autograd = true;
    pwmconf.pwm_autoscale = true;
    pwmconf.pwm_freq = 0b00;
    pwmconf.pwm_ofs = 36;
    st.PWMCONF(pwmconf.sr);

    st.GSTAT(); // Clear GSTAT

    if(mode == HOMING_MODE)
    {
      COOLCONF_t coolconf{0};
      coolconf.sgt = __sg_thr(axis);
      st.COOLCONF(coolconf.sr);
      tmc_enable_stallguard(st);
    }
    else if(mode == STEALTH_MODE)
    {
      tmc_disable_stallguard(st);
    }

  }
#endif


void tmc_init(TMC_MODE mode)
{
  selector_step_pin_reset(); //PB4  stepper pin
  pulley_step_pin_reset();   //PA11
  idler_step_pin_reset();	   //PA8

  tmc_init_axis(pulley, AX_PUL, STEALTH_MODE);
  tmc_init_axis(selector, AX_SEL, mode);
  tmc_init_axis(idler, AX_IDL, mode);
}

template<typename TMC>
uint16_t read_sg(TMC &st)
{
  uint32_t val32 = 0;
  val32 = st.SG_RESULT();
  return (val32 & 0x3ff);
}

uint16_t tmc_read_sg(TMC2209Stepper &st)
{
  uint32_t val32 = 0;
  val32 = st.SG_RESULT();
  return (val32 & 0x3ff);
}

template<typename TMC>
uint8_t read_gstat(TMC &st)
{
	uint8_t retval = 0;
  retval = st.GSTAT();
  return (retval & 0x7);
}

uint8_t tmc_read_gstat()
{
  uint32_t result = 0;
	result |= read_gstat(pulley);
  result |= read_gstat(selector);
  result |= read_gstat(idler);
	return result;
}

#ifdef TMC_DEBUG
template<typename TMC>
static bool test_connection(TMC &st) 
{
  const uint8_t test_result = st.test_connection();
  switch (test_result) {
    case 0: Serial1.println("OK"); break;
    case 1: Serial1.println("HIGH"); break;
    case 2: Serial1.println("LOW"); break;
    default:
      break;
  }
  return test_result;
}

void test_tmc_connection()
{
  uint8_t axis_connection = 0;
  Serial1.println("connection... ");
  Serial1.print("M1:");
  axis_connection += test_connection(pulley);
  Serial1.print("M2:");
  axis_connection += test_connection(selector);
  Serial1.print("M3:");
  axis_connection += test_connection(idler);
  if(axis_connection) Serial1.println("TMC ERROR");
}
#endif //TMC_DEGUBG