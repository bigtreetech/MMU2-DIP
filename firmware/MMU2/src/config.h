// main configuration file
#ifndef CONFIG_H_
#define CONFIG_H_

//UART1
#define UART1_BDR 115200

//communication uart0/1
#define UART_COM 1

#define CHOPPER_TIMING  CHOPPER_DEFAULT_12V

/* Print simple drive status information
 * Note that debug mode cannot connect the motherboard normally
*/
// #define TMC_DEBUG

#define HOLD_MULTIPLIER    0.5  // Scales down the holding current from run current
#define INTERPOLATE       true  // Interpolate X/Y/Z_MICROSTEPS to 256

/* Stepper Drivers
*  Options: spi:TMC2130, TMC5160
*           uart:TMC2209,TMC2208(TMC2208 no stallguard)
*/
#define TMC2130            1

// SG_THR stallguard treshold (sensitivity),  TMC2209: 0...255. TMC2130: 63...-64
// !!! This setting is not universal, and the values set by different machines are different !!!
#define TMC_SG_THR_SEL     50  //TMC2209 set to about 50, TMC2130 Set to about 31
#define TMC_SG_THR_IDL     20  //TMC2209 set to about 20, TMC2130 Set to about 19

#define TMC_TCOOLTHRS      450     // TCOOLTHRS default

// TCOOLTHRS coolstep treshold, usable range 0-300
#define TMC_TCOOLTHRS_0    128  // ~2s until driver lowers to hold current
#define TMC_TCOOLTHRS_1    128
#define TMC_TCOOLTHRS_2    128

/* TMC drive set the motor current, the higher the current the greater the heat generation, 
*  the lower the current the smaller the torque, choose a balance point.
*
*  Set currents   0~31    {AX_PUL , AX_SEL , AX_IDL}
*/
#define CURRENT_HOLDING_STEALTH {1, 8, 8}
#define CURRENT_HOLDING_NORMAL  {1, 8, 8}
#define CURRENT_RUNNING_STEALTH {18, 20, 20}
#define CURRENT_RUNNING_NORMAL  {20, 22, 23}
#define CURRENT_HOMING          {1, 22, 22}

//number of extruders [1 2 3 4 5]
#define EXTRUDERS 5

#endif //CONFIG_H_
