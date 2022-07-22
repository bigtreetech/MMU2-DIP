// main configuration file
#ifndef CONFIG_H_
#define CONFIG_H_

//UART1
#define UART1_BDR 115200

//communication uart0/1
#define UART_COM 1

/* Chopper Timing
*
* Options: CHOPPER_DEFAULT_12V, CHOPPER_DEFAULT_19V, CHOPPER_DEFAULT_24V
*/
#define CHOPPER_TIMING CHOPPER_DEFAULT_24V

/* Print simple drive status information
 * Note that debug mode cannot connect the motherboard normally
*/
// #define TMC_DEBUG

// filament Sensor with COM to Vcc and NC to Signal uses "true" here (most common setup).
#define FILAMENT_SENSOR_INVERTING   true // Set to true to invert the logic of the filament_Sensor.


//Inverse motor direction
#define PULLEY_DIR_INVERTING        false
#define SELE_DIR_INVERTING          false
#define IDLER_DIR_INVERTING         false


#define INTERPOLATE       true  // Interpolate X/Y/Z_MICROSTEPS to 256

/* Stepper Drivers
*  Options: spi:TMC2130, TMC5160
*           uart:TMC2209,TMC2208(TMC2208 no stallguard)
*/
#define TMC2209            1

// SG_THR stallguard treshold (sensitivity),  TMC2209: 0...255. TMC2130: 63...-64
// !!! This setting is not universal, and the values set by different machines are different !!!
#define TMC_SG_THR_SEL     50  //TMC2209 set to about 50, TMC2130 Set to about 31
#define TMC_SG_THR_IDL     20  //TMC2209 set to about 20, TMC2130 Set to about 19


#define TMC_TPOWERDOWN     68     // TCOOLTHRS default


/* TMC drives set the motor current. The higher the current, the greater the heat and noise.
*  the lower the current the smaller the torque, choose a balance point.
*
*  Set currents   0~31    {AX_PUL , AX_SEL , AX_IDL}
*/
#define CURRENT_HOLDING     {0,  2,  8}
#define CURRENT_RUNNING     {15, 15, 15}

// Override default RSENSE value
// #define R_SENSE        0.11

//number of extruders [1 2 3 4 5]
#define EXTRUDERS 5

#endif //CONFIG_H_
