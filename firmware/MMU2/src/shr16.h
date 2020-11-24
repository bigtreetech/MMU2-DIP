//shr16.h - 16bit shift register (2x74595)
#ifndef _SHR16_H
#define _SHR16_H

#include <inttypes.h>

#if defined(__cplusplus)
extern "C" {
#endif //defined(__cplusplus)

/*shift register outputs
* LEDS - hardcoded
* SHR16_LEDG0          0x0100
* SHR16_LEDR0          0x0200
* SHR16_LEDG1          0x0400
* SHR16_LEDR1          0x0800
* SHR16_LEDG2          0x1000
* SHR16_LEDR2          0x2000
* SHR16_LEDG3          0x4000
* SHR16_LEDR3          0x8000
* SHR16_LEDG4          0x0040
* SHR16_LEDR4          0x0080
*/
#define SHR16_DIR_0          0x0001
#define SHR16_ENA_0          0x0002
#define SHR16_DIR_1          0x0004
#define SHR16_ENA_1          0x0008
#define SHR16_DIR_2          0x0010
#define SHR16_ENA_2          0x0020
#define SHR16_LED_MSK        0xffc0
#define SHR16_DIR_MSK        (SHR16_DIR_0 + SHR16_DIR_1 + SHR16_DIR_2)
#define SHR16_ENA_MSK        (SHR16_ENA_0 + SHR16_ENA_1 + SHR16_ENA_2)


extern uint16_t shr16_v;

extern void shr16_init(void);

extern void shr16_set_led(uint16_t led);

extern void shr16_set_ena(uint8_t ena);

extern void shr16_set_dir(uint8_t dir);

extern uint8_t shr16_get_ena(void);

extern uint8_t shr16_get_dir(void);


#if defined(__cplusplus)
}
#endif //defined(__cplusplus)
#endif //_SHR16_H
