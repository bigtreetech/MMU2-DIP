//adc.h
#ifndef _ADC_H
#define _ADC_H
#include <Arduino.h>
#include <inttypes.h>
#include "config.h"

extern __IO uint16_t adc_val;

#if defined(__cplusplus)
extern "C" {
#endif //defined(__cplusplus)

extern void adc_init(void);

#if defined(__cplusplus)
}
#endif //defined(__cplusplus)

#endif //_ADC_H
