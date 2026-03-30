#ifndef ADC_H_
#define ADC_H_

#include "config.h"
#include "control_algorithms.h"
#include "fsl_adc16.h"
#include "fsl_debug_console.h"
#include <stdint.h>

void ADC_Init(void);
uint16_t ADC_ReadBatteryRaw(void);
float ADC_ReadBatteryVoltage(void);

#endif /* ADC_H_ */
