#include "adc.h"

void ADC_Init(void) {
    adc16_config_t adcConfig;
    ADC16_GetDefaultConfig(&adcConfig);
    ADC16_Init(ADC_BASE, &adcConfig);
    ADC16_EnableHardwareTrigger(ADC_BASE, false);
}

uint16_t ADC_ReadBatteryRaw(void) {
    adc16_channel_config_t channelConfig = {.channelNumber = BATTERY_CHANNEL,
                                            .enableInterruptOnConversionCompleted = false,
                                            .enableDifferentialConversion = false};

    ADC16_SetChannelConfig(ADC_BASE, ADC_CHANNEL_GROUP, &channelConfig);
    while (!(ADC16_GetChannelStatusFlags(ADC_BASE, ADC_CHANNEL_GROUP) &
             kADC16_ChannelConversionDoneFlag)) {
    }
    return ADC16_GetChannelConversionValue(ADC_BASE, ADC_CHANNEL_GROUP);
}

float ADC_ReadBatteryVoltage(void) {
    uint16_t adcVal = ADC_ReadBatteryRaw();
    PRINTF("adc raw: %d\r\n", adcVal);
    float voltageADC = adcVal * VREF / ADC_RESOLUTION;
    float batteryVoltage = voltageADC * DIVIDER_RATIO;
    float val = batteryVoltage;
    int partea_int = (int)val;
    int partea_zec = (int)((val - partea_int) * 100);
    PRINTF("Bateria are %d.%02d V\r\n", partea_int, partea_zec);
    return batteryVoltage;
}
