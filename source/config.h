#ifndef CONFIG_H
#define CONFIG_H

#include "fsl_clock.h"
#include "fsl_ftm.h"
#include "fsl_gpio.h"
#include "fsl_i2c.h"

/* ======================== MOTOR SETTINGS ======================== */
#define MOTOR_BRUSHED_FTM_BASEADDR  FTM0
#define MOTOR_STEERING_FTM_BASEADDR FTM3
#define MOTOR_BRUSHED               kFTM_Chnl_5
#define MOTOR_STEERING              kFTM_Chnl_6
#define MOTOR_DELAY1                50000000

/* Get source clock for FTM driver */
#define FTM_SOURCE_CLOCK CLOCK_GetFreq(kCLOCK_BusClk)
#ifndef FTM_PWM_ON_LEVEL
#define FTM_PWM_ON_LEVEL kFTM_HighTrue
#endif
#ifndef DEMO_PWM_FREQUENCY
#define DEMO_PWM_FREQUENCY (37U) /* 50Hz frequency for RC motors */
#endif

/* ======================== I2C SETTINGS ======================== */
#define EXAMPLE_I2C_MASTER_BASEADDR I2C0
#define I2C_MASTER_CLK_SRC          I2C0_CLK_SRC
#define I2C_MASTER_CLK_FREQ         CLOCK_GetFreq(I2C0_CLK_SRC)
#define I2C_MASTER_SLAVE_ADDR_7BIT  0x54U
#define I2C_BAUDRATE                100000U
/* Set default SCL stop hold time to 4us for 100kHz baudrate according to spec. For 400kHz and 1mHz
   the hold time is 0.6us and 0.26us. */
#define I2C_DATA_LENGTH 128U

/* ======================== GPIO SETTINGS ======================== */
#define BOARD_ECHO_GPIO       GPIOC
#define BOARD_ECHO_GPIO_PIN   1U
#define BOARD_TRIG_GPIO       GPIOA
#define BOARD_TRIG_GPIO_PIN   6U
#define BOARD_SWITCH_GPIO     GPIOE
#define BOARD_SWITCH_GPIO_PIN 12U

/* ======================== DELAY SETTINGS ======================== */
#define DELAY_1SEC           13300000
#define DELAY_BRUSHED_RESET  20000   /* Smooth speed change */
#define DELAY_STEERING_RESET 40000   /* Smooth steering change */
#define DELAY_BRUSHED_CMD    5000000 /* Brushed motor speed sensitivity */
#define DELAY_STEERING_CMD   50000   /* Steering smoothness */

/* ======================== DEFINE SETTINGS ======================== */
#define CENTER_X 39.0f

/* ======================== ADC SETTINGS ======================== */
#define ADC_BASE          ADC1
#define ADC_CHANNEL_GROUP 0
#define BATTERY_CHANNEL   10 // ADC1_SE10 = PTB4
#define VREF              3.3f
#define ADC_RESOLUTION    4095.0f
#define DIVIDER_RATIO     10.15f

#endif /* CONFIG_H */
