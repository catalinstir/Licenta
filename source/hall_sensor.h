/*
 * hall_sensor.h
 *
 * Hall sensor input capture driver for RDDRONE-FMUK66
 * Uses FTM0_CH2 on PTC3 (J4 Pin 9)
 *
 * Measures RPM from hall sensor pulses using hardware input capture
 */

#ifndef HALL_SENSOR_H_
#define HALL_SENSOR_H_

#include <stdint.h>
#include <stdbool.h>

/*******************************************************************************
 * Configuration
 ******************************************************************************/

/* Hardware Configuration */
#define HALL_FTM_BASEADDR       FTM0
#define HALL_FTM_CHANNEL        kFTM_Chnl_2      /* FTM0_CH2 = PTC3 */
#define HALL_FTM_IRQn           FTM0_IRQn
#define HALL_FTM_IRQ_HANDLER    FTM0_IRQHandler

/* GPIO for pin mux */
#define HALL_GPIO_PORT          PORTC
#define HALL_GPIO_PIN           3U

/* Measurement Configuration */
#define HALL_PULSES_PER_REV     1U              /* Pulses per wheel revolution */
                                                 /* Change if your sensor has multiple magnets */

#define HALL_TIMEOUT_MS         500U            /* Consider stopped if no pulse for this long */
#define HALL_MIN_RPM            10U             /* Minimum measurable RPM */
#define HALL_MAX_RPM            10000U          /* Maximum expected RPM */

/* Filter Configuration */
#define HALL_FILTER_SAMPLES     4U              /* Moving average filter size (power of 2) */

/*******************************************************************************
 * Data Types
 ******************************************************************************/

typedef struct {
    /* Raw capture data */
    uint32_t lastCaptureValue;      /* Last captured timer value */
    uint32_t captureInterval;       /* Ticks between last two captures */
    uint32_t captureCount;          /* Total number of captures (pulse count) */

    /* Calculated values */
    float rpm;                      /* Current RPM */
    float frequency;                /* Pulse frequency in Hz */
    float speed_mps;                /* Speed in m/s (if wheel diameter configured) */

    /* Status */
    bool isRunning;                 /* True if pulses are being received */
    uint32_t lastUpdateTime;        /* System tick of last valid capture */

    /* Filter buffer */
    uint32_t filterBuffer[HALL_FILTER_SAMPLES];
    uint8_t filterIndex;

    /* Configuration */
    float wheelCircumference_m;     /* Wheel circumference in meters */
    uint32_t timerFrequency;        /* FTM clock frequency */
} HallSensor_t;

/*******************************************************************************
 * API Functions
 ******************************************************************************/

/**
 * @brief Initialize the hall sensor input capture
 *
 * Configures FTM0_CH2 for input capture on rising edge.
 * Must be called before using any other hall sensor functions.
 */
void HallSensor_Init(void);

/**
 * @brief Set wheel circumference for speed calculation
 *
 * @param circumference_m Wheel circumference in meters
 */
void HallSensor_SetWheelCircumference(float circumference_m);

/**
 * @brief Get current RPM
 *
 * @return Current RPM, 0 if stopped or no valid reading
 */
float HallSensor_GetRPM(void);

/**
 * @brief Get pulse frequency
 *
 * @return Frequency in Hz
 */
float HallSensor_GetFrequency(void);

/**
 * @brief Get calculated speed
 *
 * @return Speed in meters per second (requires wheel circumference to be set)
 */
float HallSensor_GetSpeed_mps(void);

/**
 * @brief Get total pulse count since init
 *
 * @return Number of pulses detected
 */
uint32_t HallSensor_GetPulseCount(void);

/**
 * @brief Check if wheel is currently spinning
 *
 * @return true if pulses received within timeout period
 */
bool HallSensor_IsRunning(void);

/**
 * @brief Reset pulse counter and measurements
 */
void HallSensor_Reset(void);

/**
 * @brief Update function - call periodically to check for timeout
 *
 * Should be called from main loop or a periodic timer.
 * Handles timeout detection when wheel stops.
 *
 * @param currentTime_ms Current system time in milliseconds
 */
void HallSensor_Update(uint32_t currentTime_ms);

/**
 * @brief Get pointer to hall sensor data structure
 *
 * Useful for debugging or accessing all data at once.
 *
 * @return Pointer to HallSensor_t structure
 */
const HallSensor_t* HallSensor_GetData(void);

/*******************************************************************************
 * Debug Functions
 ******************************************************************************/

/**
 * @brief Print current hall sensor status to debug console
 */
void HallSensor_PrintStatus(void);

#endif /* HALL_SENSOR_H_ */
