/*
 * hall_sensor.c
 *
 * Hall sensor input capture driver for RDDRONE-FMUK66
 * Uses FTM0_CH2 on PTC3 (J4 Pin 9)
 */

#include "hall_sensor.h"
#include "fsl_ftm.h"
#include "fsl_port.h"
#include "fsl_clock.h"
#include "fsl_debug_console.h"
#include <string.h>

/*******************************************************************************
 * Private Variables
 ******************************************************************************/

static HallSensor_t g_hallSensor;

/* Flag for ISR to signal new capture */
static volatile bool g_newCapture = false;

/*******************************************************************************
 * Private Functions
 ******************************************************************************/

/**
 * @brief Apply moving average filter to capture interval
 */
static uint32_t HallSensor_FilterInterval(uint32_t newInterval)
{
    uint32_t sum = 0;

    /* Add new sample to circular buffer */
    g_hallSensor.filterBuffer[g_hallSensor.filterIndex] = newInterval;
    g_hallSensor.filterIndex = (g_hallSensor.filterIndex + 1) % HALL_FILTER_SAMPLES;

    /* Calculate average */
    for (uint8_t i = 0; i < HALL_FILTER_SAMPLES; i++) {
        sum += g_hallSensor.filterBuffer[i];
    }

    return sum / HALL_FILTER_SAMPLES;
}

/**
 * @brief Calculate RPM from capture interval
 */
static float HallSensor_CalculateRPM(uint32_t interval)
{
    if (interval == 0) {
        return 0.0f;
    }

    /*
     * RPM = (timer_frequency / interval) * 60 / pulses_per_rev
     *
     * timer_frequency = ticks per second
     * interval = ticks between pulses
     * 60 = seconds per minute
     */
    float frequency = (float)g_hallSensor.timerFrequency / (float)interval;
    float rpm = (frequency * 60.0f) / (float)HALL_PULSES_PER_REV;

    /* Clamp to valid range */
    if (rpm < (float)HALL_MIN_RPM) {
        return 0.0f;
    }
    if (rpm > (float)HALL_MAX_RPM) {
        return (float)HALL_MAX_RPM;
    }

    return rpm;
}

/**
 * @brief Calculate speed from RPM
 */
static float HallSensor_CalculateSpeed(float rpm)
{
    if (g_hallSensor.wheelCircumference_m <= 0.0f) {
        return 0.0f;
    }

    /*
     * speed (m/s) = RPM * circumference (m) / 60
     */
    return (rpm * g_hallSensor.wheelCircumference_m) / 60.0f;
}

/*******************************************************************************
 * Interrupt Handler
 ******************************************************************************/

/**
 * @brief FTM0 Interrupt Handler for input capture
 *
 * NOTE: If you already have an FTM0_IRQHandler in your project,
 * you'll need to merge this code into that handler.
 */
void FTM0_IRQHandler(void)
{
    uint32_t currentCapture;
    uint32_t interval;

    /* Check if this is a channel 2 capture event */
    if (FTM_GetStatusFlags(HALL_FTM_BASEADDR) & kFTM_Chnl2Flag) {

        /* Read the captured value */
        currentCapture = HALL_FTM_BASEADDR->CONTROLS[HALL_FTM_CHANNEL].CnV;

        /* Calculate interval (handles timer overflow) */
        if (currentCapture >= g_hallSensor.lastCaptureValue) {
            interval = currentCapture - g_hallSensor.lastCaptureValue;
        } else {
            /* Timer overflow occurred */
            interval = (0xFFFF - g_hallSensor.lastCaptureValue) + currentCapture + 1;
        }

        /* Store values */
        g_hallSensor.lastCaptureValue = currentCapture;
        g_hallSensor.captureInterval = interval;
        g_hallSensor.captureCount++;
        g_hallSensor.isRunning = true;
        g_newCapture = true;

        /* Clear the channel flag */
        FTM_ClearStatusFlags(HALL_FTM_BASEADDR, kFTM_Chnl2Flag);
    }

    /* Clear timer overflow flag if set */
    if (FTM_GetStatusFlags(HALL_FTM_BASEADDR) & kFTM_TimeOverflowFlag) {
        FTM_ClearStatusFlags(HALL_FTM_BASEADDR, kFTM_TimeOverflowFlag);
    }

    __DSB();
}

/*******************************************************************************
 * Public Functions
 ******************************************************************************/

void HallSensor_Init(void)
{
    ftm_config_t ftmConfig;

    /* Clear the data structure */
    memset(&g_hallSensor, 0, sizeof(HallSensor_t));

    /* Enable clock for PORTC */
    CLOCK_EnableClock(kCLOCK_PortC);

    /* Configure PTC3 as FTM0_CH2 (Alt4) */
    PORT_SetPinMux(HALL_GPIO_PORT, HALL_GPIO_PIN, kPORT_MuxAlt4);

    /* Optional: Enable pull-up if your hall sensor is open-drain */
    PORT_SetPinConfig(HALL_GPIO_PORT, HALL_GPIO_PIN, &(port_pin_config_t){
        .pullSelect = kPORT_PullUp,
        .slewRate = kPORT_FastSlewRate,
        .passiveFilterEnable = kPORT_PassiveFilterDisable,
        .openDrainEnable = kPORT_OpenDrainDisable,
        .driveStrength = kPORT_LowDriveStrength,
        .mux = kPORT_MuxAlt4
    });

    /* Get FTM clock frequency */
    g_hallSensor.timerFrequency = CLOCK_GetFreq(kCLOCK_BusClk);

    /* Initialize FTM with default config */
    FTM_GetDefaultConfig(&ftmConfig);

    /*
     * Configure prescaler based on expected RPM range
     *
     * For measuring RPM from ~10 to ~10000:
     * - At 60MHz bus clock with prescaler 128: 468,750 Hz timer
     * - At 10 RPM (1 pulse per 6 seconds): interval = 2,812,500 ticks (overflow!)
     * - At 10000 RPM (166.7 pulses/sec): interval = 2,812 ticks (good resolution)
     *
     * Use prescaler 128 for good resolution at high RPM
     * Handle overflow in ISR for low RPM
     */
    ftmConfig.prescale = kFTM_Prescale_Divide_128;

    /* Update timer frequency after prescaler */
    g_hallSensor.timerFrequency = g_hallSensor.timerFrequency / 128;

    /* Initialize the FTM */
    FTM_Init(HALL_FTM_BASEADDR, &ftmConfig);

    /*
     * Configure channel 2 for input capture on rising edge
     *
     * Note: If your hall sensor outputs falling edge pulses,
     * change to kFTM_FallingEdge or kFTM_RisingAndFallingEdge
     */
    FTM_SetupInputCapture(HALL_FTM_BASEADDR,
                          HALL_FTM_CHANNEL,
                          kFTM_RisingEdge,
                          0);  /* No filter */

    /* Enable channel interrupt */
    FTM_EnableInterrupts(HALL_FTM_BASEADDR, kFTM_Chnl2InterruptEnable);

    /* Enable FTM0 interrupt in NVIC */
    EnableIRQ(HALL_FTM_IRQn);

    /* Start the timer */
    FTM_StartTimer(HALL_FTM_BASEADDR, kFTM_SystemClock);

    PRINTF("Hall Sensor initialized on PTC3 (J4 Pin 9)\r\n");
    PRINTF("Timer frequency: %lu Hz\r\n", g_hallSensor.timerFrequency);
}

void HallSensor_SetWheelCircumference(float circumference_m)
{
    g_hallSensor.wheelCircumference_m = circumference_m;
    PRINTF("Wheel circumference set to %.3f m\r\n", circumference_m);
}

float HallSensor_GetRPM(void)
{
    return g_hallSensor.rpm;
}

float HallSensor_GetFrequency(void)
{
    return g_hallSensor.frequency;
}

float HallSensor_GetSpeed_mps(void)
{
    return g_hallSensor.speed_mps;
}

uint32_t HallSensor_GetPulseCount(void)
{
    return g_hallSensor.captureCount;
}

bool HallSensor_IsRunning(void)
{
    return g_hallSensor.isRunning;
}

void HallSensor_Reset(void)
{
    /* Disable interrupts while resetting */
    DisableIRQ(HALL_FTM_IRQn);

    g_hallSensor.captureCount = 0;
    g_hallSensor.captureInterval = 0;
    g_hallSensor.rpm = 0.0f;
    g_hallSensor.frequency = 0.0f;
    g_hallSensor.speed_mps = 0.0f;
    g_hallSensor.isRunning = false;
    g_hallSensor.filterIndex = 0;
    memset(g_hallSensor.filterBuffer, 0, sizeof(g_hallSensor.filterBuffer));

    EnableIRQ(HALL_FTM_IRQn);

    PRINTF("Hall sensor reset\r\n");
}

void HallSensor_Update(uint32_t currentTime_ms)
{
    /* Process new capture if available */
    if (g_newCapture) {
        g_newCapture = false;
        g_hallSensor.lastUpdateTime = currentTime_ms;

        /* Apply filter and calculate values */
        uint32_t filteredInterval = HallSensor_FilterInterval(g_hallSensor.captureInterval);

        g_hallSensor.frequency = (float)g_hallSensor.timerFrequency / (float)filteredInterval;
        g_hallSensor.rpm = HallSensor_CalculateRPM(filteredInterval);
        g_hallSensor.speed_mps = HallSensor_CalculateSpeed(g_hallSensor.rpm);
    }

    /* Check for timeout (wheel stopped) */
    if (g_hallSensor.isRunning) {
        uint32_t elapsed = currentTime_ms - g_hallSensor.lastUpdateTime;

        if (elapsed > HALL_TIMEOUT_MS) {
            g_hallSensor.isRunning = false;
            g_hallSensor.rpm = 0.0f;
            g_hallSensor.frequency = 0.0f;
            g_hallSensor.speed_mps = 0.0f;
        }
    }
}

const HallSensor_t* HallSensor_GetData(void)
{
    return &g_hallSensor;
}

void HallSensor_PrintStatus(void)
{
    PRINTF("\r\n=== Hall Sensor Status ===\r\n");
    PRINTF("Running: %s\r\n", g_hallSensor.isRunning ? "YES" : "NO");
    PRINTF("Pulse count: %lu\r\n", g_hallSensor.captureCount);
    PRINTF("Last interval: %lu ticks\r\n", g_hallSensor.captureInterval);
    PRINTF("Frequency: %.2f Hz\r\n", g_hallSensor.frequency);
    PRINTF("RPM: %.1f\r\n", g_hallSensor.rpm);

    if (g_hallSensor.wheelCircumference_m > 0.0f) {
        PRINTF("Speed: %.2f m/s (%.2f km/h)\r\n",
               g_hallSensor.speed_mps,
               g_hallSensor.speed_mps * 3.6f);
    }
    PRINTF("==========================\r\n");
}
