/*
 * hall_sensor.c
 *
 * Hall sensor driver — RPM fed externally by ESP8266 via UART4.
 * uart_handler.c calls HallSensor_SetExternalRPM() on each "H:<rpm>" packet.
 * HallSensor_Update() (called from SysTick, 1 ms) handles the timeout only.
 */

#include "hall_sensor.h"

#include "fsl_debug_console.h"
#include "globals.h"

#include <string.h>

static HallSensor_t g_hallSensor;

static float CalculateSpeed(float rpm)
{
    if (g_hallSensor.wheelCircumference_m <= 0.0f)
        return 0.0f;
    return (rpm * g_hallSensor.wheelCircumference_m) / 60.0f * 3.6f;
}

void HallSensor_Init(void)
{
    memset(&g_hallSensor, 0, sizeof(HallSensor_t));
    PRINTF("Hall Sensor init: RPM via ESP8266 UART bridge\r\n");
}

void HallSensor_SetWheelCircumference(float circumference_m)
{
    g_hallSensor.wheelCircumference_m = circumference_m;
}

void HallSensor_SetExternalRPM(float rpm)
{
    if (rpm <= 0.0f)
    {
        g_hallSensor.rpm            = 0.0f;
        g_hallSensor.frequency      = 0.0f;
        g_hallSensor.speed_kmh      = 0.0f;
        g_hallSensor.isRunning      = false;
        g_hallSensor.lastUpdateTime = g_systemTime_ms;
        return;
    }

    /* Normalize to expected range [100, 500] RPM.
     * ESP8266 occasionally measures 10x too high (WiFi blocking causes a
     * very short captured interval). Scale in whichever direction brings
     * the reading into range; leave it alone if already there. */
    if      (rpm * 10.0f >= 100.0f && rpm * 10.0f <= 500.0f) rpm *= 10.0f;
    else if (rpm / 10.0f >= 100.0f && rpm / 10.0f <= 500.0f) rpm /= 10.0f;
    else return; /* unrecoverable — outside range in both directions */

    /* Plausibility filter: compare against last_valid_rpm so a spurious H:0
     * (or timeout) can't reset the baseline and let a bad reading slip through. */
    if (g_hallSensor.last_valid_rpm > 0.0f)
    {
        float ratio = rpm / g_hallSensor.last_valid_rpm;
        if (ratio < 0.5f || ratio > 2.0f)
            return;
    }

    g_hallSensor.last_valid_rpm = rpm;
    g_hallSensor.rpm            = rpm;
    g_hallSensor.frequency      = rpm / 60.0f;
    g_hallSensor.speed_kmh      = CalculateSpeed(rpm);
    g_hallSensor.isRunning      = true;
    g_hallSensor.lastUpdateTime = g_systemTime_ms;
}

void HallSensor_Update(uint32_t currentTime_ms)
{
    if (g_hallSensor.isRunning &&
        (currentTime_ms - g_hallSensor.lastUpdateTime) > HALL_TIMEOUT_MS)
    {
        g_hallSensor.isRunning = false;
        g_hallSensor.rpm       = 0.0f;
        g_hallSensor.frequency = 0.0f;
        g_hallSensor.speed_kmh = 0.0f;
    }
}

float HallSensor_GetRPM(void)       { return g_hallSensor.rpm; }
float HallSensor_GetFrequency(void) { return g_hallSensor.frequency; }
float HallSensor_GetSpeed_kmh(void) { return g_hallSensor.speed_kmh; }
bool  HallSensor_IsRunning(void)    { return g_hallSensor.isRunning; }

void HallSensor_Reset(void)
{
    g_hallSensor.rpm       = 0.0f;
    g_hallSensor.frequency = 0.0f;
    g_hallSensor.speed_kmh = 0.0f;
    g_hallSensor.isRunning = false;
}

const HallSensor_t *HallSensor_GetData(void) { return &g_hallSensor; }
