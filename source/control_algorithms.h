#ifndef CONTROL_ALGORITHMS_H_
#define CONTROL_ALGORITHMS_H_

#include "common_types.h"
#include "config.h"
#include "globals.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

/* =========================================================================
 * Existing PID interface — unchanged
 * ========================================================================= */

uint16_t MapControlToDutyCycle(float control);
float    CalculateError(float x_center);
float    CalculateCombinedError(float x_center1, float x_center2);
void     CalculateLaneCenter2(VectorType v1, VectorType v2,
                               float *x_1, float *x_2,
                               float *y_1, float *y_2);
void     CalculateLaneCenter(VectorType v1, VectorType v2,
                              float *x_center, float *y_center);

float          PID_Control(PIDController *pid, float error);
MotorCommand_t ProcessVectorsPID(VectorType v1, VectorType v2);
uint16_t       CalculateSpeedFromDuty(uint16_t duty_cycle);
uint16_t CalculateSpeedFromControl(float control);

/* =========================================================================
 * LQR steering control
 * =========================================================================
 *
 * Model: discrete kinematic bicycle, 3 states, 1 input
 *
 *   States  x = [e_lat (m), theta_e (rad), psi_dot (rad/s)]
 *   Input   u = delta (rad)
 *
 * Sign convention:
 *   e_lat   > 0  →  car is LEFT of lane centre
 *   theta_e > 0  →  car heading is CCW (left) of lane direction
 *   psi_dot > 0  →  yaw rate CCW (turning left)
 *   delta   > 0  →  steer left
 *
 * Control law:  delta = -K * x
 *
 * The gain vector K is computed offline by tools/lqr_tuning.py and
 * hardcoded in control_algorithms.c.  No solver runs on the MCU.
 *
 * Camera-to-physical mapping used in LQRState_Update():
 *   e_lat   = (x_center - CENTER_X) * LQR_PIX2M
 *   theta_e = CalculateLaneHeading(v1, v2)
 *   psi_dot = (theta_e - prev_theta_e) / LQR_DT
 *
 * If the car steers in the wrong direction on first test, negate delta
 * inside LQR_SteerControl() (see SIGN_NOTE comment there).
 * ========================================================================= */

#define LQR_NUM_STATES    3          /* number of state variables             */
#define LQR_DT            0.558f    /* control timestep [s] — measured loop period */
#define LQR_PIX2M         0.006f    /* pixel → metres scale factor            */
                                     /* calibrate: measure track width in m   */
                                     /* divide by its width in pixels          */
#define LQR_MAX_STEER     0.524f    /* maximum steering angle [rad] ≈ 30 °   */

/** LQR controller state — one instance per controller, zero-initialise. */
typedef struct {
    float e_lat;         /**< lateral error from lane centre [m]             */
    float theta_e;       /**< heading error: car vs lane direction [rad]      */
    float psi_dot;       /**< estimated yaw rate [rad/s]                      */
    float prev_theta_e;  /**< theta_e from the previous step (derivative)     */
    bool  initialized;   /**< false until first LQRState_Update() call;       */
                         /**< prevents spurious psi_dot on first cycle        */
} LQRState_t;

/**
 * @brief Estimate the lane heading angle from two lane vectors.
 *
 * Computes atan2(dx, dy) for each valid vector (invalid ones have
 * m_index == 255) and returns the average.  Result is in [-pi/2, pi/2].
 *
 * @param v1  Left lane vector  (after PreprocessVectors normalisation)
 * @param v2  Right lane vector (after PreprocessVectors normalisation)
 * @return    Lane heading angle [rad]; 0 = straight ahead
 */
float    CalculateLaneHeading(VectorType v1, VectorType v2);

/**
 * @brief Update LQR state from the latest camera vectors.
 *
 * Must be called once per control cycle, before LQR_SteerControl().
 *
 * @param state  Controller state (in/out)
 * @param v1     Left lane vector
 * @param v2     Right lane vector
 */
void     LQRState_Update(LQRState_t *state, VectorType v1, VectorType v2);

/**
 * @brief Compute LQR steering PWM duty-cycle.
 *
 * Performs the matrix-vector multiply  u = -K * x  and maps the result
 * to a discrete PWM step {500, 600, 700, 800, 900}.
 *
 * @param state  Current LQR state (updated by LQRState_Update)
 * @return       Steering PWM duty × 100 (e.g. 700 = 7.00 % = neutral)
 */
uint16_t LQR_SteerControl(const LQRState_t *state);

/** Update the LQR gain vector at runtime (values in natural units, not ×1000). */
void LQR_SetGains(float k_e_lat, float k_theta_e, float k_psi_dot);
/** Read the current LQR gain vector. */
void LQR_GetGains(float *k_e_lat, float *k_theta_e, float *k_psi_dot);

/* =========================================================================
 * PD speed control
 * =========================================================================
 *
 * Tracks a target RPM using Hall sensor feedback (HallSensor_GetRPM()).
 * No integral term — avoids windup against the stall floor and is simpler
 * to tune.  Steady-state error is acceptable for racing.
 *
 * Feedforward: PWM_MIN_SPEED × 100 (the PWM that yields ~PD_TARGET_RPM on
 * a flat surface with a charged battery).  PD correction adds on top.
 *
 * Output is clamped to [PWM_MIN_SPEED×100, PWM_MAX_SPEED×100].
 * ========================================================================= */

typedef struct {
    float Kp;           /**< proportional gain [PWM_unit / RPM]              */
    float Kd;           /**< derivative gain   [PWM_unit / (RPM / LQR_DT)]  */
    float prev_error;   /**< error from previous cycle for finite-difference  */
    float output_min;   /**< lower PWM clamp (stall floor, percentage × 100) */
    float output_max;   /**< upper PWM clamp (percentage × 100)              */
    bool  initialized;  /**< false until the first call (skips first Kd term) */
} PDController_t;

/**
 * @brief Run one PD speed control step.
 *
 * Call once per LQR cycle after SpeedSteeringCoupling() has reduced the
 * target RPM for cornering.
 *
 * @param pd          Controller state (in/out)
 * @param target_rpm  Desired wheel speed after coupling reduction [RPM]
 * @param current_rpm Measured wheel speed from HallSensor_GetRPM() [RPM]
 * @return            Drive motor PWM duty × 100
 */
uint16_t PD_SpeedControl(PDController_t *pd, float target_rpm, float current_rpm);

/* =========================================================================
 * Speed–steering coupling
 * =========================================================================
 *
 * Reduces target RPM proportionally when the steering demand is large,
 * improving cornering stability and preventing traction loss.
 * ========================================================================= */

/**
 * @brief Apply speed-steering coupling.
 *
 * @param target_rpm   Uncoupled target RPM
 * @param steer_duty   Current steering duty × 100 (e.g. 700 = neutral)
 * @return             Adjusted target RPM (≥ target_rpm × SPEED_STEER_MIN_SCALE)
 */
float    SpeedSteeringCoupling(float target_rpm, uint16_t steer_duty);

#endif /* CONTROL_ALGORITHMS_H_ */
