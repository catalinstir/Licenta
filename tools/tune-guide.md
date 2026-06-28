"Steers late" means k_theta_e is too low — the heading error gain is what makes the car anticipate curves early (it reacts to the lane angle
  before lateral drift builds up). Try doubling it as a first step: 0.268 → 0.54. Rerun the tuner with --q2 <higher> or just edit the gain
  directly.

  ---
  LQR Tune Guide

  Each gain has a distinct job and a distinct failure mode:

  ┌───────────┬─────────┬──────────────────────────────────┬─────────────────────────────────────┬────────────────────────────────────────┐
  │   Gain    │ Current │             Controls             │               Too low               │                Too high                │
  ├───────────┼─────────┼──────────────────────────────────┼─────────────────────────────────────┼────────────────────────────────────────┤
  │ k_e_lat   │ 0.032   │ Lateral centering                │ Drifts off-center before correcting │ Overshoots center, snakes on straights │
  ├───────────┼─────────┼──────────────────────────────────┼─────────────────────────────────────┼────────────────────────────────────────┤
  │ k_theta_e │ 0.268   │ Heading alignment — anticipatory │ Steers late into curves             │ Overcorrects, oscillates in curves     │
  ├───────────┼─────────┼──────────────────────────────────┼─────────────────────────────────────┼────────────────────────────────────────┤
  │ k_psi_dot │ 0.100   │ Yaw rate damping                 │ Oscillates / snakes                 │ Sluggish turn initiation               │
  └───────────┴─────────┴──────────────────────────────────┴─────────────────────────────────────┴────────────────────────────────────────┘

  Tuning process:

  1. Fix direction first (done — delta negated)
  2. Tune k_theta_e on a curve — increase until the car starts turning early enough, back off if it oscillates in the curve
  3. Tune k_e_lat on a straight — increase until the car re-centres quickly after a disturbance, back off if it snakes
  4. Tune k_psi_dot last — increase if oscillations survive steps 2-3, decrease if turns feel sluggish

  Practical rules:
  - Change one gain at a time, ±50% per step
  - k_theta_e and k_psi_dot interact strongly — if raising k_theta_e causes oscillation, raise k_psi_dot to damp it
  - The 558 ms loop period is long, so the system is already aggressive (eigenvalues near zero) — you have more headroom to increase gains than a
  fast-loop controller would

