 During a test run — on the laptop, before flashing:
  # Terminal 1: run the car in autonomous mode as usual
  # Terminal 2: start the logger
  python tools/data_logger.py --label lqr
  # Ctrl+C when the lap is done

  After collecting runs:
  # Compare PID vs LQR vs LQR+coupling (all in one command)
  python tools/plot_results.py tools/data/pid_*.csv tools/data/lqr_*.csv --labels "PID" "LQR"

  # Save PNGs for thesis
  python tools/plot_results.py tools/data/*.csv --save tools/data/plots/

  The logger also prints a live summary table per control cycle so you can watch it in real time. The plotter prints a stats table (mean/std/max of
  e_lat, heading error std) which is directly usable as a thesis comparison table.

  For comparing PID as baseline you'll need one run with PID re-enabled in STATE_CONTROL — just swap back the ProcessVectorsPID call temporarily,
  label that run pid, then switch back to LQR.
