# AUV PID Enhancement

Simulation study that takes the kind of PID controller used on real low-cost ROVs/AUVs, shows where it breaks, and fixes each weakness one at a time with a measured before/after result.

This repo is separate from `auv-adaptive-mpc-gp` (learning-based MPC). That project is untouched; this one only reuses its plant model, reference trajectory and disturbance data.

## Idea

Real vehicles mostly run PID. It is simple and robust, but it has known weaknesses. For each one we add a fix and measure the change on the same plant and the same disturbance.

| # | PID weakness | Planned fix | Metric |
|---|--------------|-------------|--------|
| 1 | Hand-tuned gains | Automatic tuning (fminsearch) | Position RMSE |
| 2 | Integrator windup when thrusters saturate | Back-calculation anti-windup | Overshoot, recovery time |
| 3 | Reacts only after the error appears | GP disturbance feedforward | Position RMSE under current |
| 4 | Sensitive to model mismatch | Test at +/-20% parameters, then robustify | RMSE vs mismatch |
| 5 | No fault awareness | Fault detection from estimator residual | Detection delay, RMSE after fault |
| 6 | Noisy derivative term | Filtered derivative | Control effort, RMSE with noise |
| 7 | Results from one run only | Multiple seeds, mean and std | Std across seeds |

## Status

- [x] Plant, reference trajectory and real-data disturbance
- [x] Baseline PID and runner
- [ ] Fix 1 to 7 (each on its own branch / pull request)

## Run

In MATLAB, from this folder:

    run_pid_study

Expected baseline result: position RMSE of about 0.2988 m.

## Scenario and data

- 4-DOF AUV (BlueROV2-Heavy parameters), RK4, Ts = 0.2 s, Tf = 90 s.
- Reference: Lissajous-style path at z = -1 m.
- Disturbance: ocean surface current from MET Norway NORA3 (point 60.5N, 3.5E, 1-3 Mar 2017), converted to force with a scale of 15, time-compressed to the run length.

## Known limitations

- Simulation only, no hardware.
- The data is ocean-model output, not sensor readings.
- The force scale factor was chosen by us.
- The plant and the controller share the same model.
- Baseline gains are hand-tuned.
- `wrapToPi` needs a MATLAB toolbox that provides it.

## Requirements

MATLAB (tested on R2026a).
