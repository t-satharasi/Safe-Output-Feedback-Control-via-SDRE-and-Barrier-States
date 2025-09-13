# Safe-Output-Feedback-Control-via-SDRE-and-Barrier-States
This paper presents a safe output-feedback control framework for partially observable nonlinear control-affine systems, leveraging state-dependent Riccati equations (SDREs) in conjunction with robust barrier states (BaS). The developed approach integrates an SDRE-based observer. The developed Code is presented in MATLAB by Mathworks Inc, and was created on MATLAB 2021a. THe Same Code was used for Simulations and Experiments for the Paper.

## Code Structure

- SDREBasedControl.m – Baseline SDRE controller without barrier states.
- SDREBasedWithBarrier.m – Extended controller with barrier functions and robust terms.
- main.m (driver script) – Sets up system dynamics, obstacle parameters, and runs simulations.
- data/ folder – Stores simulation results. Subfolders are automatically created
  - NoBarrier/ 
  - BarrierOnly/
  - BarrierAndRobust/
Each subfolder contains .dat files with simulation results (state trajectories, estimates, errors, and parameter estimates).

## Extending the Code
- System dynamics: Update fun_A(x) and fun_G(x) in main.m for new models.
- Obstacle geometry: Adjust obs_center and obs_radius (currently circular obstacles).
- Controller/observer tuning: Modify Q, R, M, alpha, gamma, K, etc within the Controller
- Add new scenarios: Extend the cases list in SDREBasedWithBarrier.m.
