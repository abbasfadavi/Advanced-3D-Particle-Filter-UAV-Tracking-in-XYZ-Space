Advanced 3D Particle Filter – UAV Tracking in XYZ Space

This project implements an Advanced 3D Particle Filter for UAV (Unmanned Aerial Vehicle) tracking in three-dimensional space (X–Y–Z).
The goal is to accurately estimate the UAV position using a large number of particles under noisy measurement conditions.

🧠 Overview

The algorithm is first modeled and simulated in MATLAB, providing a detailed visualization of the UAV’s motion, measurement noise, and estimated trajectory.
Then, the same algorithm is efficiently implemented in Vivado HLS, targeting FPGA hardware acceleration.

A comparison between MATLAB and HLS outputs verifies the correctness and performance equivalence of both implementations.

⚙️ Technical Highlights

Number of Particles: 1000

Clock Frequency: 20 MHz

Tracking Duration: 10 seconds (simulation)

FPGA Computation Time: 100 milliseconds

Timing Status: ✅ No timing violation

FPGA Resource Utilization
Resource	Usage
BRAM	64
DSP	78
FF	37K
LUT	21K
🧩 Features

Full 3D UAV tracking with position and measurement noise.

MATLAB simulation and visualization of particle distribution.

Vivado HLS hardware implementation of the particle filter core.

Cross-validation between MATLAB and HLS results.

Real-time capable with efficient resource usage.

📊 Simulation Example

Below is a 3D visualization of the UAV trajectory and estimated path from MATLAB simulation:

Green: True Path

Blue: Filter Estimate

Red Dots: Measurements

Cyan: Particles

Blue Star: True Position

💡 Future Improvements

Integration with real sensor data (e.g., GPS + IMU).

Adaptive particle resampling for dynamic motion models.

Optimization for multi-core FPGA acceleration.
![show](https://github.com/user-attachments/assets/e41e72e2-6476-466d-b816-64b6cb1fc37d)
