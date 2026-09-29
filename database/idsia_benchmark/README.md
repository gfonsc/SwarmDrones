# IDSIA Nano-Drone System Identification Benchmark Dataset

This directory contains experimental quadrotor flight data sourced from the **IDSIA Nano-Drone System Identification Benchmark**. 

## Attribution
The data and specific physical parameters used in our data-driven control analysis strictly attribute their origin to:
**"A System Identification Benchmark for Nano-Quadrotors"**
By: the IDSIA Robotics Lab.
Repository: [https://github.com/idsia-robotics/nanodrone-sysid-benchmark](https://github.com/idsia-robotics/nanodrone-sysid-benchmark)

We maintain this explicit copy to isolate the offline training dependencies for the Data-enabled Predictive Control (DeePC) algorithm, specifically ensuring the algorithms always have access to highly persistently exciting tracking trajectories (such as `random` and `chirp`).

### Preprocessing and Conversion Note
Data is used inside the `load_benchmark_data.m` script to optimally construct valid **Square Hankel Matrices** and map raw PWM control inputs (rad/s) into physical moments and thrust forces using the experimentally determined $k_F = 3.72 \times 10^{-8}$ and $k_M = 7.73 \times 10^{-11}$ terms.
