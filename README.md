# Frenet Optimal Trajectory
[FrenetOptimalTrajectory Demo](img/fot.gif)
## Overview
This repository contains a fast, C++ implementation of the Frenet Optimal
 Trajectory algorithm with a Python wrapper. It is used as one of the motion planning models in 
 [pylot](https://github.com/erdos-project/pylot), an [erdos](https://github.com/erdos-project) project.
 
Reference Papers:
- [Optimal Trajectory Generation for Dynamic Street Scenarios in a Frenet Frame](https://www.researchgate.net/profile/Moritz_Werling/publication/224156269_Optimal_Trajectory_Generation_for_Dynamic_Street_Scenarios_in_a_Frenet_Frame/links/54f749df0cf210398e9277af.pdf)
- [Optimal trajectory generation for dynamic street scenarios in a Frenet Frame](https://www.youtube.com/watch?v=Cj6tAQe7UCY)

## Setup
```
git clone https://github.com/fangedward/frenet-optimal-trajectory-planner.git
./build.sh
```

## Example Usage
There is a Python wrapper and C++ API. The Python wrapper is located in 
`FrenetOptimalTrajectory/fot_wrapper.py` and the C++ API is under 
`src/FrenetOptimalTrajectory/fot_wrapper.cpp`.
The following command will simulate a simple scenario to run the
 FrenetOptimalTrajectory planning algorithm.
```
python3 FrenetOptimalTrajectory/fot.py
```