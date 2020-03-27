#ifndef FRENET_OPTIMAL_TRAJECTORY_CONSTANTS_H
#define FRENET_OPTIMAL_TRAJECTORY_CONSTANTS_H
// Parameter
const double MAX_SPEED = 25.0;            // maximum speed [m/s]
const double MAX_ACCEL = 6.0;             // maximum acceleration [m/ss]
const double MAX_CURVATURE = 10;          // maximum curvature [1/m]
const double MAX_ROAD_WIDTH_L = 5.0;      // maximum road width to left [m]
const double MAX_ROAD_WIDTH_R = 1.0;      // maximum road width to left [m]
const double D_ROAD_W = 0.25;             // road width sampling length [m]
const double DT = 0.25;                   // time tick [s]
const double MAXT = 6;                    // max prediction time [m]
const double MINT = 2;                    // min prediction time [m]
const double D_T_S = 0.25;                // target speed sampling length [m/s]
const double N_S_SAMPLE = 2.0;            // sampling number of target speed
const double OBSTACLE_RADIUS = 3;         // obstacle radius [m]

// cost weights
const double KJ = 0.1;                    // jerk cost
const double KT = 0.1;                    // time cost
const double KD = 1.0;                    // end state cost
const double KLAT = 1.0;                  // lateral cost
const double KLON = 1.0;                  // longitudinal cost
#endif //FRENET_OPTIMAL_TRAJECTORY_CONSTANTS_H
