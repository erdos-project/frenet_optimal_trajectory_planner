#ifndef FRENET_OPTIMAL_TRAJECTORY_FRENETPATH_H
#define FRENET_OPTIMAL_TRAJECTORY_FRENETPATH_H

#include "py_cpp_struct.h"
#include "CubicSpline2D.h"

#include <vector>
#include <tuple>

using namespace std;

class FrenetPath {
public:
    // Frenet attributes
    vector<double> t;          // time
    vector<double> d;          // lateral offset
    vector<double> d_d;        // lateral speed
    vector<double> d_dd;       // lateral acceleration
    vector<double> d_ddd;      // lateral jerk
    vector<double> s;          // s position along spline
    vector<double> s_d;        // s speed
    vector<double> s_dd;       // s acceleration
    vector<double> s_ddd;      // s jerk

    // Euclidean attributes
    vector<double> x;          // x position
    vector<double> y;          // y position
    vector<double> yaw;        // yaw in rad
    vector<double> ds;         // speed
    vector<double> c;          // curvature

    // Debug
    vector<double> ix;
    vector<double> iy;
    vector<double> iyaw;

    // Cost attributes
    // lateral costs
    double c_lateral_deviation = 0.0;
    double c_lateral_velocity = 0.0;
    double c_lateral_acceleration = 0.0;
    double c_lateral_jerk = 0.0;
    double c_lateral = 0.0;

    // longitudinal costs
    double c_longitudinal_acceleration = 0.0;
    double c_longitudinal_jerk = 0.0;
    double c_time_taken = 0.0;
    double c_end_speed_deviation = 0.0;
    double c_longitudinal = 0.0;

    // obstacle costs
    double c_inv_dist_to_obstacles = 0.0;

    // final cost
    double cf = 0.0;

    FrenetPath(FrenetHyperparameters *fot_hp_);
    bool to_global_path(CubicSpline2D* csp);
    bool is_valid_path(const vector<tuple<double, double>>& obstacles);
    bool is_collision(const vector<tuple<double, double>>& obstacles);
    double inverse_distance_to_obstacles(
        const vector<tuple<double, double>>& obstacles);

private:
    // Hyperparameters
    FrenetHyperparameters *fot_hp;
};

#endif //FRENET_OPTIMAL_TRAJECTORY_FRENETPATH_H
