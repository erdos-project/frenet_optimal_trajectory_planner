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

    // Cost attributes
    double cd = 0.0;                // lateral cost
    double cv = 0.0;                // longitudinal cost
    double cf = 0.0;                // final cost

    FrenetPath(FrenetHyperparameters *fot_hp_);
    bool to_global_path(CubicSpline2D* csp);
    bool is_valid_path(const vector<tuple<double, double>>& obstacles);
    bool is_collision(const vector<tuple<double, double>>& obstacles);
private:
    // Hyperparameters
    FrenetHyperparameters *fot_hp;
};

#endif //FRENET_OPTIMAL_TRAJECTORY_FRENETPATH_H
