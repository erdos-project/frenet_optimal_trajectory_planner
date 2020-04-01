#include "FrenetPath.h"
#include "utils.h"

#include <algorithm>
#include <iostream>

using namespace std;

FrenetPath::FrenetPath(FrenetHyperparameters *fot_hp_) {
    fot_hp = fot_hp_;
}

// Convert the frenet path to global path in terms of x, y, yaw, velocity
bool FrenetPath::to_global_path(CubicSpline2D* csp) {
    double ix, iy, iyaw, di, fx, fy, dx, dy;
    // calc global positions
    for (int i = 0; i < s.size(); i++) {
        ix = csp->calc_x(s[i]);
        iy = csp->calc_y(s[i]);
        if (isnan(ix) || isnan(iy)) break;

        iyaw = csp->calc_yaw(s[i]);
        di = d[i];
        fx = ix + di * cos(iyaw + M_PI / 2.0);
        fy = iy + di * sin(iyaw + M_PI / 2.0);
        x.push_back(fx);
        y.push_back(fy);
    }

    // not enough points to construct a valid path
    if (x.size() <= 1) {
        return false;
    }

    // calc yaw and ds
    for (int i = 0; i < x.size() - 1; i++) {
        dx = x[i+1] - x[i];
        dy = y[i+1] - y[i];
        yaw.push_back(atan2(dy, dx));
        ds.push_back(hypot(dx, dy));
    }
    yaw.push_back(yaw.back());
    ds.push_back(ds.back());

    // calc curvature
    for (int i = 0; i < yaw.size() - 1; i++) {
        c.push_back((yaw[i+1] - yaw[i]) / ds[i]);
    }

    return true;
}

// Validate the calculated frenet paths against threshold speed, acceleration,
// curvature and collision checks
bool FrenetPath::is_valid_path(const vector<tuple<double, double>>& obstacles) {
    if (any_of(s_d.begin(), s_d.end(),
            [this](int i){return abs(i) > fot_hp->max_speed;})) {
        return false;
    }
    // max accel check
    else if (any_of(s_dd.begin(), s_dd.end(),
            [this](int i){return abs(i) > fot_hp->max_accel;})) {
        return false;
    }
    // max curvature check
    else if (any_of(c.begin(), c.end(),
            [this](int i){return abs(i) > fot_hp->max_curvature;})) {
        return false;
    }
    // collision check
    else if (is_collision(obstacles)) {
        return false;
    }
    else {
        return true;
    }
}

bool FrenetPath::is_collision(const vector<tuple<double, double>>& obstacles) {
    // no obstacles
    if (obstacles.empty()) {
        return false;
    }

    // iterate over all obstacles
    for (tuple<double, double> obstacle : obstacles) {
        // calculate distance to each point in path
        for (int i = 0; i < x.size(); i++) {
            // exit if within OBSTACLE_RADIUS
            double xd = x[i] - get<0>(obstacle);
            double yd = y[i] - get<1>(obstacle);
            if (norm(xd, yd) <= fot_hp->obstacle_radius) {
                return true;
            }
        }
    }

    // no collisions
    return false;
}