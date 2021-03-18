// Author: Edward Fang
// Email: edward.fang@berkeley.edu
//
// This code is adapted from
// https://github.com/AtsushiSakai/PythonRobotics/tree/
// master/PathPlanning/FrenetOptimalTrajectory.
// Its author is Atsushi Sakai.
//
// Reference Papers:
// - [Optimal Trajectory Generation for Dynamic Street Scenarios in a Frenet
// Frame]
// (https://www.researchgate.net/profile/Moritz_Werling/publication/224156269_Optimal_Trajectory_Generation_for_Dynamic_Street_Scenarios_in_a_Frenet_Frame/links/54f749df0cf210398e9277af.pdf)
// - [Optimal trajectory generation for dynamic street scenarios in a Frenet
// Frame] (https://www.youtube.com/watch?v=Cj6tAQe7UCY)

#ifndef FRENET_OPTIMAL_TRAJECTORY_FRENET_OPTIMAL_TRAJECTORY_H
#define FRENET_OPTIMAL_TRAJECTORY_FRENET_OPTIMAL_TRAJECTORY_H

#include "CubicSpline2D.h"
#include "FrenetPath.h"
#include "Obstacle.h"
#include "py_cpp_struct.h"

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <mutex>
#include <vector>

using namespace std;
using namespace Eigen;

class FrenetOptimalTrajectory {
public:
    FrenetOptimalTrajectory(FrenetInitialConditions *fot_ic_,
                            FrenetHyperparameters *fot_hp_);
    ~FrenetOptimalTrajectory();
    FrenetPath *getBestPath();
    void setObstacles();
    void addObstacle(Vector2f first_point, Vector2f second_point);

private:
    FrenetInitialConditions *fot_ic;
    FrenetHyperparameters *fot_hp;
    mutex *mu;
    FrenetPath *best_frenet_path;
    CubicSpline2D *csp;
    vector<Obstacle *> obstacles;
    vector<double> x, y;
    vector<FrenetPath *> frenet_paths;
    void calc_frenet_paths(int start_di_index, int end_di_index,
                           bool multithreaded);
    void threaded_calc_all_frenet_paths();
};

#endif // FRENET_OPTIMAL_TRAJECTORY_FRENET_OPTIMAL_TRAJECTORY_H
