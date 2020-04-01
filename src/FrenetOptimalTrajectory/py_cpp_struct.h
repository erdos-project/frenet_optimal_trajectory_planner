#ifndef FRENETOPTIMALTRAJECTORY_PY_CPP_STRUCT_H
#define FRENETOPTIMALTRAJECTORY_PY_CPP_STRUCT_H
struct FrenetInitialConditions {
    double s0;
    double c_speed;
    double c_d;
    double c_d_d;
    double c_d_dd;
    double target_speed;
    double *wx;
    double *wy;
    int nw;
    double *ox;
    double *oy;
    int no;
};

struct FrenetHyperparameters {
    double max_speed;
    double max_accel;
    double max_curvature;
    double max_road_width_l;
    double max_road_width_r;
    double d_road_w;
    double dt;
    double maxt;
    double mint;
    double d_t_s;
    double n_s_sample;
    double obstacle_radius;
    double kj;
    double kt;
    double kd;
    double ko;
    double klat;
    double klon;
};
#endif //FRENETOPTIMALTRAJECTORY_PY_CPP_STRUCT_H
