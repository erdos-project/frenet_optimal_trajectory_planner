#ifndef FRENETOPTIMALTRAJECTORY_PY_CPP_STRUCT_H
#define FRENETOPTIMALTRAJECTORY_PY_CPP_STRUCT_H
const int MAX_PATH_LENGTH = 100;

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
    double *o_llx;
    double *o_lly;
    double *o_urx;
    double *o_ury;
    int no;
};

struct FrenetReturnValues {
    int success;
    double x_path[MAX_PATH_LENGTH];
    double y_path[MAX_PATH_LENGTH];
    double speeds[MAX_PATH_LENGTH];
    double ix[MAX_PATH_LENGTH];
    double iy[MAX_PATH_LENGTH];
    double iyaw[MAX_PATH_LENGTH];
    double d[MAX_PATH_LENGTH];
    double s[MAX_PATH_LENGTH];
    double speeds_x[MAX_PATH_LENGTH];
    double speeds_y[MAX_PATH_LENGTH];
    double params[MAX_PATH_LENGTH];
    double costs[MAX_PATH_LENGTH];
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
    double kd;
    double kv;
    double ka;
    double kj;
    double kt;
    double ko;
    double klat;
    double klon;
};
#endif //FRENETOPTIMALTRAJECTORY_PY_CPP_STRUCT_H
