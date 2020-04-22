from ctypes import c_double, c_int, POINTER, Structure, CDLL

_c_double_p = POINTER(c_double)

MAX_PATH_LENGTH = 100

class FrenetInitialConditions(Structure):
    _fields_ = [
        ("s0", c_double),
        ("c_speed", c_double),
        ("c_d", c_double),
        ("c_d_d", c_double),
        ("c_d_dd", c_double),
        ("target_speed", c_double),
        ("wx", _c_double_p),
        ("wy", _c_double_p),
        ("nw", c_int),
        ("o_llx", _c_double_p),
        ("o_lly", _c_double_p),
        ("o_urx", _c_double_p),
        ("o_ury", _c_double_p),
        ("no", c_int)
    ]
    
class FrenetReturnValues(Structure):
    _fields_ = [
        ("success", c_int),
        ("x_path", c_double * MAX_PATH_LENGTH),
        ("y_path", c_double * MAX_PATH_LENGTH),
        ("speeds", c_double * MAX_PATH_LENGTH),
        ("ix", c_double * MAX_PATH_LENGTH),
        ("iy", c_double * MAX_PATH_LENGTH),
        ("iyaw", c_double * MAX_PATH_LENGTH),
        ("d", c_double * MAX_PATH_LENGTH),
        ("s", c_double * MAX_PATH_LENGTH),
        ("speeds_x", c_double * MAX_PATH_LENGTH),
        ("speeds_y", c_double * MAX_PATH_LENGTH),
        ("params", c_double * MAX_PATH_LENGTH),
        ("costs", c_double * MAX_PATH_LENGTH)
    ]

class FrenetHyperparameters(Structure):
    _fields_ = [
        ("max_speed", c_double),
        ("max_accel", c_double),
        ("max_curvature", c_double),
        ("max_road_width_l", c_double),
        ("max_road_width_r", c_double),
        ("d_road_w", c_double),
        ("dt", c_double),
        ("maxt", c_double),
        ("mint", c_double),
        ("d_t_s", c_double),
        ("n_s_sample", c_double),
        ("obstacle_clearance", c_double),
        ("kd", c_double),
        ("kv", c_double),
        ("ka", c_double),
        ("kj", c_double),
        ("kt", c_double),
        ("ko", c_double),
        ("klat", c_double),
        ("klon", c_double),
    ]