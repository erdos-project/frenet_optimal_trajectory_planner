from ctypes import c_double, c_int, POINTER, Structure, CDLL

_c_double_p = POINTER(c_double)

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
        ("ox", _c_double_p),
        ("oy", _c_double_p),
        ("no", c_int)
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
        ("obstacle_radius", c_double),
        ("kj", c_double),
        ("kt", c_double),
        ("kd", c_double),
        ("ko", c_double),
        ("klat", c_double),
        ("klon", c_double),
    ]