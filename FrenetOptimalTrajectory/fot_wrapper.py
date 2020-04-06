import numpy as np
import os

from ctypes import c_double, c_int, POINTER, Structure, CDLL

try:
    from py_cpp_struct import FrenetInitialConditions, FrenetHyperparameters
except:
    from pylot.planning.frenet_optimal_trajectory\
        .frenet_optimal_trajectory_planner.FrenetOptimalTrajectory\
        .py_cpp_struct import FrenetInitialConditions, FrenetHyperparameters

try:
    cdll = CDLL("build/libFrenetOptimalTrajectory.so")
except:
    cdll = CDLL(
        "{}/pylot/planning/frenet_optimal_trajectory/frenet_optimal_trajectory_planner/"
        "build/libFrenetOptimalTrajectory.so".format(os.getcwd())
    )

_c_double_p = POINTER(c_double)

# func / return type declarations for C++ run_fot
_run_fot = cdll.run_fot
_run_fot.argtypes = (
    POINTER(FrenetInitialConditions),
    POINTER(FrenetHyperparameters),
    _c_double_p,
    _c_double_p,
    _c_double_p,
    _c_double_p,
    _c_double_p,
    _c_double_p,
    _c_double_p,
    _c_double_p,
)
_run_fot.restype = c_int

# func / return type declarations for C++ to_frenet_initial_conditions
_to_frenet_initial_conditions = cdll.to_frenet_initial_conditions
_to_frenet_initial_conditions.restype = None
_to_frenet_initial_conditions.argtypes = (
    c_double,
    c_double,
    c_double,
    c_double,
    c_double,
    c_double,
    _c_double_p,
    _c_double_p,
    c_int,
    _c_double_p
)

def _parse_hyperparameters(hp):
    return FrenetHyperparameters(
        hp["max_speed"],
        hp["max_accel"],
        hp["max_curvature"],
        hp["max_road_width_l"],
        hp["max_road_width_r"],
        hp["d_road_w"],
        hp["dt"],
        hp["maxt"],
        hp["mint"],
        hp["d_t_s"],
        hp["n_s_sample"],
        hp["obstacle_radius"],
        hp["kj"],
        hp["kt"],
        hp["kd"],
        hp["klat"],
        hp["klon"]
    )

def run_fot(initial_conditions, hyperparameters):
    """ Return the frenet optimal trajectory given initial conditions in
    cartesian space.

    Args:
        initial_conditions (dict): dict containing the following items
            ps (float): previous longitudinal position
            target_speed (float): target speed [m/s]
            pos (np.ndarray([float, float])): initial position in global coord
            vel (np.ndarray([float, float])): initial velocity [m/s]
            wp (np.ndarray([float, float])): list of global waypoints
            obs (np.ndarray([float, float])): list of global obstacle centers

        hyperparameters (dict): a dict of optional hyperparameters
            max_speed (float): maximum speed [m/s]
            max_accel (float): maximum acceleration [m/s^2]
            max_curvature (float): maximum curvature [1/m]
            max_road_width_l (float): maximum road width to the left [m]
            max_road_width_r (float): maximum road width to the right [m]
            d_road_w (float): road width sampling discretization [m]
            dt (float): time sampling discretization [s]
            maxt (float): max prediction horizon [s]
            mint (float): min prediction horizon [s]
            d_t_s (float): target speed sampling discretization [m/s]
            n_s_sample (float): sampling number of target speed
            obstacle_radius (float): obstacle radius [m]
            kj (float): jerk cost
            kt (float): time cost
            kd (float): end state cost
            klat (float): lateral cost
            klon (float): longitudinal cost

    Returns:
        result_x (np.ndarray(float)): x positions of fot, if it exists
        result_y (np.ndarray(float)): y positions of fot, if it exists
        speeds (np.ndarray(float)): speeds of fot, if it exists
        speeds_x (np.ndarray(float)): x speeds of fot, if it exists
        speeds_y (np.ndarray(float)): y speeds of fot, if it exists
        params (dict): next frenet coordinates, if they exist
        success (bool): whether a fot was found or not
    """
    # parse initial conditions and convert to frenet coordinates
    fot_initial_conditions, misc = to_frenet_initial_conditions(
        initial_conditions
    )

    # parse hyper parameters
    fot_hp = _parse_hyperparameters(hyperparameters)

    # create storage variables
    result_x = np.zeros(100)
    result_y = np.zeros(100)
    speeds = np.zeros(100)
    ix = np.zeros(100)
    iy = np.zeros(100)
    iyaw = np.zeros(100)
    d = np.zeros(100)
    s = np.zeros(100)
    params = np.zeros(5)

    # run the planner
    success = _run_fot(
        fot_initial_conditions,
        fot_hp,
        result_x.ctypes.data_as(_c_double_p),
        result_y.ctypes.data_as(_c_double_p),
        speeds.ctypes.data_as(_c_double_p),
        ix.ctypes.data_as(_c_double_p),
        iy.ctypes.data_as(_c_double_p),
        iyaw.ctypes.data_as(_c_double_p),
        d.ctypes.data_as(_c_double_p),
        s.ctypes.data_as(_c_double_p),
        params.ctypes.data_as(_c_double_p)
    )

    # remove values after last calculated waypoint
    ind = -1
    if success:
        ind = np.where(np.isnan(result_x))[0][0]

    return result_x[:ind], result_y[:ind], speeds[:ind], \
           ix[:ind], iy[:ind], iyaw[:ind], d[:ind], s[:ind], misc, success


def to_frenet_initial_conditions(initial_conditions):
    """ Convert the cartesian initial conditions into frenet initial conditions.

    Args:
        initial_conditions (dict): dictionary containing
            ps (float): previous longitudinal position
            target_speed (float): target speed [m/s]
            pos (np.ndarray([float, float])): initial position in global coord
            vel (np.ndarray([float, float])): initial velocity [m/s]
            wp (np.ndarray([float, float])): list of global waypoints
            obs (np.ndarray([float, float])): list of global obstacle centers

    Returns:
        FrenetInitialConditions, dictionary for debugging
    """
    # parse the initial conditions
    ps = initial_conditions['ps']
    pos = initial_conditions['pos']
    vel = initial_conditions['vel']
    wp = initial_conditions['wp']
    obs = initial_conditions['obs']
    target_speed = initial_conditions['target_speed']
    if obs.shape[0] == 0:
        obs = np.empty((0, 2))
    x = pos[0].item()
    y = pos[1].item()
    vx = vel[0].item()
    vy = vel[1].item()
    wx = wp[:, 0].astype(np.float64)
    wy = wp[:, 1].astype(np.float64)
    ox = obs[:, 0].astype(np.float64)
    oy = obs[:, 1].astype(np.float64)
    forward_speed = np.hypot(vx, vy).item()

    # construct return array and convert initial conditions
    misc = np.zeros(5)
    _to_frenet_initial_conditions(
        c_double(ps),
        c_double(x),
        c_double(y),
        c_double(vx),
        c_double(vy),
        c_double(forward_speed),
        wx.ctypes.data_as(_c_double_p),
        wy.ctypes.data_as(_c_double_p),
        c_int(len(wx)),
        misc.ctypes.data_as(_c_double_p)
    )

    # return the FrenetInitialConditions structure
    return FrenetInitialConditions(
        misc[0], # c_s
        misc[1], # c_speed
        misc[2], # c_d
        misc[3], # c_d_d
        misc[4], # c_d_dd
        target_speed, # target speed
        wx.ctypes.data_as(_c_double_p), # waypoints x position
        wy.ctypes.data_as(_c_double_p), # waypoints y position
        len(wx),
        ox.ctypes.data_as(_c_double_p), # obstacles x position
        oy.ctypes.data_as(_c_double_p), # obstacles y position
        len(ox),
    ), misc
