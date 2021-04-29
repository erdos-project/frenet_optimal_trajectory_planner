#########################################
# Testing Scripts for Anytime Algorithm
#########################################
from fot_wrapper import _parse_hyperparameters, to_frenet_initial_conditions
import fot_planner as fp
import numpy as np
import time


#########################################
# CONFIG and Set Up
#########################################
num_threads = 2
conds = {
        's0':
        0,
        'target_speed':
        20,
        'wp': [[0, 0], [50, 0], [150, 0]],
        'obs': [[48, -2, 52, 2], [98, -4, 102, 2], [98, 6, 102, 10],
                [128, 2, 132, 6]],
        'pos': [0, 0],
        'vel': [0, 0],
    }  # paste output from debug log

initial_conditions = {
    'ps': conds['s0'],
    'target_speed': conds['target_speed'],
    'pos': np.array(conds['pos']),
    'vel': np.array(conds['vel']),
    'wp': np.array(conds['wp']),
    'obs': np.array(conds['obs'])
}

hyperparameters = {
    "max_speed": 25.0,
    "max_accel": 15.0,
    "max_curvature": 15.0,
    "max_road_width_l": 5.0,
    "max_road_width_r": 5.0,
    "d_road_w": 0.5,
    "dt": 0.2,
    "maxt": 5.0,
    "mint": 2.0,
    "d_t_s": 0.5,
    "n_s_sample": 2.0,
    "obstacle_clearance": 0.1,
    "kd": 1.0,
    "kv": 0.1,
    "ka": 0.1,
    "kj": 0.1,
    "kt": 0.1,
    "ko": 0.1,
    "klat": 1.0,
    "klon": 1.0,
    "num_threads": num_threads,  # set 0 to avoid using threaded algorithm
}

# static elements of planner
wx = initial_conditions['wp'][:, 0]
wy = initial_conditions['wp'][:, 1]
obs = np.array(conds['obs'])

# simulation config
sim_loop = 200
area = 40

ic, misc = to_frenet_initial_conditions(initial_conditions)
hp = _parse_hyperparameters(hyperparameters)

# Calling Anytime Planner
planner = fp.FotPlanner(ic, hp)
planner.async_plan()

# print(planner.get_path())
# # fot_wrapper.run_anytime_fot(initial_conditions, hyperparameters)
# # best_plan = planner.getBestPath()
# # for _ in range(100):
# #     new_plan = planner.getBestPath()
# #     assert new_plan.score() >= best_plan.score()
# #     if new_plan.score() > best_plan.score():
# #         best_plan = new_plan
# #     time.sleep(0.01) # sleep for 10 ms

planner.stop_plan()