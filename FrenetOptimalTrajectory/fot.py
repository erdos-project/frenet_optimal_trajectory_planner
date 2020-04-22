import fot_wrapper
import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patch


def main():
    conds = {
        's0': 0,
        'target_speed': 20,
        'wx': [0, 50, 150],
        'wy': [0, 0, 0],
        'obstacle_list': [[48, -2, 52, 2], [98, -2, 102, 2]],
        'x': 0,
        'y': 0,
        'vx': 0,
        'vy': 0,
    }  # paste output from debug log

    initial_conditions = {
        'ps': conds['s0'],
        'target_speed': conds['target_speed'],
        'pos': np.array([conds['x'], conds['y']]),
        'vel': np.array([conds['vx'], conds['vy']]),
        'wp': np.array([conds['wx'], conds['wy']]).T,
        'obs': np.array(conds['obstacle_list'])
    }

    hyperparameters = {
        "max_speed": 25.0,
        "max_accel": 10.0,
        "max_curvature": 3.0,
        "max_road_width_l": 6.0,
        "max_road_width_r": 1.0,
        "d_road_w": 0.25,
        "dt": 0.25,
        "maxt": 8.0,
        "mint": 2.0,
        "d_t_s": 0.5,
        "n_s_sample": 2.0,
        "obstacle_clearance": 0.5,
        "kd": 1.0,
        "kv": 0.1,
        "ka": 0.1,
        "kj": 0.1,
        "kt": 0.1,
        "ko": 0.1,
        "klat": 1.0,
        "klon": 1.0
    }

    # static elements of planner
    wx = np.array(conds['wx'])
    wy = np.array(conds['wy'])
    obs = np.array(conds['obstacle_list'])

    # simulation config
    show_animation = True
    sim_loop = 200
    area = 40
    total_time = 0
    for i in range(sim_loop):
        # run FOT and keep time
        print("Iteration: {}".format(i))
        start_time = time.time()
        result_x, result_y, speeds, ix, iy, iyaw, d, s, speeds_x, \
            speeds_y, misc, costs, success = \
            fot_wrapper.run_fot(initial_conditions, hyperparameters)
        end_time = time.time() - start_time
        print("Time taken: {}".format(end_time))
        total_time += end_time

        # reconstruct initial_conditions
        if success:
            initial_conditions['pos'] = np.array([result_x[1], result_y[1]])
            initial_conditions['vel'] = np.array([speeds_x[1], speeds_y[1]])
            initial_conditions['ps'] = misc['s']
            print(costs)
        else:
            print("Failed unexpectedly")
            break

        # break if near goal
        if np.hypot(result_x[1] - wx[-1], result_y[1] - wy[-1]) <= 3.0:
            print("Goal")
            break

        if show_animation:  # pragma: no cover
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None]
            )
            plt.plot(wx, wy)
            if obs.shape[0] == 0:
                obs = np.empty((0, 2))
            ax = plt.gca()
            for o in obs:
                rect = patch.Rectangle((o[0], o[1]),
                                       o[2] - o[0],
                                       o[3] - o[1])
                ax.add_patch(rect)
            plt.plot(result_x[1:], result_y[1:], "-or")
            plt.plot(result_x[1], result_y[1], "vc")
            plt.xlim(result_x[1] - area, result_x[1] + area)
            plt.ylim(result_y[1] - area, result_y[1] + area)
            plt.xlabel("X axis")
            plt.ylabel("Y axis")
            plt.title("v[m/s]:" + str(
                      np.linalg.norm(initial_conditions['vel']))[0:4]
            )
            plt.grid(True)
            plt.pause(0.1)

    print("Finish")
    print("Average time per iteration: {}".format(total_time / i))

if __name__ == '__main__':
    main()
