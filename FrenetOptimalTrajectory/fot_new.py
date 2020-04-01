import fot_wrapper
import time
import numpy as np
import matplotlib.pyplot as plt


def main():
    conds = {
        's0': 0.0,
        'target_speed': 10,
        'wx': [132.67, 128.67, 124.67, 120.67, 116.67, 112.67, 108.67,
               104.67, 101.43,  97.77,  94.84,  92.89,  92.4 ,  92.4 ,
               92.4 ,  92.4 ,  92.4 ,  92.4 ,  92.4 ,  92.39,  92.39,
               92.39,  92.39,  92.39,  92.39],
        'wy': [195.14, 195.14, 195.14, 195.14, 195.14, 195.14, 195.14,
               195.14, 195.14, 195.03, 193.88, 191.75, 188.72, 185.32,
               181.32, 177.32, 173.32, 169.32, 165.32, 161.32, 157.32,
               153.32, 149.32, 145.32, 141.84],
        'obstacle_list': [[92.89, 191.75]],
        'x': 132.67,
        'y': 195.14,
        'vx': -5,
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
        "max_accel": 6.0,
        "max_curvature": 10.0,
        "max_road_width_l": 5.0,
        "max_road_width_r": 1.0,
        "d_road_w": 0.25,
        "dt": 0.25,
        "maxt": 6.0,
        "mint": 2.0,
        "d_t_s": 0.25,
        "n_s_sample": 2.0,
        "obstacle_radius": 3.0,
        "kj": 0.1,
        "kt": 0.1,
        "kd": 1.0,
        "klat": 1.0,
        "klon": 1.0
    }

    # static elements of planner
    wx = np.array(conds['wx'])
    wy = np.array(conds['wy'])
    obs = np.array(conds['obstacle_list'])

    # simulation config
    show_animation = True
    sim_loop = 400
    area = 20
    total_time = 0
    for i in range(sim_loop):
        # run FOT and keep time
        print("Iteration: {}".format(i))
        start_time = time.time()
        result_x, result_y, speeds, speeds_x, speeds_y, misc, success = \
            fot_wrapper.run_fot(initial_conditions, hyperparameters)
        end_time = time.time() - start_time
        print("Time taken: {}".format(end_time))
        total_time += end_time

        # reconstruct initial_conditions
        if success:
            initial_conditions['pos'] = np.array([result_x[1], result_y[1]])
            initial_conditions['vel'] = np.array([speeds_x[1], -speeds_y[1]])
            initial_conditions['ps'] = misc[0]
        else:
            break

        # break if near goal
        if np.hypot(result_x[1] - wx[-1], result_y[1] - wy[-1]) <= 1.0:
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
            plt.scatter(obs[:, 0], obs[:, 1], marker='o', s=(3*6)**2)
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
