import fot_wrapper
import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patch
import argparse
from pathlib import Path


def run_fot(show_animation=False,
            show_info=False,
            num_threads=0,
            save_frame=False):
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
    total_time = 0
    time_list = []
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
        time_list.append(end_time)

        # reconstruct initial_conditions
        if success:
            initial_conditions['pos'] = np.array([result_x[1], result_y[1]])
            initial_conditions['vel'] = np.array([speeds_x[1], speeds_y[1]])
            initial_conditions['ps'] = misc['s']
            if show_info:
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
                "key_release_event",
                lambda event: [exit(0) if event.key == "escape" else None])
            plt.plot(wx, wy)
            if obs.shape[0] == 0:
                obs = np.empty((0, 4))
            ax = plt.gca()
            for o in obs:
                rect = patch.Rectangle((o[0], o[1]), o[2] - o[0], o[3] - o[1])
                ax.add_patch(rect)
            plt.plot(result_x[1:], result_y[1:], "-or")
            plt.plot(result_x[1], result_y[1], "vc")
            plt.xlim(result_x[1] - area, result_x[1] + area)
            plt.ylim(result_y[1] - area, result_y[1] + area)
            plt.xlabel("X axis")
            plt.ylabel("Y axis")
            plt.title("v[m/s]:" +
                      str(np.linalg.norm(initial_conditions['vel']))[0:4])
            plt.grid(True)
            if save_frame:
                Path("img/frames").mkdir(parents=True, exist_ok=True)
                plt.savefig("img/frames/{}.jpg".format(i))
            plt.pause(0.1)

    print("Finish")

    print("======================= SUMMARY ========================")
    print("Total time for {} iterations taken: {}".format(i, total_time))
    print("Average time per iteration: {}".format(total_time / i))
    print("Max time per iteration: {}".format(max(time_list)))

    return time_list


if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-d",
        "--display",
        action="store_true",
        help="show animation, ensure you have X11 forwarding server open")
    parser.add_argument("-v",
                        "--verbose",
                        action="store_true",
                        help="verbose mode, show all state info")
    parser.add_argument("-s",
                        "--save",
                        action="store_true",
                        help="save each frame of simulation")
    parser.add_argument(
        "-c",
        "--compare",
        action="store_true",
        help="compare threaded runtime with unthreaded baseline")
    parser.add_argument("-t",
                        "--thread",
                        type=int,
                        default=0,
                        help="set number of threads to run with")
    parser.add_argument("-p",
                        "--profile",
                        action="store_true",
                        help="enable time profiling")
    parser.add_argument("-f",
                        "--full",
                        action="store_true",
                        help="full time profiling")
    args = parser.parse_args()

    # run planner with args passed in, record time
    planner_time = run_fot(args.display, args.verbose, args.thread, args.save)

    if args.compare:
        baseline_time = run_fot(False, False, 0, False)

        print("======================= SPEED UP ========================")
        print("Average Speed Up per Iteration: {} x".format(
            np.mean(baseline_time) / np.mean(planner_time)))

    if args.profile:
        if args.compare:
            plt.plot(baseline_time, color="k", label="Baseline")
            plt.hlines(np.mean(baseline_time),
                       0,
                       len(baseline_time),
                       colors="k",
                       label="Baseline mean",
                       linestyles="dashed")

        plt.plot(planner_time,
                 color="g",
                 label="{}-Threads".format(args.thread))
        plt.hlines(np.mean(planner_time),
                   0,
                   len(planner_time),
                   colors="g",
                   label="Threaded mean",
                   linestyles="dashed")
        plt.legend()
        plt.xlabel("Iteration Number")
        plt.ylabel("Time Per Iteration (s)")
        plt.title("Planning Execution Time Per Iteration")
        plt.gcf().canvas.mpl_connect(
            "key_release_event",
            lambda event: [exit(0) if event.key == "escape" else None])
        plt.show()

    print("=========================================================")

    # For comparing runtime and speedup across different number of threads
    # this will profile runtime for running with k threads,
    # for all k < num_thread_upper_bound
    # Also calculate a speedup ratio compared to the baseline,
    # which is single thread execution
    if args.full:
        num_thread_upper_bound = 9
        full_times = []
        for i in range(1, num_thread_upper_bound):
            full_times.append(run_fot(False, False, i, False))

        full_times_average_time = [
            np.mean(full_times[i - 1])
            for i in range(1, num_thread_upper_bound)
        ]  # runtime per iteration
        full_times_speedup = [
            np.sum(full_times[0]) / np.sum(full_times[i - 1])
            for i in range(1, num_thread_upper_bound)
        ]  # speed up ratio

        fig, ax1 = plt.subplots()
        num_threads_x_axis = [i for i in range(1, num_thread_upper_bound)]

        color = "tab:red"
        ax1.set_xlabel("Number of Threads")
        ax1.set_ylabel("Average Planner Execution Time Per Iteration (s)",
                       color=color)
        ax1.plot(num_threads_x_axis, full_times_average_time, color=color)
        ax1.tick_params(axis="y", labelcolor=color)

        ax2 = ax1.twinx()
        color = "tab:blue"
        ax2.set_ylabel("Speed Up", color=color)
        ax2.plot(num_threads_x_axis, full_times_speedup, color=color)
        ax2.tick_params(axis="y", labelcolor=color)

        plt.title("Performance vs Number of Threads")
        fig.tight_layout()
        plt.show()
