from fot import fot
import numpy as np
import matplotlib.pyplot as plt
import argparse

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
parser.add_argument("-c",
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
planner_time = fot(args.display, args.verbose, args.thread, args.save)

if args.compare:
    baseline_time = fot(False, False, 0, False)

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

    plt.plot(planner_time, color="g", label="{}-Threads".format(args.thread))
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
        full_times.append(fot(False, False, i, False))

    full_times_average_time = [
        np.mean(full_times[i - 1]) for i in range(1, num_thread_upper_bound)
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
