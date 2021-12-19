import sys
import yaml
import cv2
import matplotlib.pyplot as plt
import numpy as np
import queue
import collections

def plot_pos(stat_time_p, stat_measured, stat_predict, title="Tracking Performance - Stationary"):
    fig, ax = plt.subplots(nrows=4, ncols=1, figsize=(8,12))
    (ax0, ax1, ax2, ax3) = ax.flatten()
    fig.tight_layout(pad=3.0)
    fig.subplots_adjust(top=0.92)

    fig.suptitle(title, size=18)

    ax0.plot(stat_time_p, stat_measured[:,0], label="Measured")
    ax0.plot(stat_time_p, stat_predict[:,0], label="Predicted")
    ax0.set_title('X position')
    ax0.set_xlabel('Time (s)')
    ax0.set_ylabel('Position (m)')
    ax0.grid()
    ax0.legend()
    # ax0.plot(moving_time_p, moving_measured[:,0])
    ax1.plot(stat_time_p, stat_measured[:,1], label="Measured")
    ax1.plot(stat_time_p, stat_predict[:,1], label="Predicted")
    ax1.set_title('Y position')
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Position (m)')
    ax1.grid()
    ax1.legend()

    ax2.plot(stat_time_p, stat_measured[:,2], label="Measured")
    ax2.plot(stat_time_p, stat_predict[:,2], label="Predicted")
    ax2.set_title('Z position')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Position (m)')
    ax2.grid()
    ax2.legend()
    stat_norm = np.array([np.linalg.norm(e) for e in stat_measured - stat_predict])
    ax3.plot(stat_time_p, stat_norm)
    ax3.set_title('Error')
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Position Error (m)')
    ax3.grid()
    ax3.legend()

offset = 0.557

stat_predict = np.load('position_pred_stat.npy')
stat_time = np.load('time_stat.npy')
stat_measured = np.load('position_measured_stat.npy')

stat_predict = np.flip(stat_predict, axis=0)
stat_time = np.flip(stat_time, axis=0)
stat_measured = np.flip(stat_measured, axis=0)
stat_time_p = stat_time - stat_time[0]
# stat_time_p = np.array([x - stat_time[0] for x in stat_time])

# stat_predict = np.array(stat_predict)
# stat_time = np.array(stat_time)
# stat_measured = np.array(stat_measured)

# import pdb; pdb.set_trace()
moving_predict = np.load('position_pred_moving.npy')
moving_time = np.load('time_moving.npy')
moving_measured = np.load('position_measured_moving.npy', allow_pickle=True)
# Reindex/reshape?

moving_predict = np.flip(moving_predict, axis=0)
moving_time = np.flip(moving_time, axis=0)
moving_measured = np.flip(moving_measured, axis=0)
# moving_measured[0] = moving_measured[1]
moving_time_p = moving_time - moving_time[0]


# stat_predict[:,2] += offset
# moving_predict[:,2] += offset

# import pdb; pdb.set_trace()
plot_pos(stat_time_p, stat_measured, stat_predict, title="Tracking Performance - Stationary")
plot_pos(moving_time_p, moving_measured, moving_predict, title="Tracking Performance - Mobile")


plt.show()
# import pdb; pdb.set_trace()
