#!/usr/bin/python2
import argparse
import pathlib
import utils
import os
import scipy.io as sio
import seaborn as sns
import matplotlib.pyplot as plt
import pickle
import numpy as np
plt.rcParams.update({
    "text.usetex": True,
    "font.family": "sans-serif",
    "font.sans-serif": ["Helvetica"]})
## for Palatino and other serif fonts use:
plt.rcParams.update({
    "text.usetex": True,
    "font.family": "serif",
    "font.serif": ["Palatino"],
})

def header_to_time(header):
    return np.array(header['stamp']['secs'], float) + np.array(header['stamp']['nsecs'], float) * 1e-9

def plot_xy(x, y, xLim=None, yLim=None):
    plt.style.use('seaborn-whitegrid')
    plt.plot(x, y, 'r', LineWidth=1)
    if xLim is not None:
        plt.xlim(xLim[0], xLim[1])
    if yLim is not None:
        plt.ylim(yLim[0], yLim[1])
    plt.grid(b=True, which='major', color='#666666', linestyle='-')
    plt.minorticks_on()
    plt.grid(b=True, which='minor', color='#999999', linestyle=':')

def plot_mean_std(x, y_mean, y_std_dev):
    plt.style.use('seaborn-whitegrid')
    plt.plot(x, y_mean, 'r', LineWidth=1)
    plt.fill_between(x, y_mean-3*y_std_dev,
                     y_mean + 3*y_std_dev, color='red', alpha=0.2)
    plt.grid(b=True, which='major', color='#666666', linestyle='-')
    plt.minorticks_on()
    plt.grid(b=True, which='minor', color='#999999', linestyle=':')

def plot_imu(imu):
    imu['t'] = header_to_time(imu['header'])
    t_ = imu['t'][0]
    imu['t'] = imu['t']-t_
    tf = imu['t'][-1]

    plt.subplot(321)
    plot_xy(imu['t'], imu['angular_velocity']['x'], xLim=(0, tf), yLim=(-3,3))

    plt.subplot(322)
    plot_xy(imu['t'], imu['linear_acceleration']['x'], xLim=(0, tf), yLim=(-3,3))

    plt.subplot(323)
    plot_xy(imu['t'], imu['angular_velocity']['y'], xLim=(0, tf), yLim=(-3,3))

    plt.subplot(324)
    plot_xy(imu['t'], imu['linear_acceleration']['y'], xLim=(0, tf), yLim=(-3,3))

    plt.subplot(325)
    plot_xy(imu['t'], imu['angular_velocity']['z'], xLim=(0, tf), yLim=(-3,3))

    plt.subplot(326)
    plot_xy(imu['t'], imu['linear_acceleration']['z'], xLim=(0, tf), yLim=(-1,12))

class OdomPlotter(object):
    """
    Module to plot odometry info along with standard-deviation
    """
    def __init__(self):
        pass

    @staticmethod
    def extract_position(_dict_in):
        _dict_out = {
            't': np.array(_dict_in['header']['stamp']['secs'], float) + np.array(_dict_in['header']['stamp']['nsecs'],
                                                                                 float) * 1e-9,
            'x': np.array(_dict_in['pose']['pose']['position']['x'], float),
            'y': np.array(_dict_in['pose']['pose']['position']['y'], float),
            'z': np.array(_dict_in['pose']['pose']['position']['z'], float)}

        pose_covar = np.array(_dict_in['pose']['covariance'], float)

        _dict_out['x_std'] = np.sqrt(pose_covar[:, 0])
        _dict_out['y_std'] = np.sqrt(pose_covar[:, 7])
        _dict_out['z_std'] = np.sqrt(pose_covar[:, 14])

        return _dict_out



def main_plot_odom():
    parser = argparse.ArgumentParser()
    parser.add_argument('-b', '--bag', nargs=1, help='Bag filename')
    parser.add_argument('-p', '--pickle', nargs=1,
                        help='Pre-saved pickle file')
    args = parser.parse_args()
    if not (args.bag or args.pickle):
        parser.error('No argument given, either --bag(-b) or --pickle(-p)')

    if args.bag is not None:
        bag_ = utils.ros.BagReader(args.bag[0])
        bag_.read()
        data = bag_.data
        with open(os.path.splitext(args.bag[0])[0] + '.pickle', 'wb') as handle:
            pickle.dump(data, handle, protocol=pickle.HIGHEST_PROTOCOL)

    if args.pickle is not None:
        with open(args.pickle[0].name, 'rb') as handle:
            data = pickle.load(handle)

    # plot the pose info
    pos_est = OdomPlotter.extract_position(data['dega_base_odom_estimate'])
    t0 = pos_est['t'][0]
    pos_est['t'] = pos_est['t']-t0

    pos_true = OdomPlotter.extract_position(data['dega_base_odom_truth'])
    pos_true['t'] = pos_true['t']-t0

    tf = pos_est['t'][-1]

    plt.figure(1)
    plt.subplot(311)
    plot_mean_std(pos_est['t'], pos_est['x'], pos_est['x_std'])
    plt.plot(pos_true['t'], pos_true['x'], 'k', LineWidth=2, linestyle=':')
    plt.xlim(0, tf)
    plt.ylim(-1, 1.)
    plt.ylabel(r'$x~[m]$', fontsize=15)

    plt.subplot(312)
    plt.style.use('seaborn-whitegrid')
    plot_mean_std(pos_est['t'], pos_est['y'], pos_est['y_std'])
    plt.plot(pos_true['t'], pos_true['y'], 'k', LineWidth=2, linestyle=':')
    plt.xlim(0, tf)
    plt.ylim(-1.5, 0.5)
    plt.ylabel(r'$y~[m]$', fontsize=15)

    plt.subplot(313)
    plt.style.use('seaborn-whitegrid')
    plot_mean_std(pos_est['t'], pos_est['z'], pos_est['z_std'])
    plt.plot(pos_true['t'], pos_true['z'], 'k', LineWidth=2, linestyle=':')
    plt.xlim(0, tf)
    plt.ylim(-0.25, 1.5)
    plt.ylabel(r'$z~[m]$', fontsize=15)
    plt.xlabel(r'$ \texttt{time } [s]$', fontsize=15)
    plt.show()

    if 'dega_base_imu_raw' in data.keys():
        plt.figure(2)
        plot_imu(data['dega_base_imu_raw'])
        plt.show()
    else:
        print("skipping dega_base_imu_raw")
    # plt.ylim(-1, 1)

    pass


if __name__ == "__main__":
    main_plot_odom()