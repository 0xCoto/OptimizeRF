#!/usr/bin/python
# -*- coding: utf-8 -*-

from datetime import datetime
import argparse
from nanovna import NanoVNAV2
import skrf
from skrf.media import Coaxial
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator
from matplotlib import gridspec
import time
from datetime import timedelta
import threading
import os
from skopt import gp_minimize
from scipy import interpolate
import ast
from tkinter import *
import sys

### Matplotlib configuration ###

default_params = {
    'toolbar': 'None',
    'figure.dpi': 120,
    'figure.figsize': [4, 3],
    'figure.subplot.left': 0.15,
    'figure.subplot.right': 0.9,
    'figure.subplot.bottom': 0.12,
    'axes.titlesize': 'medium',
    'axes.labelsize': 14,
    'ytick.labelsize': 'small',
    'xtick.labelsize': 'small',
    'legend.fontsize': 12,
    'legend.loc': 'best',
    'font.size': 16,
    'font.family': 'serif',
    'text.usetex': True,
    }
plt.rcParams.update(default_params)


def optimize(
    f_start,
    f_stop,
    sweep_pts,
    cal,
    load_cal,
    f_goal,
    goal,
    s_parameter,
    parameters,
    bounds,
    reset,
    max_calls,
    exp_ratio,
    conv_ratio,
    output,
    ):
    global start_time, positions, best, ntw_best, s, grid_x, grid_y, \
        s11_progress, s21_progress, f_goal_label, flag

    # ## Input parameters ###
    # Goals

    if goal.lower() == 'min' or goal.lower() == 'minimize':
        sign = 1  # Minimize S-Parameter
    elif goal.lower() == 'max' or goal.lower() == 'maximize':
        sign = -1  # Maximize S-Parameter

    # Import parameters

    modules = []
    for parameter in parameters:
        try:
            modules.append(__import__(parameter))
        except ImportError:
            print ('Error: Failed to import \'', parameter, '\'.')
            return 1

    # Parameter bounds

    bounds = ast.literal_eval(bounds)

    f_arr = np.linspace(f_start / 1e6, f_stop / 1e6, num=sweep_pts)
    f = skrf.Frequency.from_f(f_arr, unit='MHz')

    # ## Clear previous data/logs ###

    files = ['Current.s2p', 'Initial.s2p', 'Best.s2p', 'log.txt']

    for file in files:
        if os.path.isfile(file):
            os.remove(file)
        else:
            pass

    # ## Device initiation ###
    # Initiate device

    nv = NanoVNAV2()

    # Set frequency range

    nv.set_sweep(f_start, f_stop)
    nv.fetch_frequencies()

    calibrated = False
    if load_cal != '':
        calibrated = True
    else:
        load_cal = '.'
        if cal:

            # print('Performing measurement calibration...\n')

            with open('log.txt', 'a') as log:
                log.write('===============================\n')
                log.write('[*] Performing measurement calibration...\n')

            # ## OPEN ###

            input('Connect OPEN to Port 1 and press Enter.')

            # Measure S11 and S21

            s11 = nv.data(0)
            s21 = nv.data(1)

            f_arr = np.linspace(f_start / 1e6, f_stop / 1e6,
                                num=sweep_pts)
            f = skrf.Frequency.from_f(f_arr, unit='MHz')
            s = np.zeros((len(f), 2, 2), dtype='complex_')
            s[:, 0, 0] = s11
            s[:, 0, 1] = s21  # # np.zeros_like(s11) #np.ones_like(s11)*np.nan
            s[:, 1, 0] = s21
            s[:, 1, 1] = s11  # # np.ones_like(s11) #np.ones_like(s11)*np.nan
            ntw_init = skrf.Network(frequency=f, s=s)

            ntw_init.write_touchstone(load_cal + '/open',
                    skrf_comment=False)

            # ## SHORT ###

            input('Connect SHORT to Port 1 and press Enter.')

            # Measure S11 and S21

            s11 = nv.data(0)
            s21 = nv.data(1)

            f_arr = np.linspace(f_start / 1e6, f_stop / 1e6,
                                num=sweep_pts)
            f = skrf.Frequency.from_f(f_arr, unit='MHz')
            s = np.zeros((len(f), 2, 2), dtype='complex_')
            s[:, 0, 0] = s11
            s[:, 0, 1] = s21  # # np.zeros_like(s11) #np.ones_like(s11)*np.nan
            s[:, 1, 0] = s21
            s[:, 1, 1] = s11  # # np.zeros_like(s11) #np.ones_like(s11)*np.nan
            ntw_init = skrf.Network(frequency=f, s=s)

            ntw_init.write_touchstone(load_cal + '/short',
                    skrf_comment=False)

            # ## LOAD ###

            input('Connect LOAD to Port 1 and press Enter.')

            # Measure S11 and S21

            s11 = nv.data(0)
            s21 = nv.data(1)

            f_arr = np.linspace(f_start / 1e6, f_stop / 1e6,
                                num=sweep_pts)
            f = skrf.Frequency.from_f(f_arr, unit='MHz')
            s = np.zeros((len(f), 2, 2), dtype='complex_')
            s[:, 0, 0] = s11
            s[:, 0, 1] = s21  # # np.zeros_like(s11) #np.ones_like(s11)*np.nan
            s[:, 1, 0] = s21
            s[:, 1, 1] = s11  # # np.ones_like(s11) #np.ones_like(s11)*np.nan
            ntw_init = skrf.Network(frequency=f, s=s)

            ntw_init.write_touchstone(load_cal + '/load',
                    skrf_comment=False)

            # ## THRU ###

            input('Connect THRU between Port 1 & 2 and press Enter.')

            # Measure S11 and S21

            s11 = nv.data(0)
            s21 = nv.data(1)

            f_arr = np.linspace(f_start / 1e6, f_stop / 1e6,
                                num=sweep_pts)
            f = skrf.Frequency.from_f(f_arr, unit='MHz')
            s = np.zeros((len(f), 2, 2), dtype='complex_')
            s[:, 0, 0] = s11
            s[:, 0, 1] = s21  # # np.ones_like(s11) #np.ones_like(s11)*np.nan
            s[:, 1, 0] = s21
            s[:, 1, 1] = s11  # # np.zeros_like(s11) #np.ones_like(s11)*np.nan
            ntw_init = skrf.Network(frequency=f, s=s)

            ntw_init.write_touchstone(load_cal + '/thru',
                    skrf_comment=False)
            calibrated = True
    if calibrated:

        # ## Create ideals ###

        coax = Coaxial(frequency=f, z0=50)
        my_ideals = [coax.short(nports=2), coax.open(nports=2),
                     coax.match(nports=2), coax.thru()]
        my_measured = [skrf.Network(load_cal + '/short.s2p'),
                       skrf.Network(load_cal + '/open.s2p'),
                       skrf.Network(load_cal + '/load.s2p'),
                       skrf.Network(load_cal + '/thru.s2p')]

        # # Create TwoPortOnePath instance

        calibration = \
            skrf.calibration.TwoPortOnePath(measured=my_measured,
                ideals=my_ideals)

        # Run calibration algorithm

        calibration.run()
        if cal:
            input('Calibration complete. Press Enter to begin DUT optimization.'
                  )

    # Measure initial S11 and S21

    s11 = nv.data(0)
    s21 = nv.data(1)

    f_arr = np.linspace(f_start / 1e6, f_stop / 1e6, num=sweep_pts)
    f = skrf.Frequency.from_f(f_arr, unit='MHz')
    s = np.zeros((len(f), 2, 2), dtype='complex_')
    s[:, 0, 0] = s11
    s[:, 0, 1] = s21  # # np.ones_like(s11)*np.nan #np.zeros_like(s11)
    s[:, 1, 0] = s21
    s[:, 1, 1] = s11  # # np.ones_like(s11)*np.nan #np.ones_like(s11)
    ntw_init = skrf.Network(frequency=f, s=s)
    if calibrated:
        ntw_init = calibration.apply_cal((ntw_init, ntw_init))

    s = ntw_init.s

    ntw_init.write_touchstone('Initial', skrf_comment=False)
    ntw_init = skrf.Network('Initial.s2p')

    ntw_best = ntw_init
    ntw_best.write_touchstone('Best', skrf_comment=False)
    ntw_best = skrf.Network('Best.s2p')

    f_goal /= 1e6

    idx = np.abs(f_arr - f_goal).argmin()

    if f_goal == int(f_goal):
        f_goal_label = str(int(f_goal))
    else:
        f_goal_label = str(f_goal)

    positions = np.empty((0, len(bounds)))
    s11_progress = np.array([])
    s21_progress = np.array([])
    if len(bounds) == 2:
        (grid_x, grid_y) = np.mgrid[bounds[0][0]:bounds[0][1]:250j,
                                    bounds[1][0]:bounds[1][1]:250j]

    plt.ion()

    fig = plt.figure('Network Optimizer', figsize=(16, 10))

    if len(bounds) == 2:
        width_ratios = [2.15, 2.15, 0.9]
    else:
        width_ratios = [2.15, 2.15, 0.75]
    spec = gridspec.GridSpec(ncols=3, nrows=2,
                             width_ratios=width_ratios)  # , hspace=0.075)

    mng = plt.get_current_fig_manager()
    mng.full_screen_toggle()

    start_time = time.time()
    best = None
    flag = True

    def s_mag(goto):
        global start_time, positions, best, ntw_best, s, grid_x, \
            grid_y, s11_progress, s21_progress, f_goal_label, flag
        try:

            # Float to Int

            goto = [int(pos) for pos in goto]

            if not flag:
                positions = np.append(positions, [goto], axis=0)
            with open('log.txt', 'a') as log:
                log.write('==================================\n')
                for parameter_idx in range(len(parameters)):
                    log.write('[*] Shifting x' + str(parameter_idx + 1)
                              + ' to position: '
                              + str(goto[parameter_idx]) + '\n')

            # print('a')

            threads = []
            plt.tight_layout(pad=0.7)
            plt.show()
            plt.draw()
            plt.pause(0.01)
            plt.clf()

            # for plt_thread in plt_threads:
            # kil....threads.append(threading.Thread(target=plt_thread, args=(0,)))

            for parameter_idx in range(len(parameters)):
                threads.append(threading.Thread(target=modules[parameter_idx].drive,
                               args=(goto[parameter_idx], )))
            for thread in threads:
                thread.start()
            for thread in threads:
                thread.join()

            # Measure current S11 and S21

            s11 = nv.data(0)
            s21 = nv.data(1)

            f_arr = np.linspace(f_start / 1e6, f_stop / 1e6,
                                num=sweep_pts)
            f = skrf.Frequency.from_f(f_arr, unit='MHz')
            s = np.zeros((len(f), 2, 2), dtype='complex_')
            s[:, 0, 0] = s11
            s[:, 0, 1] = s21  # # np.ones_like(s11)*np.nan #np.zeros_like(s11) # S12 (incompatible with NanoVNA)
            s[:, 1, 0] = s21
            s[:, 1, 1] = s11  # # np.ones_like(s11)*np.nan #np.ones_like(s11) # S22 (incompatible with NanoVNA)

            ntw = skrf.Network(frequency=f, s=s)
            if calibrated:
                ntw = calibration.apply_cal((ntw, ntw))

            s = ntw.s
            if not flag:
                s11_progress = np.append(s11_progress,
                        skrf.mathFunctions.complex_2_db(s[idx, 0, 0]))
                s21_progress = np.append(s21_progress,
                        skrf.mathFunctions.complex_2_db(s[idx, 1, 0]))

            ntw.write_touchstone('Current', skrf_comment=False)
            ntw = skrf.Network('Current.s2p')

            if flag:
                best = sign * skrf.mathFunctions.complex_2_db(s[idx, 1,
                        0])
                ntw_best = ntw
                ntw_best.write_touchstone('Best', skrf_comment=False)
                ntw_best = skrf.Network('Best.s2p')
            elif sign * skrf.mathFunctions.complex_2_db(s[idx, 1, 0]) \
                < best:
                best = sign * skrf.mathFunctions.complex_2_db(s[idx, 1,
                        0])
                ntw_best = ntw
                ntw_best.write_touchstone('Best', skrf_comment=False)
                ntw_best = skrf.Network('Best.s2p')

            ax1 = fig.add_subplot(spec[0])  # Initial vs Best
            ax2 = fig.add_subplot(spec[3])  # Current
            ax3 = fig.add_subplot(spec[4])  # Positions vs Iteration
            ax4 = fig.add_subplot(spec[1])  # |S11|, |S21| vs Iteration
            ax5 = fig.add_subplot(spec[1:, -1])  # x1 vs x2 (S21)
            ax6 = fig.add_subplot(spec[2])  # x1 vs x2 (S11)
            if len(bounds) != 2:
                ax5.axis('off')
                ax6.axis('off')
            ax3.xaxis.set_major_locator(MaxNLocator(integer=True))  # Force integer x-ticks
            ax4.xaxis.set_major_locator(MaxNLocator(integer=True))  # Force integer x-ticks

            # plt.subplots_adjust(wspace=0,hspace=0) #hspace=0.075)

            # Initial vs Best

            ax1.set_title('$\mathrm{Optimization \ Progress}$',
                          fontsize=19)

            ax1.axvline(x=f_goal * 1e6, color='#9467bd', linestyle='--')
            ntw_init.plot_s_db(n=0, m=0, ax=ax1)
            ntw_init.plot_s_db(n=0, m=1, ax=ax1)

            ntw_best.plot_s_db(n=0, m=0, ax=ax1)
            ntw_best.plot_s_db(n=0, m=1, ax=ax1)

            ax1.set_xlabel('$\mathrm{Frequency \ (MHz)}$')
            ax1.set_ylabel('$\mathrm{Magnitude \ (dB)}$')
            ax1.set_ylim(top=0)
            ax1.grid()
            ax1.legend(loc='lower left')

            # Current

            ax2.axvline(x=f_goal * 1e6, color='#9467bd', linestyle='--')
            ntw.plot_s_db(n=0, m=0, ax=ax2)
            ntw.plot_s_db(n=0, m=1, ax=ax2)
            ax1.get_shared_x_axes().join(ax1, ax2)
            ax1.set_xticklabels([])
            x_axis = ax1.axes.get_xaxis()
            x_axis.set_label_text('')
            x_label = x_axis.get_label()
            x_label.set_visible(False)
            ax2.set_ylabel('$\mathrm{Magnitude \ (dB)}$')
            ax2.set_ylim(top=0)
            ax2.grid()
            ax2.legend(loc='lower left')

            # Positions vs Iteration #

            for parameter_idx in range(len(parameters)):
                ax3.plot(range(1, len(positions) + 1), positions[:,
                         parameter_idx], label='$x_{'
                         + str(parameter_idx + 1) + r'}$')
            ax3.set_xlim(left=1)
            ax3.grid()
            if len(positions) < 2:
                ax3.set_xticklabels([])
            ax3.legend(loc='best')
            ax3.set_xlabel('$\mathrm{Iteration \ \#}$')
            ax3.set_ylabel('$\mathrm{Position \ (steps)}$')

            # |S11|, |S21| vs Iteration #

            ax4.set_title('$\mathrm{Parameter \ Sequence}$',
                          fontsize=19)
            ax4.plot(range(1, len(s11_progress) + 1), s11_progress,
                     label='$S_{11} \ \mathrm{(' + f_goal_label
                     + ' \ \mathrm{MHz})}$')
            ax4.plot(range(1, len(s21_progress) + 1), s21_progress,
                     label='$S_{21} \ \mathrm{(' + f_goal_label
                     + ' \ \mathrm{MHz})}$')
            ax4.set_xlim(left=1)
            ax4.get_shared_x_axes().join(ax4, ax3)
            ax4.set_xticklabels([])
            x_axis = ax1.axes.get_xaxis()
            x_axis.set_label_text('')
            x_label = x_axis.get_label()
            x_label.set_visible(False)
            ax4.grid()
            ax4.legend(loc='best')
            ax4.set_ylabel('$\mathrm{Magnitude \ (dB)}$')

            # |S21| vs x1 vs x2

            if len(bounds) == 2:
                ax5.set_aspect('equal', anchor='N')
                if not flag and len(positions) >= 4:
                    grid_z2 = interpolate.griddata(positions,
                            s21_progress, (grid_x, grid_y),
                            method='cubic')
                    ax5.imshow(grid_z2.T, extent=(bounds[0][0],
                               bounds[0][1], bounds[1][0],
                               bounds[1][1]), origin='lower')
                    levels = np.arange(np.amin(s21_progress),
                            np.amax(s21_progress),
                            abs(np.amax(s21_progress)
                            - np.amin(s21_progress)) / 3)
                    ax5.contour(grid_z2.T, levels, colors='k',
                                origin='lower', extent=(bounds[0][0],
                                bounds[0][1], bounds[1][0],
                                bounds[1][1]))
                    ax5.plot(positions[:, 0], positions[:, 1], 'r.',
                             ms=3)
                    ax5.set_aspect((bounds[0][1] - bounds[0][0])
                                   / (bounds[1][1] - bounds[1][0]),
                                   anchor='N')
                ax5.set_xlabel('$x_{1}$')
                ax6.set_xticks([])
                ax6.get_shared_x_axes().join(ax6, ax5)
                ax6.set_xticklabels([])
                x_axis = ax6.axes.get_xaxis()
                x_axis.set_label_text('')
                x_label = x_axis.get_label()
                x_label.set_visible(False)
                ax5.set_ylabel('$x_{2}$')

                # |S11| vs x1 vs x2

                ax6.set_aspect('equal', anchor='S')
                if not flag and len(positions) >= 4:
                    grid_z2 = interpolate.griddata(positions,
                            s11_progress, (grid_x, grid_y),
                            method='cubic')
                    ax6.imshow(grid_z2.T, extent=(bounds[0][0],
                               bounds[0][1], bounds[1][0],
                               bounds[1][1]), origin='lower')
                    levels = np.arange(np.amin(s11_progress),
                            np.amax(s11_progress),
                            abs(np.amax(s11_progress)
                            - np.amin(s11_progress)) / 3)
                    ax6.contour(grid_z2.T, levels, colors='k',
                                origin='lower', extent=(bounds[0][0],
                                bounds[0][1], bounds[1][0],
                                bounds[1][1]))
                    ax6.plot(positions[:, 0], positions[:, 1], 'r.',
                             ms=3)
                    ax6.set_aspect((bounds[0][1] - bounds[0][0])
                                   / (bounds[1][1] - bounds[1][0]),
                                   anchor='S')

                ax5.set_xlabel('$x_{1}$')
                ax6.set_title('$\mathrm{Evaluation \ Heatmaps}$\n',
                              fontsize=19)
                ax6.set_xlabel('$x_{1}$')
                ax6.set_ylabel('$x_{2}$')

                fig.text(x=0.91, y=0.786, s=r'$|S_{11}|$', fontsize=13)
                fig.text(x=0.91, y=0.506, s=r'$|S_{21}|$', fontsize=13)

            fig.text(x=0.850, y=0.926,
                     s=r'$\mathrm{\underline{Optimize\textbf{RF}}}$',
                     fontsize=26, color='seagreen')

            y_offset = 0
            if len(bounds) != 2:
                y_offset = 0.6

            fig.text(x=0.847, y=0.160 + y_offset,
                     s=r'$\mathrm{\underline{Optimization \ Status}}$',
                     fontsize=16)

            if len(positions) >= int(exp_ratio * max_calls):
                search_space = 'Local \ (exploiting)'
            else:
                search_space = 'Global \ (exploring)'

            fig.text(x=0.820, y=0.125 + y_offset,
                     s=r'$\mathrm{Search \ Space}$: $\mathrm{'
                     + search_space + r'}$', fontsize=14)

            # Check convergence

            convergence = 'Not \ reached'
            if s_parameter.lower() == 's21':
                s_progress = s21_progress
            else:
                s_progress = s11_progress
            if len(s_progress) > int(exp_ratio * max_calls):
                s_progress_reversed = s_progress[::-1]
                if sign == -1:  # Maximize magnitude
                    last_best_idx = np.argmax(s_progress_reversed)
                else:

                      # Minimize magnitude

                    last_best_idx = np.argmin(s_progress_reversed)
                if last_best_idx >= int(conv_ratio * max_calls):
                    convergence = 'Reached'

            fig.text(x=0.839, y=0.095 + y_offset,
                     s=r'$\mathrm{Convergence}$: $\mathrm{'
                     + convergence + r'}$', fontsize=14)

            percentage = str(round(100 * (len(positions) / max_calls),
                             1))
            fig.text(x=0.844, y=0.065 + y_offset,
                     s=r'$\mathrm{Progress}$: $\mathrm{'
                     + str(len(positions)) + r'/' + str(max_calls)
                     + r'\ (' + percentage + r'\%)}$', fontsize=14)

            runtime = int(time.time() - start_time)
            fig.text(x=0.866, y=0.035 + y_offset,
                     s=r'$\mathrm{Runtime}$: $\mathrm{'
                     + str(timedelta(seconds=runtime))[:-3].replace(':'
                     , r'}$:$\mathrm{') + r'}$', fontsize=14)

            if flag:
                flag = False
            plt.savefig(str(len(positions) + 1) + '.png', dpi=300)
            if len(positions) == max_calls and output != '':
                plt.savefig(output, dpi=300)
        except KeyboardInterrupt:

            # plt.show()
            # plt.draw()
            # plt.pause(0.01)
            # plt.clf()

            raise ValueError('\n[-] Sweep terminated by user.')

        if s_parameter.lower() == 's21':
            magnitude = sign * skrf.mathFunctions.complex_2_db(s[idx,
                    1, 0])
        else:
            magnitude = sign * skrf.mathFunctions.complex_2_db(s[idx,
                    0, 0])

        return magnitude

    # ## Optimization ###

    try:

        # Initiate plot (goto=0 to begin plot)

        with open('log.txt', 'a') as log:
            log.write('=-=-=-=-=-=-=-=-=-=-=-=-=-=-=\n')
            log.write('[+] Initiating parameters...\n')
        s_mag([0] * len(bounds))

        # Begin optimization

        result = gp_minimize(
            s_mag,
            bounds,
            n_calls=max_calls,
            n_initial_points=int(exp_ratio * max_calls),
            initial_point_generator='lhs',
            verbose=True,
            )
    except Exception, e:

        with open('log.txt', 'a') as log:
            print e
            log.write(str(e))
            log.write('''
[-] Optimization interrupted.
''')

    if reset:
        with open('log.txt', 'a') as log:
            if len(bounds) == 1:
                log.write('[*] Resetting parameter back to initial position (0 steps).\n'
                          )
            else:
                log.write('[*] Resetting parameters back to initial positions (0 steps).\n'
                          )

        threads = []
        for parameter_idx in range(len(parameters)):
            threads.append(threading.Thread(target=modules[parameter_idx].drive,
                           args=(0, )))

        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join()

        for parameter_idx in range(len(parameters)):
            modules[parameter_idx].cleanup()


def main(gui=False):

    # Load argument values

    parser = argparse.ArgumentParser()

    if gui:

        # GUI

        parser.add_argument('-G', '--gui', dest='gui',
                            action='store_true',
                            help='Start graphical user interface')
        parser.set_defaults(gui=False)
    else:

        # GUI (included for -h)

        parser.add_argument('-G', '--gui', dest='gui',
                            action='store_true',
                            help='Start graphical user interface')
        parser.set_defaults(gui=False)

        # Sweep arguments

        parser.add_argument(
            '-f0',
            '--f_start',
            dest='f_start',
            help='Start frequency (Hz)',
            type=float,
            required=True,
            )
        parser.add_argument(
            '-f1',
            '--f_stop',
            dest='f_stop',
            help='Stop frequency (Hz)',
            type=float,
            required=True,
            )
        parser.add_argument(
            '-t',
            '--sweep_pts',
            dest='sweep_pts',
            help='Sweep points (Hz)',
            type=int,
            default=101,
            )

        # Measurement calibration

        parser.add_argument('-c', '--cal', dest='cal',
                            action='store_true',
                            help='Perform interactive measurement calibration'
                            )
        parser.set_defaults(cal=False)
        parser.add_argument(
            '-l',
            '--load_cal',
            dest='load_cal',
            help='Import local measurement calibration from directory',
            type=str,
            default='',
            )

        # Parameter properties

        parser.add_argument(
            '-p',
            '--parameters',
            dest='parameters',
            nargs='+',
            help='Tuning parameters (module names)',
            type=str,
            required=True,
            )
        parser.add_argument(
            '-b',
            '--bounds',
            dest='bounds',
            help='Parameters boundaries. Format: \'[(x1_min, x1_max), (x2_min, x2_max), ..., (xn_min, xn_max)]\''
                ,
            type=str,
            required=True,
            )
        parser.add_argument('-r', '--reset', dest='reset',
                            action='store_true',
                            help='Reset parameters back to initial positions'
                            )
        parser.set_defaults(reset=True)

        # Goal arguments

        parser.add_argument(
            '-f',
            '--f_goal',
            dest='f_goal',
            help='Target frequency (Hz)',
            type=float,
            required=True,
            )
        parser.add_argument(
            '-g',
            '--goal',
            dest='goal',
            help='Goal operator (minimize/maximize)',
            type=str,
            default='max',
            )
        parser.add_argument(
            '-s',
            '--s_parameter',
            dest='s_parameter',
            help='Scattering parameter\'s magnitude to optimize',
            type=str,
            default='S21',
            )

        # Optimizer settings

        parser.add_argument(
            '-i',
            '--max_calls',
            dest='max_calls',
            help='Maximum number of function iterations (calls)',
            type=int,
            default=100,
            )
        parser.add_argument(
            '-e',
            '--exp_ratio',
            dest='exp_ratio',
            help='Exploration ratio (initial global search)',
            type=float,
            default=0.1,
            )
        parser.add_argument(
            '-q',
            '--conv_ratio',
            dest='conv_ratio',
            help='Convergence ratio',
            type=float,
            default=0.3,
            )

        # Result output

        parser.add_argument(
            '-o',
            '--output',
            dest='output',
            help='Results output filename',
            type=str,
            default='',
            )
    args = parser.parse_args()

    if not gui:

        # ## Begin optimization ###

        optimize(
            args.f_start,
            args.f_stop,
            args.sweep_pts,
            args.cal,
            args.load_cal,
            args.f_goal,
            args.goal,
            args.s_parameter,
            args.parameters,
            args.bounds,
            args.reset,
            args.max_calls,
            args.exp_ratio,
            args.conv_ratio,
            args.output,
            )
    else:

        # Create window object

        app = Tk()

        def start_optimizer():
            top = Toplevel()
            top.geometry('500x100')
            top.title('OptimizeRF')
            Message(top, text='Starting Optimizer...', font=('Helvetica'
                    , 18, 'italic'), padx=50, pady=50).pack()
            top.after(10000, top.destroy)
            time.sleep(1)

            # ## Begin optimization ###
            # print(
            # ....float(f_start_text.get()),
            # ....float(f_stop_text.get()),
            # ....int(sweep_pts_text.get()),
            # ....var_cal.get() == 1,
            # ....str(load_cal_entry.get()) if str(load_cal_entry.get()) != '' else '',
            # ....float(f_goal_text.get()),
            # ....str((goal_text.get()).lower()),
            # ....str(objective),
            # ....list(str(parameters_text.get()).split()),
            # ....str(bounds_text.get()),
            # ....int(var_reset.get()) == 1,
            # ....int(max_calls_text.get()),
            # ....float(exploration_ratio_text.get()),
            # ....float(convergence_ratio_text.get()),
            # ....str(output_text.get())
            # )

            optimize(
                float(f_start_text.get()),
                float(f_stop_text.get()),
                int(sweep_pts_text.get()),
                var_cal.get() == 1,
                (str(load_cal_entry.get()) if str(load_cal_entry.get())
                 != '' else ''),
                float(f_goal_text.get()),
                str(goal_text.get().lower()),
                str(objective),
                list(str(parameters_text.get()).split()),
                str(bounds_text.get()),
                int(var_reset.get()) == 1,
                int(max_calls_text.get()),
                float(exploration_ratio_text.get()) / 100,
                float(convergence_ratio_text.get()) / 100,
                str(output_text.get()),
                )

        # ## Sweep arguments ###

        sweep_args_label = Label(app, text='Sweep Arguments',
                                 font=('Helvetica', 18, 'bold',
                                 'underline'))
        sweep_args_label.grid(
            row=0,
            column=0,
            columnspan=2,
            pady=10,
            padx=10,
            sticky=W,
            )

        # f_start

        f_start_text = DoubleVar()
        f_start_label = Label(app, text=' Start frequency: ',
                              font=('Helvetica', 14))
        f_start_label.grid(row=1, column=0, padx=20, sticky=W)
        f_start_entry = Entry(app, textvariable=f_start_text, width=12)
        f_start_entry.grid(row=1, column=1)
        f_start_label_hz = Label(app, text='Hz', font=('Helvetica', 14))
        f_start_label_hz.grid(row=1, column=2, sticky=W)

        # f_stop

        f_stop_text = DoubleVar()
        f_stop_label = Label(app, text=' Stop frequency: ',
                             font=('Helvetica', 14))
        f_stop_label.grid(row=2, column=0, padx=20, sticky=W)
        f_stop_entry = Entry(app, textvariable=f_stop_text, width=12)
        f_stop_entry.grid(row=2, column=1)
        f_stop_label_hz = Label(app, text='Hz', font=('Helvetica', 14))
        f_stop_label_hz.grid(row=2, column=2, sticky=W)

        # sweep_pts

        sweep_pts_text = IntVar(value=101)
        sweep_pts_label = Label(app, text=' Sweep Points: ',
                                font=('Helvetica', 14))
        sweep_pts_label.grid(row=3, column=0, padx=20, sticky=W)
        sweep_pts_entry = Entry(app, textvariable=sweep_pts_text,
                                width=12)
        sweep_pts_entry.grid(row=3, column=1)

        # ## Measurement Calibration ###

        measurement_calibration_label = Label(app,
                text='Measurement Calibration', font=('Helvetica', 18,
                'bold', 'underline'))
        measurement_calibration_label.grid(
            row=4,
            column=0,
            columnspan=3,
            pady=10,
            padx=10,
            sticky=W,
            )
        var_cal = IntVar()

        # cal

        cal = Radiobutton(app, text='Perform interactive calibration',
                          variable=var_cal, value=1, font=('Helvetica',
                          14))
        cal.grid(row=5, column=0, columnspan=3, padx=10, sticky=W)

        # load_cal

        load_cal_text = StringVar(value='measured')
        load_cal = Radiobutton(app,
                               text='Import local calibration from dir:'
                               , variable=var_cal, value=2,
                               font=('Helvetica', 14))
        load_cal.grid(row=6, column=0, columnspan=3, padx=10, sticky=W)
        load_cal_entry = Entry(app, textvariable=load_cal_text, width=9)
        load_cal_entry.grid(row=6, column=1, padx=30)

        # uncalibrated

        uncalibrated = Radiobutton(app, text='None (uncalibrated)',
                                   variable=var_cal, value=3,
                                   font=('Helvetica', 14))
        uncalibrated.grid(row=7, column=0, columnspan=3, padx=10,
                          sticky=W)
        uncalibrated.select()

        # ## Parameters ###

        params_label = Label(app, text='Parameter Properties',
                             font=('Helvetica', 18, 'bold', 'underline'
                             ))
        params_label.grid(
            row=8,
            column=0,
            columnspan=3,
            pady=10,
            padx=10,
            sticky=W,
            )

        # parameters

        parameters_text = StringVar(value='module_x1 module_x2')
        parameters_label = Label(app, text=' Parameters:',
                                 font=('Helvetica', 14))
        parameters_label.grid(row=9, column=0, padx=20, sticky=W)
        parameters_entry = Entry(app, textvariable=parameters_text,
                                 width=32)
        parameters_entry.grid(row=9, column=1, columnspan=3)

        # boundaries

        bounds_text = \
            StringVar(value='[(x1_min, x1_max), (x2_min, x2_max)]')
        bounds_label = Label(app, text=' Boundaries:', font=('Helvetica'
                             , 14))
        bounds_label.grid(row=10, column=0, padx=20, sticky=W)
        bounds_entry = Entry(app, textvariable=bounds_text, width=32)
        bounds_entry.grid(row=10, column=1, columnspan=3)

        # reset

        var_reset = IntVar()
        reset = Checkbutton(app,
                            text='Reset parameters back to initial positions'
                            , variable=var_reset, font=('Helvetica',
                            14))
        reset.grid(row=11, column=0, columnspan=11, padx=10, sticky=W)
        reset.select()

        # ## Goals ###

        goals_label = Label(app, text='Optimization Goal',
                            font=('Helvetica', 18, 'bold', 'underline'))
        goals_label.grid(
            row=12,
            column=0,
            columnspan=3,
            pady=10,
            padx=10,
            sticky=W,
            )

        # f_goal

        f_goal_text = DoubleVar()
        f_goal_label = Label(app, text=' Target frequency: ',
                             font=('Helvetica', 14))
        f_goal_label.grid(row=13, column=0, padx=20, sticky=W)
        f_goal_entry = Entry(app, textvariable=f_goal_text, width=12)
        f_goal_entry.grid(row=13, column=1)
        f_goal_label_hz = Label(app, text='Hz', font=('Helvetica', 14))
        f_goal_label_hz.grid(row=13, column=2, sticky=W)

        # goal

        goal_text = StringVar(app)
        goal_text.set('Maximize')
        goal_label = Label(app, text=' Optimization goal:',
                           font=('Helvetica', 14))
        goal_label.grid(row=14, column=0, padx=20, sticky=W)
        goal_entry = OptionMenu(app, goal_text, 'Minimize', 'Maximize')
        goal_entry.grid(row=14, column=1)

        # objective S-Parameter

        objective_text = StringVar(app)
        objective_text.set('     |S21|  ')
        objective_label = Label(app, text=' Objective S-Parameter:',
                                font=('Helvetica', 14))
        objective_label.grid(row=15, column=0, padx=20, sticky=W)
        objective_entry = OptionMenu(app, objective_text, '     |S11|  '
                , '     |S21|  ')
        objective_entry.grid(row=15, column=1)
        if objective_text.get() == '     |S21|  ':
            objective = 'S21'
        else:
            objective = 'S11'

        # ## Optimizer Settings ###

        settings_label = Label(app, text='Optimizer Settings',
                               font=('Helvetica', 18, 'bold',
                               'underline'))
        settings_label.grid(
            row=16,
            column=0,
            columnspan=3,
            pady=10,
            padx=10,
            sticky=W,
            )

        # max_calls

        max_calls_text = IntVar(value=100)
        max_calls_label = Label(app, text=' Max calls: ',
                                font=('Helvetica', 14))
        max_calls_label.grid(row=17, column=0, padx=20, sticky=W)
        max_calls_entry = Entry(app, textvariable=max_calls_text,
                                width=12)
        max_calls_entry.grid(row=17, column=1)

        # exploration_ratio

        exploration_ratio_text = IntVar(value=15)
        exploration_ratio_label = Label(app, text=' Exploration ratio: '
                , font=('Helvetica', 14))
        exploration_ratio_label.grid(row=18, column=0, padx=20,
                sticky=W)
        exploration_ratio_entry = Entry(app,
                textvariable=exploration_ratio_text, width=12)
        exploration_ratio_entry.grid(row=18, column=1)
        exploration_ratio_label_percent = Label(app, text='%',
                font=('Helvetica', 14))
        exploration_ratio_label_percent.grid(row=18, column=2, sticky=W)

        # convergence_ratio

        convergence_ratio_text = IntVar(value=25)
        convergence_ratio_label = Label(app, text=' Convergence ratio: '
                , font=('Helvetica', 14))
        convergence_ratio_label.grid(row=19, column=0, padx=20,
                sticky=W)
        convergence_ratio_entry = Entry(app,
                textvariable=convergence_ratio_text, width=12)
        convergence_ratio_entry.grid(row=19, column=1)
        convergence_ratio_label_percent = Label(app, text='%',
                font=('Helvetica', 14))
        convergence_ratio_label_percent.grid(row=19, column=2, sticky=W)

        # ## Results ###

        results_label = Label(app, text='Results', font=('Helvetica',
                              18, 'bold', 'underline'))
        results_label.grid(
            row=20,
            column=0,
            columnspan=3,
            pady=10,
            padx=10,
            sticky=W,
            )

        # output

        output_text = StringVar(value='results.pdf')
        output_label = Label(app, text=' Output filename:',
                             font=('Helvetica', 14))
        output_label.grid(row=21, column=0, padx=20, sticky=W)
        output_entry = Entry(app, textvariable=output_text, width=12)
        output_entry.grid(row=21, column=1)

        # fullscreen

        var_fullscreen = IntVar()
        fullscreen = Checkbutton(app, text='Enable fullscreen',
                                 variable=var_fullscreen,
                                 font=('Helvetica', 14))
        fullscreen.grid(row=22, column=0, columnspan=11, padx=10,
                        sticky=W)
        fullscreen.select()
        start = Button(app, text='Start Optimizer',
                       command=start_optimizer, width=59, bg='#2edc71')
        start.grid(
            row=23,
            column=0,
            columnspan=3,
            padx=10,
            sticky=W,
            pady=10,
            )

        app.title('OptimizeRF | GUI')
        app.geometry('520x960')

        # Start GUI

        app.mainloop()


if __name__ == '__main__':
    if '-G' in sys.argv or '--gui' in sys.argv:
        gui = True
    else:
        gui = False
    main(gui)
