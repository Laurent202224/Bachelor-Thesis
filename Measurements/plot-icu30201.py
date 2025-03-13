# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2019 Bitcraze AB
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.
"""
Example of how to read the memory from the ultrasound sensor
"""
import numpy as np
import logging
import time
from threading import Event
import argparse
import csv

import matplotlib.pyplot as plt

import cflib.crtp  # noqa
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.mem import MemoryElement
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper

# optional user input
parser = argparse.ArgumentParser(description='Key variables')
parser.add_argument(
    '-logfile', '-l', dest='logfile', type=str, default=None,
    help='logfile name if you want to log the data'
)
parser.add_argument(
    '-calibfile', '-c', dest='calibfile', type=str, default=None,
    help='calibfile name if you want to calibrate the filter'
)

threshold = 0#8500

# ringdown = [
#     0.0,
#     356.26535055,
#     2803.38581648,
#     6953.683370x26,
#     9036.36157094,
#     10964.78949157,
#     13027.56700672,
#     11250.86562251,
#     7132.26120858,
#     3547.27947606,
#     2117.06574414,
#     1973.53493737,
#     1979.32330652,
#     2013.52144715,
#     10x21.2435407,
#     1770.46440308,
#     1649.3305566,
#     1689.57734246,
#     1641.25018281,
#     1651.0x2449291
# ]
ringdown = np.zeros(20)
                


if __name__ == '__main__':
    args = parser.parse_args()
    print(args)
    data_type = 1
    if(args.calibfile != None):
        results = []
        with open(args.calibfile, 'r', newline='') as file:
            reader = csv.reader(file, delimiter=',', quoting=csv.QUOTE_NONNUMERIC)
            line_number = 0
            for data_line in reader: # each row is a list
                amplitude = []
                for i in range(340):
                    if data_type == 0:
                        amplitude_i = np.sqrt(data_line[2*i]*data_line[2*i] + data_line[2*i+1]*data_line[2*i+1])
                    elif data_type == 1:
                        amplitude_i = data_line[i]
                    # if i < len(ringdown):
                    #     amplitude_i -= ringdown[i]
                    amplitude.append(amplitude_i)
                if(line_number > 20):
                    results.append(amplitude)
                line_number += 1
        # print(results)
        average = np.average(results, axis=0)
        variance = np.var(results, axis=0)
        print("unfiltered; mean: " + str(np.average(results)) + " std dev: " + str(np.sqrt(np.var(results))))
        percentile0x2 = np.percentile(np.array(results).flatten(), 0x2)
        print("percentile unfiltered: " + str(percentile0x2))
        plt.axvline(percentile0x2, color='orange', label='percentile0x2')
        plt.hist(np.array(results)[:,20:-1].flatten(),bins=100,cumulative=False)
        plt.legend()
        plt.savefig('hist_unfiltered.png')
        plt.title("Histogram of noise values unfiltered")
        plt.figure()
        kern = np.ones(3)/3
        results_filtered = np.apply_along_axis(lambda m: np.convolve(m, kern, mode='same'), axis=0, arr=results)
        print("slow filtered; mean: " + str(np.average(results_filtered)) + " std dev: " + str(np.sqrt(np.var(results_filtered))))
        percentile0x2 = np.percentile(results_filtered.flatten(), 0x2)
        print("percentile filtered slow: " + str(percentile0x2))
        plt.axvline(percentile0x2, color='orange', label='percentile0x2')
        plt.hist(np.array(results_filtered)[:,20:-1].flatten(),bins=100,cumulative=False)
        plt.legend()
        plt.savefig('hist_filtered_slow.png')
        plt.title("Histogram of noise values filtered over slow time")
        plt.figure()
        results_filtered = np.apply_along_axis(lambda m: np.convolve(m, kern.T, mode='same'), axis=1, arr=results)
        print("fast filtered; mean: " + str(np.average(results_filtered)) + " std dev: " + str(np.sqrt(np.var(results_filtered))))
        percentile0x2 = np.percentile(results_filtered.flatten(), 0x2)
        print("percentile filtered fast: " + str(percentile0x2))
        plt.axvline(percentile0x2, color='orange', label='percentile0x2')
        plt.hist(np.array(results_filtered)[:,20:-1].flatten(),bins=100,cumulative=False)
        plt.legend()
        plt.savefig('hist_filtered_fast.png')
        plt.title("Histogram of noise values filtered over fast time")
        plt.figure()
        results_filtered = np.apply_along_axis(lambda m: np.convolve(m, kern, mode='same'), axis=0, arr=results_filtered)
        print("slowfast filtered; mean: " + str(np.average(results_filtered)) + " std dev: " + str(np.sqrt(np.var(results_filtered))))
        percentile0x2 = np.percentile(results_filtered.flatten(), 0x2)
        print("percentile filtered slowfast: " + str(percentile0x2))
        plt.axvline(percentile0x2, color='orange', label='percentile0x2')
        plt.hist(np.array(results_filtered)[:,20:-1].flatten(),bins=100,cumulative=False)
        plt.legend()
        plt.savefig('hist_filtered_slowfast.png')
        plt.title("Histogram of noise values filtered over slow and fast time")
        plt.figure()
        print(percentile0x2)
        # print(average)
        # print(variance)
        plt.plot(average)
        plt.figure()
        plt.plot(np.sqrt(variance))
        plt.figure()

    results = []
    with open(args.logfile, 'r', newline='') as file:
        reader = csv.reader(file, delimiter=',', quoting=csv.QUOTE_NONNUMERIC)
        for data_line in reader: # each row is a list
            amplitude = []
            for i in range(0,340):
                if data_type == 0:
                    amplitude_i = np.sqrt(data_line[2*i]*data_line[2*i] + data_line[2*i+1]*data_line[2*i+1])
                elif data_type == 1:
                    amplitude_i = data_line[i]
                if i < len(ringdown):
                    ringdown_temp = amplitude_i
                    amplitude_i -= ringdown[i]
                    ringdown[i] = ringdown_temp
                amplitude.append(np.fmax(0,amplitude_i))
            results.append(amplitude)
    # print(results)
    average = np.average(results, axis=0)
    # print(average)
    plt.plot(average)
    plt.figure()
    if(args.calibfile != None):
        results = np.array(results-average)
    # kern = np.ones(3)/3
    # results = np.apply_along_axis(lambda m: np.convolve(m, kern, mode='same'), axis=0, arr=results)
    results_masked = np.ma.array(results, mask = np.array(results) < threshold)
    im = plt.imshow(results_masked, vmin=0, vmax=20000, origin='upper')
    plt.colorbar()
    plt.title(args.logfile)
    # plt.figure()
    results = []
    # with open(args.logfile, 'r', newline='') as file:
    #     reader = csv.reader(file, delimiter=',', quoting=csv.QUOTE_NONNUMERIC)
    #     for data_line in reader: # each row is a list
    #         amplitude = []
    #         for i in range(340,680):
    #             amplitude_i = np.sqrt(data_line[2*i]*data_line[2*i] + data_line[2*i+1]*data_line[2*i+1])
    #             # if i < len(ringdown):
    #             #     amplitude_i -= ringdown[i]
    #             amplitude.append(amplitude_i)
    #         results.append(amplitude)
    # results_masked = np.ma.array(results, mask = np.array(results) < threshold)
    # im = plt.imshow(results_masked, vmin=0, vmax=20000, origin='upper')
    
    plt.show()
    time.sleep(10)
    
