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
from threading import Event,Timer
import argparse
import csv

import matplotlib.pyplot as plt

import cflib.crtp  # noqa
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.mem import MemoryElement
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig
from cflib.utils import uri_helper

# optional user input
parser = argparse.ArgumentParser(description='Key variables')
parser.add_argument(
    '-logfile', '-l', dest='logfile', type=str, default=None,
    help='logfile name if you want to log the data'
)
parser.add_argument(
    '-logfileCF', '-lcf', dest='logfileCF', type=str, default=None,
    help='CF logfile name if you want to log the CF data'
)
parser.add_argument(
    '-takeoff', '-t', dest='takeoff', type=int, default=0,
    help='1 if takeoff, 0 if not (default)'
)
parser.add_argument(
    '-fly_time_s', '-time', dest='fly_time_s', type=int, default=0,
    help='enter the time in seconds you want to fly, 0 if not (default)'
)



# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

#The weighting of the early measurements
ringdown = [
    0.0,
    43.14188669, 
    200.57727046, 
    454.82517427, 
    794.77298914,
    977.6688749, 
    950.72305559, 
    819.67561478, 
    453.88945013, 
    117.77104725,
    32.67534404,  
    30.97135393,  
    28.56346846,  
    27.99968167,  
    26.10425345
]


class LoggingExample:
    """
    Simple logging example class that logs the Stabilizer from a supplied
    link uri and disconnects after 5s.
    """

    def __init__(self, link_uri):
        """ Initialize and run the example with the specified link_uri """

        self._cf = Crazyflie(rw_cache='./cache')

        # Connect some callbacks from the Crazyflie API
        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

        print('Connecting to %s' % link_uri)

        # Try to connect to the Crazyflie
        self._cf.open_link(link_uri)

        # Variable used to keep main loop occupied until disconnect
        self.is_connected = True

    def _connected(self, link_uri):
        """ This callback is called from the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""
        print('Connected to %s' % link_uri)

        # The definition of the logconfig can be made before connecting
        self._lg_stab = LogConfig(name='Stabilizer', period_in_ms=100)
        self._lg_stab.add_variable('stateEstimate.x', 'float')
        self._lg_stab.add_variable('stateEstimate.y', 'float')
        self._lg_stab.add_variable('stateEstimate.z', 'float')
        self._lg_stab.add_variable('stabilizer.roll', 'float')
        self._lg_stab.add_variable('stabilizer.pitch', 'float')
        self._lg_stab.add_variable('stabilizer.yaw', 'float')
        # The fetch-as argument can be set to FP16 to save space in the log packet
        self._lg_stab.add_variable('pm.vbat', 'FP16')

        # Adding the configuration cannot be done until a Crazyflie is
        # connected, since we need to check that the variables we
        # would like to log are in the TOC.
        try:
            self._cf.log.add_config(self._lg_stab)
            # This callback will receive the data
            self._lg_stab.data_received_cb.add_callback(self._stab_log_data)
            # This callback will be called on errors
            self._lg_stab.error_cb.add_callback(self._stab_log_error)
            # Start the logging
            self._lg_stab.start()
        except KeyError as e:
            print('Could not start log configuration,'
                  '{} not found in TOC'.format(str(e)))
        except AttributeError:
            print('Could not add Stabilizer log config, bad configuration.')

        # Start a timer to disconnect in 10s
        # t = Timer(5, self._cf.close_link)
        # t.start()

    def _stab_log_error(self, logconf, msg):
        """Callback from the log API when an error occurs"""
        print('Error when logging %s: %s' % (logconf.name, msg))

    def _stab_log_data(self, timestamp, data, logconf):
        """Callback from a the log API when data arrives"""
        print(f'[{timestamp}][{logconf.name}]: ', end='')
        for name, value in data.items():
            print(f'{name}: {value:3.3f} ', end='')
        print(data['pm.vbat'])
        # if(data['pm.vbat'] < 3.7):
        #     print('low battery, landing\n')
        #     land = 1
        #     time.sleep(2)
        #     self._cf.close_link()


    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the specified address)"""
        print('Connection to %s failed: %s' % (link_uri, msg))
        self.is_connected = False

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        print('Connection to %s lost: %s' % (link_uri, msg))

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        print('Disconnected from %s' % link_uri)
        self.is_connected = False

class ReadMem:
    def __init__(self, uri):
        self._event = Event()
        self._cf = Crazyflie(rw_cache='./cache')
        self.args = parser.parse_args()
        self.NUM_SENSORS = 1 #number of sensors, for later set higher probably
        self.data_type = 1 # 0 is IQ data, 1 amplitude
        self.land = 0
        self.memread_finished = 0
        self.last_no_low_battery = time.time()
        print(self.args)
        ringdown_dynamic = np.ones(20, dtype=float)*20000 # [20000, 20000, 20000, ..., 20000] array of 20 elements

        with SyncCrazyflie(uri, cf=self._cf) as scf:
            # adding the different variables we would like to log
            self._lg_stab = LogConfig(name='Stabilizer', period_in_ms=100)
            self._lg_stab.add_variable('stateEstimate.x', 'float')
            self._lg_stab.add_variable('stateEstimate.y', 'float')
            self._lg_stab.add_variable('stateEstimate.z', 'float')
            # self._lg_stab.add_variable('stabilizer.roll', 'float')
            # self._lg_stab.add_variable('stabilizer.pitch', 'float')
            self._lg_stab.add_variable('stabilizer.yaw', 'float')
            self._lg_stab.add_variable('us.range_0', 'float')
            # self._lg_stab.add_variable('us.MAX_RANGE', 'float')
            self._lg_stab.add_variable('range.front', 'uint16_t')
            # The fetch-as argument can be set to FP16 to save space in the log packet
            # self._lg_stab.add_variable('pm.vbat', 'FP16')

            # Adding the configuration cannot be done until a Crazyflie is
            # connected, since we need to check that the variables we
            # would like to log are in the TOC.
            try:
                self._cf.log.add_config(self._lg_stab)
                # This callback will receive the data
                self._lg_stab.data_received_cb.add_callback(self._stab_log_data)
                # This callback will be called on errors
                self._lg_stab.error_cb.add_callback(self._stab_log_error)
                # Start the logging
                self._lg_stab.start()
            except KeyError as e:
                print('Could not start log configuration,'
                    '{} not found in TOC'.format(str(e)))
            except AttributeError:
                print('Could not add Stabilizer log config, bad configuration.')
            mems = self._cf.mem.get_mems(MemoryElement.TYPE_DECK_ICU30201) # look in read_deck_mem.py for more explanation

            count = len(mems)
            if count != 1:
                raise Exception('Unexpected nr of memories found:', count)

            mem = mems[0] # should only contain one memory element


            data = [[0 for x in range(340)] for y in range(100)] # creates an 2D array of 100 rows and 340 columns with each element = 0
            im = plt.imshow(data, vmin=0, vmax=20000, origin='upper') # plot is created with color scale from 0 to 20000 in the upper left corner 
            data2 = [[0 for x in range(340)] for y in range(100)]
            if self.NUM_SENSORS > 1:     # if we have more than one sensor, another plot is created (for the other sensor), what if we have more than 2 sensors??
                plt.figure()
                im2 = plt.imshow(data2, vmin=0, vmax=20000, origin='upper')
            plt.colorbar()

            if self.args.takeoff == 1:  # takeoff configuration
                print("Reset estimator")
                self._cf.param.set_value('kalman.resetEstimation', '1')
                time.sleep(0.1)
                self._cf.param.set_value('kalman.resetEstimation', '0')
                self._cf.param.set_value('us.fly', '1')
                ##### TO STAY STABLE 0.7m ABOVE GROUND FOR MEASUREMENTS #####
                #self._cf.param.set_value('flightmode.althold', '1') ## doesn't work yet
                ##### TO STAY STABLE 0.7m ABOVE GROUND FOR MEASUREMENTS #####

                commander = scf.cf.high_level_commander
                commander.takeoff(0.7, 2.0)
                time.sleep(2)
            

            start_time = time.time()
            # data_line_ringdown = mem.read_data_sync()
            # print(data_line_ringdown)
            ringdown_dynamic_new = np.zeros(20, dtype=float) # put weight of the first 20 samples to 0 to filter out the early/bad/noisy measurements
            frames = 0
            time_one_iteration = 0
            iter_counter = 0
            # for frames in range(100):
            while True:
                if count == 1:
                    data_line = mem.read_data_sync() # read the data and store it in data_line
                    self.memread_finished = 1
                    if self.args.logfile is not None:
                        with open(self.args.logfile, 'a', newline='') as file: # write the log variables to a .csv file
                            writer = csv.writer(file)
                            writer.writerow(data_line)
                    # print(np.shape(data))
                    # print(np.shape(data_line))
                    # print(np.max(data_line))
                    amplitude = []
                    for i in range(340):
                        if self.data_type == 0:
                            amplitude_i = np.sqrt(data_line[2*i]*data_line[2*i] + data_line[2*i+1]*data_line[2*i+1]) # A = sqrt(I^2 + Q^2)
                        elif self.data_type == 1:
                            amplitude_i = data_line[i]
                        # print(amplitude_i)
                        # if i < len(ringdown):
                        #     amplitude_i -= ringdown[i]
                        amplitude.append(amplitude_i)
                    if frames == 0:
                        ringdown_dynamic_new = np.array(amplitude[:20]) # ringdown weights = first 20 amplitude samples -> only for the very first frame
                    else:
                        for i in range(20):
                            ringdown_dynamic_new[i] = ringdown_dynamic[i]*(10-1)/10 + amplitude[i]/10 # ringdown gets updated with the static ringdown_dynamic vector
                    amplitude[:20] = amplitude[:20] - ringdown_dynamic # first 20 amplitudes get reduced to discard noise errors
                    ringdown_dynamic = ringdown_dynamic_new
                    data[frames%100] = np.array(amplitude) # the amplitudes get saved into data[] array, if we have more than 100 frames, can't the data be overwritten??
                    im.set_data(data) # updates the image plot with the new data
                    print(np.max(amplitude))
                    if self.NUM_SENSORS > 1: # if more than 1 sensor, what if more than 2 sensors?? -> for-loop?
                        amplitude = []
                        for i in range(340,680):
                            amplitude_i = np.sqrt(data_line[2*i]*data_line[2*i] + data_line[2*i+1]*data_line[2*i+1])
                            if i < len(ringdown):
                                amplitude_i -= ringdown[i]
                            amplitude.append(amplitude_i)
                        data2[frames%100] = np.array(amplitude)
                        im2.set_data(data2)
                    plt.pause(0.01)
                    if self.args.takeoff == 1 and self.land == 1: # or (self.args.fly_time_s*1 <= time_one_iteration*iter_counter):
                     
                        commander.land(0.0, 2.0)
                        time.sleep(2)
                        commander.stop()
                        break
                    #if self.args.fly_time_s*1 <= time_one_iteration*iter_counter:  #### ADDED , only works if we actually read something####
                       # self.land = 1
                        #commander.land(0.0, 2.0)
                       # print("LANDING\n")
                       # time.sleep(2)
                       # commander.stop()
                       # break
                else:
                    time.sleep(0.2)
                    self.memread_finished = 1
                if iter_counter == 0:
                    time_one_iteration = time.time()-start_time
                iter_counter +=1
                if iter_counter >= 300:
                    self.land = 1

            print(np.average(data, axis=0)) # computes the average of each distance to an object (column) over all time frames (rows)

            end_time = time.time()
            print('FPS: {}'.format(100/(end_time - start_time))) # print the frames per second
            
            time.sleep(5)
    def _stab_log_error(self, logconf, msg):
        """Callback from the log API when an error occurs"""
        print('Error when logging %s: %s' % (logconf.name, msg))

    def _stab_log_data(self, timestamp, data, logconf):
        """Callback from a the log API when data arrives"""
        print(f'[{timestamp}][{logconf.name}]: ', end='')
        for name, value in data.items():
            print(f'{name}: {value:3.3f} ', end='') # print the data to the console??
        if self.memread_finished == 1:
            self.memread_finished = 0
            if self.args.logfileCF is not None:
                    with open(self.args.logfileCF, 'a', newline='') as file: # write the data to a .csv file, 'a' means that any data will be appended to the end of the file
                        writer = csv.writer(file)
                        row_content = [timestamp]
                        for name, value in data.items():
                            row_content.append(name)
                            row_content.append(value)  
                        writer.writerow(row_content)
        # we want to land if 5sec < 3.2V       
        # if (data['pm.vbat'] < 3.2):
        #     if self.last_no_low_battery + 5 < time.time():
        #         print('low battery, landing\n')
        #         self.land = 1
        #         time.sleep(2)
        #         self._cf.close_link()
        # else:
        #     self.last_no_low_battery = time.time()


    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the specified address)"""
        print('Connection to %s failed: %s' % (link_uri, msg))
        self.is_connected = False

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        print('Connection to %s lost: %s' % (link_uri, msg))

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        print('Disconnected from %s' % link_uri)
        self.is_connected = False


if __name__ == '__main__':
    # URI to the Crazyflie to connect to
    uri = uri_helper.uri_from_env(default='radio://0/99/2M/E7E7E7E7E7')
    # uri = uri_helper.uri_from_env(default='usb://0')

    # Initialize the low-level drivers
    cflib.crtp.init_drivers()
    # le = LoggingExample(uri)

    rm = ReadMem(uri)
