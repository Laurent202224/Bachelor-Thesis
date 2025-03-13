# Obstacle avoidance app for Crazyflie 2.X

This folder contains the app layer application for the Crazyflie to avoid obstacles, which can be started and configured from the [cfclient](https://github.com/bitcraze/crazyflie-clients-python). 

See App layer API guide [here](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/userguides/app_layer/)

## Build and flash

Make sure that you are in the app_hello_world folder (not the main folder of the crazyflie firmware, in our case in the Firmware folder). Then type the following to build and flash it while the crazyflie is put into bootloader mode:

```
make clean
make 
make cload
```

If you want to compile the application elsewhere in your machine, make sure to update ```CRAZYFLIE_BASE``` in the **Makefile**.
This app is tested with the crazyflie-firmware commit b15eebc864e7e734daa3766d116fa26edbc940a9, but be sure to replace the crazyflie-firmware/src/modules/interface/mem.h file with the <a href="https://git.ee.ethz.ch/pbl/FS2024/laurent_schroeder_392/-/blob/main/Firmware/crazyflie-firmware-directory/mem.h?ref_type=heads">mem.h</a> file in the <a href="https://git.ee.ethz.ch/pbl/FS2024/laurent_schroeder_392/-/tree/main/Firmware/crazyflie-firmware-directory?ref_type=heads">crazyflie-firmware-directory</a> directory and delete this directory afterwards (you don't need it anymore).

## Run

- Turn on the drone  
- Connect to it via <a href="https://www.bitcraze.io/documentation/repository/crazyflie-clients-python/master/">cfclient</a>
- Optional: Adapt params like max vel, height, etc. if wanted (in the Parameters tab, ToF_FLY_PARAMS)
- Optional: Adapt params of the sensor fusion function if wanted (in the Parameters tab, SENS_FUS_Func)
- Enter a number (in seconds) for how long it should fly (it will anyway land once the battery runs out) in the ToFly parameter (in the Parameters tab, ToF_FLY_PARAMS)
- Press enter, it will take off and start flying around!
- Optional: Look at the cmds in the Plotter tab (add a config under "Settings" "logging configurations")