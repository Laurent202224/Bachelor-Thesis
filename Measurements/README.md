# Measurements with the ICU-30201 Sensor

This folder contains the measurements done with the ICU-30201 sensor with different setups. For more details about the measurements, please have a look at this <a href="https://git.ee.ethz.ch/pbl/FS2024/laurent_schroeder_392/-/blob/main/README.md?ref_type=heads">../README.md</a> file.

In order to make a measurement by yourself, follow the following steps:

## Build and Flash

Open a terminal in the <a href="https://git.ee.ethz.ch/pbl/FS2024/laurent_schroeder_392/-/tree/main/Firmware?ref_type=heads">../Firmware</a> folder and execute the following commands while the drone is in the bootloader drone:

```
make clean
make 
make cload
```

## Read Data

- Turn on the drone  
- Open a terminal in the <a href="https://git.ee.ethz.ch/pbl/FS2024/laurent_schroeder_392/-/tree/main/Measurements?ref_type=heads">Measurements</a> folder
- Execute the following command: 

```
python3.11 read-icu30201.py -l test_us.csv -lcf test_cf.csv -t 1
```

This will store the logged data in the test_cf.csv file and the detected amplitude values in the test_us.csv file

## Plot Data

After reading the data, you can visualize it by plotting it with the following commands:

- Open a terminal in the <a href="https://git.ee.ethz.ch/pbl/FS2024/laurent_schroeder_392/-/tree/main/Measurements?ref_type=heads">Measurements</a> folder
- Execute the following command: 

```
python3.11 plot-icu30201.py -l test_us.csv
```

If you stored the test_us.csv file in another folder, replace test_us.csv with the correct path to the file
