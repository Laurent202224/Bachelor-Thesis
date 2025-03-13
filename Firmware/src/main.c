/* 
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Authors: Hanna Müller, Vlad Niculescu, Tommaso Polonelli, Iman Ostovar
 */

#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "app.h"

/* FreeRtos includes */
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include "debug.h"
#include "crtp.h"
#include "vl53l5cx_api.h"
#include "deck.h"
#include "param.h"

//---------------------------Ultrasound test_tof.c----------------------------------

#include "log.h"
#include "mem.h"
#include <stdio.h>
#include <invn/soniclib/soniclib.h>			// Chirp SonicLib sensor API definitions
#include "chirp_board_config.h"	// required header with basic device counts etc.
#include "app_config.h"
#include "app_version.h"
#include <invn/soniclib/chirp_bsp.h>			// board support package function definitions

//-------------------Custom libraries-------------------//
#include "I2C_expander.h"
#include "ToF_process.h"
#include "Fly_control.h"

#define AVG_FILTER_N 5 //filter over more samples to increase reliability of ultrasound sensor

static float THRESHOLD_LEVEL = 0; //gets computed later on during calibration

#define INTERVAL_DIRECTION_CHANGE 10000 // in Systick (ms at the moment of writing)

volatile uint8_t execution_started = 0;

uint16_t avg_filtered_amp[APP_DATA_MAX_SAMPLES] = {0};

static float ringdown_filter[CHIRP_RINGDOWN_FILTER_SAMPLES] = {0};

#if (OUTPUT_AMP_LOG || OUTPUT_IQ_LOG)
#include "time.h"
#endif

/* Number of LEDs to blink */
#define NUM_LEDS			(3)

/* Measurement number to use in this example */
#define MEAS_NUM			(CH_DEFAULT_MEAS_NUM)

/* Check build option dependencies */
#if ((OUTPUT_AMP_DATA_CSV + OUTPUT_AMP_LOG + OUTPUT_IQ_DATA + OUTPUT_IQ_LOG) > 1)
#error Cannot specify multiple output options at the same time (OUTPUT_AMP_xxx or OUTPUT_IQ_xxx).
#endif

#if (OUTPUT_AMP_DATA_CSV || OUTPUT_AMP_LOG)
#define READ_AMPLITUDE_DATA  1
#else
#define READ_AMPLITUDE_DATA	 0
#endif

#ifndef READ_IQ_DATA
#if (OUTPUT_IQ_DATA || OUTPUT_IQ_LOG || OUTPUT_IQ_DATA_SUMMARY)
#define READ_IQ_DATA		1
#else
#define READ_IQ_DATA		1
#endif
#endif


/* chirp_data_t - Structure to hold measurement data for one sensor
 *   This structure is used to hold the data from one measurement cycle from
 *   a sensor.  The data values include the measured range, the ultrasonic
 *   signal amplitude, the number of valid samples  in the measurement, and
 *   (optionally) the full amplitude data or raw I/Q data
 *   from the measurement.
 *
 *  The format of this data structure is specific to this example application, so
 *  you may change it as desired.
 *
 *  A "chirp_data[]" array of these structures, one for each possible sensor,
 *  is declared in the main.c file.  The sensor's device number is
 *  used to index the array.
 */
typedef struct
{
	uint8_t	 rx_sensor_num;				// receiving sensor number
	uint8_t	 tx_sensor_num;				// transmitting sensor number
	uint32_t range;						// from ch_get_range()
	uint16_t amplitude;					// from ch_get_amplitude()
	uint16_t num_samples;				// from ch_get_num_samples()
#if READ_AMPLITUDE_DATA
	uint16_t amp_data[APP_DATA_MAX_SAMPLES];
										// from ch_get_amplitude_data()
#endif
#if READ_IQ_DATA
	ch_iq_sample_t	iq_data[APP_DATA_MAX_SAMPLES];
										// from ch_get_iq_data()
#endif
} chirp_data_t;

// static volatile uint8_t rawDataArray[2][sizeof(ch_iq_sample_t)*APP_DATA_MAX_SAMPLES]; // let's try this without double buffering, I think the iq_data should be updated all at once without other tasks jumping in between
static volatile uint8_t rawDataArrayMemFrame[sizeof(ch_iq_sample_t)*APP_DATA_MAX_SAMPLES*2]; // TODO remove hardcoded 2 sensors
// static uint8_t dblBuf = 0;

// Handling from the memory module
static uint32_t handleMemGetSize(void) { return sizeof(ch_iq_sample_t)*APP_DATA_MAX_SAMPLES; }
static bool handleMemRead(const uint32_t memAddr, const uint8_t readLen, uint8_t* buffer);
static const MemoryHandlerDef_t memDef = {
  .type = MEM_TYPE_ICU30201,
  .getSize = handleMemGetSize,
  .read = handleMemRead,
  .write = 0, // Write not supported
};

#ifndef INCLUDE_ALGO_EXTERNAL
/* Detection level settings
 *   This structure is passed to the ch_meas_init() function to set the target
 *   detection thresholds.  Each threshold entry consists of a starting sample
 *   number and a threshold amplitude level.  The values are defined in app_config.h.
 */
ch_thresholds_t chirp_detect_thresholds = {
	.threshold = {
		{CHIRP_THRESH_0_START, CHIRP_THRESH_0_LEVEL},      /* threshold 0 */
		{CHIRP_THRESH_1_START, CHIRP_THRESH_1_LEVEL},      /* threshold 1 */
		{CHIRP_THRESH_2_START, CHIRP_THRESH_2_LEVEL},      /* threshold 2 */
		{CHIRP_THRESH_3_START, CHIRP_THRESH_3_LEVEL},      /* threshold 3 */
		{CHIRP_THRESH_4_START, CHIRP_THRESH_4_LEVEL},      /* threshold 4 */
		{CHIRP_THRESH_5_START, CHIRP_THRESH_5_LEVEL},      /* threshold 5 */
		{CHIRP_THRESH_6_START, CHIRP_THRESH_6_LEVEL},      /* threshold 6 */
		{CHIRP_THRESH_7_START, CHIRP_THRESH_7_LEVEL},      /* threshold 7 */
		}
};
ch_thresholds_t *chirp_detect_thresholds_ptr = &chirp_detect_thresholds;
#else
ch_thresholds_t *chirp_detect_thresholds_ptr = NULL;
#endif

#if  IMPORT_MEASUREMENT
	/* Imported measurement definition from measurement_config.c */
	extern measurement_queue_t    measurement_config_queue;
	extern ICU_ALGO_SHASTA_CONFIG measurement_config_cfg;
#else
	/* Measurement configuration struct - starting/default values */
	static ch_meas_config_t  meas_config = {
			.num_ranges 			= CHIRP_MAX_TARGETS,
			.odr 					= CHIRP_SENSOR_ODR,
			.meas_period 			= 0,
			.ringdown_cancel_samples = CHIRP_RINGDOWN_FILTER_SAMPLES,
			.static_filter_samples 	= 0,			// may be changed later using ch_set_static_range()
			.iq_output_format 		= CH_OUTPUT_IQ, 	/* return (Q, I) */
			.filter_update_interval = 0, 		/* update filter every sample */
	};
#endif

/* Array of structs to hold measurement data, one for each possible device */
chirp_data_t chirp_data[CHIRP_MAX_NUM_SENSORS];

/* Array of ch_dev_t device descriptors, one for each possible device */
static ch_dev_t chirp_devices[CHIRP_MAX_NUM_SENSORS];

/* Descriptor structure for group of sensors */
ch_group_t chirp_group;

/* Task flag word
 *   This variable contains the TIMER_FLAG, DATA_READY_FLAG and READ_DONE_FLAG
 *   bit flags that are set in interrupt callback routines.  The flags are
 *   checked in the main() loop and, if set, will cause an appropriate function to
 *   be called to process sensor data.
 */

volatile uint32_t taskflags = 0;

/* Device tracking variables
 *   These are bit-field variables which contain a separate bit assigned to
 *   each (possible) sensor, indexed by the device number.  The active_devices
 *   variable contains the bit pattern describing which ports have active
 *   sensors connected.  The data_ready_devices variable is set bit-by-bit
 *   as sensors interrupt, indicating they have completed a measurement
 *   cycle.  The two variables are compared to determine when all active
 *   devices have interrupted.
 */
static uint32_t active_devices;
static uint32_t data_ready_devices;

/* Number of connected sensors */
static uint8_t num_connected_sensors = 0;  //maybe put to 1??

/* Number of sensors that use external triggering to start measurement */
static uint8_t num_triggered_sensors = 0; //maybe put to 1??


#if (READ_DATA_NONBLOCKING  && (READ_AMPLITUDE_DATA || READ_IQ_DATA))
/* Count of non-blocking data reads queued */
static uint16_t	num_io_queued = 0;
#endif

#if (OUTPUT_AMP_LOG || OUTPUT_IQ_LOG)
/* Data log support - only needed if using formal logging interface */
#define ODR_TO_DECIMATION(odr) ((odr == CH_ODR_FREQ_DIV_32) ? 4 : \
								((odr == CH_ODR_FREQ_DIV_16) ? 2 : 1))	// default = 1

static uint8_t log_id = 0;

ch_log_cfg_t log_config = {
	.interval_ms = MEASUREMENT_INTERVAL_MS,
	.output_type = (OUTPUT_AMP_LOG == 1) ? CH_OUTPUT_AMP : CH_OUTPUT_IQ,
	.decimation_factor = ODR_TO_DECIMATION(CHIRP_SENSOR_ODR),
	.start_sample = 0,
};
#endif //  (OUTPUT_AMP_LOG || OUTPUT_IQ_LOG)


/* Forward declarations */
static uint8_t configure_sensors(ch_group_t *grp_ptr, uint8_t num_ports, uint32_t *active_devices_ptr,
		                         uint8_t *num_connected_ptr, uint8_t *num_triggered_ptr);
float *handle_data_ready(ch_group_t *grp_ptr);
float handle_data_ready_return_avg_amp(ch_group_t *grp_ptr); //AVG_CALIB
static void    sensor_int_callback(ch_group_t *grp_ptr, uint8_t dev_num, ch_interrupt_type_t int_type);
static void    periodic_timer_callback(void);
static void ultrasound_data_collection_task_f(void *);
FlyCommand_t Decision_Making_US(float* log_range);
static void ringdown_filter_define(ch_group_t *grp_ptr);
//void handle_data_ready_return_max_amp(ch_group_t *grp_ptr); //AVG_MAX

#if (READ_AMPLITUDE_DATA || READ_IQ_DATA)
static uint8_t read_measurement_data(ch_dev_t *dev_ptr);
#endif
#if (OUTPUT_AMP_DATA_CSV || OUTPUT_IQ_DATA)
static void    display_sensor_data(chirp_data_t * sensor_data);
#endif
#if (OUTPUT_AMP_LOG || OUTPUT_IQ_LOG)
static void    log_sensor_data(chirp_data_t * sensor_data);
#endif
#if (READ_DATA_NONBLOCKING  && (READ_AMPLITUDE_DATA || READ_IQ_DATA))
static void    io_complete_callback(ch_group_t *grp_ptr);
static uint8_t handle_read_done(ch_group_t *grp_ptr);
#endif


//---------------------------ToF test_tof.c-----------------------------------------



#define  ARM_CM_DEMCR      (*(uint32_t *)0xE000EDFC)
#define  ARM_CM_DWT_CTRL   (*(uint32_t *)0xE0001000)
#define  ARM_CM_DWT_CYCCNT (*(uint32_t *)0xE0001004)

#define DEBUG_MODULE "HELLOWORLD"

//-------------------Control MAcros-------------------//
#define SESNSOR_FORAWARD_ENABLE

// #define SEND_DATA
#define ON_BOARD_PROCESS
#define START_FLIGHT



//-------------------Global Defines-------------------//
//Sensors Addresses
#define VL53L5CX_FORWARD_I2C_ADDRESS            ((uint16_t)(VL53L5CX_DEFAULT_I2C_ADDRESS*4))
#define VL53L5CX_BACKWARD_I2C_ADDRESS            ((uint16_t)(VL53L5CX_FORWARD_I2C_ADDRESS+2))
//Data Length
// #define ToF_DISTANCES_LEN             (128)
// #define ToF_TARGETS_DETECTED_LEN      (64)
// #define ToF_TARGETS_STATUS_LEN        (64)
#define FRONT_SENSOR_OFFSET        (0)
#define BACK_SENSOR_OFFSET        (ToF_DISTANCES_LEN+ToF_TARGETS_DETECTED_LEN+ToF_TARGETS_STATUS_LEN)



//-------------------Global Variables-------------------//
#ifdef SESNSOR_FORAWARD_ENABLE
static VL53L5CX_Configuration vl53l5dev_f;
static VL53L5CX_ResultsData vl53l5_res_f;
#endif


// IRQ
volatile uint8_t irq_status = 0;
//uint8_t vl53l5_buffer[VL53L5CX_MAX_RESULTS_SIZE];

uint32_t timestamp;
uint8_t get_data_success_f = false;

//-------------------Functions defins-------------------//
void send_command(uint8_t command, uint8_t arg);
void send_data_packet(uint8_t *data, uint16_t data_len);
void send_data_packet_28b(uint8_t *data, uint8_t size, uint8_t index);
//
bool initialize_sensors_I2C(VL53L5CX_Configuration *p_dev, uint8_t mode);
bool config_sensors(VL53L5CX_Configuration *p_dev, uint16_t new_i2c_address);
bool get_sensor_data(VL53L5CX_Configuration *p_dev,VL53L5CX_ResultsData *p_results);
static void tof_data_collection_task_f(void *);

//
#ifdef START_FLIGHT
uint16_t ToFly = 0; //default configuration if START_FLIGHT hasn't been defined, otherwise #else will be compiled
#else
uint16_t ToFly = 1;
#endif

//------------------------------structs & flags for data of sensors---------------------------//
static float log_range[2] = {0.0f, 0.0f}; //use this to store ultrasound measurement
static float meas_rate = 0.0;
static uint32_t time_last_cmd = 0;
float yaw_direction = 1.0;  //turning direction (to left or right)
float vel_x_old = 0.0;
static float MAX_RANGE = 4.1; // just default, will be computed from pmut frequency

/*
If you want to calibrate the ultrasound sensor by computing only the average of the NR_MAX_AMPL maximum amplitude samples,
then uncomment all the lines with AVG_MAX commented and comment out all the lines with AVG_CALIB.
*/
// #define NR_MAX_AMPL 35 //AVG_MAX //number of maximum amplitudes to average over
// float max_amp_array[NR_MAX_AMPL] = {0}; //AVG_MAX
// float min_amp_array = 0; //AVG_MAX
#define NB_LAST_US_MEAS 5 //The last NB_LAST_US_MEAS measurements of the ultrasound sensorget use in the decision tree of the obstacle avoidance algorithm


target_t nearest_ToF_target;
bool ToF_preprocess_ready = false;
bool start = false;
bool start_OA_algorithm = false;

float ultrasound_meas[2] = {0.0f, 0.0f};
float *ultrasound_meas_res = ultrasound_meas;
//float max_5_amp[NR_MAX_AMPL] = {0}; //AVG_MAX
//float *max_5_amp_res = max_5_amp; //AVG_MAX
static uint8_t ret_val = 0;
bool ultrasound_preprocess_ready = false;

////////////////////////////////////////sensor fusion defines//////////////////////////////////
#define ULTRASOUND_FF_STATUS_UNDEFINED 0
#define ULTRASOUND_FF_STATUS_CONN_WORKS 1
#define ULTRASOUND_FF_STATUS_NOT_CONN 2
#define ULTRASOUND_FF_STATUS_CONN_NOT_WORKS_NOT_FLY 3
#define ULTRASOUND_FF_STATUS_CONN_NOT_WORKS_FLY 4
#define ULTRASOUND_FF_STATUS_READY_TO_CALIB 5
uint8_t ultrasound_status = ULTRASOUND_FF_STATUS_UNDEFINED; //can define more if we also include the ultrasound sensor which points backwards

#define ToF_FF_STATUS_UNDEFINED 0
#define ToF_FF_STATUS_CONN_WORKS 1
#define ToF_FF_STATUS_NOT_CONN 2
#define ToF_FF_STATUS_READY_START 3
uint8_t tof_status = ToF_FF_STATUS_UNDEFINED;
/////////////////////////////////////////sensor fusion defines/////////////////////////////////

/////////////////////////////////////////MUTEXES//////////////////////////////////////////////
static SemaphoreHandle_t ToF_read;
static SemaphoreHandle_t ultrasound_read;

//MAIN
void appMain()
{ 
  memoryRegisterHandler(&memDef); // have to do this early, as registration of handlers has to be done before they can be queried
  vTaskPrioritySet( NULL, 0 );

  xTaskCreate(tof_data_collection_task_f, "ToFDataTask", 7*configMINIMAL_STACK_SIZE, NULL, 0, NULL); //initialize task preprocessing ToF sensor
  xTaskCreate(ultrasound_data_collection_task_f, "UltrasoundDataCollection", 6*configMINIMAL_STACK_SIZE, NULL, 0, NULL); //initialize task preprocessing ultrasound sensor

  vTaskDelay(M2T(3000)); //no rush to start, let both sensors innitialize themselves

	DEBUG_PRINT("Start main task\n");

	FlyCommand_t flight_command;

	while (ultrasound_status == ULTRASOUND_FF_STATUS_UNDEFINED || tof_status == ToF_FF_STATUS_UNDEFINED) { //Maybe unnecessary
		vTaskDelay(M2T(200));
	}
	while ((ultrasound_status != ULTRASOUND_FF_STATUS_READY_TO_CALIB && ultrasound_status != ULTRASOUND_FF_STATUS_NOT_CONN) && (tof_status != ToF_FF_STATUS_READY_START && tof_status != ToF_FF_STATUS_NOT_CONN)) { //wait for ultrasound and tof task to know which sensor is connected
		DEBUG_PRINT("ultrasound_status: %d\n", ultrasound_status);
		DEBUG_PRINT("tof_status: %d\n", tof_status);
		vTaskDelay(M2T(1000));
	} 

	if (ultrasound_status == ULTRASOUND_FF_STATUS_NOT_CONN && tof_status == ToF_FF_STATUS_NOT_CONN) { //no sensor connected, cannot fly
		DEBUG_PRINT("No sensor connected, cannot fly\n");
		while(1) {
			vTaskDelay(M2T(10000));
		}
	}
	else if (ultrasound_status == ULTRASOUND_FF_STATUS_CONN_NOT_WORKS_NOT_FLY) {
		DEBUG_PRINT("Ultrasound doesn't work, restart again\n");
		while (1) {
			vTaskDelay(M2T(10000));
		}
	}

	while(ToFly == 0)
    vTaskDelay(200);
  #ifdef START_FLIGHT
    fly_task_take_off();
    // fly_task((FlyCommand_t){take_off,0.0f});
	start = true;
  #endif
	
	FlyCommand_t flight_command_new;

	if (ultrasound_status != ULTRASOUND_FF_STATUS_NOT_CONN) { //checks if the ultrasound sensor is actually connected
		while (ultrasound_status != ULTRASOUND_FF_STATUS_CONN_WORKS) { //wait for the calibration and ringdown to complete before starting the OA algorithm
			flight_command_new = (FlyCommand_t) {0.0f, 0.0f, 0.0f};
			bool flight_status = fly_task(flight_command_new);
			if (ultrasound_status == ULTRASOUND_FF_STATUS_CONN_NOT_WORKS_FLY) { //ringdown didn't work and we want to land in order to restart afterwards
				fly_task_land();
				//DEBUG_PRINT("Ringdown not worked, landing\n");
				while (1) {
					vTaskDelay(M2T(10000));
				}
			}
			vTaskDelay(M2T(100));
		}
	}
	while (tof_status != ToF_FF_STATUS_CONN_WORKS && tof_status != ToF_FF_STATUS_NOT_CONN) { //only necessary to wait for tof when only tof sensor is connected
		vTaskDelay(M2T(50));
	}
	//if we get until here, everything is set up to start the OA algorithm

	if (tof_status == ToF_FF_STATUS_CONN_WORKS && ultrasound_status == ULTRASOUND_FF_STATUS_CONN_WORKS) {
		DEBUG_PRINT("\nToF and Ultrasound sensor connected and working\n");
		vTaskDelay(M2T(270)); //wait for the ToF sensor to initialize itself and have its first measurement ready
	}
	else if (tof_status == ToF_FF_STATUS_CONN_WORKS) {
		DEBUG_PRINT("\nOnly ToF sensor is connected and working\n");
	}
	else {
		DEBUG_PRINT("\nOnly ultrasound sensor is connected and working\n");
	}

	start_OA_algorithm = true;
	DEBUG_PRINT("Start OA algorithm\n");
	bool flight_status = true; //flying
	float ultrasound_meas_ff[NB_LAST_US_MEAS] = {1, 1, 1, 1, 1}; //not too high not that the loss function blows up and not too small such that the loss function doesn't become to high
	// for (int i = 0; i<5; i++) { //for debugging
	// 	DEBUG_PRINT("ultrasound_meas_ff[%d]: %f", i, ultrasound_meas_ff[i]);
	// }
	static uint8_t us_meas_ff_index = 0;
	bool something_changed = false; //if new measurement is ready
	
	while(1) {

		if (tof_status == ToF_FF_STATUS_CONN_WORKS && ultrasound_status == ULTRASOUND_FF_STATUS_CONN_WORKS) { //both sensors are connected
			if (ToF_preprocess_ready && xSemaphoreTake(ToF_read, 0) == pdTRUE) { //make sure nothing gets written in the nearest_ToF_target variable
				//update the ToF array command (nearest_ToF_target)
				ToF_preprocess_ready = false;
				something_changed = true;
				xSemaphoreGive(ToF_read);
			}
			if (ultrasound_preprocess_ready && xSemaphoreTake(ultrasound_read, 0) == pdTRUE) { //make sure nothing gets written in the ultrasound_meas_res array
				//update the us array command, store the distance
				ultrasound_meas_ff[us_meas_ff_index++] = log_range[0];
				us_meas_ff_index %= NB_LAST_US_MEAS;
				// for (int i = 0; i<5; i++) { //for debugging
				// 	DEBUG_PRINT("ultrasound_meas_ff[%d]: %f", i, ultrasound_meas_ff[i]);
				// }
				ultrasound_preprocess_ready = false;
				something_changed = true;
				xSemaphoreGive(ultrasound_read);
			}
			if (something_changed) { //if we have new data recompute the flight command
				flight_command_new = Decision_Making_ToF_and_US(ultrasound_meas_ff, nearest_ToF_target, (us_meas_ff_index-1)%NB_LAST_US_MEAS);
				something_changed = false;
			}
			else {
				flight_command_new = flight_command;
			}
		}

		else if (tof_status == ToF_FF_STATUS_CONN_WORKS) { //only the ToF sensor is connected
			if (ToF_preprocess_ready && xSemaphoreTake(ToF_read, 0) == pdTRUE) { //make sure nothing gets written in the nearest_ToF_target variable
				if (nearest_ToF_target.min_distance != UINT16_MAX) {  //make sure the ToF sensor has detected an object
					DEBUG_PRINT("Distance %d\n", nearest_ToF_target.min_distance);
					flight_command_new = Decision_Making_ToF(nearest_ToF_target); //only ToF decision
				}
				else {
					flight_command_new = (FlyCommand_t) {1.0f, 0.0f, 0.0f}; //no object detected, so full speed (1m/s) forward without turning
				}
				ToF_preprocess_ready = false;
				xSemaphoreGive(ToF_read);
			}
			else {
				flight_command_new = flight_command;
			}
		}
		else if (ultrasound_status == ULTRASOUND_FF_STATUS_CONN_WORKS) { //only ultrasound connected
			if (ultrasound_preprocess_ready && xSemaphoreTake(ultrasound_read, 0) == pdTRUE) {  //use only for ultrasound alone
				flight_command_new = Decision_Making_US(log_range);
				ultrasound_preprocess_ready = false;
				xSemaphoreGive(ultrasound_read); //watch out for race conditions
				//DEBUG_PRINT("flight_command: %f, %f, %f\n", flight_command_new.command_velocity_x, flight_command_new.command_velocity_z, flight_command_new.command_turn); //for debugging
			}
			else {
				flight_command_new = flight_command;
			}
		}

		if (flight_status) {
            flight_status = fly_task(flight_command_new);
        }

		flight_command = flight_command_new;
		vTaskDelay(M2T(5));
	}
}


//------------------------------------ToF data collection & preprocessing task----------------------------------

void tof_data_collection_task_f(void *pvParameters) { //-------------------works alone------------------
    (void) pvParameters; // Unused parameter
	DEBUG_PRINT("Started ToF task\n");

	ToF_read = xSemaphoreCreateMutex(); //initialize mutex

	//-----------------------------------Initilaize The Deck -----------------------------------------------------//
	bool gpio_exp_status = false;
	bool sensors_status = true;
	gpio_exp_status = I2C_expander_initialize();
	DEBUG_PRINT("ToFDeck I2C_GPIO Expander: %s\n", gpio_exp_status ? "OK." : "ERROR!");  
	#ifdef SESNSOR_FORAWARD_ENABLE
		vTaskDelay(M2T(100)); 
		sensors_status = initialize_sensors_I2C(&vl53l5dev_f,1); //forward
		DEBUG_PRINT("ToFDeck Forward Sensor Initialize 1: %s\n", sensors_status ? "OK." : "ERROR!");
	#endif


	if(gpio_exp_status == false || sensors_status == false)
	{
		DEBUG_PRINT("ERROR LOOP_1!"); 
		tof_status = ToF_FF_STATUS_NOT_CONN;
		vTaskDelete(NULL);
		while (1)
		{//stay in ERROR LOOP
		vTaskDelay(M2T(10000)); 
		}
	}

	DEBUG_PRINT("ToFDeck GPIO & Interrupt Initialized. \n");  
	tof_status = ToF_FF_STATUS_READY_START;

  //------------------------------------ToF sensor initially finished----------------------------------

	//-----------------------------------Take Drone Off -----------------------------------------------------//

  while (!start) { //gets set after takeoff
	vTaskDelay(M2T(200));
  }

  //-----------------------------------Start Sensors Ranging -----------------------------------------------------//
  #ifdef SESNSOR_FORAWARD_ENABLE
    vTaskDelay(M2T(100));
    uint64_t t0 = xTaskGetTickCount();
    uint8_t ranging_start_res_f = vl53l5cx_start_ranging(&vl53l5dev_f);
    uint64_t t1 = xTaskGetTickCount();
    DEBUG_PRINT("Time start ranging %d. \n", (uint32_t)(t1-t0));  


    DEBUG_PRINT("ToFDeck Start Sensor Forward Ranging: %s\n", (ranging_start_res_f == VL53L5CX_STATUS_OK) ? "OK." : "ERROR!"); 
  #else
    uint8_t ranging_start_res_f = VL53L5CX_STATUS_OK;
  #endif

  if(ranging_start_res_f != VL53L5CX_STATUS_OK){
    DEBUG_PRINT("ERROR LOOP_2!"); 
    while (1)
    {//stay in ERROR LOOP
      vTaskDelay(M2T(10000));  //If ToF sensor is not connect we remain in this while loop
    }
  }
  tof_status = ToF_FF_STATUS_CONN_WORKS;

  //-----------------------------------Collect and Send Data------------------------------------------------------//

  #ifdef SESNSOR_FORAWARD_ENABLE
    //go into ifdef if we send some data, otherwise not
    #ifdef SEND_DATA 
      uint8_t to_send_buffer_f[ToF_DISTANCES_LEN+ToF_TARGETS_DETECTED_LEN+ToF_TARGETS_STATUS_LEN];
    #endif
  #endif
  DEBUG_PRINT("ToF ready\n");
  while (ultrasound_status != ULTRASOUND_FF_STATUS_NOT_CONN && ultrasound_status != ULTRASOUND_FF_STATUS_CONN_WORKS) { //wait until the ultrasound is ready (calibrated and ringdown filter defined) or not connected at all
	vTaskDelay(M2T(200));
  }

  while(1) {
    if (ToFly == 0){ //TODO: probably unnecessary
      fly_task_land();
      break;
    }
	vTaskDelay(M2T(67)); // Task Delay, we want to run this task at 15Hz, maximum frequency at which the ToF sensor can run in continuous mode

    // Collect Data
    #ifdef SESNSOR_FORAWARD_ENABLE
      get_data_success_f = get_sensor_data(&vl53l5dev_f, &vl53l5_res_f); //if we get some new data from the ToF, then we send new commands to our commander (adapt the commands, by including the data received from the ultrasound sensor)
      if (get_data_success_f == true)
      {
        
        #ifdef SEND_DATA
          //
          send_command(1, (ToF_DISTANCES_LEN+ToF_TARGETS_DETECTED_LEN+ToF_TARGETS_STATUS_LEN)/28 + 1);  
          //
          memcpy(&to_send_buffer_f[0], (uint8_t *)(&vl53l5_res_f.distance_mm[0]), ToF_DISTANCES_LEN);
          memcpy(&to_send_buffer_f[ToF_DISTANCES_LEN], (uint8_t *)(&vl53l5_res_f.nb_target_detected[0]), ToF_TARGETS_DETECTED_LEN);
          memcpy(&to_send_buffer_f[ToF_DISTANCES_LEN+ToF_TARGETS_DETECTED_LEN], (uint8_t *)(&vl53l5_res_f.target_status[0]), ToF_TARGETS_STATUS_LEN);
          //
          send_data_packet(&to_send_buffer_f[0], ToF_DISTANCES_LEN+ToF_TARGETS_DETECTED_LEN+ToF_TARGETS_STATUS_LEN);
        #endif

        #ifdef ON_BOARD_PROCESS
		  if (xSemaphoreTake(ToF_read, 0) == pdTRUE) { //watch out for race conditions
			nearest_ToF_target = Process_ToF_Image(&vl53l5_res_f); //where the magic happens
			ToF_preprocess_ready = true;
			xSemaphoreGive(ToF_read);
		  }
        #endif

        // Clear Flag
        get_data_success_f = false;

      }
    #endif
    }
}


// //-----------------------------------Finish ToF data collection & preprocessing task----------------------------




// //-----------------------------------Ultrasound_f data collection & preprocessing task--------------------------
#define AMP_CALIBRATE_FILTER_LEN 340 //define the length over how many samples the calibration gets done (340 is maximum)
float amp_array[AMP_CALIBRATE_FILTER_LEN] = {0};
void ultrasound_data_collection_task_f(void *pvParameters) {
	//DEBUG_PRINT("Ultrasound task started\n");
	   //------------------------Ultrasound deck initializing----------------------------
  DEBUG_PRINT("Started ultrasound task\n"); 

  ch_group_t *grp_ptr;			// pointer to group descriptor
	ch_dev_t *dev_ptr;				// pointer to individual device descriptor
	uint8_t chirp_error = 0;
	uint8_t num_ports;
	uint8_t dev_num;
	bool ultrasound_connected = false;
	ultrasound_read = xSemaphoreCreateMutex(); //initialize mutex

	/* Initialize board hardware functions
	 *   This call to the board support package (BSP) performs all necessary
	 *   hardware initialization for the application to run on this board.
	 *   This includes setting up memory regions, initializing clocks and
	 *   peripherals (including SPI and serial port), and any processor-specific
	 *   startup sequences.
	 *
	 *   The chbsp_board_init() function also initializes fields within the
	 *   sensor group descriptor, including number of supported sensors and
	 *   the RTC clock calibration pulse length.
	 */
	grp_ptr = &chirp_group;
	chbsp_board_init(grp_ptr);
	vTaskDelay(M2T(10));

	// DEBUG_PRINT("\n\nHello Chirp! - SonicLib Example Application for ICU Sensors\n");
	// DEBUG_PRINT("    Version: %u.%u.%u-%s", APP_VERSION_MAJOR, APP_VERSION_MINOR, APP_VERSION_REV, APP_VERSION_SUFFIX);
	// DEBUG_PRINT("    SonicLib version: %u.%u.%u%s\n", SONICLIB_VER_MAJOR, SONICLIB_VER_MINOR, SONICLIB_VER_REV, SONICLIB_VER_SUFFIX);
	// DEBUG_PRINT("\n");

	/* Get the number of (possible) sensor devices on the board
	 *   Set by the BSP during chbsp_board_init()
	 */
	num_ports = ch_get_num_ports(grp_ptr);

	/* Initialize sensor descriptors.
	 *   This loop initializes each (possible) sensor's ch_dev_t descriptor,
	 *   although we don't yet know if a sensor is actually connected.
	 *
	 *   The call to ch_init() specifies the sensor descriptor, the sensor group
	 *   it will be added to, the device number within the group, and the sensor
	 *   firmware initialization routine that will be used.
	 */
	DEBUG_PRINT("Initializing %d sensor(s)... ", num_ports);

	for (dev_num = 0; dev_num < num_ports; dev_num++) {
		dev_ptr = &(chirp_devices[dev_num]);	// init struct in array
		/* Init device descriptor
		 *   Note that this assumes all sensors will use the same sensor
		 *   firmware as specified by CHIRP_SENSOR_FW_INIT_FUNC in app_config.h.
		 */
		chirp_error |= ch_init(dev_ptr, grp_ptr, dev_num, CHIRP_SENSOR_FW_INIT_FUNC);
	}
	vTaskDelay(M2T(10));

	/* Start all sensors.
	 *   The ch_group_start() function will search each port (that was
	 *   initialized above) for a sensor. If it finds one, it programs it (with
	 *   the firmware specified above during ch_init()) and waits for it to
	 *   perform a self-calibration step.  Then, once it has found all the
	 *   sensors, ch_group_start() completes a timing reference calibration by
	 *   applying a pulse of known length to the sensor's INT line.
	 */
	if (chirp_error == 0) {
		DEBUG_PRINT("starting group... ");
		
		chirp_error = ch_group_start(grp_ptr);
		
	}
 	vTaskDelay(M2T(10)); //no rush to start

	if (chirp_error == 0) {
		DEBUG_PRINT("OK\n");
	} else {
		DEBUG_PRINT("FAILED: %d\n", chirp_error);
		//ultrasound_connected = false;
		ultrasound_status = ULTRASOUND_FF_STATUS_NOT_CONN;
		//calibrated = true;

	}
	DEBUG_PRINT("\n");

	// /* Get and display the initialization results for each connected sensor.
	//  *   This loop checks each device number in the sensor group to determine
	//  *   if a sensor is actually connected.  If so, it makes a series of
	//  *   function calls to get different operating values, including the
	//  *   operating frequency, clock calibration values, and firmware version.
	//  */
	// DEBUG_PRINT("Dev   Type       ID     Freq Hz  B/W Hz   RTC Cal    CPU Freq    Firmware\n");

	for (dev_num = 0; dev_num < num_ports; dev_num++) {
		dev_ptr = ch_get_dev_ptr(grp_ptr, dev_num);
	DEBUG_PRINT("%d\n", dev_num);

	// vTaskDelay(M2T(1000)); //give the enormous print time
		if (ch_sensor_is_connected(dev_ptr)) { //ultrasound sensor is connected
	DEBUG_PRINT("is connected\n");
			// DEBUG_PRINT(" %d  ICU-%d  %-7s  %6lu    %4u   %u@%ums  %0.2fMHz  %s\n", dev_num,
			// 		ch_get_part_number(dev_ptr),
			// 		ch_get_sensor_id(dev_ptr),
			// 		ch_get_frequency(dev_ptr),
			// 		ch_get_bandwidth(dev_ptr),
			// 		ch_get_rtc_cal_result(dev_ptr),
			// 		ch_get_rtc_cal_pulselength(dev_ptr),
			// 		(ch_get_cpu_frequency(dev_ptr) / 1000000.0f),
			// 		ch_get_fw_version_string(dev_ptr));
			// DEBUG_PRINT(" %d  ICU-%d  \n",
			// 		ch_get_part_number(dev_ptr),
			// 		ch_get_sensor_id(dev_ptr));
			ultrasound_connected = true;
		}
	}
	if (!ultrasound_connected) {
		ultrasound_status = ULTRASOUND_FF_STATUS_NOT_CONN;
	}
	// DEBUG_PRINT("\n");
	// vTaskDelay(M2T(1000)); //give the enormous print time

// 	/* Register callback function to be called when Chirp sensor interrupts */
	ch_io_int_callback_set(grp_ptr, sensor_int_callback);

// #if (READ_DATA_NONBLOCKING  && (READ_AMPLITUDE_DATA || READ_IQ_DATA))
// 	/* Register callback function called when non-blocking data readout completes */
// 	ch_io_complete_callback_set(grp_ptr, io_complete_callback);
// #endif

	/* Configure each sensor with its operating parameters */
 	vTaskDelay(M2T(200)); //no rush to start
	
	chirp_error = configure_sensors(grp_ptr, num_ports, &active_devices,
			&num_connected_sensors, &num_triggered_sensors);
 	vTaskDelay(M2T(100)); //no rush to start
	dev_ptr = ch_get_dev_ptr(grp_ptr, 0);
	MAX_RANGE = 340.0/dev_ptr->orig_pmut_freq*4.0*343.0/2.0; //TODO fix for multiple sensors and different ODRs 

	if (!chirp_error) {
		DEBUG_PRINT("\nStarting measurements...\n");
	}
	
	//-----------------------------moved to task-------------------------
	taskflags |= TIMER_FLAG;
	int timeout_counter = 0;
	uint32_t start_time = xTaskGetTickCount();
	uint32_t meas_counter = 0;
	//-----------------------------moved to task f-----------------------
  //--------------------------Finish-----------------------------------
	DEBUG_PRINT("Test ultrasound sensor is working\n");
	int counter = 0;
	for (int i=0; i < 202; i++) { //test sensor works
			timeout_counter++;
    	// 	vTaskDelay(M2T(10));
		// 	// chbsp_proc_sleep();			// put processor in low-power sleep mode
		// 	/* We only continue here after an interrupt wakes the processor */
		// }
		if ((taskflags & (INTERRUPT_FLAG_0 | INTERRUPT_FLAG_1)) == (INTERRUPT_FLAG_0 | INTERRUPT_FLAG_1))
		{
			// DEBUG_PRINT("ch_interrupt_triggered taskflags: 0x%x 0x%x\n", taskflags, (INTERRUPT_FLAG_0 | INTERRUPT_FLAG_1));
			taskflags &= ~(INTERRUPT_FLAG_0 | INTERRUPT_FLAG_1);
			// vTaskDelay(M2T(100));
			ch_interrupt_triggered();
		}
		

		// vTaskDelay(M2T(10)); 
		/* Check for sensor data-ready interrupt(s) */
		if (taskflags & DATA_READY_FLAG) {
			ch_group_trigger(grp_ptr); // trigger sensor
			/* All sensors have interrupted - handle sensor data */
			taskflags &= ~DATA_READY_FLAG;				// clear flag
			//DEBUG_PRINT("Before read\n");
			ultrasound_meas_res = handle_data_ready(grp_ptr);					// read and display measurement
			//DEBUG_PRINT("After read\n");
			// taskflags |= TIMER_FLAG;
			meas_counter++;
			meas_rate = (xTaskGetTickCount() - start_time)/meas_counter;
			timeout_counter = 0;


#if ((READ_AMPLITUDE_DATA || READ_IQ_DATA)  && READ_DATA_NONBLOCKING)
            /* Start any pending non-blocking data reads */
            if (num_io_queued != 0) {
                ch_io_start_nb(grp_ptr);
                num_io_queued = 0;
            }
#endif
        }  // end if (taskflags & DATA_READY_FLAG)

		if (taskflags & TIMER_FLAG) {
			taskflags &= ~TIMER_FLAG;
			taskflags = 0;
			execution_started = 1;

			ch_group_trigger(grp_ptr); // trigger sensor
			vTaskDelay(M2T(26)); // takes 26ms to measure
			
			timeout_counter = 0;
		}
	


#if ((READ_AMPLITUDE_DATA || READ_IQ_DATA)  && READ_DATA_NONBLOCKING)
		/* Check for non-blocking data read complete */
		if (taskflags & READ_DONE_FLAG) {

			/* All non-blocking data readouts have completed */
			taskflags &= ~READ_DONE_FLAG;		// clear flag
			handle_read_done(grp_ptr);			// display measurement data
		}
#endif
		if(timeout_counter >= 100)
		{
			taskflags |= TIMER_FLAG;
			timeout_counter = 0;
			DEBUG_PRINT("timeout\n");  //often get this
			counter += 1;
		}
		vTaskDelay(M2T(5));
	}

	if (counter < 2) {
		DEBUG_PRINT("\nUltrasound works\n");
	}
	else {
		//DEBUG_PRINT("Too many timeouts, ultrasound not working, restart\n");
		if (ultrasound_connected) {
			ultrasound_status = ULTRASOUND_FF_STATUS_CONN_NOT_WORKS_NOT_FLY;
		}
		else {
			DEBUG_PRINT("Ultrasound not connected\n");
		}
		vTaskDelete(NULL);
		DEBUG_PRINT("Shouldn't print this\n");
		while(1) {
			vTaskDelay(M2T(200));
		}
	}
	DEBUG_PRINT("If you start the flight, the ultrasound sensor first gets calibrated\n");
	ultrasound_status = ULTRASOUND_FF_STATUS_READY_TO_CALIB;
	

	/*
	If you want to calibrate with the average over AMP_CALIBRATE_FILTER_LEN samples,
	 the uncomment every line with the comment //AVG_CALIB an coment every line with
	  the comment //AVG_MAX*/
	float NOISE_AVG = 0; //AVG_CALIB
	while (!start) { //gets set in the main task
		vTaskDelay(M2T(200));
	}
	
	//////////////////////////////Calibration starts///////////////////////////////////
	float amplitude_avg = 0; //AVG_CALIB

	taskflags |= TIMER_FLAG;
	timeout_counter = 0;
	start_time = xTaskGetTickCount();
	meas_counter = 0;
	uint16_t counter_amp = 0;

	DEBUG_PRINT("Calibation starts\n");
	for (int i = 0; i<500;i++) {
		timeout_counter++;
    	// 	vTaskDelay(M2T(10));
		// 	s// chbsp_proc_sleep();			// put processor in low-power sleep mode
		// 	/* We only continue here after an interrupt wakes the processor */
		// }
		if ((taskflags & (INTERRUPT_FLAG_0 | INTERRUPT_FLAG_1)) == (INTERRUPT_FLAG_0 | INTERRUPT_FLAG_1))
		{
			// DEBUG_PRINT("ch_interrupt_triggered taskflags: 0x%x 0x%x\n", taskflags, (INTERRUPT_FLAG_0 | INTERRUPT_FLAG_1));
			taskflags &= ~(INTERRUPT_FLAG_0 | INTERRUPT_FLAG_1);
			// vTaskDelay(M2T(100));
			ch_interrupt_triggered();
		}
		

		// vTaskDelay(M2T(10)); 
		/* Check for sensor data-ready interrupt(s) */
		if (taskflags & DATA_READY_FLAG) {
			ch_group_trigger(grp_ptr); // trigger sensor
			/* All sensors have interrupted - handle sensor data */
			taskflags &= ~DATA_READY_FLAG;				// clear flag
			//handle_data_ready_return_max_amp(grp_ptr); //AVG_MAX
			amplitude_avg = handle_data_ready_return_avg_amp(grp_ptr);					// read and display measurement //AVG_CALIB
			
			NOISE_AVG += amplitude_avg; //AVG_CALIB
			counter_amp += 1; //AVG_CALIB
			// taskflags |= TIMER_FLAG;
			meas_counter++;
			meas_rate = (xTaskGetTickCount() - start_time)/meas_counter;
			timeout_counter = 0;


#if ((READ_AMPLITUDE_DATA || READ_IQ_DATA)  && READ_DATA_NONBLOCKING)
            /* Start any pending non-blocking data reads */
            if (num_io_queued != 0) {
                ch_io_start_nb(grp_ptr);
                num_io_queued = 0;
            }
#endif
        }  // end if (taskflags & DATA_READY_FLAG)

		if (taskflags & TIMER_FLAG) {
			taskflags &= ~TIMER_FLAG;
			taskflags = 0;
			execution_started = 1;

			ch_group_trigger(grp_ptr); // trigger sensor
			vTaskDelay(M2T(26)); // takes 26ms to measure
			
			timeout_counter = 0;
		}
	


#if ((READ_AMPLITUDE_DATA || READ_IQ_DATA)  && READ_DATA_NONBLOCKING)
		/* Check for non-blocking data read complete */
		if (taskflags & READ_DONE_FLAG) {

			/* All non-blocking data readouts have completed */
			taskflags &= ~READ_DONE_FLAG;		// clear flag
			handle_read_done(grp_ptr);			// display measurement data
		}
#endif
		if(timeout_counter >= 100)
		{
			taskflags |= TIMER_FLAG;
			timeout_counter = 0;
			DEBUG_PRINT("timeout\n");  //often get this
			counter += 1;
		}
		vTaskDelay(M2T(5));
	}
  //#endif

//   float max_amp_avg = 0; //AVG_MAX
//   for (int i = 0; i < NR_MAX_AMPL;i++) { //AVG_MAX
// 	  max_amp_avg += max_5_amp[i]; //AVG_MAX
//   } //AVG_MAX
//   max_amp_avg /= NR_MAX_AMPL; //AVG_MAX

//   DEBUG_PRINT("Calibration succeeded with average maximum amplitude of %f\n", max_amp_avg); //AVG_MAX
//   THRESHOLD_LEVEL = 1.1*max_amp_avg + 1000; //AVG_MAX
//   DEBUG_PRINT("threshold level: %f\n", THRESHOLD_LEVEL); //AVG_MAX

  float NOISE_AVG_FINAL = NOISE_AVG/counter_amp; //AVG_CALIB
  DEBUG_PRINT("\nCalibrated always with %d out of 340 samples\n", AMP_CALIBRATE_FILTER_LEN); //AVG_CALIB
  DEBUG_PRINT("Calibration succeeded with average amplitude of %f\n", NOISE_AVG_FINAL); //AVG_CALIB
  THRESHOLD_LEVEL = 1.5*NOISE_AVG_FINAL + 1500.0; //AVG_CALIB
  DEBUG_PRINT("threshold level: %f\n", THRESHOLD_LEVEL); //AVG_CALIB
  //////////////////////////////calibration ends///////////////////////////////////

  /////////////////////////////reconfigure measurement queue to transmit and receive///////////////////////////////// 
  DEBUG_PRINT("Reconfigure the sensor to transmit and receive...\n");
  
  measurement_config_queue.meas[0].trx_inst[0].cmd_config = 33569;
  measurement_config_queue.meas[0].trx_inst[1].cmd_config = 1057;
  measurement_config_queue.meas[0].trx_inst[2].cmd_config = 289;
  measurement_config_queue.meas[0].trx_inst[3].cmd_config = 4385;
  measurement_config_queue.meas[0].trx_inst[4].cmd_config = 4385;
  measurement_config_queue.meas[0].trx_inst[5].cmd_config = 4385;

  measurement_config_queue.meas[1].trx_inst[0].cmd_config = 33569;
  measurement_config_queue.meas[1].trx_inst[1].cmd_config = 1057;
  measurement_config_queue.meas[1].trx_inst[2].cmd_config = 289;
  measurement_config_queue.meas[1].trx_inst[3].cmd_config = 4385;
  measurement_config_queue.meas[1].trx_inst[4].cmd_config = 4385;
  measurement_config_queue.meas[1].trx_inst[5].cmd_config = 4385;

  ////////////////////////////////////new measurement queue configured////////////////////////////

  //////////////////////////////////reconfigure the sensors with the new configuration////////////
  chirp_error = configure_sensors(grp_ptr, num_ports, &active_devices,
			&num_connected_sensors, &num_triggered_sensors);
	vTaskDelay(M2T(100)); //no rush to start
	dev_ptr = ch_get_dev_ptr(grp_ptr, 0);
	MAX_RANGE = 340.0/dev_ptr->orig_pmut_freq*4.0*343.0/2.0; //TODO fix for multiple sensors and different ODRs 

	if (!chirp_error) {
		DEBUG_PRINT("\nSensor configured again, start ringdown measurement\n");
	}
	
  /////////////////////////////////Sensor is reconfigured, start ringdown measurement//////////////
  taskflags |= TIMER_FLAG;
  timeout_counter = 0;
  start_time = xTaskGetTickCount();
  meas_counter = 0;
  uint16_t ringdown_counter = 0;
  counter = 0;
  for ( int i = 0; i < 202; i++) {
  		timeout_counter++;
    	// 	vTaskDelay(M2T(10));
		// 	s// chbsp_proc_sleep();			// put processor in low-power sleep mode
		// 	/* We only continue here after an interrupt wakes the processor */
		// }
		if ((taskflags & (INTERRUPT_FLAG_0 | INTERRUPT_FLAG_1)) == (INTERRUPT_FLAG_0 | INTERRUPT_FLAG_1))
		{
			// DEBUG_PRINT("ch_interrupt_triggered taskflags: 0x%x 0x%x\n", taskflags, (INTERRUPT_FLAG_0 | INTERRUPT_FLAG_1));
			taskflags &= ~(INTERRUPT_FLAG_0 | INTERRUPT_FLAG_1);
			// vTaskDelay(M2T(100));
			ch_interrupt_triggered();
		}
		

		// vTaskDelay(M2T(10)); 
		/* Check for sensor data-ready interrupt(s) */
		if (taskflags & DATA_READY_FLAG) {
			ch_group_trigger(grp_ptr); // trigger sensor
			/* All sensors have interrupted - handle sensor data */
			taskflags &= ~DATA_READY_FLAG;				// clear flag
			ringdown_filter_define(grp_ptr);					// read and display measurement
			ringdown_counter += 1;
			// taskflags |= TIMER_FLAG;
			meas_counter++;
			meas_rate = (xTaskGetTickCount() - start_time)/meas_counter;
			timeout_counter = 0;


#if ((READ_AMPLITUDE_DATA || READ_IQ_DATA)  && READ_DATA_NONBLOCKING)
            /* Start any pending non-blocking data reads */
            if (num_io_queued != 0) {
                ch_io_start_nb(grp_ptr);
                num_io_queued = 0;
            }
#endif
        }  // end if (taskflags & DATA_READY_FLAG)

		if (taskflags & TIMER_FLAG) {
			taskflags &= ~TIMER_FLAG;
			taskflags = 0;
			execution_started = 1;

			ch_group_trigger(grp_ptr); // trigger sensor
			vTaskDelay(M2T(26)); // takes 26ms to measure
			
			timeout_counter = 0;
		}
	


#if ((READ_AMPLITUDE_DATA || READ_IQ_DATA)  && READ_DATA_NONBLOCKING)
		/* Check for non-blocking data read complete */
		if (taskflags & READ_DONE_FLAG) {

			/* All non-blocking data readouts have completed */
			taskflags &= ~READ_DONE_FLAG;		// clear flag
			handle_read_done(grp_ptr);			// display measurement data
		}
#endif
		if(timeout_counter >= 100)
		{
			taskflags |= TIMER_FLAG;
			timeout_counter = 0;
			DEBUG_PRINT("timeout\n");  //often get this
			counter += 1;
		}
		vTaskDelay(M2T(5));
	}

  if (counter >= 2) {
	DEBUG_PRINT("Ringdown didn't work, landing\n");
	ultrasound_status = ULTRASOUND_FF_STATUS_CONN_NOT_WORKS_FLY;
	
	while(1) {
		vTaskDelay(M2T(200));
	}
  }

  for (int i=0; i < CHIRP_RINGDOWN_FILTER_SAMPLES; i++) {
	ringdown_filter[i] = ringdown_filter[i]/((float) ringdown_counter);
	DEBUG_PRINT("ringdown_filter[%d] = %f\n", i, ringdown_filter[i]);
  }
  DEBUG_PRINT("Ringdown filter done\n");
  ultrasound_status = ULTRASOUND_FF_STATUS_CONN_WORKS;
  /////////////////////////////Ringdown filter done, start OA algorithm///////////////////////
  
	while(1) {

		if (ToFly == 0) { //TODO: unnecssary probably, remove
			DEBUG_PRINT("ToFLy = 0\n");
			fly_task_land();
			break;
		}

	// vTaskDelay(M2T(2000)); // let's start at 1Hz, that should sure work... 
	// ch_group_trigger(&chirp_group); // trigger sensor
    // DEBUG_PRINT("In US task\n"); 
	/*
		 * Put processor in light sleep mode if there are no pending tasks, but
		 * never turn off the main clock, so that interrupts can still wake
		 * the processor.
		 */

		/* *** WARNING - Race Condition ***
		 * The following test of the taskflags is used to determine if the processor
		 * should be put in sleep mode.  However, because the task flags are updated
		 * at interrupt level, it is possible for them to change before the sleep
		 * state is entered.  This can result in a "missed" measurement cycle or
		 * similar problems.
		 *
		 * It is recommended to lock interrupts at this point (at least blocking the
		 * sensor interrupt, periodic timer, and non-blocking I/O) before testing
		 * taskflags, and then restore the interrupt enable state after the test or
		 * as part of the sleep function.
		 *
		 * Because such solutions are hardware-specific, they are not included in this
		 * simple example.  However, you should add this protection or otherwise handle
		 * this timing possibility if adapting this code for your actual application.
		 */

		// if (taskflags == 0) {
			timeout_counter++;
    	// 	vTaskDelay(M2T(10));
		// 	// chbsp_proc_sleep();			// put processor in low-power sleep mode
		// 	/* We only continue here after an interrupt wakes the processor */
		// }
		if ((taskflags & (INTERRUPT_FLAG_0 | INTERRUPT_FLAG_1)) == (INTERRUPT_FLAG_0 | INTERRUPT_FLAG_1))
		{
			// DEBUG_PRINT("ch_interrupt_triggered taskflags: 0x%x 0x%x\n", taskflags, (INTERRUPT_FLAG_0 | INTERRUPT_FLAG_1));
			taskflags &= ~(INTERRUPT_FLAG_0 | INTERRUPT_FLAG_1);
			// vTaskDelay(M2T(100));
			ch_interrupt_triggered();
		}
		

		// vTaskDelay(M2T(10)); 
		/* Check for sensor data-ready interrupt(s) */
		if (taskflags & DATA_READY_FLAG) {
			ch_group_trigger(grp_ptr); // trigger sensor
			/* All sensors have interrupted - handle sensor data */
			taskflags &= ~DATA_READY_FLAG;				// clear flag
			if (xSemaphoreTake(ultrasound_read, 0) == pdTRUE) {
				ultrasound_meas_res = handle_data_ready(grp_ptr);					// read and display measurement
				ultrasound_preprocess_ready = true;
				xSemaphoreGive(ultrasound_read); //ready to process data
			}
			// taskflags |= TIMER_FLAG;
			meas_counter++;
			meas_rate = (xTaskGetTickCount() - start_time)/meas_counter;
			timeout_counter = 0;


#if ((READ_AMPLITUDE_DATA || READ_IQ_DATA)  && READ_DATA_NONBLOCKING)
            /* Start any pending non-blocking data reads */
            if (num_io_queued != 0) {
                ch_io_start_nb(grp_ptr);
                num_io_queued = 0;
            }
#endif
        }  // end if (taskflags & DATA_READY_FLAG)

		if (taskflags & TIMER_FLAG) {
			taskflags &= ~TIMER_FLAG;
			taskflags = 0;
			execution_started = 1;
			// ch_dev_t *dev_ptr = ch_get_dev_ptr(grp_ptr, dev_num);
			// ch_meas_set_ringdown_cancel(dev_ptr, 0, CHIRP_RINGDOWN_FILTER_SAMPLES);
			// uint16_t ringdown_cancel_readback = ch_meas_get_ringdown_cancel(dev_ptr, 0);
			// DEBUG_PRINT("ringdown cancel samples: %d\n", ringdown_cancel_readback);
		// DEBUG_PRINT("trig_pin: %d int_pin: %d\n", grp_ptr->sensor_trig_pin,  grp_ptr->sensor_int_pin);

			ch_group_trigger(grp_ptr); // trigger sensor
			vTaskDelay(M2T(26)); // takes 26ms to measure
			
			timeout_counter = 0;
			// if (CHIRP_TRIGGER_TYPE == CH_TRIGGER_TYPE_SW){
			// 	for (dev_num = 0; dev_num < num_ports; dev_num++) {
			// 		dev_ptr = ch_get_dev_ptr(grp_ptr, dev_num);
			// 		if (ch_sensor_is_connected(dev_ptr))
			// 			ch_trigger_soft(dev_ptr);
			// 	}
			// }
		}
	


#if ((READ_AMPLITUDE_DATA || READ_IQ_DATA)  && READ_DATA_NONBLOCKING)
		/* Check for non-blocking data read complete */
		if (taskflags & READ_DONE_FLAG) {

			/* All non-blocking data readouts have completed */
			taskflags &= ~READ_DONE_FLAG;		// clear flag
			handle_read_done(grp_ptr);			// display measurement data
		}
#endif
		if(timeout_counter >= 100)
		{
			taskflags |= TIMER_FLAG;
			timeout_counter = 0;
			DEBUG_PRINT("timeout\n");  //often get this
		}
		vTaskDelay(M2T(5));
	
	// *** End while(1) infinite measurement loop ***
	}
}

//-----------------------------------Finish ultrasound_f data collection & preprocessing task-------------------


void send_data_packet(uint8_t *data, uint16_t data_len)
{
  uint8_t packets_nr = 0;
  if (data_len%28 > 0)
    packets_nr = data_len/28 + 1;
  else
    packets_nr = data_len/28;

  for (uint8_t idx=0; idx<packets_nr; idx++)
    if(data_len - 28*idx >= 28)
      send_data_packet_28b(&data[28*idx], 28, idx);
    else
      send_data_packet_28b(&data[28*idx], data_len - 28*idx, idx);
}

void send_data_packet_28b(uint8_t *data, uint8_t size, uint8_t index)
{
  CRTPPacket pk;
  pk.header = CRTP_HEADER(1, 0); // first arg is the port number
  pk.size = size + 2;
  pk.data[0] = 'D';
  pk.data[1] = index;
  memcpy(&(pk.data[2]), data, size);
  crtpSendPacketBlock(&pk);
}


void send_command(uint8_t command, uint8_t arg)
{
  //uint32_t timestamp = get_time_stamp();
  CRTPPacket pk;
  pk.header = CRTP_HEADER(1, 0); // first arg is the port number
  pk.size = 7;
  pk.data[0] = 'C';
  pk.data[1] = command;
  pk.data[2] = arg;
  memcpy(&pk.data[3], (uint8_t *)(&timestamp), 4);
  //DEBUG_PRINT("cmd PK 1:%d, 2:%d, 3:%d, 4:%d, ",pk.data[3],pk.data[4],pk.data[5],pk.data[6]); //todo delete
  crtpSendPacketBlock(&pk);
}

bool config_sensors(VL53L5CX_Configuration *p_dev, uint16_t new_i2c_address)
{
  p_dev->platform = VL53L5CX_DEFAULT_I2C_ADDRESS; // use default adress for first use

  // initialize the sensor
  uint8_t tof_res = vl53l5cx_init(p_dev);   if (tof_res != VL53L5CX_STATUS_OK) return false ;
  //DEBUG_PRINT("ToF Config Result: %d \n", tof_init_res);

  // Configurations
  //change i2c address
  tof_res = vl53l5cx_set_i2c_address(p_dev, new_i2c_address);if (tof_res != VL53L5CX_STATUS_OK) return false ;
  tof_res = vl53l5cx_set_resolution(p_dev, VL53L5CX_RESOLUTION_8X8);if (tof_res != VL53L5CX_STATUS_OK) return false ; 
  // 15hz
  tof_res = vl53l5cx_set_ranging_frequency_hz(p_dev, 15);if (tof_res != VL53L5CX_STATUS_OK) return false ; 
  tof_res = vl53l5cx_set_target_order(p_dev, VL53L5CX_TARGET_ORDER_CLOSEST);if (tof_res != VL53L5CX_STATUS_OK) return false ;
  tof_res = vl53l5cx_set_ranging_mode(p_dev, VL53L5CX_RANGING_MODE_CONTINUOUS);if (tof_res != VL53L5CX_STATUS_OK) return false ;
  //tof_res = vl53l5cx_set_ranging_mode(p_dev, VL53L5CX_RANGING_MODE_AUTONOMOUS);if (tof_res != VL53L5CX_STATUS_OK) return false ;// TODO test it

  //Check for sensor to be alive
  uint8_t isAlive;
  tof_res =vl53l5cx_is_alive(p_dev,&isAlive);if (tof_res != VL53L5CX_STATUS_OK) return false;
  if (isAlive != 1) return false;
  
  // All Good!
  return true;
}

bool initialize_sensors_I2C(VL53L5CX_Configuration *p_dev, uint8_t mode)
{
  bool status = false;

  //reset I2C  //configure pins out/in for forward only

  //status = I2C_expander_set_register(OUTPUT_PORT_REG_ADDRESS,I2C_RST_BACKWARD_PIN,I2C_RST_FORWARD_PIN);if (status == false)return status;

  if (mode == 1 && p_dev != NULL){
    //enable forward only and config
    status = I2C_expander_set_register(OUTPUT_PORT_REG_ADDRESS,LPN_FORWARD_PIN | LED_FORWARD_PIN );if (status == false)return status; 
    status = config_sensors(p_dev,VL53L5CX_FORWARD_I2C_ADDRESS);if (status == false)return status; 
  }
  if (mode == 2 && p_dev != NULL){
    //enable backward only and config
    status = I2C_expander_set_register(OUTPUT_PORT_REG_ADDRESS,LPN_BACKWARD_PIN | LED_BACKWARD_PIN); if (status == false)return status; 
    status = config_sensors(p_dev,VL53L5CX_BACKWARD_I2C_ADDRESS);if (status == false)return status; 
  }
  //status = I2C_expander_set_register(OUTPUT_PORT_REG_ADDRESS,0x00); //all off
  if (mode == 3){
    //enable both forward & backward
    status = I2C_expander_set_register(OUTPUT_PORT_REG_ADDRESS,LPN_BACKWARD_PIN | LED_BACKWARD_PIN|LPN_FORWARD_PIN | LED_FORWARD_PIN); if (status == false)return status; 
  }
  return status;
}

bool get_sensor_data(VL53L5CX_Configuration *p_dev,VL53L5CX_ResultsData *p_results){
 
  // Check  for data ready I2c
  uint8_t ranging_ready = 2;
  //ranging_ready --> 0 if data is not ready, or 1 if a new data is ready.
  uint8_t status = vl53l5cx_check_data_ready(p_dev, &ranging_ready);if (status != VL53L5CX_STATUS_OK) return false;

  // 1 Get data in case it is ready
  if (ranging_ready == 1){
    status = vl53l5cx_get_ranging_data(p_dev, p_results);if (status != VL53L5CX_STATUS_OK) return false;
  }else {
    //0  data in not ready yet
    return false;
  }

  // All good then
  //return false;// TODO delete
  return true;
}

//------------------------------already from ultrasound sensor included-----------------------------

/*
 * configure_sensors() - apply configuration settings
 *
 * This function performs the detailed configuration of the sensor, including
 * definition of the measurement and selecting various optional features.
 * Values defined in the app_config.h file are used here as parameters to
 * individual SonicLib API functions.
 *
 * If IMPORT_MEASUREMENT is defined as non-zero, this function will import a defined
 * measurement instead.
 */
uint8_t configure_sensors(
		ch_group_t * grp_ptr,					// pointer to sensor group
		uint8_t      num_ports,					// number of possible sensors on board
		uint32_t *   active_devices_ptr,		// pointer to active devices tracking var
		uint8_t *    num_connected_ptr,			// pointer to count of connected devices
		uint8_t *    num_triggered_ptr)			// pointer to count of triggered devices
{
	uint8_t dev_num;
	ch_dev_t *dev_ptr;
	ch_mode_t mode;
	char const *mode_string = "";
	uint8_t chirp_error;
	uint8_t first_connected_num = 0;

	chirp_error = 0;
	*num_connected_ptr = 0;
	*active_devices_ptr = 0;

#if (CHIRP_IMPROVE_PITCH_CATCH_SENSITIVITY == 1)
	uint32_t sensors_mean_fop = 0;

	sensors_mean_fop = compute_common_fop(grp_ptr, num_ports);
#endif // (CHIRP_IMPROVE_PITCH_CATCH_SENSITIVITY == 1)

	/* Set configuration values for each connected sensor */
	for (dev_num = 0; dev_num < num_ports; dev_num++) {
		dev_ptr = ch_get_dev_ptr(grp_ptr, dev_num);		// get this device's ch_dev_t addr

		if (ch_sensor_is_connected(dev_ptr)) {
			DEBUG_PRINT("Configuring Device %d...\n", dev_num);

			(*num_connected_ptr)++;						// count one more connected sensor
			(*active_devices_ptr) |= (1 << dev_num);	// add to active device bit mask

			if (*num_connected_ptr == 1) {	// if this is the first sensor
				mode = CHIRP_FIRST_SENSOR_MODE;
				first_connected_num = dev_num;
			} else {
				mode = CHIRP_OTHER_SENSOR_MODE;
			}

			switch (mode) {
				case CH_MODE_FREERUN:
					mode_string = "CH_MODE_FREERUN";
					break;
				case CH_MODE_TRIGGERED_TX_RX:
					mode_string = "CH_MODE_TRIGGERED_TX_RX";
					break;
				case CH_MODE_TRIGGERED_RX_ONLY:
					mode_string = "CH_MODE_TRIGGERED_RX_ONLY";
					break;
				case CH_MODE_IDLE:
					mode_string = "CH_MODE_IDLE";
					break;
				default:
					chirp_error = 1;					// bad mode specified in app_config.h
					DEBUG_PRINT("ERROR - bad sensor mode %d\n", mode);
					break;
			}

			/* Initialize application structure(s) to hold measurement data */
			if (!chirp_error) {
				chirp_data[dev_num].rx_sensor_num = dev_num;		// this will be the receiving sensor
				if (mode == CH_MODE_TRIGGERED_RX_ONLY) {
					chirp_data[dev_num].tx_sensor_num = first_connected_num;	// first sensor will transmit
				} else {
					chirp_data[dev_num].tx_sensor_num = dev_num;				// this sensor will transmit
				}
			}

#if (CHIRP_IMPROVE_PITCH_CATCH_SENSITIVITY == 1)
			if (!chirp_error && (sensors_mean_fop != 0)) {
				DEBUG_PRINT("  Setting common fop... ");
				/* Change of fop shall be done before optimizing measurement */
				chirp_error = ch_set_frequency(dev_ptr, sensors_mean_fop);
				DEBUG_PRINT("%s\n", chirp_error ? "ERROR":"OK");
			}
#endif // (CHIRP_IMPROVE_PITCH_CATCH_SENSITIVITY == 1)

#if IMPORT_MEASUREMENT			// If importing a whole pre-defined measurement
			(void) num_triggered_ptr;
			if (!chirp_error && (CHIRP_MEAS_OPTIMIZE == 0)) {
				DEBUG_PRINT("  Importing measurement: ");
				chirp_error = ch_meas_import(dev_ptr, &measurement_config_queue, &measurement_config_cfg);
				DEBUG_PRINT("%s\n", chirp_error ? "ERROR":"OK");
			} else {
				DEBUG_PRINT("  Importing & optimizing measurement: ");
				chirp_error = ch_meas_optimize(dev_ptr, &measurement_config_queue, &measurement_config_cfg);
				DEBUG_PRINT("%s\n", chirp_error ? "ERROR":"OK");
			}

#else					// If defining a measurement using SonicLib functions

			/* Initialize measurement */
			if (!chirp_error) {
				DEBUG_PRINT("  Initializing measurement %d... ", MEAS_NUM);
				chirp_error = ch_meas_init(dev_ptr, MEAS_NUM, &meas_config, chirp_detect_thresholds_ptr);
				DEBUG_PRINT("%s\n", chirp_error ? "ERROR":"OK");
			}

			/* Add transmit segment unless rx-only */
			if (!chirp_error) {
				if (mode != CH_MODE_TRIGGERED_RX_ONLY) {
					DEBUG_PRINT("  Adding tx segment... ");
					chirp_error = ch_meas_add_segment_tx(dev_ptr, MEAS_NUM, CHIRP_TX_SEG_CYCLES,
								CHIRP_TX_SEG_PULSE_WIDTH, CHIRP_TX_SEG_PHASE, CHIRP_TX_SEG_INT_EN);
				} else {
					DEBUG_PRINT("  Adding count (delay) segment to match Tx sensor... ");
					chirp_error = ch_meas_add_segment_count(dev_ptr, MEAS_NUM, CHIRP_TX_SEG_CYCLES, 0);
				}
				DEBUG_PRINT("%s\n", chirp_error ? "ERROR":"OK");
			}

			/* Add first receive segment for very early part of measurement */
			if (!chirp_error) {
				DEBUG_PRINT("  Adding rx segment 0... ");
				chirp_error = ch_meas_add_segment_rx(dev_ptr, MEAS_NUM, CHIRP_RX_SEG_0_SAMPLES,
								CHIRP_RX_SEG_0_GAIN_REDUCE, CHIRP_RX_SEG_0_ATTEN, CHIRP_RX_SEG_0_INT_EN);
				DEBUG_PRINT("%s\n", chirp_error ? "ERROR":"OK");
			}

			/* Add second receive segment for remainder of measurement */
			if (!chirp_error) {
				DEBUG_PRINT("  Adding rx segment 1... ");
				chirp_error = ch_meas_add_segment_rx(dev_ptr, MEAS_NUM, CHIRP_RX_SEG_1_SAMPLES,
								CHIRP_RX_SEG_1_GAIN_REDUCE, CHIRP_RX_SEG_1_ATTEN, CHIRP_RX_SEG_1_INT_EN);
				DEBUG_PRINT("%s\n", chirp_error ? "ERROR":"OK");
			}

			/* Write complete meas config (measurement queue) to device */
			if (!chirp_error) {
				DEBUG_PRINT("  Writing meas queue... ");
				chirp_error = ch_meas_write_config(dev_ptr);
				DEBUG_PRINT("%s\n", chirp_error ? "ERROR":"OK");
			}
			/* Optimize measurement segments, if specified */
			if (!chirp_error && CHIRP_MEAS_OPTIMIZE) {
				DEBUG_PRINT("  Optimizing measurement segments... ");
				chirp_error = ch_meas_optimize(dev_ptr, NULL, NULL);
				DEBUG_PRINT("%s\n", chirp_error ? "ERROR":"OK");
			}

			/* Set sensing range based on sensor model - this can change the total
			 * number of rx samples set during ch_meas_add_segment_rx() above.
			 */
			if (!chirp_error) {
#ifndef INCLUDE_ALGO_EXTERNAL
				uint16_t max_range = 0;
				uint16_t part_number = ch_get_part_number(dev_ptr);

				if (part_number == ICU30201_PART_NUMBER) {
					max_range = CHIRP_ICU30201_MAX_RANGE_MM;
				} else if (part_number == ICU10201_PART_NUMBER) {
					max_range = CHIRP_ICU10201_MAX_RANGE_MM;
				} else {
					max_range = CHIRP_ICU20201_MAX_RANGE_MM;
				}
				DEBUG_PRINT("  Setting max range to %d mm... ", max_range);
				chirp_error = ch_set_max_range(dev_ptr, max_range);
#endif
				DEBUG_PRINT("%s\n", chirp_error ? "ERROR":"OK");
			}

			/* Set static target rejection range, if specified */
			if (!chirp_error && (CHIRP_STATIC_REJECT_SAMPLES != 0)) {
				DEBUG_PRINT("  Setting static target rejection (%d samples)... ", CHIRP_STATIC_REJECT_SAMPLES);
				chirp_error = ch_set_static_range(dev_ptr, CHIRP_STATIC_REJECT_SAMPLES);
				DEBUG_PRINT("%s\n", chirp_error ? "ERROR":"OK");
			}

			/* Set rx holdoff, if specified */
			if (!chirp_error && (CHIRP_RX_HOLDOFF_SAMPLES != 0)) {
				DEBUG_PRINT("  Setting rx holdoff (%d samples)... ", CHIRP_RX_HOLDOFF_SAMPLES);
				chirp_error = ch_set_rx_holdoff(dev_ptr, CHIRP_RX_HOLDOFF_SAMPLES);
				DEBUG_PRINT("%s\n", chirp_error ? "ERROR":"OK");
			}
			/* Set ringdown filter, if specified */
			if (!chirp_error && (CHIRP_RINGDOWN_FILTER_SAMPLES != 0)) {
				DEBUG_PRINT("  Setting ringdown filter samples (%d samples)... ", CHIRP_RINGDOWN_FILTER_SAMPLES);
				chirp_error = ch_meas_set_ringdown_cancel(dev_ptr, 0, CHIRP_RINGDOWN_FILTER_SAMPLES);
				uint16_t ringdown_cancel_readback = ch_meas_get_ringdown_cancel(dev_ptr, 0);
				DEBUG_PRINT("ringdown cancel samples: %d\n", ringdown_cancel_readback);
				DEBUG_PRINT("%s\n", chirp_error ? "ERROR":"OK");
			}

			/* Enable target interrupt filtering, if specified */
			if (!chirp_error && CHIRP_TARGET_INT_FILTER) {
				DEBUG_PRINT("  Enabling target interrupt filtering... ");
				chirp_error = ch_set_target_interrupt(dev_ptr, CH_TGT_INT_FILTER_ANY);
				DEBUG_PRINT("%s\n", chirp_error ? "ERROR":"OK");
			}
#endif // IMPORT_MEASUREMENT

			/* Enable amplitude output from sensor if selected */
			if (!chirp_error && READ_AMPLITUDE_DATA) {
				DEBUG_PRINT("  Enabling amplitude output... ");
				chirp_error = ch_meas_set_iq_output(dev_ptr, MEAS_NUM, CH_OUTPUT_AMP);
				DEBUG_PRINT("%s\n", chirp_error ? "ERROR":"OK");
			}

			/* Enable software triggering if selected */
			if (!chirp_error && (CHIRP_TRIGGER_TYPE == CH_TRIGGER_TYPE_SW)) {

				DEBUG_PRINT("  Enabling soft triggering... ");
				ch_set_trigger_type(dev_ptr, CH_TRIGGER_TYPE_SW);
				DEBUG_PRINT("OK\n");
			}

			if (!chirp_error) {
				/* If sensor will be free-running, set internal sample interval */
				if (mode == CH_MODE_FREERUN) {
					ch_set_sample_interval(dev_ptr, MEASUREMENT_INTERVAL_MS);
				} else {
					(*num_triggered_ptr)++;	// count one more triggered sensor
				}

				/* Set interrupt line(s) as input and enable, if not also used for triggering */
				if ((mode == CH_MODE_FREERUN) || (CHIRP_TRIGGER_TYPE == CH_TRIGGER_TYPE_SW) ||
					(CHIRP_SENSOR_INT_PIN != CHIRP_SENSOR_TRIG_PIN)) {

					// DEBUG_PRINT("  Setting INT line as input and enabling... ");
					chdrv_int_group_set_dir_in(grp_ptr);			// set INT line as input
					chdrv_int_group_interrupt_enable(grp_ptr);		// enable interrupt
					// DEBUG_PRINT("OK\n");
				}

				/* Set sensor mode - must come last, because sensing will be enabled and may begin */
				// DEBUG_PRINT("  Enabling in %s mode... ", mode_string);
				chirp_error = ch_set_mode(dev_ptr, mode);
				DEBUG_PRINT("%s\n", chirp_error ? "ERROR":"OK");
			}

			/* Read back and display config settings */
			if (!chirp_error) {
				DEBUG_PRINT("Device %d: Configuration OK\n\n", dev_num);
				// ultrasound_display_config_info(dev_ptr);
			} else {
				DEBUG_PRINT("Device %d: ERROR during configuration\n", dev_num);
			}
		}  /*  end if ch_sensor_is_connected() */
        // vTaskDelay(100); // wait cause we cannot bloc other tasks too long
	}  /*  end for dev_num < num_ports */

	/* Enable receive sensor rx pre-triggering for group, if specified */
	ch_set_rx_pretrigger(grp_ptr, CHIRP_RX_PRETRIGGER_ENABLE);

	return chirp_error;
}

/*
 * handle_data_ready() - get and display data from a measurement
 *
 * This routine is called from the main() loop after all sensors have
 * interrupted. It shows how to read the sensor data once a measurement is
 * complete.  This routine always reads out the range and amplitude, and
 * optionally will read out the amplitude data or raw I/Q for all samples
 * in the measurement.
 *
 * See the comments in app_config.h for information about the amplitude data
 * and I/Q readout build options.
 */
float *handle_data_ready(ch_group_t *grp_ptr)
{
	// DEBUG_PRINT("handle data ready\n");
	uint8_t  dev_num;
	uint16_t num_samples = 0;
	uint8_t  ret_val = 0;
#ifndef INCLUDE_ALGO_EXTERNAL
	uint32_t range;
	uint16_t amplitude;
	ch_range_t range_type;
#endif

	/* Read and display data from each connected sensor
	 *   This loop will write the sensor data to this application's "chirp_data"
	 *   array.  Each sensor has a separate chirp_data_t structure in that
	 *   array, so the device number is used as an index.
	 *
	 *   Multiple detected targets will be displayed if DISPLAY_MULTI_TARGET is non-zero.
	 *   Otherwise, only the closest detected target will be displayed.
	 */

	for (dev_num = 0; dev_num < ch_get_num_ports(grp_ptr); dev_num++) {
		ch_dev_t *dev_ptr = ch_get_dev_ptr(grp_ptr, dev_num);

		if (ch_sensor_is_connected(dev_ptr)) {
#ifndef INCLUDE_ALGO_EXTERNAL
			 /*  For sensors in transmit/receive mode, report one-way echo
			 *   range.  For sensors in receive-only mode, report direct
			 *   one-way range from transmitting sensor.
			 */
			if (ch_get_mode(dev_ptr) == CH_MODE_TRIGGERED_RX_ONLY) {
				range_type = CH_RANGE_DIRECT;
			} else {
				range_type = CH_RANGE_ECHO_ONE_WAY;
			}
#endif

			// DEBUG_PRINT("Dev %d:  ", dev_num);

#ifndef INCLUDE_ALGO_EXTERNAL
#if DISPLAY_MULTI_TARGET
			/* Display multiple detected targets (if any) */

			uint8_t num_targets = ch_get_num_targets(dev_ptr);

			if (num_targets > 0) {

				for (uint8_t target_num = 0; target_num < num_targets;
						target_num++) {

					range = ch_get_target_range(dev_ptr, target_num, range_type);
					amplitude = ch_get_target_amplitude(dev_ptr, target_num);

					if (target_num == 0) {					// store first target values
						chirp_data[dev_num].range = range;
						chirp_data[dev_num].amplitude = amplitude;
					}

					DEBUG_PRINT("Tgt %d: %0.1f mm ", target_num, (float) range / 32.0f);

					if (DISPLAY_SAMPLE_NUM) {
						uint16_t num_mm = (range / 32);

						/* ch_mm_to_samples() assumes num_mm is one-way (1/2 round-trip time-of-flight) */
						if (range_type != CH_RANGE_ECHO_ONE_WAY) {
							num_mm /= 2;		// mm distance was entire time-of-flight, so convert
						}
						DEBUG_PRINT("(sample %u) ", ch_mm_to_samples(dev_ptr, num_mm));
					}

					if (DISPLAY_AMP_VALUE) {
						DEBUG_PRINT("amp=%5u ", amplitude);
					}
					DEBUG_PRINT("  ");
				}
			} else {
				DEBUG_PRINT("        no target found  ");
			}
#else	// DISPLAY_MULTI_TARGET
			/* Display single detected target (if any) */

			range = ch_get_range(dev_ptr, range_type);
			chirp_data[dev_num].range = range;

			if (range != CH_NO_TARGET) {
				/* Target object was successfully detected (range available) */

				 /* Get the new amplitude value - it's only updated if range
				  * was successfully measured.  */
				amplitude = ch_get_amplitude(dev_ptr);
				chirp_data[dev_num].amplitude = amplitude;

				DEBUG_PRINT("Range: %0.1f mm ", (float) range/32.0f);
				DEBUG_PRINT("Amplitude: %d ", amplitude);

				if (DISPLAY_SAMPLE_NUM) {
					uint16_t num_mm = (range / 32);

					/* ch_mm_to_samples() assumes num_mm is one-way (1/2 round-trip time-of-flight) */
					if (range_type != CH_RANGE_ECHO_ONE_WAY) {
						num_mm /= 2;		// mm distance was entire time-of-flight, so convert
					}
					// DEBUG_PRINT("(sample %u) ", ch_mm_to_samples(dev_ptr, num_mm));
				}

				if (DISPLAY_AMP_VALUE) {
					// DEBUG_PRINT("amp=%5u ", chirp_data[dev_num].amplitude);
				}

			} else {
				/* No target object was detected - no range value */
				chirp_data[dev_num].amplitude = 0;  /* no updated amplitude */

				DEBUG_PRINT("        no target found  ");
			}
#endif	// DISPLAY_MULTI_TARGET
#endif

			/* Store number of active samples in this measurement */
			num_samples = ch_get_num_samples(dev_ptr);
			chirp_data[dev_num].num_samples = num_samples;

#if (READ_AMPLITUDE_DATA || READ_IQ_DATA)
			/* Optionally read all samples in measurement from sensor */
			read_measurement_data(dev_ptr);

			if (!READ_DATA_NONBLOCKING) {
				/* Data read is already complete - log or display sample data */
  #if (OUTPUT_AMP_LOG || OUTPUT_IQ_LOG)
				log_sensor_data(&chirp_data[dev_num]);
  #elif (OUTPUT_AMP_DATA_CSV || OUTPUT_IQ_DATA)
				display_sensor_data(&chirp_data[dev_num]);
  #else
				// DEBUG_PRINT("read %u IQ samples\n", chirp_data[dev_num].num_samples);
				// float THRESHOLD_LEVEL_SUM = 0.0;
				// for (int i = chirp_data[dev_num].num_samples - 10; i < chirp_data[dev_num].num_samples; i++)
				// {
				// 	float sample_i = (float)chirp_data[dev_num].iq_data[i].i;
				// 	float sample_q = (float)chirp_data[dev_num].iq_data[i].q;
				// 	THRESHOLD_LEVEL_SUM += sqrtf(sample_i*sample_i + sample_q*sample_q);
				// }
				// THRESHOLD_LEVEL = THRESHOLD_LEVEL*9/10 + 1.5*THRESHOLD_LEVEL_SUM/10/10; // adapt dynamically to motor noise
				uint8_t first_obstacle = 1;
				for (int i = 0; i < chirp_data[dev_num].num_samples; i++)
				{
					float sample_i = (float)chirp_data[dev_num].iq_data[i].i;
					float sample_q = (float)chirp_data[dev_num].iq_data[i].q;
					float amp = sqrtf(sample_i*sample_i + sample_q*sample_q);
					if(i < CHIRP_RINGDOWN_FILTER_SAMPLES)
					{
						amp -= ringdown_filter[i];  //ringdown filter is now statically defined not dynamically
					}
					avg_filtered_amp[i] = avg_filtered_amp[i]*(AVG_FILTER_N - 1)/AVG_FILTER_N + amp/AVG_FILTER_N; //exponential filter
					amp = avg_filtered_amp[i];
					if (first_obstacle && amp >  THRESHOLD_LEVEL)
					{
						DEBUG_PRINT("Number of sample: %d\n", i);
						log_range[dev_num] = (float)(i+8)/340.0f*MAX_RANGE;
						first_obstacle = 0;
						// break; // cannot break if we want to calculate/transmit all amplitude data
					}
					// if(i < CHIRP_THRESH_1_START)
					// {
					// 	if (amp > CHIRP_THRESH_0_LEVEL + THRESHOLD_LEVEL)
					// 	{
					// 		// DEBUG_PRINT("%f, %f\n", (float)i/340.0f*7.0f, amp);
					// 		log_range[dev_num] = (float)i/340.0f*MAX_RANGE;
					// 		// vTaskDelay(M2T(20));
					// 		break;
					// 	}
					// } else if (i < CHIRP_THRESH_2_START)
					// {
					// 	if (amp > CHIRP_THRESH_1_LEVEL + THRESHOLD_LEVEL)
					// 	{
					// 		log_range[dev_num] = (float)i/340.0f*MAX_RANGE;
					// 		break;
					// 	}
					// } else if (i < CHIRP_THRESH_3_START)
					// {
					// 	if (amp > CHIRP_THRESH_2_LEVEL + THRESHOLD_LEVEL)
					// 	{
					// 		log_range[dev_num] = (float)i/340.0f*MAX_RANGE;
					// 		break;
					// 	}
					// } else if (i < CHIRP_THRESH_4_START)
					// {
					// 	if (amp > CHIRP_THRESH_3_LEVEL + THRESHOLD_LEVEL)
					// 	{
					// 		log_range[dev_num] = (float)i/340.0f*MAX_RANGE;
					// 		break;
					// 	}
					// } else if (i < CHIRP_THRESH_5_START)
					// {
					// 	if (amp > CHIRP_THRESH_4_LEVEL + THRESHOLD_LEVEL)
					// 	{
					// 		log_range[dev_num] = (float)i/340.0f*MAX_RANGE;
					// 		break;
					// 	}
					// } else if (i < CHIRP_THRESH_6_START)
					// {
					// 	if (amp > CHIRP_THRESH_5_LEVEL + THRESHOLD_LEVEL)
					// 	{
					// 		log_range[dev_num] = (float)i/340.0f*MAX_RANGE;
					// 		break;
					// 	}
					// } else
					// {
					// 	if (amp > CHIRP_THRESH_6_LEVEL + THRESHOLD_LEVEL)
					// 	{
					// 		log_range[dev_num] = (float)i/340.0f*MAX_RANGE;
					// 		break;
					// 	}
					// }
					if(first_obstacle)
					{
						log_range[dev_num] = MAX_RANGE;
					}					
				}
				
  #endif
			}

#else	// not reading amp or I/Q data
			DEBUG_PRINT("\n");		// final EOL for this device/measurement result
#endif 	// (READ_AMPLITUDE_DATA || READ_IQ_DATA)

		}	// end if (ch_sensor_is_connected(dev_ptr))
	}	// end for (dev_num < ch_get_num_ports(grp_ptr))
	DEBUG_PRINT("log_range_0: %f", log_range[0]);
	
	return log_range;
}


FlyCommand_t Decision_Making_US(float* log_range) {
	
	float command_velocity_z_us = 0.0f;
	float command_turn_us = 0.0f;
	float distance = log_range[0];
	FlyCommand_t return_val = {0, 0, 0};
	//DEBUG_PRINT("Distance log_range: %f\n", distance);

	if (ToFly) {
		float rate_yaw = 5.5;
		float vel_x = 0.0f;
		//rate_yaw = 1; //33.3 should equal around 1degree in between measurements
		//float vel_x = 0.0;
		if (distance > 0.8)
		{
			vel_x = distance/4.0;
			rate_yaw = 0.0;
		} 
		else if (distance > 0.4)
		{
			vel_x = distance/4.0;
			rate_yaw = rate_yaw*(0.8-distance)/0.8;
		} 
		else if (distance > 0.3) {
			vel_x = distance/4.0;
		}
		else {
			vel_x = 0;
		}

		// else if (distance > 0.2) {
		// 	rate_yaw = rate_yaw*(0.9-distance)/0.8;
		// }
		if(vel_x > vel_x_old + 0.05)
		{
			vel_x = vel_x_old + 0.05;
		}
		if (vel_x > 1) {
			vel_x = 1;
		}
		vel_x_old = vel_x;
		
		uint32_t current_time = xTaskGetTickCount();
		if(distance > 0.4 && ((current_time - time_last_cmd) > INTERVAL_DIRECTION_CHANGE) && rand() > RAND_MAX/2)
		{
			yaw_direction *= -1.0;
			time_last_cmd = current_time;
		}
		//DEBUG_PRINT("Rate_yaw: %f", rate_yaw); 
		return_val.command_velocity_x = vel_x;
		return_val.command_turn = rate_yaw*yaw_direction;
	}
	return return_val;
}


#if (READ_AMPLITUDE_DATA || READ_IQ_DATA)
uint8_t read_measurement_data(ch_dev_t *dev_ptr)
{
	uint16_t start_sample = 0;
	uint8_t  error = 0;
	uint16_t num_samples = ch_get_num_samples(dev_ptr);
	uint8_t  dev_num = ch_get_dev_num(dev_ptr);

#if !(READ_DATA_NONBLOCKING)
	/* Reading sensor data in normal, blocking mode */

  #if READ_AMPLITUDE_DATA
	error = ch_get_amplitude_data(dev_ptr, chirp_data[dev_num].amp_data,
					  			start_sample, num_samples, CH_IO_MODE_BLOCK);
  #elif READ_IQ_DATA
	error = ch_get_iq_data(dev_ptr, chirp_data[dev_num].iq_data,
								start_sample, num_samples, CH_IO_MODE_BLOCK);
  #endif
	if (error) {
		printf("  ERROR reading %d samples", num_samples);
	}

#else
	/* Reading sensor data in non-blocking mode - queue a read operation */
	printf("     queuing %d samples...", num_samples);

  #if READ_AMPLITUDE_DATA
	error = ch_get_amplitude_data(dev_ptr, chirp_data[dev_num].amp_data,
					  			start_sample, num_samples, CH_IO_MODE_NONBLOCK);
  #elif READ_IQ_DATA
	error = ch_get_iq_data(dev_ptr, chirp_data[dev_num].iq_data,
								start_sample, num_samples, CH_IO_MODE_NONBLOCK);
  #endif

	if (!error) {
		num_io_queued++;		// record a pending non-blocking read
		printf("OK");
	} else {
		printf("**ERROR**");
	}
	printf("\n");
#endif // !(READ_DATA_NONBLOCKING)

	return error;
}
#endif // (READ_AMPLITUDE_DATA || READ_IQ_DATA)


/*
 * periodic_timer_callback() - periodic timer callback routine
 *
 * This function is called by the periodic timer interrupt when the timer
 * expires.  It sets a flag to that will be checked in the main() loop.
 *
 * If the sensor is operating in hardware-triggered mode, this function calls
 * ch_group_trigger() during each execution.
 *
 * This function is registered by the call to chbsp_periodic_timer_init()
 * in main().
 */
static void periodic_timer_callback(void)
{
	/* Set timer flag - it will be checked and cleared in main() loop */
	taskflags |= TIMER_FLAG;

	/* If using normal h/w triggering, trigger all sensors now
	 *   If software triggering selected, it will be done at task level in
	 *   main() loop based on TIMER_FLAG.
	 */
	if ((CHIRP_TRIGGER_TYPE == CH_TRIGGER_TYPE_HW) && (num_triggered_sensors > 0)) {
		ch_group_trigger(&chirp_group);
	}
}

/*
 * sensor_int_callback() - sensor interrupt callback routine
 *
 * This function is called by the board support package's interrupt handler for
 * the sensor's INT line every time that the sensor interrupts.  The device
 * number parameter, dev_num, is used to identify the interrupting device
 * within the sensor group.  (Generally the device number is same as the port
 * number used in the BSP to manage I/O pins, etc.)
 *
 * Each time this function is called, a bit is set in the data_ready_devices
 * variable to identify the interrupting device.  When all active sensors have
 * interrupted (found by comparing with the active_devices variable), the
 * DATA_READY_FLAG is set.  That flag will be detected in the main() loop.
 *
 * This callback function is registered by the call to ch_io_int_callback_set()
 * in main().
 */
static void sensor_int_callback(ch_group_t *grp_ptr, uint8_t dev_num, ch_interrupt_type_t int_type)
{
	// DEBUG_PRINT("sensor_int_callback\n");
	ch_dev_t *dev_ptr = ch_get_dev_ptr(grp_ptr, dev_num);

#ifndef INCLUDE_ALGO_EXTERNAL
	if (int_type != CH_INTERRUPT_TYPE_DATA_RDY) {
		/* Check soniclib.h CH_INTERRUPT_TYPE_* for values details */
		DEBUG_PRINT("Sensor %u : Bad interrupt type : %04X\r\n", dev_num, int_type);
		return;
	}
#else
	// allow ALGO_ERROR flag because ASIC FW has no ALGO and we won't read out algo data
	if ((int_type != CH_INTERRUPT_TYPE_DATA_RDY) && (int_type != CH_INTERRUPT_TYPE_ALGO_ERROR)) {
		/* Check soniclib.h CH_INTERRUPT_TYPE_* for values details */
		DEBUG_PRINT("Sensor %u : Bad interrupt type : %04X\r\n", dev_num, int_type);
		return;
	}
#endif
    // chbsp_led_toggle(dev_num);              // toggle led to show sensor active

	data_ready_devices |= (1 << dev_num);	// add to data-ready bit mask
	// DEBUG_PRINT("data ready: %x active %x\n", data_ready_devices, active_devices);
	if (data_ready_devices == active_devices) {
		/* All active sensors have interrupted after performing a measurement */
		data_ready_devices = 0;

		/* Set data-ready flag - it will be checked in main() loop */
		taskflags |= DATA_READY_FLAG;

		/* Disable interrupt for shared int/trig pin if in h/w triggered mode
		*   It will automatically be re-enabled by the next ch_group_trigger()
		*/
		if ((ch_get_mode(dev_ptr) != CH_MODE_FREERUN) &&
			(ch_get_trigger_type(dev_ptr) == CH_TRIGGER_TYPE_HW) &&
			(CHIRP_SENSOR_INT_PIN == CHIRP_SENSOR_TRIG_PIN)) {

			chdrv_int_group_interrupt_disable(grp_ptr);
		} else {
			chdrv_int_set_dir_in(dev_ptr);			// set INT line as input and enable int
			chdrv_int_group_interrupt_enable(grp_ptr);
		}
	}
}

static bool handleMemRead(const uint32_t memAddr, const uint8_t readLen, uint8_t* buffer)
{
#ifdef READ_IQ_DATA_CF
  if (memAddr == 0)
  {
    memcpy((void *)rawDataArrayMemFrame, (void *)chirp_data[0].iq_data, ICU_MAX_NUM_SAMPLES*sizeof(ch_iq_sample_t)); 
    // memcpy((void *)(rawDataArrayMemFrame + ICU_MAX_NUM_SAMPLES*sizeof(ch_iq_sample_t)), (void *)chirp_data[1].iq_data, ICU_MAX_NUM_SAMPLES*sizeof(ch_iq_sample_t)); //TODO remove hardcoded 2 sensors
  }

  if (memAddr + readLen <= ICU_MAX_NUM_SAMPLES*sizeof(ch_iq_sample_t)) //TODO remove hardcoded 2 sensors
  {
    memcpy((void *)buffer, (void *)(&rawDataArrayMemFrame[memAddr]), readLen);
  }
#endif // READ_IQ_DATA_CF
#ifdef READ_AMP_DATA_CF
  if (memAddr == 0)
  {
    memcpy((void *)rawDataArrayMemFrame, (void *)avg_filtered_amp, ICU_MAX_NUM_SAMPLES*sizeof(uint16_t)); 
    // memcpy((void *)(rawDataArrayMemFrame + ICU_MAX_NUM_SAMPLES*sizeof(ch_iq_sample_t)), (void *)chirp_data[1].iq_data, ICU_MAX_NUM_SAMPLES*sizeof(ch_iq_sample_t)); //TODO remove hardcoded 2 sensors
  }

  if (memAddr + readLen <= ICU_MAX_NUM_SAMPLES*sizeof(uint16_t)) //TODO remove hardcoded 2 sensors
  {
    memcpy((void *)buffer, (void *)(&rawDataArrayMemFrame[memAddr]), readLen);
  }
#endif // READ_AMP_DATA_CF
  return true;
}

float handle_data_ready_return_avg_amp(ch_group_t *grp_ptr) //AVG_CALIB ->remove the whole function if you use the AVG_MAX
{
	// DEBUG_PRINT("handle data ready\n");
	uint8_t  dev_num;
	uint16_t num_samples = 0;
	uint8_t  ret_val = 0;
#ifndef INCLUDE_ALGO_EXTERNAL
	uint32_t range;
	uint16_t amplitude;
	ch_range_t range_type;
#endif

	/* Read and display data from each connected sensor
	 *   This loop will write the sensor data to this application's "chirp_data"
	 *   array.  Each sensor has a separate chirp_data_t structure in that
	 *   array, so the device number is used as an index.
	 *
	 *   Multiple detected targets will be displayed if DISPLAY_MULTI_TARGET is non-zero.
	 *   Otherwise, only the closest detected target will be displayed.
	 */
	for (dev_num = 0; dev_num < ch_get_num_ports(grp_ptr); dev_num++) {
		ch_dev_t *dev_ptr = ch_get_dev_ptr(grp_ptr, dev_num);

		if (ch_sensor_is_connected(dev_ptr)) {
#ifndef INCLUDE_ALGO_EXTERNAL
			 /*  For sensors in transmit/receive mode, report one-way echo
			 *   range.  For sensors in receive-only mode, report direct
			 *   one-way range from transmitting sensor.
			 */
			if (ch_get_mode(dev_ptr) == CH_MODE_TRIGGERED_RX_ONLY) {
				DEBUG_PRINT("receive only works\n"); //TODO: REMOVE AGAIN
				range_type = CH_RANGE_DIRECT;
			} else {
				range_type = CH_RANGE_ECHO_ONE_WAY;
			}
#endif

			// DEBUG_PRINT("Dev %d:  ", dev_num);

#ifndef INCLUDE_ALGO_EXTERNAL
#if DISPLAY_MULTI_TARGET
			/* Display multiple detected targets (if any) */

			uint8_t num_targets = ch_get_num_targets(dev_ptr);

			if (num_targets > 0) {

				for (uint8_t target_num = 0; target_num < num_targets;
						target_num++) {

					range = ch_get_target_range(dev_ptr, target_num, range_type);
					amplitude = ch_get_target_amplitude(dev_ptr, target_num);

					if (target_num == 0) {					// store first target values
						chirp_data[dev_num].range = range;
						chirp_data[dev_num].amplitude = amplitude;
					}

					DEBUG_PRINT("Tgt %d: %0.1f mm ", target_num, (float) range / 32.0f);

					if (DISPLAY_SAMPLE_NUM) {
						uint16_t num_mm = (range / 32);

						/* ch_mm_to_samples() assumes num_mm is one-way (1/2 round-trip time-of-flight) */
						if (range_type != CH_RANGE_ECHO_ONE_WAY) {
							num_mm /= 2;		// mm distance was entire time-of-flight, so convert
						}
						DEBUG_PRINT("(sample %u) ", ch_mm_to_samples(dev_ptr, num_mm));
					}

					if (DISPLAY_AMP_VALUE) {
						DEBUG_PRINT("amp=%5u ", amplitude);
					}
					DEBUG_PRINT("  ");
				}
			} else {
				DEBUG_PRINT("        no target found  ");
			}
#else	// DISPLAY_MULTI_TARGET
			/* Display single detected target (if any) */

			range = ch_get_range(dev_ptr, range_type);
			chirp_data[dev_num].range = range;

			if (range != CH_NO_TARGET) {
				/* Target object was successfully detected (range available) */

				 /* Get the new amplitude value - it's only updated if range
				  * was successfully measured.  */
				amplitude = ch_get_amplitude(dev_ptr);
				chirp_data[dev_num].amplitude = amplitude;

				DEBUG_PRINT("Range: %0.1f mm ", (float) range/32.0f);
				DEBUG_PRINT("Amplitude: %d ", amplitude);

				if (DISPLAY_SAMPLE_NUM) {
					uint16_t num_mm = (range / 32);

					/* ch_mm_to_samples() assumes num_mm is one-way (1/2 round-trip time-of-flight) */
					if (range_type != CH_RANGE_ECHO_ONE_WAY) {
						num_mm /= 2;		// mm distance was entire time-of-flight, so convert
					}
					// DEBUG_PRINT("(sample %u) ", ch_mm_to_samples(dev_ptr, num_mm));
				}

				if (DISPLAY_AMP_VALUE) {
					//DEBUG_PRINT("amp=%5u ", chirp_data[dev_num].amplitude);
				}

			} else {
				/* No target object was detected - no range value */
				chirp_data[dev_num].amplitude = 0;  /* no updated amplitude */

				DEBUG_PRINT("        no target found  ");
			}
#endif	// DISPLAY_MULTI_TARGET
#endif

			/* Store number of active samples in this measurement */
			num_samples = ch_get_num_samples(dev_ptr);
			chirp_data[dev_num].num_samples = num_samples;

#if (READ_AMPLITUDE_DATA || READ_IQ_DATA)
			/* Optionally read all samples in measurement from sensor */
			read_measurement_data(dev_ptr);

			if (!READ_DATA_NONBLOCKING) {
				/* Data read is already complete - log or display sample data */
  #if (OUTPUT_AMP_LOG || OUTPUT_IQ_LOG)
				log_sensor_data(&chirp_data[dev_num]);
  #elif (OUTPUT_AMP_DATA_CSV || OUTPUT_IQ_DATA)
				display_sensor_data(&chirp_data[dev_num]);
  #else
				//DEBUG_PRINT("read %u IQ samples\n", chirp_data[dev_num].num_samples);
				//float amp_sum = 0.0;
				for (int i = chirp_data[dev_num].num_samples-AMP_CALIBRATE_FILTER_LEN; i < chirp_data[dev_num].num_samples; i++)
				{
					float sample_i = (float)chirp_data[dev_num].iq_data[i].i;
					float sample_q = (float)chirp_data[dev_num].iq_data[i].q;
					//amp_sum += sqrtf(sample_i*sample_i + sample_q*sample_q);
					amp_array[i+AMP_CALIBRATE_FILTER_LEN-chirp_data[dev_num].num_samples] = sqrtf(sample_i*sample_i + sample_q*sample_q);
				}
				
  #endif
			}
			

#else	// not reading amp or I/Q data
			DEBUG_PRINT("\n");		// final EOL for this device/measurement result
#endif 	// (READ_AMPLITUDE_DATA || READ_IQ_DATA)

		}	// end if (ch_sensor_is_connected(dev_ptr))
	}	// end for (dev_num < ch_get_num_ports(grp_ptr))
	float amp_sum = 0;
	for (int i = 0; i < AMP_CALIBRATE_FILTER_LEN; i++) {
		amp_sum += amp_array[i];
		//DEBUG_PRINT("amp_array: %d", amp_array[i]);
	}
	float amp_avg = amp_sum/AMP_CALIBRATE_FILTER_LEN;
	//DEBUG_PRINT("amp_avg: %f\n", amp_avg);
	
	return amp_avg;
}

/*	Uncomment this whole function if you want to use the average of the NR_MAX_AMPL 
	maximum amplitudes samples for the calibration 
	instead of the average over all the samples //AVG_MAX
*/
// void handle_data_ready_return_max_amp(ch_group_t *grp_ptr)
// {
// 	// DEBUG_PRINT("handle data ready\n");
// 	uint8_t  dev_num;
// 	uint16_t num_samples = 0;
// 	uint8_t  ret_val = 0;
// #ifndef INCLUDE_ALGO_EXTERNAL
// 	uint32_t range;
// 	uint16_t amplitude;
// 	ch_range_t range_type;
// #endif

// 	/* Read and display data from each connected sensor
// 	 *   This loop will write the sensor data to this application's "chirp_data"
// 	 *   array.  Each sensor has a separate chirp_data_t structure in that
// 	 *   array, so the device number is used as an index.
// 	 *
// 	 *   Multiple detected targets will be displayed if DISPLAY_MULTI_TARGET is non-zero.
// 	 *   Otherwise, only the closest detected target will be displayed.
// 	 */
// 	for (dev_num = 0; dev_num < ch_get_num_ports(grp_ptr); dev_num++) {
// 		ch_dev_t *dev_ptr = ch_get_dev_ptr(grp_ptr, dev_num);

// 		if (ch_sensor_is_connected(dev_ptr)) {
// #ifndef INCLUDE_ALGO_EXTERNAL
// 			 /*  For sensors in transmit/receive mode, report one-way echo
// 			 *   range.  For sensors in receive-only mode, report direct
// 			 *   one-way range from transmitting sensor.
// 			 */
// 			if (ch_get_mode(dev_ptr) == CH_MODE_TRIGGERED_RX_ONLY) {
// 				DEBUG_PRINT("receive only works\n"); //TODO: REMOVE AGAIN
// 				range_type = CH_RANGE_DIRECT;
// 			} else {
// 				range_type = CH_RANGE_ECHO_ONE_WAY;
// 			}
// #endif

// 			// DEBUG_PRINT("Dev %d:  ", dev_num);

// #ifndef INCLUDE_ALGO_EXTERNAL
// #if DISPLAY_MULTI_TARGET
// 			/* Display multiple detected targets (if any) */

// 			uint8_t num_targets = ch_get_num_targets(dev_ptr);

// 			if (num_targets > 0) {

// 				for (uint8_t target_num = 0; target_num < num_targets;
// 						target_num++) {

// 					range = ch_get_target_range(dev_ptr, target_num, range_type);
// 					amplitude = ch_get_target_amplitude(dev_ptr, target_num);

// 					if (target_num == 0) {					// store first target values
// 						chirp_data[dev_num].range = range;
// 						chirp_data[dev_num].amplitude = amplitude;
// 					}

// 					DEBUG_PRINT("Tgt %d: %0.1f mm ", target_num, (float) range / 32.0f);

// 					if (DISPLAY_SAMPLE_NUM) {
// 						uint16_t num_mm = (range / 32);

// 						/* ch_mm_to_samples() assumes num_mm is one-way (1/2 round-trip time-of-flight) */
// 						if (range_type != CH_RANGE_ECHO_ONE_WAY) {
// 							num_mm /= 2;		// mm distance was entire time-of-flight, so convert
// 						}
// 						DEBUG_PRINT("(sample %u) ", ch_mm_to_samples(dev_ptr, num_mm));
// 					}

// 					if (DISPLAY_AMP_VALUE) {
// 						DEBUG_PRINT("amp=%5u ", amplitude);
// 					}
// 					DEBUG_PRINT("  ");
// 				}
// 			} else {
// 				DEBUG_PRINT("        no target found  ");
// 			}
// #else	// DISPLAY_MULTI_TARGET
// 			/* Display single detected target (if any) */

// 			range = ch_get_range(dev_ptr, range_type);
// 			chirp_data[dev_num].range = range;

// 			if (range != CH_NO_TARGET) {
// 				/* Target object was successfully detected (range available) */

// 				 /* Get the new amplitude value - it's only updated if range
// 				  * was successfully measured.  */
// 				amplitude = ch_get_amplitude(dev_ptr);
// 				chirp_data[dev_num].amplitude = amplitude;

// 				DEBUG_PRINT("Range: %0.1f mm ", (float) range/32.0f);
// 				DEBUG_PRINT("Amplitude: %d ", amplitude);

// 				if (DISPLAY_SAMPLE_NUM) {
// 					uint16_t num_mm = (range / 32);

// 					/* ch_mm_to_samples() assumes num_mm is one-way (1/2 round-trip time-of-flight) */
// 					if (range_type != CH_RANGE_ECHO_ONE_WAY) {
// 						num_mm /= 2;		// mm distance was entire time-of-flight, so convert
// 					}
// 					// DEBUG_PRINT("(sample %u) ", ch_mm_to_samples(dev_ptr, num_mm));
// 				}

// 				if (DISPLAY_AMP_VALUE) {
// 					DEBUG_PRINT("amp=%5u ", chirp_data[dev_num].amplitude);  //TODO: REMOVE again
// 				}

// 			} else {
// 				/* No target object was detected - no range value */
// 				chirp_data[dev_num].amplitude = 0;  /* no updated amplitude */

// 				DEBUG_PRINT("        no target found  ");
// 			}
// #endif	// DISPLAY_MULTI_TARGET
// #endif

// 			/* Store number of active samples in this measurement */
// 			num_samples = ch_get_num_samples(dev_ptr);
// 			chirp_data[dev_num].num_samples = num_samples;

// #if (READ_AMPLITUDE_DATA || READ_IQ_DATA)
// 			/* Optionally read all samples in measurement from sensor */
// 			read_measurement_data(dev_ptr);

// 			if (!READ_DATA_NONBLOCKING) {
// 				/* Data read is already complete - log or display sample data */
//   #if (OUTPUT_AMP_LOG || OUTPUT_IQ_LOG)
// 				log_sensor_data(&chirp_data[dev_num]);
//   #elif (OUTPUT_AMP_DATA_CSV || OUTPUT_IQ_DATA)
// 				display_sensor_data(&chirp_data[dev_num]);
//   #else
// 				DEBUG_PRINT("read %u IQ samples\n", chirp_data[dev_num].num_samples);
// 				//float amp_sum = 0.0;
// 				for (int i = chirp_data[dev_num].num_samples-AMP_CALIBRATE_FILTER_LEN; i < chirp_data[dev_num].num_samples; i++)
// 				{
// 					float sample_i = (float)chirp_data[dev_num].iq_data[i].i;
// 					float sample_q = (float)chirp_data[dev_num].iq_data[i].q;
// 					//amp_sum += sqrtf(sample_i*sample_i + sample_q*sample_q);
// 					amp_array[i+AMP_CALIBRATE_FILTER_LEN-chirp_data[dev_num].num_samples] = sqrtf(sample_i*sample_i + sample_q*sample_q); //TODO: remove some unnecessary stuff around here
// 				}
// 				//uint16_t amp_avg11 = amp_sum/10; // adapt dynamically to motor noise
// 				//uint8_t first_obstacle = 1; //TODO: maybe merge the 2 for loops into one to be more efficient
// 				for (int i = 0; i < chirp_data[dev_num].num_samples; i++)
// 				{
// 					float sample_i = (float)chirp_data[dev_num].iq_data[i].i;
// 					float sample_q = (float)chirp_data[dev_num].iq_data[i].q;
// 					float amp = sqrtf(sample_i*sample_i + sample_q*sample_q);
					
// 					if (amp > min_amp_array) {
// 						float min_amp_new = amp;
// 						bool changed_one_val = false;
// 						for (int i = 0; i<NR_MAX_AMPL;i++) {
// 							//DEBUG_PRINT("amp_array[%d]: %f", i, max_5_amp[i]);
// 							//DEBUG_PRINT("amp = %f", amp);
// 							if ((max_5_amp[i] >= min_amp_array-0.01) && (max_5_amp[i] <= min_amp_array+0.01) && !changed_one_val) {
// 								max_5_amp[i] = amp;
// 								changed_one_val = true;
// 								//DEBUG_PRINT("amp update: %f\n", amp);
// 							}
// 							if (max_5_amp[i] < amp) {
// 								min_amp_new = max_5_amp[i];
// 							}
// 						}
// 						min_amp_array = min_amp_new;
						
// 					}		
// 				}
				
//   #endif
// 			}
			

// #else	// not reading amp or I/Q data
// 			DEBUG_PRINT("\n");		// final EOL for this device/measurement result
// #endif 	// (READ_AMPLITUDE_DATA || READ_IQ_DATA)

// 		}	// end if (ch_sensor_is_connected(dev_ptr))
// 	}	// end for (dev_num < ch_get_num_ports(grp_ptr))
// }


static void ringdown_filter_define(ch_group_t *grp_ptr)
{
	// DEBUG_PRINT("handle data ready\n");
	uint8_t  dev_num;
	uint16_t num_samples = 0;
#ifndef INCLUDE_ALGO_EXTERNAL
	uint32_t range;
	uint16_t amplitude;
	ch_range_t range_type;
#endif

	/* Read and display data from each connected sensor
	 *   This loop will write the sensor data to this application's "chirp_data"
	 *   array.  Each sensor has a separate chirp_data_t structure in that
	 *   array, so the device number is used as an index.
	 *
	 *   Multiple detected targets will be displayed if DISPLAY_MULTI_TARGET is non-zero.
	 *   Otherwise, only the closest detected target will be displayed.
	 */

	for (dev_num = 0; dev_num < ch_get_num_ports(grp_ptr); dev_num++) {
		ch_dev_t *dev_ptr = ch_get_dev_ptr(grp_ptr, dev_num);

		if (ch_sensor_is_connected(dev_ptr)) {
#ifndef INCLUDE_ALGO_EXTERNAL
			 /*  For sensors in transmit/receive mode, report one-way echo
			 *   range.  For sensors in receive-only mode, report direct
			 *   one-way range from transmitting sensor.
			 */
			if (ch_get_mode(dev_ptr) == CH_MODE_TRIGGERED_RX_ONLY) {
				range_type = CH_RANGE_DIRECT;
			} else {
				range_type = CH_RANGE_ECHO_ONE_WAY;
			}
#endif

			// DEBUG_PRINT("Dev %d:  ", dev_num);

#ifndef INCLUDE_ALGO_EXTERNAL
#if DISPLAY_MULTI_TARGET
			/* Display multiple detected targets (if any) */

			uint8_t num_targets = ch_get_num_targets(dev_ptr);

			if (num_targets > 0) {

				for (uint8_t target_num = 0; target_num < num_targets;
						target_num++) {

					range = ch_get_target_range(dev_ptr, target_num, range_type);
					amplitude = ch_get_target_amplitude(dev_ptr, target_num);

					if (target_num == 0) {					// store first target values
						chirp_data[dev_num].range = range;
						chirp_data[dev_num].amplitude = amplitude;
					}

					DEBUG_PRINT("Tgt %d: %0.1f mm ", target_num, (float) range / 32.0f);

					if (DISPLAY_SAMPLE_NUM) {
						uint16_t num_mm = (range / 32);

						/* ch_mm_to_samples() assumes num_mm is one-way (1/2 round-trip time-of-flight) */
						if (range_type != CH_RANGE_ECHO_ONE_WAY) {
							num_mm /= 2;		// mm distance was entire time-of-flight, so convert
						}
						DEBUG_PRINT("(sample %u) ", ch_mm_to_samples(dev_ptr, num_mm));
					}

					if (DISPLAY_AMP_VALUE) {
						DEBUG_PRINT("amp=%5u ", amplitude);
					}
					DEBUG_PRINT("  ");
				}
			} else {
				DEBUG_PRINT("        no target found  ");
			}
#else	// DISPLAY_MULTI_TARGET
			/* Display single detected target (if any) */

			range = ch_get_range(dev_ptr, range_type);
			chirp_data[dev_num].range = range;

			if (range != CH_NO_TARGET) {
				/* Target object was successfully detected (range available) */

				 /* Get the new amplitude value - it's only updated if range
				  * was successfully measured.  */
				amplitude = ch_get_amplitude(dev_ptr);
				chirp_data[dev_num].amplitude = amplitude;

				DEBUG_PRINT("Range: %0.1f mm ", (float) range/32.0f);
				DEBUG_PRINT("Amplitude: %d ", amplitude);

				if (DISPLAY_SAMPLE_NUM) {
					uint16_t num_mm = (range / 32);

					/* ch_mm_to_samples() assumes num_mm is one-way (1/2 round-trip time-of-flight) */
					if (range_type != CH_RANGE_ECHO_ONE_WAY) {
						num_mm /= 2;		// mm distance was entire time-of-flight, so convert
					}
					// DEBUG_PRINT("(sample %u) ", ch_mm_to_samples(dev_ptr, num_mm));
				}

				if (DISPLAY_AMP_VALUE) {
					// DEBUG_PRINT("amp=%5u ", chirp_data[dev_num].amplitude);
				}

			} else {
				/* No target object was detected - no range value */
				chirp_data[dev_num].amplitude = 0;  /* no updated amplitude */

				DEBUG_PRINT("        no target found  ");
			}
#endif	// DISPLAY_MULTI_TARGET
#endif

			/* Store number of active samples in this measurement */
			num_samples = ch_get_num_samples(dev_ptr);
			chirp_data[dev_num].num_samples = num_samples;

#if (READ_AMPLITUDE_DATA || READ_IQ_DATA)
			/* Optionally read all samples in measurement from sensor */
			read_measurement_data(dev_ptr);

			if (!READ_DATA_NONBLOCKING) {
				/* Data read is already complete - log or display sample data */
  #if (OUTPUT_AMP_LOG || OUTPUT_IQ_LOG)
				log_sensor_data(&chirp_data[dev_num]);
  #elif (OUTPUT_AMP_DATA_CSV || OUTPUT_IQ_DATA)
				display_sensor_data(&chirp_data[dev_num]);
  #else
				// DEBUG_PRINT("read %u IQ samples\n", chirp_data[dev_num].num_samples);
				// float THRESHOLD_LEVEL_SUM = 0.0;
				// for (int i = chirp_data[dev_num].num_samples - 10; i < chirp_data[dev_num].num_samples; i++)
				// {
				// 	float sample_i = (float)chirp_data[dev_num].iq_data[i].i;
				// 	float sample_q = (float)chirp_data[dev_num].iq_data[i].q;
				// 	THRESHOLD_LEVEL_SUM += sqrtf(sample_i*sample_i + sample_q*sample_q);
				// }
				// THRESHOLD_LEVEL = THRESHOLD_LEVEL*9/10 + 1.5*THRESHOLD_LEVEL_SUM/10/10; // adapt dynamically to motor noise
				uint8_t first_obstacle = 1;
				for (int i = 0; i < chirp_data[dev_num].num_samples; i++)
				{
					float sample_i = (float)chirp_data[dev_num].iq_data[i].i;
					float sample_q = (float)chirp_data[dev_num].iq_data[i].q;
					float amp = sqrtf(sample_i*sample_i + sample_q*sample_q);
					if(i < CHIRP_RINGDOWN_FILTER_SAMPLES)
					{
						ringdown_filter[i] += amp;
					}
				}
				
  #endif
			}

#else	// not reading amp or I/Q data
			DEBUG_PRINT("\n");		// final EOL for this device/measurement result
#endif 	// (READ_AMPLITUDE_DATA || READ_IQ_DATA)

		}	// end if (ch_sensor_is_connected(dev_ptr))
	}	// end for (dev_num < ch_get_num_ports(grp_ptr))
}

LOG_GROUP_START(us)
LOG_ADD(LOG_FLOAT, range_0, &(log_range[0]))  //WHAT I CAN LOG
LOG_ADD(LOG_FLOAT, range_1, &(log_range[1]))
LOG_ADD(LOG_FLOAT, MAX_RANGE, &MAX_RANGE)
LOG_ADD(LOG_FLOAT, THR, &THRESHOLD_LEVEL)
LOG_GROUP_STOP(us)

PARAM_GROUP_START(ToF_FLY_PARAMS)
PARAM_ADD(PARAM_UINT16, ToFly, &ToFly)
PARAM_GROUP_STOP(ToF_FLY_PARAMS)