/*!
 * \file chbsp_dummy.c
 *
 * \brief Dummy implementations of optional board support package IO functions allowing
 * platforms to selectively support only needed functionality.  These are placeholder
 * routines that will satisfy references from other code to avoid link errors, but they
 * do not peform any actual operations.
 *
 * See chirp_bsp.h for descriptions of all board support package interfaces, including
 * details on these optional functions.
 */

/*
 * Copyright (c) 2017-2021 Chirp Microsystems.  All rights reserved.
 */
#include <invn/soniclib/chirp_bsp.h>
#include "FreeRTOS.h"
#include "task.h"
#include "nvicconf.h"

#include "deck.h"
#include "deck_spi.h"
#include "debug.h"
#include "system.h"
#include "log.h"
#include "param.h"
#include "sleepus.h"
#include "chirp_board_config.h"

extern volatile uint32_t taskflags;
extern volatile uint32_t interrupt_sensors;
extern volatile uint8_t execution_started;

uint8_t dummy[0x17c0] = {0};

#define DECK_GPIO_CS_0      0//DECK_GPIO_RX1
#define DECK_GPIO_CS_1  	1//DECK_GPIO_TX1
#define DECK_GPIO_INT2_0	    8//DECK_GPIO_TX2 // FIX ME - this is a workaround to use 2 INT pins
#define DECK_GPIO_INT2_1      9//DECK_GPIO_RX2 // FIX ME - this is a workaround to use 2 INT pins


static const deckPin_t icu_cs_pins[] = {DECK_GPIO_CS_0, DECK_GPIO_CS_1};//, DECK_GPIO_CS_2, DECK_GPIO_CS_3};

#define DECK_GPIO_INT1_0    5//DECK_GPIO_IO2
#define DECK_GPIO_INT1_1  	4//DECK_GPIO_IO1
// #define DECK_GPIO_INT1_2	5//DECK_GPIO_IO2
// #define DECK_GPIO_INT1_3    5//DECK_GPIO_IO2


static const deckPin_t icu_int1_pins[] = {DECK_GPIO_INT1_0, DECK_GPIO_INT1_1};//, DECK_GPIO_INT1_2, DECK_GPIO_INT1_3};
static const deckPin_t icu_int2_pins[] = {DECK_GPIO_INT2_0, DECK_GPIO_INT2_1}; // FIX ME - this is a workaround to use 2 INT pins

// #define GPIO_PIN_IRQ 	  DECK_GPIO_INT1_0
// #define EXTI_PortSource EXTI_PortSourceGPIOB
// #define EXTI_PinSource 	EXTI_PinSource5
// #define EXTI_LineN 		  EXTI_Line5

#define GPIO_PIN_IRQ_0 	  DECK_GPIO_INT1_0
#define EXTI_PortSource_0 EXTI_PortSourceGPIOB
#define EXTI_PinSource_0 	EXTI_PinSource5
#define EXTI_LineN_0 		  EXTI_Line5

#define GPIO_PIN_IRQ_1 	  DECK_GPIO_INT1_1
#define EXTI_PortSource_1 EXTI_PortSourceGPIOB
#define EXTI_PinSource_1 	EXTI_PinSource8
#define EXTI_LineN_1 		  EXTI_Line8

IRQn_Type IRQn_array[] = {EXTI9_5_IRQn, EXTI3_IRQn};

static ch_group_t *sensor_group_ptr;

EXTI_InitTypeDef EXTI_InitStructure_0;
EXTI_InitTypeDef EXTI_InitStructure_1;


/* Functions supporting debugging */

__attribute__((weak)) void chbsp_debug_toggle(uint8_t __attribute__((unused)) dbg_pin_num) {}

__attribute__((weak)) void chbsp_debug_on(uint8_t __attribute__((unused)) dbg_pin_num) {}

__attribute__((weak)) void chbsp_debug_off(uint8_t __attribute__((unused)) dbg_pin_num) {}

__attribute__((weak)) void chbsp_print_str(char *str) {
	(void)(str);
}

uint32_t chbsp_timestamp_ms() {
	return xTaskGetTickCount();
}



__attribute__((weak)) void chbsp_group_int2_interrupt_enable(ch_group_t *grp_ptr) {
	(void)(grp_ptr);
}

__attribute__((weak)) void chbsp_int2_interrupt_enable(ch_dev_t *dev_ptr) {
	(void)(dev_ptr);
}

__attribute__((weak)) void chbsp_group_int2_interrupt_disable(ch_group_t *grp_ptr) {
	(void)(grp_ptr);
}

__attribute__((weak)) void chbsp_int2_interrupt_disable(ch_dev_t *dev_ptr) {
	(void)(dev_ptr);
}





/* Functions supporting non-blocking operation */

__attribute__((weak)) int chbsp_i2c_mem_write_nb(ch_dev_t *dev_ptr, uint16_t mem_addr, uint8_t *data, uint16_t num_bytes) {
	(void)(dev_ptr);
	(void)(mem_addr);
	(void)(data);
	(void)(num_bytes);
	return 1;
}

__attribute__((weak)) int chbsp_i2c_read_nb(ch_dev_t *dev_ptr, uint8_t *data, uint16_t num_bytes) {
	(void)(dev_ptr);
	(void)(data);
	(void)(num_bytes);
	return 1;
}

__attribute__((weak)) int chbsp_i2c_mem_read_nb(ch_dev_t *dev_ptr, uint16_t mem_addr, uint8_t *data, uint16_t num_bytes) {
	(void)(dev_ptr);
	(void)(mem_addr);
	(void)(data);
	(void)(num_bytes);
	return 1;
}

__attribute__((weak)) void chbsp_external_irq_handler(chdrv_transaction_t *trans){
	(void)(trans);
}

__attribute__((weak)) int chbsp_spi_mem_read_nb(ch_dev_t *dev_ptr, uint16_t mem_addr, uint8_t *data, uint16_t num_bytes) {
	(void)(dev_ptr);
	(void)(mem_addr);
	(void)(data);
	(void)(num_bytes);
	return 1;
}


void chbsp_int2_clear(ch_dev_t *dev_ptr) {
	digitalWrite(icu_int2_pins[ch_get_dev_num(dev_ptr)], LOW);
}

void chbsp_int2_set(ch_dev_t *dev_ptr) {
	digitalWrite(icu_int2_pins[ch_get_dev_num(dev_ptr)], HIGH);
}

void chbsp_group_int2_set(ch_group_t *grp_ptr) {
	for (int i = 0; i < CHIRP_MAX_NUM_SENSORS; i++)
	{
		digitalWrite(icu_int2_pins[i], HIGH);
	}
}

void chbsp_group_int2_clear(ch_group_t *grp_ptr) {
	for (int i = 0; i < CHIRP_MAX_NUM_SENSORS; i++)
	{
		digitalWrite(icu_int2_pins[i], LOW);
	}
}

/*!
 * \brief Configure the Chirp sensor INT1 pin as an output for a group of sensors
 * 
 * \param grp_ptr 		pointer to the ch_group_t config structure for a group of sensors
 *
 * This function should configure each Chirp sensor's INT1 pin as an output (from the perspective 
 * of the host system).
 */
void chbsp_group_set_int1_dir_out(ch_group_t *grp_ptr) {
	for (int i = 0; i < CHIRP_MAX_NUM_SENSORS; i++)
	{
		pinMode(icu_int1_pins[i], OUTPUT);
	}
	// DEBUG_PRINT("int1 group dir out\n");
}

void chbsp_group_set_int2_dir_out(ch_group_t *grp_ptr) {
	for (int i = 0; i < CHIRP_MAX_NUM_SENSORS; i++)
	{
		pinMode(icu_int2_pins[i], OUTPUT);
	}
	// DEBUG_PRINT("int1 group dir out\n");
}

/*!
 * \brief Configure the Chirp sensor INT1 pin as an output for one sensor.
 *
 * \param dev_ptr 		pointer to the ch_dev_t config structure for a sensor
 *
 * This function configures the Chirp sensor INT1 pin as an output (from the perspective 
 * of the host system).
 */
void chbsp_set_int1_dir_out(ch_dev_t *dev_ptr) {
	pinMode(icu_int1_pins[ch_get_dev_num(dev_ptr)], OUTPUT);
	// DEBUG_PRINT("int1 dir out %d\n" ,ch_get_dev_num(dev_ptr));
}

void chbsp_set_int2_dir_out(ch_dev_t *dev_ptr) {
	pinMode(icu_int2_pins[ch_get_dev_num(dev_ptr)], OUTPUT);
	// DEBUG_PRINT("int1 dir out %d\n" ,ch_get_dev_num(dev_ptr));
}

/*!
 * \brief Configure the Chirp sensor INT1 pins as inputs for a group of sensors
 * 
 * \param grp_ptr 		pointer to the ch_group_t config structure for a group of sensors
 * 
 * This function should configure each Chirp sensor's INT1 pin as an input (from the perspective 
 * of the host system).
 */
void chbsp_group_set_int1_dir_in(ch_group_t *grp_ptr) {
	for (int i = 0; i < CHIRP_MAX_NUM_SENSORS; i++)
	{
		pinMode(icu_int1_pins[i], INPUT_PULLDOWN);
	}
	// DEBUG_PRINT("int1 group dir in\n");
}

void chbsp_group_set_int2_dir_in(ch_group_t *grp_ptr) {
	for (int i = 0; i < CHIRP_MAX_NUM_SENSORS; i++)
	{
		pinMode(icu_int2_pins[i], INPUT_PULLDOWN);
	}
	// DEBUG_PRINT("int1 group dir in\n");
}

/*!
 * \brief Configure the Chirp sensor INT1 pin as an input for one sensor.
 *
 * \param dev_ptr 		pointer to the ch_dev_t config structure for a sensor
 *
 * This function configures the Chirp sensor INT1 pin as an input (from the perspective of 
 * the host system).
 */
void chbsp_set_int1_dir_in(ch_dev_t *dev_ptr) {
	pinMode(icu_int1_pins[ch_get_dev_num(dev_ptr)], INPUT_PULLDOWN);
	// DEBUG_PRINT("int1 dir in %d\n", ch_get_dev_num(dev_ptr));
}
void chbsp_set_int2_dir_in(ch_dev_t *dev_ptr) {
	pinMode(icu_int2_pins[ch_get_dev_num(dev_ptr)], INPUT_PULLDOWN);
	// DEBUG_PRINT("int1 dir in %d\n", ch_get_dev_num(dev_ptr));
}

/*!
 * \brief Set the INT1 pins low for a group of sensors.
 *
 * \param grp_ptr 		pointer to the ch_group_t config structure for a group of sensors
 * 
 * This function drives the INT1 line low for each sensor in the group.
 */
void chbsp_group_int1_clear(ch_group_t *grp_ptr) {
	for (int i = 0; i < CHIRP_MAX_NUM_SENSORS; i++)
	{
		digitalWrite(icu_int1_pins[i], LOW);
	}
	// DEBUG_PRINT("int1 group clear\n");
}

void chbsp_int1_clear(ch_dev_t *dev_ptr) {
	digitalWrite(icu_int1_pins[ch_get_dev_num(dev_ptr)], LOW);
	// DEBUG_PRINT("int1 clear %d\n", ch_get_dev_num(dev_ptr));
}

void chbsp_int1_set(ch_dev_t *dev_ptr) {
	digitalWrite(icu_int1_pins[ch_get_dev_num(dev_ptr)], HIGH);
	// DEBUG_PRINT("int1 set %d\n", ch_get_dev_num(dev_ptr));
}

 /*!
 * \brief Set the INT1 pins high for a group of sensors.
 * 
 * \param grp_ptr 		pointer to the ch_group_t config structure for a group of sensors
 *
 * This function drives the INT1 line high for each sensor in the group.
 */
void chbsp_group_int1_set(ch_group_t *grp_ptr) {
	for (int i = 0; i < CHIRP_MAX_NUM_SENSORS; i++)
	{
		digitalWrite(icu_int1_pins[i], HIGH);
	}
	// DEBUG_PRINT("int1 group set\n");
}

/*!
 * \brief Disable interrupts for a group of sensors
 * 
 * \param grp_ptr 	pointer to the ch_group_t config structure for a group of sensors
 *
 * For each sensor in the group, this function enables the host interrupt associated 
 * with the Chirp sensor device's INT1 line.
 */
void chbsp_group_int1_interrupt_enable(ch_group_t *grp_ptr) {
	DEBUG_PRINT("enable interrupt\n");
	for (int i = 0; i < CHIRP_MAX_NUM_SENSORS; i++)
	{
		NVIC_EnableIRQ(IRQn_array[i]);
	}
}

/*!
 * \brief Enable the interrupt for one sensor
 *
 * \param dev_ptr	pointer to the ch_dev_t config structure for a sensor
 *
 * This function enables the host interrupt associated with the Chirp sensor device's 
 * INT1 line.
 */
void chbsp_int1_interrupt_enable(ch_dev_t *dev_ptr) {
	DEBUG_PRINT("enable interrupt for sensor %d\n", ch_get_dev_num(dev_ptr));
	DEBUG_PRINT("use interrupt %d\n", IRQn_array[ch_get_dev_num(dev_ptr)]);
	NVIC_EnableIRQ(IRQn_array[ch_get_dev_num(dev_ptr)]);
}

/*!
 * \brief Disable interrupts for a group of sensors
 * 
 * \param grp_ptr 		pointer to the ch_group_t config structure for a group of sensors
 *
 * For each sensor in the group, this function disables the host interrupt associated 
 * with the Chirp sensor device's INT1 line.
 */
void chbsp_group_int1_interrupt_disable(ch_group_t *grp_ptr) {
	for (int i = 0; i < CHIRP_MAX_NUM_SENSORS; i++)
	{
		NVIC_DisableIRQ(IRQn_array[i]);
	}
	// DEBUG_PRINT("interrup disable\n");
}

/*!
 * \brief Disable the interrupt for one sensor
 *
 * \param dev_ptr 		pointer to the ch_dev_t config structure for a sensor
 *
 * This function disables the host interrupt associated with the Chirp sensor device's 
 * INT1 line.
 */
void chbsp_int1_interrupt_disable(ch_dev_t *dev_ptr) {
	NVIC_DisableIRQ(IRQn_array[ch_get_dev_num(dev_ptr)]);
	// DEBUG_PRINT("interrup disable\n");
}

/*!
 * \brief Set callback routine for Chirp sensor I/O interrupt
 *
 * \param callback_func_ptr 	pointer to application function to be called when interrupt occurs
 *
 * This function sets up the specified callback routine to be called whenever the interrupt 
 * associated with the sensor's INT1 line occurs.  The callback routine address in stored in
 * a pointer variable that will later be accessed from within the interrupt handler to call 
 * the function.
 *
 * The callback function will be called at interrupt level from the interrupt 
 * service routine.
 */
// void chbsp_int1_callback_set(ch_io_int_callback_t callback_func_ptr) {
// }

void __attribute__((used)) EXTI5_Callback(void)
{
	// DEBUG_PRINT("callback!\n");
	interrupt_sensors = 1; //very unclean
	if(execution_started)
	{
		taskflags |= INTERRUPT_FLAG_0;
		// ch_interrupt(sensor_group_ptr, 0);
	}
}
void __attribute__((used)) EXTI8_Callback(void)
{
	// DEBUG_PRINT("callback!\n");
	interrupt_sensors = 2; //very unclean
	if(execution_started)
	{
		taskflags |= INTERRUPT_FLAG_1;
		// ch_interrupt(sensor_group_ptr, 0);
	}
}

void ch_interrupt_triggered(void)
{
	// DEBUG_PRINT("all sensors interrupted\n");
	ch_interrupt(sensor_group_ptr, 0);
	// ch_interrupt(sensor_group_ptr, 1);
}

/*!
 * \brief Delay for specified number of microseconds
 * 
 * \param us  	number of microseconds to delay before returning
 *
 * This function waits for the specified number of microseconds before returning to 
 * the caller.
 */
void chbsp_delay_us(uint32_t us) {
	sleepus(us);
}

/*!
 * \brief Delay for specified number of milliseconds.
 *
 * \param ms 	number of milliseconds to delay before returning
 *
 * This function waits for the specified number of milliseconds before returning to 
 * the caller.
 */
void chbsp_delay_ms(uint32_t ms) {
	// vTaskDelay(M2T(ms));
	sleepus(ms*1000);
}

/*!
 * \brief Assert the SPI chip select line for a device
 *
 * \param dev_ptr 		pointer to the ch_dev_t config structure for a sensor
 *
 */
void chbsp_spi_cs_on(ch_dev_t *dev_ptr) {
	// DEBUG_PRINT("CS %d ON\n", ch_get_dev_num(dev_ptr));
	spiBeginTransaction(SPI_BAUDRATE_12MHZ);
	digitalWrite(icu_cs_pins[ch_get_dev_num(dev_ptr)], LOW);
}


/*!
 * \brief De-assert the SPI chip select line for a device
 *
 * \param dev_ptr 		pointer to the ch_dev_t config structure for a sensor
 *
 */
void chbsp_spi_cs_off(ch_dev_t *dev_ptr) {
	// DEBUG_PRINT("CS %d OFF\n", ch_get_dev_num(dev_ptr));
	digitalWrite(icu_cs_pins[ch_get_dev_num(dev_ptr)], HIGH);
	spiEndTransaction();
}

/*!
 * \brief Write bytes to an SPI slave.
 * 
 * \param dev_ptr 		pointer to the ch_dev_t config structure for a sensor
 * \param data 			data to be transmitted
 * \param num_bytes 	length of data to be transmitted
 *
 * \return 0 if successful, 1 on error or NACK
 *
 * This function should write one or more bytes of data to an SPI slave device.
 * The SPI interface will have already been initialized using \a chbsp_spi_init().
 */
int chbsp_spi_write(ch_dev_t *dev_ptr, uint8_t *data, uint16_t num_bytes) {
	// DEBUG_PRINT("write num_bytes: %d\n", num_bytes);
	spiExchange(num_bytes, data, dummy);

	return 0;
}

/*!
 * \brief Read bytes from an SPI slave.
 * 
 * \param dev_ptr 		pointer to the ch_dev_t config structure for a sensor
 * \param data 			pointer to receive data buffer
 * \param num_bytes 	number of bytes to read
 *
 * \return 0 if successful, 1 on error or NACK
 *
 * This function should read the specified number of bytes from an SPI slave device.
 * The SPI interface must have already been initialized using \a chbsp_spi_init().
 *
 */
int chbsp_spi_read(ch_dev_t *dev_ptr, uint8_t *data, uint16_t num_bytes) {
	
	memset(dummy,0,num_bytes);
	// DEBUG_PRINT("read num_bytes: %d\n", num_bytes);
	spiExchange(num_bytes, dummy, data);
	
	return 0;
}

/*!
 * \brief Initialize board hardware
 *
 * \note This function performs all necessary initialization on the board.
 */
void chbsp_board_init(ch_group_t *grp_ptr) {
		/* Make local copy of group pointer */
	sensor_group_ptr = grp_ptr;

	/* Initialize group descriptor */
	grp_ptr->num_ports = 2;
	grp_ptr->num_buses = 1;
	grp_ptr->rtc_cal_pulse_ms = 100;
	grp_ptr->disco_hook = NULL;

	// Initialize CS Pin
	for (int i = 0; i < CHIRP_MAX_NUM_SENSORS; i++)
	{
		pinMode(icu_cs_pins[i], OUTPUT);
		digitalWrite(icu_cs_pins[i], HIGH);
	}
	// vTaskDelay(M2T(400)); // we want the flow deck to be initialized first
	if (!spiTest())
	{
		spiBegin();
		// vTaskDelay(M2T(40));
	}
	// digitalWrite(icu_cs_pins[0], HIGH);
	// vTaskDelay(M2T(2));
	// digitalWrite(icu_cs_pins[0], LOW);
	// vTaskDelay(M2T(2));
	// digitalWrite(icu_cs_pins[0], HIGH);
	// vTaskDelay(M2T(2));

	SYSCFG_EXTILineConfig(EXTI_PortSource_0, EXTI_PinSource_0);

	EXTI_InitStructure_0.EXTI_Line = EXTI_LineN_0;
	EXTI_InitStructure_0.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure_0.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure_0.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure_0);

	SYSCFG_EXTILineConfig(EXTI_PortSource_1, EXTI_PinSource_1);
	EXTI_InitStructure_1.EXTI_Line = EXTI_LineN_1;
	EXTI_InitStructure_1.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure_1.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure_1.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure_1);


	pinMode(icu_int2_pins[0], OUTPUT);
	pinMode(icu_int2_pins[1], OUTPUT);
	digitalWrite(icu_int2_pins[0], HIGH);
	digitalWrite(icu_int2_pins[1], HIGH);

	pinMode(icu_int1_pins[0], INPUT_PULLDOWN);
	pinMode(icu_int1_pins[1], INPUT_PULLDOWN);
	EXTI_ClearITPendingBit(EXTI_LineN_0);
	EXTI_ClearITPendingBit(EXTI_LineN_1);
	NVIC_EnableIRQ(EXTI9_5_IRQn);
	// NVIC_EnableIRQ(EXTI3_IRQn);
	DEBUG_PRINT("board init\n");

	
}