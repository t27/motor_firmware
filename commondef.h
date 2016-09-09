/*
 * commondef.h
 *
 *  Created on: Jul 31, 2016
 *      Author: Kaustubh
 *      Contains all header files, definitions
 */

#ifndef COMMONDEF_H_
#define COMMONDEF_H_

/********************
Programing Style Guide
********************/

 /*
 - Names should be descriptive, no non-standard abbreviations, we have auto complete (PWM, PID is allowed)
 - Variable Names: under_scored, all small with underscores
 - Class Names: CamelCasing again, start with capital letter
 - File Names: under_scored, start with small letter, use underscore, class FooBar will have name foo_bar.cpp and foo_bar.h
 - File Names: if a file has abstract preceed the name of the file with a_
 - Class Instance: leading 'c', FooBar cFooBar. in case of multiple instances, specify a differentiator later with underscore
 - Class data members/variables: have a trailing underscore
 - Function Names: camelCasing, start with small letters
 - Macro Names: All caps #define MACRO
 - All header files to include the usage guide of the functions present in them, abstract classes in our case
 - All TODOs, if any to be mentioned on top of the header file, CCS catches them all if required later
 - All printing statements inside #ifdef DEBUG
 - Only the following type of int data types in the code, no generic terms:
 		 int8_t		uint8_t  (0 to 255)								byte
		 int16_t	uint16_t (0 to 65,535)							short
		 int32_t	uint32_t (0 to 4,294,967,295)					int
		 int64_t	uint64_t (0 to 18,446,744,073,709, 551,615)		long
*/


#include <stdio.h>
#include <stdint.h>
#include <math.h>

#include "inc/hw_uart.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "driverlib/systick.h"

//#include "Encoder/ams_position_encoder.h"
#include "Encoder/cui_position_encoder.h"
#include "Encoder/cui_load_encoder.h"
#include "ForceSensor/current_force_sensor.h"
#include "MainBus/can_bus.h"
#include "MotorDriver/motor_driver_5015a.h"
#include "Params/params.h"
#include "PID/pid.h"

#define DEBUG
#define MAX_ENCODER_COUNT 16384 //0-16383
#define F_CPU 80000000

volatile bool is_position_homing_done;
volatile bool is_load_homing_done;
volatile uint64_t TIME_MICROS;

#endif /* COMMONDEF_H_ */
