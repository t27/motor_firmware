/*
 * cui_load_encoder.cpp
 *
 *  Created on: 09-Sep-2016
 *      Author: Tarang
 */

#include "cui_load_encoder.h"

void loadIndexInterrupt();

CUILoadEncoder :: CUILoadEncoder() {

	//Enable QEI Peripherals
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI1);

//	//Enable GPIOD7 - it's used for NMI
//	HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY; //In Tiva include this is the same as "_DD" in older versions (0x4C4F434B)
//	HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= 0x80;
//	HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0;

	//Set Pins to be PHA1 PHB1
	GPIOPinConfigure(GPIO_PC5_PHA1);
	GPIOPinConfigure(GPIO_PC6_PHB1);

	//Enable pin PC4 for QEI1 IDX1
	GPIOPinConfigure(GPIO_PC4_IDX1);

	//Set GPIO pins for QEI. PhA1 -> PC5, PhB1 ->PC6, IDX1 -> PC4
	GPIOPinTypeQEI(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5 |  GPIO_PIN_6);



	//Disable QEI and Int before configuration
	QEIDisable(QEI1_BASE);
	QEIIntDisable(QEI1_BASE,QEI_INTERROR | QEI_INTDIR | QEI_INTTIMER | QEI_INTINDEX);


	// Configure the quadrature encoder to capture edges on both signals and
	// maintain an absolute position by resetting on index pulses. Using a
	// 4096 line encoder at four edges per line, there are 16,384 pulses per
	// revolution; therefore set the maximum position to 16,383 as the count
	// is zero based.

	QEIConfigure(QEI1_BASE, (QEI_CONFIG_CAPTURE_A_B  | QEI_CONFIG_RESET_IDX 	| QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_SWAP), 16383);

	//Enable the QEI again
	QEIEnable(QEI1_BASE);
	QEIIntEnable(QEI1_BASE, QEI_INTINDEX);
	QEIIntRegister(QEI1_BASE, loadIndexInterrupt);
}

CUILoadEncoder::~CUILoadEncoder() {
}


uint32_t CUILoadEncoder::getPosition(){
	uint32_t position;
	position = QEIPositionGet(QEI1_BASE);
	return position;
}


void CUILoadEncoder::setPosition(uint32_t position){
	QEIPositionSet(QEI1_BASE, position);
}


void loadIndexInterrupt(){
	is_load_homing_done = true;
	QEIIntClear(QEI1_BASE, QEI_INTINDEX);
	QEIIntDisable(QEI1_BASE, QEI_INTINDEX);
}
