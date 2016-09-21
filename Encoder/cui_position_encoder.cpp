/*
 * cui_position_encoder.cpp
 *
 *  Created on: Aug 26, 2016
 *      Author: Kaustubh
 */

#include "cui_position_encoder.h"

void positionIndexInterrupt();

CUIPositionEncoder::CUIPositionEncoder() {

	//Enable QEI Peripherals
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI0);

	//Enable GPIOD7 - it's used for NMI
	HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY; //In Tiva include this is the same as "_DD" in older versions (0x4C4F434B)
	HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= 0x80;
	HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0;

	//Set Pins to be PHA0 PHB
	GPIOPinConfigure(GPIO_PD6_PHA0);
	GPIOPinConfigure(GPIO_PD7_PHB0);
	//Set GPIO pins for QEI. PhA0 -> PD6, PhB0 ->PD7.
	GPIOPinTypeQEI(GPIO_PORTD_BASE, GPIO_PIN_6 |  GPIO_PIN_7);

	//Enable pin PF4 for QEI0 IDX0
	GPIOPinConfigure(GPIO_PF4_IDX0);
	GPIOPinTypeQEI(GPIO_PORTF_BASE, GPIO_PIN_4);


	//Disable QEI and Int before configuration
	QEIDisable(QEI0_BASE);
	QEIIntDisable(QEI0_BASE,QEI_INTERROR | QEI_INTDIR | QEI_INTTIMER | QEI_INTINDEX);


	// Configure the quadrature encoder to capture edges on both signals and
	// maintain an absolute position by resetting on index pulses. Using a
	// 4096 line encoder at four edges per line, there are 16,384 pulses per
	// revolution; therefore set the maximum position to 16,383 as the count
	// is zero based.

	QEIConfigure(QEI0_BASE, (QEI_CONFIG_CAPTURE_A_B  | QEI_CONFIG_RESET_IDX 	| QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_SWAP), 16383);

	//Enable the QEI again
	QEIEnable(QEI0_BASE);

//	QEIVelocityConfigure(QEI0_BASE, QEI_VELDIV_1, 2);
//	QEIVelocityEnable(QEI0_BASE);

	QEIIntEnable(QEI0_BASE, QEI_INTINDEX);
	QEIIntRegister(QEI0_BASE, positionIndexInterrupt);


}

CUIPositionEncoder::~CUIPositionEncoder() {
}

//void CUIPositionEncoder::setIndexHandler(void (*handler)()){
//	QEIIntEnable(QEI0_BASE, QEI_INTINDEX);
//	QEIIntRegister(QEI0_BASE, indexInterrupt);
//}

uint16_t CUIPositionEncoder::getIndex() {
	uint16_t val = GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_4) >> 1;
	return val;
}

uint32_t CUIPositionEncoder::getPosition() {
	uint32_t position;
	position = QEIPositionGet(QEI0_BASE);
	return position;
}

uint32_t CUIPositionEncoder::getVelocity() {
	uint32_t velocity;
	velocity = QEIVelocityGet(QEI0_BASE);
	return velocity;
}


void CUIPositionEncoder::setPosition(uint32_t position){
	QEIPositionSet(QEI0_BASE, position);
}


void positionIndexInterrupt() {
	is_position_homing_done = true;
	QEIIntClear(QEI0_BASE, QEI_INTINDEX);
	QEIIntDisable(QEI0_BASE, QEI_INTINDEX);
}

