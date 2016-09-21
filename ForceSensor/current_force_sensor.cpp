/*
 * CurrentForceSensor.cpp
 *
 *  Created on: 25-Jul-2016
 *      Author: Tarang
 */

#include "current_force_sensor.h"

CurrentForceSensor::CurrentForceSensor() {
	// TODO Auto-generated constructor stub

	// Initialise ADC for reading current sensor value
	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
	//
	// The actual port and pins used may be different on your part, consult
	// the data sheet for more information. GPIO port E needs to be enabled
	// so these pins can be used.
	//
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

	//
	// Select the analog ADC function for these pins.
	// Consult the data sheet to see which functions are allocated per pin.
	// TODO: change this to select the port/pin you are using.
	//
	GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);

	ADCSequenceDisable(ADC0_BASE, sequenceNum);

//	ADCClockConfigSet(ADC0_BASE, ADC_CLOCK_SRC_PLL | ADC_CLOCK_RATE_HALF, 1);

	// Enable the first sample sequencer to capture the value of channel 0 when
	// the processor trigger occurs.
	ADCSequenceConfigure(ADC0_BASE, sequenceNum, ADC_TRIGGER_PROCESSOR, 0);
//	ADCSoftwareOversampleConfigure(ADC0_BASE, 3, 8);

//	ADCSoftwareOversampleConfigure(ADC0_BASE, 0, 2);
//	ADCSoftwareOversampleStepConfigure(ADC0_BASE, 0, 0, (ADC_CTL_CH0 | ADC_CTL_IE | ADC_CTL_END));

	ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_CH1);
	ADCSequenceStepConfigure(ADC0_BASE, 0, 1, ADC_CTL_CH1);
	ADCSequenceStepConfigure(ADC0_BASE, 0, 2, ADC_CTL_CH1);
	ADCSequenceStepConfigure(ADC0_BASE, 0, 3, ADC_CTL_CH1);
	ADCSequenceStepConfigure(ADC0_BASE, 0, 4, ADC_CTL_CH1);
	ADCSequenceStepConfigure(ADC0_BASE, 0, 5, ADC_CTL_CH1);
	ADCSequenceStepConfigure(ADC0_BASE, 0, 6, ADC_CTL_CH1);
	ADCSequenceStepConfigure(ADC0_BASE, sequenceNum, 7, ADC_CTL_CH1 | ADC_CTL_IE |
		                             ADC_CTL_END);
	ADCHardwareOversampleConfigure(ADC0_BASE, 32);
	//
	// Since sample sequence 3 is now configured, it must be enabled.
	//
	ADCSequenceEnable(ADC0_BASE, sequenceNum);

	//
	// Clear the interrupt status flag.  This is done to make sure the
	// interrupt flag is cleared before we sample.
	//
	ADCIntClear(ADC0_BASE, sequenceNum);

	total = 0;
	readIndex = 0;
	numReadings = 8;
	for(int i=0; i<numReadings;i++){
		readings[i]=0;
	}
}

CurrentForceSensor::~CurrentForceSensor() {
	// TODO Auto-generated destructor stub
}

uint32_t CurrentForceSensor::getCurrent() {
	uint32_t current[8];
	// Trigger the sample sequence.
	//
	// Trigger the ADC conversion.
	//
	ADCProcessorTrigger(ADC0_BASE, sequenceNum);

	//
	// Wait for conversion to be completed.
	//
	while(!ADCIntStatus(ADC0_BASE, sequenceNum, false))
	{
	}

	//
	// Clear the ADC interrupt flag.
	//
	ADCIntClear(ADC0_BASE, sequenceNum);

	//
	// Read ADC Value.
	//
	ADCSequenceDataGet(ADC0_BASE, sequenceNum, current);

//	ADCSoftwareOversampleDataGet(ADC0_BASE, 0, &current, 1);
//	current = (4096 - current)/2048 * 40;

//	return 4096 - current;
	uint32_t currentSum=0;
	for(int i = 0; i < 8; i++){
		currentSum += current[i];
	}
	return 4096 - (currentSum>>3); // Divide by 8
}
uint32_t CurrentForceSensor::getCurrentAvg() {
	uint32_t sum;
	for(int i = 0; i < 8; i++) {
		sum += getCurrent();
	}

	return sum >> 3; // (Sum / 8)
}

uint32_t CurrentForceSensor::getCurrentMovingAvg() {

	// subtract the last reading:
	total = total - readings[readIndex];
	// read from the sensor:
	readings[readIndex] = getCurrent();
	// add the reading to the total:
	total = total + readings[readIndex];
	// advance to the next position in the array:
	readIndex++;
	// if we're at the end of the array...
	if (readIndex >= numReadings) {
		// ...wrap around to the beginning:
		readIndex = 0;
	}

	return total/numReadings;

}

uint32_t CurrentForceSensor::getFilteredCurrent() {
	if(total == 0) {
		//Ensure moving average has enough previous readings
		for(int i=0;i<numReadings;i++) {
			uint32_t temp = getCurrentMovingAvg();
		}
	}
	return getCurrentMovingAvg();
}
uint32_t CurrentForceSensor::getTorque() {
	// Read Current Value and convert to Torque
	uint32_t current = getCurrent();

	// convert to Torque representation
	uint32_t torque = current * 1;

	return torque;
}
