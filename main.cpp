/*
 * main.c
 */
#include "commondef.h"

void InitConsole(void) // Debug logging
{
	// Enable GPIO port A which is used for UART0 pins.
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	// Configure the pin muxing for UART0 functions on port A0 and A1.
	GPIOPinConfigure(GPIO_PA0_U0RX);
	GPIOPinConfigure(GPIO_PA1_U0TX);
	// Enable UART0 so that we can configure the clock.
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
	// Use the internal 16MHz oscillator as the UART clock source.
	UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
	// Select the alternate (UART) function for these pins.
	GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	// Initialize the UART for console I/O.
	UARTStdioConfig(0, 115200, 16000000);
}

volatile bool g_bErrFlag = 0;
volatile uint32_t g_ui32MsgCount = 0;
void canHandler(void) {
	uint32_t ui32Status;
	// Read the CAN interrupt status to find the cause of the interrupt
	ui32Status = CANIntStatus(CAN0_BASE, CAN_INT_STS_CAUSE);
	// If the cause is a controller status interrupt, then get the status
	if(ui32Status == CAN_INT_INTID_STATUS)
	{
		// Read the controller status.  This will return a field of status
		// error bits that can indicate various errors.  Error processing
		// is not done in this example for simplicity.  Refer to the
		// API documentation for details about the error status bits.
		// The act of reading this status will clear the interrupt.  If the
		// CAN peripheral is not connected to a CAN bus with other CAN devices
		// present, then errors will occur and will be indicated in the
		// controller status.
		ui32Status = CANStatusGet(CAN0_BASE, CAN_STS_CONTROL);

		// Set a flag to indicate some errors may have occurred.
		g_bErrFlag = 1;
	}
	// Check if the cause is message object 1, which what we are using for
	// sending messages.
	else if(ui32Status == 1)
	{
		// Getting to this point means that the TX interrupt occurred on
		// message object 1, and the message TX is complete.  Clear the
		// message object interrupt.
		CANIntClear(CAN0_BASE, 1);
		// Increment a counter to keep track of how many messages have been
		// sent.  In a real application this could be used to set flags to
		// indicate when a message is sent.
		g_ui32MsgCount++;

		// Since the message was sent, clear any error flags.
		g_bErrFlag = 0;
	}
	// Otherwise, something unexpected caused the interrupt.  This should
	// never happen.
	else
	{
		// Spurious interrupt handling can go here.
	}
}
void
SimpleDelay(void)
{
	//
	// Delay cycles for 1 second
	//
	SysCtlDelay(F_CPU/ 3); //50ms
}

void handler()
{
	is_position_homing_done = true;
	printf("\nboom\n");
}

void SysTickInt(void) {
	// called every 1 us
	TIME_MICROS ++;
}

void startTimer() {
	TIME_MICROS = 0;
	SysTickPeriodSet(F_CPU/1000000);// for microseconds, F_CPU/1000 for millis
	SysTickIntRegister(SysTickInt);
	SysTickIntEnable();
	SysTickEnable();
}
#define DATA_COUNT 3000
uint16_t positions[DATA_COUNT];
uint16_t deltaTimes[DATA_COUNT];



int main(void)
{


	SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ); //80 Mhz clock cycle

#ifdef DEBUG	// Set up the serial console to use for displaying messages
	InitConsole();
#endif

	startTimer();

#ifdef DEBUG
//	printf("Current,Target,Speed,Time\n");
	printf("Current\n");
#endif

	/*** Init classes, variables ***/
	is_position_homing_done = false; //false if not done, true if done
//	AMSPositionEncoder cAMSPositionEncoder;
	CUIPositionEncoder cCUIPositionEncoder;
	CUILoadEncoder cCUILoadEncoder;
	CurrentForceSensor cCurrentForceSensor;
	Params cParams;
	//	PID cPID(1, 100, -100, 0.01, 0.0085, 0.000003);//0.0031
//	PID cPID(50000, 100, -100, 0.014, 0.045, 0);//0.007,d=2.9 @50
//	PID cPID(50000, 100, -100, 0.015, 0.058, 0.00003);//0.007,d=2.9 @50
	double DEFAULT_KP = 0.006;
	PID cPID(50, 100, -100, 0.006, 0, 0);//0.007,d=2.9 @50

	MotorDriver5015a cMotorDriver5015a;
	uint16_t current_position=0;
	uint16_t previous_position=0;
	uint16_t target_position;
	float speed;
	uint64_t prevTime = TIME_MICROS; // clock cycles
//	uint64_t currTime;
	uint64_t count = 0;
	bool first_time = true;
	cParams.setTargetPos(12000);
	int zeroCurrent=0;
	int target = 12000;
	float speed_val = 50.0;
	int dir = 1;// Direction;
	uint8_t cooloff = 0;
	bool training_mode = true;

	cCurrentForceSensor.getFilteredCurrent();

//	int32_t total=0;
//	int32_t readings[4]={0};
//	int32_t readIndex=0;
//	int8_t numReadings=4;
//
//	int32_t load_value;
	int no_movement_count = 0;
	int load_direction=-1;
	while(1) {
		if (is_position_homing_done) {
			if(first_time) {

				first_time = false;

//				cMotorDriver5015a.setSpeed(0);
				cMotorDriver5015a.emergencyBrake();

				//Current Sensor Zero Reading
				for(int i = 0;i < 32;i++){
					zeroCurrent+=cCurrentForceSensor.getFilteredCurrent();
				}

				zeroCurrent = zeroCurrent/32;
//				printf("ZC=%d\n",zeroCurrent);
				cMotorDriver5015a.brakeRelease();
//				cCUILoadEncoder.setPosition(0);// Setting current position as home
				// TODO: handle the case where the load may be in a non zero position initially(Motor is experiencing a default load)
				prevTime = TIME_MICROS;
//				while(1){};
			}
			cMotorDriver5015a.setSpeed(speed_val);

			// Read the current position from the encoder and udpate the params class
			previous_position = current_position;
			current_position = cCUIPositionEncoder.getPosition();
			int posn_diff = current_position - previous_position;
			if(posn_diff>0){
				load_direction = 1;
			} else if (posn_diff <0) {
				load_direction = -1;
			}
//			if (posn_diff < 50 && posn_diff > -50) {
//				no_movement_count++;
//				if(no_movement_count > 100) {
//					printf("---------");
//					cPID.setKp(50*DEFAULT_KP);
//				}
//			} else {
//				no_movement_count = 0;
//				cPID.setKp(DEFAULT_KP);
//			}
			cParams.setCurrentPos(current_position);
			int current_val = cCurrentForceSensor.getFilteredCurrent();

			// Load Encoder window avg Measurements
//			uint16_t load_encoder_reading = cCUILoadEncoder.getPosition();
//			float expected_load = 0.56 * sin(current_position * 2 * 3.1415/16383.0);
//
//			load_value = load_encoder_reading - current_position;
//
//			total = total - readings[readIndex];
//			// read from the sensor:
//			readings[readIndex] = load_value;
//			// add the reading to the total:
//			total = total + readings[readIndex];
//			// advance to the next position in the array:
//			readIndex++;
//			// if we're at the end of the array...
//			if (readIndex >= numReadings) {
//				// ...wrap around to the beginning:
//				readIndex = 0;
//			}
//
//			load_value = total >> 2;


//			// Current Sensor Thresholding
//			uint32_t current = cCurrentForceSensor.getFilteredCurrent();
//			if(cooloff > 0)
//				cooloff--;
//
//			if(current > 1045 && cooloff == 0) {
//				dir*=-1;
//				cooloff = 5;
//				printf("CHDR\n");
//			}
//
//			if (dir < 0)
//				cMotorDriver5015a.setDirection(MotorDriver5015a::CLOCKWISE);
//			else
//				cMotorDriver5015a.setDirection(MotorDriver5015a::ANTICLOCKWISE);
//			// Current Sensor Thresholding end

#ifdef DEBUG
			int curr_time = TIME_MICROS;
			int diff = curr_time - prevTime;
//			if(count==0) {
				positions[count] = current_position;
//			} else {
//				positions[count] = (positions[count-1] + current_position)/2;
//			}

			deltaTimes[count] = diff;
//			printf("%d, %d, %lu\n", current_position, diff,vel);
			prevTime = curr_time;
			count++;
//			if(count == 1700) {
//				speed_val = 0;
//			}
			if(count> DATA_COUNT - 1) {
				for(int i = 0;i < DATA_COUNT; i++) {
					printf("%d,%d\n",positions[i],deltaTimes[i]);
				}
				while(1){}
			}
//			printf("%d,%d,%d,%d\n", current_position, current_val, current_val-zeroCurrent, current_position - previous_position);
//			int printLoad = (int)(expected_load * 10000);
//			printf("%d,%d\n", printLoad, load_value);

#endif
			//	 Read the target position from the Params class
			target_position = cParams.getTargetPos();
//			if(count%30 == 0) {
//				cParams.setTargetPos(target);
//				target+=(dir)*2000;
//				if(target>16380) {
//					target = 0;
//				} else if(target<0) {
//					target = 16383;
//				}
//			}
//#ifdef DEBUG
//			printf("%d,", target_position);
//#endif

			/*** Call PID class main function and get PWM speed as the output ***/
			speed = cPID.calculate(target_position, current_position);
//#ifdef DEBUG
//			printf("%d\n", (int)speed);
//#endif
			if (speed <= 100 && speed >= -100) { // Speed can be greater than +/-100 if PID fails due to sample time issues
//				count++;
//				float spd = fabsf(speed);
//				cMotorDriver5015a.setSpeed(spd);
//
//				if (speed < 0)
//					cMotorDriver5015a.setDirection(MotorDriver5015a::CLOCKWISE);
//				else
//					cMotorDriver5015a.setDirection(MotorDriver5015a::ANTICLOCKWISE);

//				currTime = TIME_MICROS;
//#ifdef DEBUG
//			    printf("%llu\n", currTime - prevTime);
//#endif
//				prevTime = currTime;
			}

			// Training Code(Current)
//			if (training_mode) {
//				int THRES = 10;
//				int DELTA = 50;
//				int load = current_val;
//				int absLoad = load>0? load: -load;
//
//				if(absLoad > THRES) {
//					if(load_direction == -1) {
//						target += DELTA;
//					} else if (load_direction == 1) {
//						target -= DELTA;
//					}
//					if(target > 16383) {
//						target -= 16383;
//					} else if (target < 0) {
//						target += 16383;
//					}
//					cParams.setTargetPos(target);
//				}
//			}


			// Training Code(Spring)
//			if (training_mode) {
//				int THRES = 65;
//				int DELTA = 50;
//				int load = load_value;
//				int absLoad = load>0? load: -load;
//
//				if(absLoad > THRES) {
//					if(load > 0) {
//						target += DELTA;
//					} else {
//						target -= DELTA;
//					}
//					if(target > 16383) {
//						target -= 16383;
//					} else if (target < 0) {
//						target += 16383;
//					}
//					cParams.setTargetPos(target);
//				}
//			}


		} else {
			cMotorDriver5015a.setDirection(MotorDriver5015a::CLOCKWISE);
			cMotorDriver5015a.setSpeed(5.0);
		}
	}

	return 0;
}

