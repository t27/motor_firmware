/*
 * pid.cpp
 *
 *  Created on: Jul 31, 2016
 *      Author: Kaustubh
 */

#ifndef _PID_SOURCE_
#define _PID_SOURCE_


#include "pid.h"

PID::PID( double period, double max, double min, double Kp, double Kd, double Ki ) :
max_(max),
min_(min),
Kp_(Kp),
Kd_(Kd),
Ki_(Ki),
period_(period),
pre_error_(0),
integral_(0)
{
	dt_ = 1;
	prev_time_ = TIME_MICROS;
}

double PID::calculate( double setpoint, double pv )
{

	uint64_t curr_time = TIME_MICROS;
	uint64_t deltaT = curr_time - prev_time_;

	if (deltaT > period_) {
//	if (true) {
		// Calculate error
		double error = setpoint - pv;

		// Handling encoder rollover for 360 degree encoder
		if(error > MAX_ENCODER_COUNT/2)
			error = error - MAX_ENCODER_COUNT;
		else if(error < -(MAX_ENCODER_COUNT/2))
			error = error + MAX_ENCODER_COUNT;

		// Error tolerance
		if (error < 2 && error > -2) {
			error = 0;
		}
		// Proportional term
		double Pout = Kp_ * error;

		// Integral term
		integral_ += error * dt_;
		double Iout = Ki_ * integral_;
		if( Iout > max_ )
			Iout = max_;
		else if( Iout < min_ )
			Iout = min_;

		// Derivative term
		double derivative = (error - pre_error_) / dt_;
//		if (derivative > 4000 || derivative < -4000) {
//			derivative = 0;
//		}
		double Dout = Kd_ * derivative;

		// Calculate total output
		double output = Pout + Iout + Dout;
		//    double output = Pout;
		//    printf("output=%lf",output);
		//     Restrict to max/min
		if( output > max_ )
			output = max_;
		else if( output < min_ )
			output = min_;

		// Save error to previous error
		pre_error_ = error;
		prev_time_ = curr_time;
		return output;
	} else {
		return -1000.0;
	}

}

PID::~PID()
{
}

double PID::getKd() const {
	return Kd_;
}

void PID::setKd(double kd) {
	Kd_ = kd;
}

double PID::getKi() const {
	return Ki_;
}

void PID::setKi(double ki) {
	Ki_ = ki;
}

double PID::getKp() const {
	return Kp_;
}

void PID::setKp(double kp) {
	Kp_ = kp;
}

double PID::getMax() const {
	return max_;
}

void PID::setMax(double max) {
	max_ = max;
}

double PID::getMin() const {
	return min_;
}

void PID::setMin(double min) {
	min_ = min;
}

#endif


