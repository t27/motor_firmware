/*
 * pid.h
 *
 *  Created on: Jul 31, 2016
 *      Author: Kaustubh
 */

#ifndef PID_PID_H_
#define PID_PID_H_

#include "../commondef.h"

class PID
{
    public:
        // Kp -  proportional gain
        // Ki -  Integral gain
        // Kd -  derivative gain
        // dt -  loop interval time
        // max - maximum value of manipulated variable
        // min - minimum value of manipulated variable
        PID( double period, double max, double min, double Kp, double Kd, double Ki );

        // Returns the manipulated variable given a setpoint and current process value
        double calculate( double setpoint, double pv );
        ~PID();
		double getKd() const;
		void setKd(double kd);
		double getKi() const;
		void setKi(double ki);
		double getKp() const;
		void setKp(double kp);
		double getMax() const;
		void setMax(double max);
		double getMin() const;
		void setMin(double min);

    private:
        double dt_;
        double max_;
        double min_;
        double Kp_;
        double Kd_;
        double Ki_;
        double pre_error_;
        double integral_;
        double period_;
        uint64_t prev_time_;
};




#endif /* PID_PID_H_ */
