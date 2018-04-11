#include "PID.h"
#include <iostream>
/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	Kp_ = Kp;
	Ki_ = Ki;
	Kd_ = Kd;

	p_error_ = 0.0;
	i_error_ = 0.0;
	d_error_ = 0.0;
	prev_error_ = 0.0;

	frame_counter_ = 0;
}

void PID::UpdateError(double cte) {

	p_error_ = cte;
	frame_counter_++;
	abs_error_ += cte;
	i_error_ = abs_error_ / frame_counter_;
	d_error_ = cte - prev_error_;
	prev_error_ = cte;
}

double PID::TotalError() {

	return - Kp_ * p_error_ - Ki_ * i_error_ - Kd_ * d_error_;
}

