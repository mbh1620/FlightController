#include "pid_loop.h"

PID_LOOP::PID_LOOP(float p_value, float i_value, float d_value) {
	_p_value = p_value;
	_i_value = i_value;
	_d_value = d_value;

	current_error = 0;
	previous_error = 0;

	proportional = 0;
	integral = 0;
	differential = 0;

	pid_output = 0;
}

float PID_LOOP::compute_loop(float desired_value, float actual_value){

	current_error = desired_value - actual_value;
	
	proportional = current_error;
	integral += current_error;
	differential = current_error - previous_error;

	pid_output = (proportional*_p_value) + (integral*_i_value) + (differential*_d_value);

	previous_error = current_error;

	return pid_output;
}