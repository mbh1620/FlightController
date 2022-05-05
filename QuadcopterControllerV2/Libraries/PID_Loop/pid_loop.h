#ifndef PID_LOOP_H
#define PID_LOOP_H

#include <Arduino.h>

class PID_LOOP {

public:
	PID_LOOP(float p_value, float i_value, float d_value);
	float compute_loop(float desired_value, float actual_value);
private:
	
	float current_error;
	float previous_error;

	float desired_value;
	float actual_value;

	float _p_value;
	float _i_value;
	float _d_value;

	float proportional;
	float integral;
	float differential;	

	float pid_output;

};
#endif