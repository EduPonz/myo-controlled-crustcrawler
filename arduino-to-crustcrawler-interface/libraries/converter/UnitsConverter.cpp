/*
 Name:		UnitsConverter.cpp
 Created:	12/1/2017 11:42:29 AM
 Author:	Steffan Svendsen, Vincent Joly, Simone Jensen, David Michalik, Eduardo Ponz Segrelles, Ivelin Krasimirov Penchev
 Editor:	http://www.visualmicro.com
*/

#include "Arduino.h"
#include "UnitsConverter.h"

int UnitsConverter::degree_to_unit(float degree) {
	return(degree / this->_UNIT_TO_DEGREE);
}

int UnitsConverter::torque_to_PWM_unit(int servo_id, float vel, float torque) {	

	float B, k, R;	

	if (servo_id == 1) {		// For MX-106 servo.
		B = 0.0137;
		k = 1.8031;
		R = 9.7281;
	}else {				// For MX 106 servo.
		B = 0.0109; 
		k = 1.2691;
		R = 8.3133;
	}

	int goalPWM = (((((torque + B*vel) / k)*R) + k*vel) / 12.5) * 885;

	return(goalPWM);
}

float UnitsConverter::unit_to_degree(int unit) {
	return(unit * this->_UNIT_TO_DEGREE);
}

float UnitsConverter::unit_to_radians(int unit) {
	return(unit * this->_UNIT_TO_RAD);
}

float UnitsConverter::position_degrees_to_radians(float theta) {
	return(theta * this->_DEGREE_TO_RAD);
}

float UnitsConverter::speed_degrees_to_radians(float omega) {
	return(omega / this->_SPEED_RAD_TO_DEGREE);
}

float UnitsConverter::acceleration_degrees_to_radians(float alpha) {
	return(alpha * this->_ACC_DEGREE_TO_RAD);
}

float UnitsConverter::unit_to_speed_degree(int unit){
  return(unit * _UNIT_TO_SPEED_DEGREE);
}





