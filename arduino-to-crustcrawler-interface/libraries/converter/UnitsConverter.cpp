/*
 Name:		UnitsConverter.cpp
 Created:	12/1/2017 11:42:29 AM
 Author:	Steffan Svendsen, Vincent Joly, Simone Jensen, David Michalik, Eduardo Ponz Segrelles, Ivelin Krasimirov Penchev
 Editor:	http://www.visualmicro.com
*/

#include "Arduino.h"
#include "UnitsConverter.h"
#include "DynamixelPro2.h"

int UnitsConverter::degree_to_unit(float degree) {
	return(degree / this->_UNIT_TO_DEGREE);
}

int UnitsConverter::torque_to_PWM_unit(int servo_id, float torque) {	// This function needs to be looked at to see if the logic is correct.

	float B, k, R;

	float vel = unit_to_degree(dynamixel.read_current_velocity(servo_id));

	if (servo_id == 0) {		// For MX-64 servo.		Check that the servo_id correspond with the correct servo.
		B = 2.3 / 33;
		k = 2.4 / 2.1;
		R = 18.2;
	}else {						// For MX 106 servo.
		B = 2.6 / 18; 
		k = 3.3 / 2.1;
		R = 29.3;
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
