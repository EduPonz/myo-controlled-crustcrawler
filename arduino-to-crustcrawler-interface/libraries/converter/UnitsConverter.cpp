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

int UnitsConverter::torque_to_unit(float torque) {
	return(1); //To Do
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
