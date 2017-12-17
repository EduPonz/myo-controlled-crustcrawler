/*
 Name:		UnitsConverter.h
 Created:	12/1/2017 11:42:29 AM
 Author:	Steffan Svendsen, Vincent Joly, Simone Jensen, David Michalik, Eduardo Ponz Segrelles, Ivelin Krasimirov Penchev, Jesper Bro Kirstein Rosenberg
 Editor:	http://www.visualmicro.com
*/

#ifndef UnitsConverter_h
#define UnitsConverter_h

#include "Arduino.h"

class UnitsConverter
{
public:
	int degree_to_unit(float degree);
	int torque_to_PWM_unit(int servo_id, float vel, float torque);
	int acceleration_degree_to_unit(float acc_degree);
	float unit_to_degree(int unit);
	float unit_to_radians(int unit);
	float position_degrees_to_radians(float theta);
	float speed_degrees_to_radians(float omega);
	float acceleration_degrees_to_radians(float alpha);
	float unit_to_speed_degree(int unit);
	float speed_degree_to_unit(float speed_degree);

private:

	const float _PI = 3.1415927;
	const float _UNIT_TO_DEGREE = 0.088;
	const float _UNIT_TO_RAD = _UNIT_TO_DEGREE * _PI / 180;
	const float _DEGREE_TO_RAD = _PI / 180;
	const float _SPEED_RAD_TO_DEGREE = 57.29578;
	const float _ACC_DEGREE_TO_RAD = 0.0174533;
	const float _UNIT_TO_SPEED_DEGREE = 1.374;
	const float _UNIT_TO_ACC_DEGREE = 21.4577;
};

#endif

