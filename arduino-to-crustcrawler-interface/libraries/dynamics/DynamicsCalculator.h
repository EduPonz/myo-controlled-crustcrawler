/*
Name:		DynamicsCalculator.h
Created:	12/5/2017 3:25:09 PM
Author:		Steffan Svendsen, Vincent Joly, Simone Jensen, David Michalik, Eduardo Ponz Segrelles, Ivelin Krasimirov Penchev, Jesper Bro Kirstein Rosenberg
*/


#ifndef DynamicsCalculator_h
#define DynamicsCalculator_h

#include "Arduino.h"
#include <math.h>

class DynamicsCalculator{
public:
	DynamicsCalculator();
	void set_thetas(float theta_1, float theta_2, float theta_3);
	void set_omegas(float omega_1, float omega_2, float omega_3);
	void set_alphas(float alpha_1, float alpha_2, float alpha_3);
	float* get_thetas();
	float* get_omegas();
	float* get_alphas();
	float* get_M();
	float* get_C();
	float* get_G();
	void get_torque(float* torque);

private:
	const float _PI = 3.1415927;
	const float _g = -9.82;
	const float _c [3] = {0.0254,0.1099,0.0939};
	const float _m [3] = {0.15,0.23,0.31};
	const float _L [3] = {0.0508,0.2197,0.1877};
	const float _I_1 [3] = {0.0000323,0.0000323,0.00003};
	const float _I_2 [3] = {0.000046,0.000925,0.000925};
	const float _I_3 [3] = {0.000062,0.000910,0.000910};
	float _theta [3];
	float _omega [3];
	float _alpha [3];
	float _M [3];
	float _C [3];
	float _G [3];

	void _calculate_M();
	void _calculate_C();
	void _calculate_G();
};

#endif
