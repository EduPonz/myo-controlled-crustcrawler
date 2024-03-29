/*
Name:		DynamicsCalculator.cpp
Created:	12/5/2017 3:25:09 PM
Author:		Steffan Svendsen, Vincent Joly, Simone Jensen, David Michalik, Eduardo Ponz Segrelles, Ivelin Krasimirov Penchev, Jesper Bro Kirstein Rosenberg
*/


#include "Arduino.h"
#include <math.h>
#include "DynamicsCalculator.h"
#include "MatrixMath.h"

DynamicsCalculator::DynamicsCalculator(){

}

void DynamicsCalculator::set_thetas(float theta_1, float theta_2, float theta_3){
	this->_theta [0] = theta_1;
	this->_theta [1] = theta_2;
	this->_theta [2] = theta_3;
}

void DynamicsCalculator::set_omegas(float omega_1, float omega_2, float omega_3){
	this->_omega [0] = omega_1;
	this->_omega [1] = omega_2;
	this->_omega [2] = omega_3;
}

void DynamicsCalculator::set_alphas(float alpha_1, float alpha_2, float alpha_3){
	this->_alpha [0] = alpha_1;
	this->_alpha [1] = alpha_2;
	this->_alpha [2] = alpha_3;
}

float* DynamicsCalculator::get_thetas(){
	return (float*)this->_theta;
}

float* DynamicsCalculator::get_omegas(){
	return (float*)this->_omega;
}

float* DynamicsCalculator::get_alphas(){
	return (float*)this->_alpha;
}

float* DynamicsCalculator::get_M(){
	return (float*)this->_M;
}

float* DynamicsCalculator::get_C(){
	return (float*)this->_C;
}

float* DynamicsCalculator::get_G(){
	return (float*)this->_G;
}

void DynamicsCalculator::get_torque(float* torque){
	this->_calculate_M();
	this->_calculate_C();
	this->_calculate_G();
	float aux_vector [3];
	Matrix.Subtract((float*)this->_M, (float*)this->_C, 3, 1, (float*)aux_vector);
	Matrix.Add((float*)aux_vector, (float*)this->_G, 3, 1, (float*)torque);
}

void DynamicsCalculator::_calculate_M(){
	this->_M [0] = (this->_alpha [0] * (this->_I_2 [0] + this->_I_3 [0] + this->_I_2 [1] + this->_I_3 [1] + 2 * this->_I_1 [2]
		- this->_I_2 [0] * cos(2 * this->_theta [1]) + this->_I_2 [1] * cos(2 * this->_theta [1]) + sq(this->_L [1])*this->_m [2]
		+ sq(this->_c [1])*this->_m [1] + sq(this->_c [2])*this->_m [2] - this->_I_3 [0] * cos(2 * this->_theta [1]
		+ 2 * this->_theta [2]) + this->_I_3 [1] * cos(2 * this->_theta [1] + 2 * this->_theta [2])
		+ sq(this->_L [1])*this->_m [2] * cos(2 * this->_theta [1]) + sq(this->_c [1])*this->_m [1] * cos(2 * this->_theta [1])
		+ sq(this->_c [2])*this->_m [2] * cos(2 * this->_theta [1] + 2 * this->_theta [2])
		+ 2 * this->_L [1] * this->_c [2] * this->_m [2] * cos(2 * this->_theta [1] + this->_theta [2])
		+ 2 * this->_L [1] * this->_c [2] * this->_m [2] * cos(this->_theta [2]))) / 2
		- (this->_omega [0] * (2 * this->_I_2 [1] * sin(2 * this->_theta [1])*this->_omega [1] - 2 * this->_I_2 [0] * sin(2 * this->_theta [1])*this->_omega [1]
		- this->_I_3 [0] * sin(2 * this->_theta [1] + 2 * this->_theta [2])*(2 * this->_omega [1] + 2 * this->_omega [2]) + this->_I_3 [1] * sin(2 * this->_theta [1]
		+ 2 * this->_theta [2])*(2 * this->_omega [1] + 2 * this->_omega [2]) + 2 * sq(this->_c [1])*this->_m [1] * sin(2 * this->_theta [1])*this->_omega [1]
		+ sq(this->_c [2])*this->_m [2] * sin(2 * this->_theta [1] + 2 * this->_theta [2])*(2 * this->_omega [1] + 2 * this->_omega [2])
		+ 2 * sq(this->_L [1])*this->_m [2] * sin(2 * this->_theta [1])*this->_omega [1]
		+ 2 * this->_L [1] * this->_c [2] * this->_m [2] * sin(this->_theta [2])*this->_omega [2]
		+ 2 * this->_L [1] * this->_c [2] * this->_m [2] * sin(2 * this->_theta [1] + this->_theta [2])*(2 * this->_omega [1] + this->_omega [2]))) / 2;

	this->_M [1] = this->_I_2 [2] * this->_alpha [1] + this->_I_3 [2] * this->_alpha [1]
		+ this->_I_3 [2] * this->_alpha [2] + sq(this->_L [1])*this->_m [2] * this->_alpha [1]
		+ sq(this->_c [1])*this->_m [1] * this->_alpha [1] + sq(this->_c [2])*this->_m [2] * this->_alpha [1] + sq(this->_c [2])*this->_m [2] * this->_alpha [2]
		+ 2 * this->_L [1] * this->_c [2] * this->_m [2] * cos(this->_theta [2])*this->_alpha [1]
		+ this->_L [1] * this->_c [2] * this->_m [2] * cos(this->_theta [2])*this->_alpha [2]
		- 2 * this->_L [1] * this->_c [2] * this->_m [2] * sin(this->_theta [2])*this->_omega [1] * this->_omega [2]
		- this->_L [1] * this->_c [2] * this->_m [2] * sin(this->_theta [2])*this->_omega [2] * this->_omega [2];

	this->_M [2] = this->_I_3 [2] * this->_alpha [1] + this->_I_3 [2] * this->_alpha [2] + sq(this->_c [2])*this->_m [2] * this->_alpha [1]
		+ sq(this->_c [2])*this->_m [2] * this->_alpha [2]
		+ this->_L [1] * this->_c [2] * this->_m [2] * cos(this->_theta [2])*this->_alpha [1]
		- this->_L [1] * this->_c [2] * this->_m [2] * sin(this->_theta [2])*this->_omega [1] * this->_omega [2];
}
void DynamicsCalculator::_calculate_C(){
	this->_C [0] = 0;

	this->_C [1] = -(sq(this->_omega [0])*(this->_m [2] * sin(2 * this->_theta [1])*sq(this->_L [1])
		+ 2 * this->_m [2] * sin(2 * this->_theta [1] + this->_theta [2])*this->_L [1] * this->_c [2]
		+ this->_m [1] * sin(2 * this->_theta [1])*sq(this->_c [1])
		+ this->_m [2] * sin(2 * this->_theta [1] + 2 * this->_theta [2])*sq(this->_c [2])
		- this->_I_3 [0] * sin(2 * this->_theta [1] + 2 * this->_theta [2])
		+ this->_I_3 [1] * sin(2 * this->_theta [1] + 2 * this->_theta [2])
		- this->_I_2 [0] * sin(2 * this->_theta [1]) + this->_I_2 [1] * sin(2 * this->_theta [1]))) / 2;

	this->_C [2] = (this->_I_3 [0] * sin(2 * this->_theta [1] + 2 * this->_theta [2])*sq(this->_omega [0])) / 2
		- (this->_I_3 [1] * sin(2 * this->_theta [1] + 2 * this->_theta [2])*sq(this->_omega [0])) / 2
		- (sq(this->_c [2])*this->_m [2] * sin(2 * this->_theta [1] + 2 * this->_theta [2])*sq(this->_omega [0])) / 2
		- (this->_L [1] * this->_c [2] * this->_m [2] * sin(this->_theta [2])*sq(this->_omega [0])) / 2
		- this->_L [1] * this->_c [2] * this->_m [2] * sin(this->_theta [2])*sq(this->_omega [1])
		- (this->_L [1] * this->_c [2] * this->_m [2] * sin(2 * this->_theta [1] + this->_theta [2])*sq(this->_omega [0])) / 2
		- this->_L [1] * this->_c [2] * this->_m [2] * sin(this->_theta [2])*this->_omega [1] * this->_omega [2];
}

void DynamicsCalculator::_calculate_G(){
	this->_G [0] = 0;
	this->_G [1] = -this->_g*this->_m [2] * (this->_c [2] * cos(this->_theta [1] + this->_theta [2])
		+ this->_L [1] * cos(this->_theta [1])) - this->_c [1] * this->_g*this->_m [1] * cos(this->_theta [1]);
	this->_G [2] = -this->_c [2] * this->_g*this->_m [2] * cos(this->_theta [1] + this->_theta [2]);
}
