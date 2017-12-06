#include "Arduino.h"
#include <math.h>
#include "DynamicsCalculator.h"

DynamicsCalculator::DynamicsCalculator(){

}

void DynamicsCalculator::set_thetas(float theta_1, float theta_2, float theta_3){
	this->_theta[0] = this->_position_degrees_to_radians(theta_1);
	this->_theta[1] = this->_position_degrees_to_radians(theta_2);
	this->_theta[2] = this->_position_degrees_to_radians(theta_3);
}

void DynamicsCalculator::set_omegas(float omega_1, float omega_2, float omega_3){
	this->_omega[0] = this->_speed_degrees_to_radians(omega_1);
	this->_omega[1] = this->_speed_degrees_to_radians(omega_2);
	this->_omega[2] = this->_speed_degrees_to_radians(omega_3);
}

void DynamicsCalculator::set_alphas(float alpha_1, float alpha_2, float alpha_3){
	this->_alpha[0] = this->_acceleration_degrees_to_radians(alpha_1);
	this->_alpha[1] = this->_acceleration_degrees_to_radians(alpha_2);
	this->_alpha[2] = this->_acceleration_degrees_to_radians(alpha_3);
}

float* DynamicsCalculator::get_torque(){
	return (float*)this->_M;
}

float DynamicsCalculator::_position_degrees_to_radians(float theta){
	float theta_rad = theta * this->_PI / 180;
	return theta_rad;
}
float DynamicsCalculator::_speed_degrees_to_radians(float omega){
	float omega_rad = omega / 57.29578;
	return omega_rad;
}
float DynamicsCalculator::_acceleration_degrees_to_radians(float alpha){
	float alpha_rad = alpha * 0.0174533;
	return alpha_rad;
}
void DynamicsCalculator::_calculate_M(){
	//////////////////////////// FIRST ROW ////////////////////////////
	this->_M[0] = (this->_I_2[0] + this->_I_3[0] + this->_I_2[1] + this->_I_3[1]  
				+ 2*this->_I_1[2] - this->_I_2[0]*cos(2*this->_theta[1]) 
				+ this->_I_2[1]*cos(2*this->_theta[1]) 
				+ sq(this->_L[1])*this->_m[2] + sq(this->_c[1])*this->_m[1] + sq(this->_c[2])*this->_m[2] 
				- this->_I_3[0]*cos(2*this->_theta[1] + 2*this->_theta[2]) 
				+ this->_I_3[1] *cos(2*this->_theta[1] + 2*this->_theta[2]) 
				+ sq(this->_L[1])*this->_m[2]*cos(2*this->_theta[1]) 
				+ sq(this->_c[1])*this->_m[1]*cos(2*this->_theta[1]) 
				+ sq(this->_c[2])*this->_m[2]*cos(2*this->_theta[1] + 2*this->_theta[2]) 
				+ 2*this->_L[1]*this->_c[2]*this->_m[2]*cos(2*this->_theta[1] + this->_theta[2]) 
				+ 2*this->_L[1]*this->_c[2]*this->_m[2]*cos(this->_theta[2]))/2;
	this->_M[1] = 0;
	this->_M[2] = 0;

	//////////////////////////// SECOND ROW ////////////////////////////
	this->_M[3] = 0;
	this->_M[4] = this->_I_2[2] + this->_I_3[2] + sq(this->_L[1])*this->_m[2] 
				+ sq(this->_c[1])*this->_m[1] + sq(this->_c[2])*this->_m[2]
				+ 2*this->_L[1]*this->_c[2]*this->_m[2]*cos(this->_theta[2]);
	this->_M[5] = this->_I_3[2] + sq(this->_c[2])*this->_m[2] 
				+ this->_L[1]*this->_c[2]*this->_m[2]*cos(this->_theta[2]);

	//////////////////////////// THIRD ROW ////////////////////////////
	this->_M[6] = 0;
	this->_M[7] = this->_M[5];
	this->_M[8] = this->_I_3[2] + sq(this->_c[2])*this->_m[2];
}
void DynamicsCalculator::_calculate_C(){

}
void DynamicsCalculator::_calculate_G(){

}