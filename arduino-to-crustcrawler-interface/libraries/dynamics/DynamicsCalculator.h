#ifndef DynamicsCalculator_h
#define DynamicsCalculator_h

#include "Arduino.h"
#include <math.h>
#include "UnitsConverter.h"

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
		const float _c [3] = {1,1,1};
		const float _m [3] = {1,1,1};
		const float _L [3] = {1,1,1};
		const float _I_1 [3] = {1,2,3};
		const float _I_2 [3] = {1,2,3};
		const float _I_3 [3] = {1,2,3};
		UnitsConverter _converter;
		float _theta [3];
		float _omega [3];
		float _alpha [3];
		// float _M [3][3];
		float _M [3];
		float _C [3];
		float _G [3];

		void _calculate_M();
		void _calculate_C();
		void _calculate_G();
};

#endif