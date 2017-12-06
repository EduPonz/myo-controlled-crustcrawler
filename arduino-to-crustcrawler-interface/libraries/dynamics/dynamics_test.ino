#include "DynamicsCalculator.h"
#include "MatrixMath.h"
DynamicsCalculator myCalculator;

void setup(){
	Serial.begin(9600);
	unsigned long time_1 = micros();
	myCalculator.set_thetas(30, 60, 45);
	myCalculator.set_omegas(30, 60, 45);
	myCalculator.set_alphas(30, 60, 45);
	float torque [3];
	myCalculator.get_torque((float*)torque);
	unsigned long time_2 = micros();
	Matrix.Print((float*)torque, 3, 1, "Torque");
	Serial.print("\ntime_1: ");
	Serial.print(time_1);
	Serial.print(" us \ntime_2: ");
	Serial.print(time_2);
	Serial.print(" us \ntime difference (us): ");
	int time_ms = (time_2 - time_1);
	Serial.println(time_ms);
}

void loop(){
	
}
