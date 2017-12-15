/*
 Name:		Path_Planning_Demo.ino
 Created:	11/30/2017 11:05:15 PM
 Author:	Steffan Svendsen, Jesper Bro Kirstein Rosenberg, Vincent Joly, Simone Jensen, David Michalik, Eduardo Ponz Segrelles, Ivelin Krasimirov Penchev 
*/

/*
The maximum acceleration (140deg/s^2) and the acceleration duration (0.25seconds), is predefined in the library.
*/

#include "PathPlanning.h"

PathPlanning path;

float angleInputServo1_0 = 90;        // This value changes with the movements of the 1st joint
float angleInputServo2_0 = 45;         // This value changes with the movements of the 2nd joint 
float angleInputServo3_0 = 45;        // This value changes with the movements of the 3rd joint

float sampleTime = 0.25;
String instruction = "home";


void print_example() {

	if (path.calculate_path(angleInputServo1_0, angleInputServo2_0, angleInputServo3_0, instruction)) {

		Serial.print("Total time = ");
		Serial.println(path.totalTime);

		Serial.print("Constant velocity time = ");
		Serial.println(path.constantVelocityTime);
		Serial.println("");

		for (int i = 0; i < 3; i++) {
			Serial.print("Servo.");
			Serial.print(i + 1);
			Serial.print(" acceleration = ");
			Serial.println(path.servo_acceleration[i]);

			Serial.print("Servo.");
			Serial.print(i + 1);
			Serial.print(" decelaration = ");
			Serial.println(path.servo_deceleration[i]);

			Serial.print("Servo.");
			Serial.print(i + 1);
			Serial.print(" acceleration segment end-position = ");
			Serial.println(path.servo_accelerationSegmentEnd[i]);

			Serial.print("Servo.");
			Serial.print(i + 1);
			Serial.print(" deceleration segment start-position = ");
			Serial.println(path.servo_decelerationSegmentStart[i]);

			Serial.println("");
		}
	}
	else {
		Serial.print("The start position is inside the user-space");
	}

}

void timing_example() {

	int startTime = micros();

	if (path.calculate_path(angleInputServo1_0, angleInputServo2_0, angleInputServo3_0, instruction)) {

		for (int i = 0; i < 3; i++) {
			path.get_position_sample(i, sampleTime);
			path.get_velocity_sample(i, sampleTime);
			path.get_acceleration_sample(i, sampleTime);
			path.get_goal_position(i);
		}
	}

	int endTime = micros();

	Serial.print("Execution time = ");
	Serial.print(endTime - startTime);
	Serial.println(" micro seconds");
}

void timing_calculate_path_example() {

  int startTime = micros();

  path.calculate_path(angleInputServo1_0, angleInputServo2_0, angleInputServo3_0, instruction);

  int endTime = micros();

  Serial.print("calculate_path() execution time = ");
  Serial.print(endTime - startTime);
  Serial.println(" micro seconds");
}

void timing_get_position_example() {

  int startTime = 0;
  int endTime = 0;
  path.calculate_path(angleInputServo1_0, angleInputServo2_0, angleInputServo3_0, instruction);

  startTime = micros();

  for(int i = 0; i < 3; i++){
  path.get_position_sample(i, sampleTime);
  }

  endTime = micros();

  Serial.print("get_position_sample() execution time = ");
  Serial.print(endTime - startTime);
  Serial.println(" micro seconds");
}

void timing_get_velocity_example() {

  int startTime = 0;
  int endTime = 0;
  path.calculate_path(angleInputServo1_0, angleInputServo2_0, angleInputServo3_0, instruction);

  startTime = micros();

  for(int i = 0; i < 3; i++){
  path.get_velocity_sample(i, sampleTime);
  }

  endTime = micros();

  Serial.print("get_velocity_sample() execution time = ");
  Serial.print(endTime - startTime);
  Serial.println(" micro seconds");
}

void timing_get_acceleration_example() {

  int startTime = 0;
  int endTime = 0;
  path.calculate_path(angleInputServo1_0, angleInputServo2_0, angleInputServo3_0, instruction);

  startTime = micros();

  for(int i = 0; i < 3; i++){
  path.get_acceleration_sample(i, sampleTime);
  }
  
  endTime = micros();

  Serial.print("get_acceleration_sample() execution time = ");
  Serial.print(endTime - startTime);
  Serial.println(" micro seconds");
}

void setup() {
	Serial.begin(9600);
	delay(500);

	Serial.println("");
	Serial.println("Ready");
	Serial.println("");
	delay(500);

	//print_example();
}

void loop() {

	timing_example();
  //timing_calculate_path_example();
  //timing_get_position_example();
  //timing_get_velocity_example();
  //timing_get_acceleration_example();
	delay(2000);
}
