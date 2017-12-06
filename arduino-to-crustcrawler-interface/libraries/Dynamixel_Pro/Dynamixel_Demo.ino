/*
 Name:		Dynamixel_Demo.ino
 Created:	12/5/2017 3:25:09 PM
 Author:	Steffan Svendsen, Vincent Joly, Simone Jensen, David Michalik, Eduardo Ponz Segrelles, Ivelin Krasimirov Penchev 
*/

#include "DynamixelPro2.h"

#define Baudrate 57600

DynamixelPro2 dynamixel;

void setup() {

	Serial.begin(115200);
	dynamixel.begin(Baudrate);
	dynamixel.write_holding_torque();
	delay(3000);
	Serial.println("");
	Serial.println("Ready");
	Serial.println("");

}


void loop() {
  
}

void read_demo() {

	for (int i = 0; i < 5; i++) {
		Serial.print("Servo ");
		Serial.print(i+1);
		Serial.print(" current position = ");
		Serial.println(dynamixel.read_current_position(i + 1));

		Serial.println("");
		Serial.print("Servo ");
		Serial.print(i + 1);
		Serial.print(" current velocity = ");
		Serial.println(dynamixel.read_current_velocity(i + 1));
	}
}

void write_demo() {

	int servo_id = 1;
	unsigned int position = 90;
	dynamixel.write_goal_position(servo_id, position);
}