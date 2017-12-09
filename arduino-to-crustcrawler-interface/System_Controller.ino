
#include <SoftwareSerial.h>
#include <ArduinoJson.h>
#include "DynamicsCalculator.h"
#include "PathPlanning.h"
#include "DynamixelPro2.h"
#include "UnitsConverter.h"

const int k_p = 400;
const int k_v = 40;
const float ALLOWED_ERROR = 2; // Some value
const int GRIPPER_DELAY = 3000;
const int SAMPLING_RATE = 50;
const int time_for_torque = 2000; // I don't think we need a time on it.
const String JOB_FAILED = "failed";
const String JOB_SUCCESS = "success";
const String JOB_IN_PROGRESS = "in progress";

SoftwareSerial _software_serial(10, 11);
DynamicsCalculator dynamics_calculator;
PathPlanning path_planning;
DynamixelPro2 dynamixel;
UnitsConverter convert;

float servo_position[3] = {0, 0, 0};
float servo_velocity[3] = {0, 0, 0};
float servo_acceleration[3] = {0, 0, 0};
float desired_position[3] = {0, 0, 0};
float desired_velocity[3] = {0, 0, 0};
float desired_acceleration[3] = {0, 0, 0};

bool first_ok = false;
bool second_ok = false;
bool third_ok = false;

int operation_id = 1;
String job_status = JOB_SUCCESS;
int mode = 0;
String instruction = "";
bool path_status = false;
String gesture = dynamixel.UNKNOWN_GESTURE;

String prev_instruction = "";
unsigned long time = millis();
unsigned long prev_time = millis();
unsigned long prev_read_millis = millis();


void setup(){
	
	Serial.flush();                                       
	_software_serial.begin(57600);                  
	Serial.begin(9600);                                  
	dynamixel.begin(_software_serial); 

	dynamixel.initialization();

	pinMode(52, OUTPUT);
	pinMode(53, OUTPUT);
	pinMode(46, OUTPUT);

}

void loop(){
	
	while (!Serial) {}

	if (Serial.available() > 0) {
		if (millis() - prev_read_millis > 100) {
			String input = "";
			input = serialInput();
			applyInstructions(input);
		}
	}


	if (mode == dynamixel.PRE_SET_MODE){
		digitalWrite(53, LOW);
		digitalWrite(52, HIGH);
		instruction = dynamixel.get_instruction(dynamixel.PRE_SET_MODE, gesture);
		if (instruction == dynamixel.EXTENDED || instruction == dynamixel.TO_USER || instruction == dynamixel.HOME){
			job_status = JOB_IN_PROGRESS;
			get_servo_positions();
			path_status = path_planning.calculate_path(servo_position[0], servo_position[1], servo_position[2], instruction);
			send_json();
			if (path_status == true){
				prev_time = millis();
				while (true){
					time = millis() - prev_time;
					float servo_error_pos [3];
					float servo_error_vel [3];
					float servo_acc [3];
					if (time >= SAMPLING_RATE || time == 0){
						for (int i = 0; i < 3; ++i){
							servo_error_pos[i] = convert.position_degrees_to_radians(convert.unit_to_degree(dynamixel.read_current_position(i)) 
							- path_planning.get_position_sample(i, time)) * k_p;
							servo_error_vel[i] = convert.position_degrees_to_radians(convert.unit_to_degree(dynamixel.read_current_velocity(i)) 
							- path_planning.get_velocity_sample(i, time)) * k_v;
							servo_acc[i] = path_planning.get_acceleration_sample(i, time);
						}
						dynamics_calculator.set_thetas(servo_error_pos[0], servo_error_pos[1], servo_error_pos[2]);
						dynamics_calculator.set_omegas(servo_error_vel[0], servo_error_vel[1], servo_error_vel[2]);
						dynamics_calculator.set_alphas(servo_acc[0], servo_acc[1], servo_acc[2]);
						float* torque;
						dynamics_calculator.get_torque(torque); // we need some conversion here
						for (int i = 0; i < 3; ++i){
							dynamixel.write_torque(i, torque[i], time_for_torque);
						}
						prev_time = millis();
					}

					if (abs(path_planning.get_goal_position(0) - convert.unit_to_degree(dynamixel.read_current_position(0))) < ALLOWED_ERROR){
						first_ok = true;
					}
					if (abs(path_planning.get_goal_position(1) - convert.unit_to_degree(dynamixel.read_current_position(1))) < ALLOWED_ERROR){
						second_ok = true;
					}
					if (abs(path_planning.get_goal_position(2) - convert.unit_to_degree(dynamixel.read_current_position(2))) < ALLOWED_ERROR){
						third_ok = true;
					}
					if (first_ok && second_ok && third_ok){
						dynamixel.write_holding_torque(true); 
						break;
					}
				}
				get_servo_positions();
				job_status = JOB_SUCCESS;
				send_json();
				job_status = JOB_FAILED;
				path_status = false;
				first_ok = false;
				second_ok = false;
				third_ok = false;
			}else{
				get_servo_positions();
				job_status = JOB_FAILED;
			}

		}else if (instruction == dynamixel.GRIPPER){
			dynamixel.use_gripper();
			delay(GRIPPER_DELAY);
			job_status = JOB_SUCCESS;
			get_servo_positions();
			send_json();
		}else{
			get_servo_positions();
			job_status = JOB_FAILED;
		}
		operation_id++;
		mode = 0;

	}else if (mode == dynamixel.MANUAL_MODE){

			digitalWrite(52, LOW);
			digitalWrite(53, HIGH);

			instruction = dynamixel.get_instruction(dynamixel.MANUAL_MODE, gesture);
			job_status = JOB_SUCCESS;

			prev_instruction = instruction;

				if (instruction == dynamixel.DOWN) {
					dynamixel.move_down();
					digitalWrite(46, HIGH);
				}
				else if (instruction == dynamixel.UP) {
					dynamixel.move_up();
					digitalWrite(46, LOW);
				}
				else if (instruction == dynamixel.LEFT) {
					dynamixel.move_left();
				}
				else if (instruction == dynamixel.RIGHT) {
					dynamixel.move_right();
				}
				else {
					job_status = JOB_FAILED;
				}
		

			/*if (prev_instruction != instruction) {
				get_servo_positions();
				send_json();
				prev_instruction = instruction;
				operation_id++;
			}*/
			mode = 0;
		
	}
	//if (mode >= 2){
	//	send_json();
	//	mode = 0;
	//}
}

void get_servo_positions(){
	for (int i = 0; i < 3; ++i){
		servo_position[i] = convert.unit_to_degree(dynamixel.read_current_position(i));
	}
	
}


String serialInput() {
	int n = Serial.available();
	String s = Serial.readString();
	return s;
}

void applyInstructions(String instructions) {
	StaticJsonBuffer<200> jsonBuffer;
	JsonObject& jsonInstructions = jsonBuffer.parseObject(instructions);
	String string_mode = jsonInstructions["mode"];
	mode = string_mode.toInt();
	String string_gesture = jsonInstructions["gesture"];
	gesture = string_gesture;
}

void send_json(){
	StaticJsonBuffer<200> jsonBuffer;
	JsonObject& jsonMessage = jsonBuffer.createObject();
	jsonMessage ["from arduino"] = true;      				// bool
	jsonMessage["operation id"] = operation_id;      		// int
	jsonMessage["mode"] = mode;              				// int
	jsonMessage["gesture"] = gesture;          				// String
	jsonMessage["job status"] = job_status;        			// String
	jsonMessage["instruction"] = instruction;      			// String
	jsonMessage["path status"] = path_status;      			// bool
	jsonMessage["servo 1 position"] = servo_position[0]; 	// float
	jsonMessage["servo 2 position"] = servo_position[1]; 	// float
	jsonMessage["servo 3 position"] = servo_position[2]; 	// float
	jsonMessage.printTo(Serial);
	Serial.println();
	Serial.flush();
}