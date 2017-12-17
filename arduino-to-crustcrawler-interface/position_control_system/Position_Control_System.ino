#include <SoftwareSerial.h>
#include <ArduinoJson.h>
#include "DynamicsCalculator.h"
#include "PathPlanning.h"
#include "DynamixelPro2.h"
#include "UnitsConverter.h"

const int k_p = 400;
const int k_v = 40;
const float ALLOWED_ERROR = 5; // Some value
const int GRIPPER_DELAY = 12000;
const int SAMPLING_RATE = 200;
const int time_for_torque = 2000; // I don't think we need a time on it.
const String JOB_FAILED = "failed";
const String JOB_SUCCESS = "success";
const String JOB_IN_PROGRESS = "in progress";

SoftwareSerial _software_serial(10, 11);
DynamicsCalculator dynamics_calculator;
PathPlanning path_planning;
DynamixelPro2 dynamixel;
UnitsConverter convert;

float servo_position[3] = { 0, 0, 0 };
float servo_error_pos[3];
float servo_error_vel[3];
float servo_acc[3];

bool path_status = false;
bool input_empty = true;

int mode = 0;
int operation_id = 1;
int overwrite_delay = 250;
String instruction = "";
String job_status = JOB_SUCCESS;
String gesture = dynamixel.UNKNOWN_GESTURE;
String prev_instruction = "";

void setup() {

	_software_serial.begin(57600);
	Serial.begin(115200);
	clear_pc_buffer();
	dynamixel.begin(_software_serial);
  delay(1500);
	dynamixel.initialization();

}

void loop() {

	while (!Serial) {}

	if (Serial.available() > 0) {
		String input = "";
		input = serialInput();
		if (input != "") {
			applyInstructions(input);
			input_empty = false;
		}else {
			input_empty = true;
		}
	}

	if (mode == dynamixel.PRE_SET_MODE && !input_empty) {
        
		if (instruction == dynamixel.EXTENDED || instruction == dynamixel.TO_USER || instruction == dynamixel.HOME) {
			job_status = JOB_IN_PROGRESS;
		
			send_json();
		
        dynamixel.write_profile_acceleration(1, 10);
        dynamixel.write_profile_velocity(1, 10);
        dynamixel.write_profile_acceleration(2, 40);
        dynamixel.write_profile_velocity(2, 15);
        
        
				if (instruction == dynamixel.HOME) {
					for (int i = 0; i < 3; i++) {
						dynamixel.write_goal_position(i, convert.degree_to_unit(path_planning.home_position[i]));
					}
				}else if (instruction == dynamixel.EXTENDED) {
					for (int i = 1; i < 3; i++) {
						dynamixel.write_goal_position(i, convert.degree_to_unit(path_planning.extended_position[i]));
					}
				}else if (instruction == dynamixel.TO_USER) {
					for (int i = 0; i < 3; i++) {
						dynamixel.write_goal_position(i, convert.degree_to_unit(path_planning.to_user_position[i]));
					}
				}

				job_status = JOB_SUCCESS;
				send_json();
				job_status = JOB_FAILED;
				path_status = false;
		
		}else if (instruction == dynamixel.GRIPPER) {
			dynamixel.use_gripper();
			instruction = "";
			delay(GRIPPER_DELAY);
			job_status = JOB_SUCCESS;
			send_json();
		}
		else {
			job_status = JOB_FAILED;
			send_json();
		}
		operation_id++;
		mode = 0;

	}
	else if (mode == dynamixel.MANUAL_MODE && !input_empty) {

		if (instruction == dynamixel.DOWN) {
			dynamixel.move_down();
			delay(overwrite_delay);
		}
		else if (instruction == dynamixel.UP) {
			dynamixel.move_up();
			delay(overwrite_delay);
		}
		else if (instruction == dynamixel.LEFT) {
			dynamixel.move_left();
			delay(overwrite_delay);
		}
		else if (instruction == dynamixel.RIGHT) {
			dynamixel.move_right();
			delay(overwrite_delay);
		}
		else {
			mode = 0;
			delay(overwrite_delay);
		}
	}
}

void get_servo_positions() {
	for (int i = 0; i < 3; ++i) {
		servo_position[i] = convert.unit_to_degree(dynamixel.read_current_position(i));
	}
}

String serialInput() {
	delay(100);
	String input = "";
	char c;
	bool open_brace = false;
	while (Serial.available() > 0) {
		c = Serial.read();
		if ((c == '{') || (open_brace == true)) {
			open_brace = true;
			input += c;
		}
		if (c == '}') {
			open_brace = false;
		}
	}

	return input;
}

void clear_pc_buffer() {
	while (Serial.available() > 0) {
		Serial.read();
	}
}

void applyInstructions(String instructions) {
	StaticJsonBuffer<200> jsonBuffer;
	JsonObject& jsonInstructions = jsonBuffer.parseObject(instructions);

	if (jsonInstructions.success()) {
		String string_mode = jsonInstructions["mode"];
		mode = string_mode.toInt();
		String string_gesture = jsonInstructions["gesture"];
		gesture = string_gesture;

		if (mode == dynamixel.MANUAL_MODE) {
			instruction = dynamixel.get_instruction(dynamixel.MANUAL_MODE, gesture);
		}
		else if (mode == dynamixel.PRE_SET_MODE) {
			instruction = dynamixel.get_instruction(dynamixel.PRE_SET_MODE, gesture);
		}
		else {
			instruction = "";
		}
	}
	else {
		clear_pc_buffer();
	}
}

void send_json() {
	get_servo_positions();
	StaticJsonBuffer<200> jsonBuffer;
	JsonObject& jsonMessage = jsonBuffer.createObject();
	jsonMessage["operation id"] = operation_id;      // int
	jsonMessage["mode"] = mode;              // int
	jsonMessage["gesture"] = gesture;            // String
	jsonMessage["job status"] = job_status;        // String
	jsonMessage["instruction"] = instruction;        // String
	jsonMessage["path status"] = path_status;        // bool
	jsonMessage["servo 1 position"] = servo_position[0];  // float
	jsonMessage["servo 2 position"] = servo_position[1];  // float
	jsonMessage["servo 3 position"] = servo_position[2];  // float
	clear_pc_buffer();
	jsonMessage.printTo(Serial);
	Serial.println();
	Serial.flush();
}
