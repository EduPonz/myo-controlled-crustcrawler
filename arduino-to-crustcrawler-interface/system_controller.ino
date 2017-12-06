#include "DynamicsCalculator.h"
#include "PathPlanning.h"
#include "jesperWhatever.h"

const int k_p = 400;
const int k_v = 40;
DynamicsCalculator dynamics_calculator;
pathPlanning path_planning;
jesperWhatever jesper_whatever;
float servo_position[3] = {0, 0, 0};
float servo_velocity[3] = {0, 0, 0};
float servo_acceleration[3] = {0, 0, 0};
float desired_position[3] = {0, 0, 0};
float desired_velocity[3] = {0, 0, 0};
float desired_acceleration[3] = {0, 0, 0};
bool preset_mode = true;

void setup(){
	Serial.begin(9600);
}

void loop(){
	if (preset_mode){
		String instruction = jesper_whatever.instruction;
		for (int i = 0; i < 3; ++i){
			servo_position[i] = jesper_whatever.get_current_position(i);
		}
		path_planning.calculate_path(instruction, servo_position[0], servo_position[1], servo_position[2]);
		prev_time = millis();
		while true{
			time = millis() - prev_time;
			float servo_error_pos [3];
			float servo_error_vel [3];
			float servo_acc [3];
			if (time >= sampling_rate || time == 0){
				for (int i = 0; i < 3; ++i){
					servo_error_pos[i] = (jesper_whatever.get_current_position(i) - path_planning.get_positionSample(i, time)) * k_p;
					servo_error_vel[i] = (jesper_whatever.get_current_velocity(i) - path_planning.get_velocitySample(i, time)) * k_v;
					servo_acc[i] = path_planning.get_accelerationSample(i, time);
				}
				dynamics_calculator.set_thetas(servo_error_pos[0], servo_error_pos[1], servo_error_pos[2]);
				dynamics_calculator.set_omegas(servo_error_vel[0], servo_error_vel[1], servo_error_vel[2]);
				dynamics_calculator.set_alphas(servo_acc[0], servo_acc[1], servo_acc[2]);
				float torque [3];
				dynamics_calculator.get_torque((float*)torque);
				jesper_whatever.send_torque((float*)torque);
				prev_time = millis();
			}
			if (abs(path_planning.get_goal_position(0, instruction) - jesper_whatever.get_current_position(0)) < allowed_error){
				first_ok = true;
			}
			if (abs(path_planning.get_goal_position(1, instruction) - jesper_whatever.get_current_position(1)) < allowed_error){
				second_ok = true;
			}
			if (abs(path_planning.get_goal_position(2, instruction) - jesper_whatever.get_current_position(2)) < allowed_error){
				third_ok = true;
			}
			if (first_ok && second_ok && third_ok){
				jesper_whatever.hold_position();
				break;
			}
		}
		first_ok = false;
		second_ok = false;
		third_ok = false;
	}
}