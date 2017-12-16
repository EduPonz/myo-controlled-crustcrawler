#include <SoftwareSerial.h>
#include <ArduinoJson.h>
#include "DynamicsCalculator.h"
#include "PathPlanning.h"
#include "DynamixelPro2.h"
#include "UnitsConverter.h"

const int k_p = 400;
const int k_v = 40;
const float ALLOWED_ERROR = 2; // Some value
const int GRIPPER_DELAY = 12000;
const int SAMPLING_RATE = 110;
const int time_for_torque = 2000; // I don't think we need a time on it.
const String JOB_FAILED = "failed";
const String JOB_SUCCESS = "success";
const String JOB_IN_PROGRESS = "in progress";

SoftwareSerial _software_serial(10, 11);
DynamicsCalculator dynamics_calculator;
PathPlanning path_planning;
DynamixelPro2 dynamixel;
UnitsConverter convert;

float servo_position [3] = {0, 0, 0};
float servo_velocity [3] = {0, 0, 0};
float servo_acceleration [3] = {0, 0, 0};
float desired_position [3] = {0, 0, 0};
float desired_velocity [3] = {0, 0, 0};
float desired_acceleration [3] = {0, 0, 0};
float servo_error_pos [3];
float servo_error_vel [3];
float servo_acc [3];

bool first_ok = false;
bool second_ok = false;
bool third_ok = false;
bool path_status = false;
bool input_empty = true;

int mode = 0;
int operation_id = 1;
int overwrite_delay = 250;
String instruction = "";
String job_status = JOB_SUCCESS;
String gesture = dynamixel.UNKNOWN_GESTURE;
String prev_instruction = "";

unsigned long loopTime = millis();
unsigned long prev_time = millis();

//unsigned long startTime = 0;
//unsigned long endTime = 0;

void setup(){

  _software_serial.begin(57600);
  Serial.begin(115200);
  clear_pc_buffer();
  dynamixel.begin(_software_serial);
  dynamixel.initialization();

}

void loop(){

  while (!Serial){}

  if (Serial.available() > 0){
    String input = "";
    input = serialInput();
    if (input != ""){
      applyInstructions(input);
      input_empty = false;
    } else{
      input_empty = true;
    }
  }

  if (mode == dynamixel.PRE_SET_MODE && !input_empty){

    if (instruction == dynamixel.EXTENDED || instruction == dynamixel.TO_USER || instruction == dynamixel.HOME){
      job_status = JOB_IN_PROGRESS;
      path_status = path_planning.calculate_path(servo_position [0], servo_position [1], servo_position [2], instruction);
      send_json();

      if (path_status == true){
        dynamixel.switching_operating_mode(dynamixel.PRE_SET_MODE);
        prev_time = millis();

        while (true){
          Serial.println("Control loop");
          //startTime = millis();
          
          loopTime = millis() - prev_time;

          if (loopTime >= SAMPLING_RATE || loopTime == 0){

            for (int i = 0; i < 3; ++i){
              servo_error_pos[i] = convert.position_degrees_to_radians(convert.unit_to_degree(dynamixel.read_current_position(i))
                           - path_planning.get_position_sample(i, loopTime)) * k_p;

              servo_error_vel[i] = convert.speed_degrees_to_radians(convert.unit_to_speed_degree(dynamixel.read_current_velocity(i))
                         - path_planning.get_velocity_sample(i, loopTime)) * k_v;
              
              servo_acc[i] = convert.acceleration_degrees_to_radians(path_planning.get_acceleration_sample(i, loopTime));
            }

            dynamics_calculator.set_thetas(servo_error_pos[0], servo_error_pos[1], servo_error_pos[2]);
            dynamics_calculator.set_omegas(servo_error_vel[0], servo_error_vel[1], servo_error_vel[2]);
            dynamics_calculator.set_alphas(servo_acc[0], servo_acc[1], servo_acc[2]);
            
            float* torque;
            dynamics_calculator.get_torque(torque); // we need some conversion here

            for (int i = 0; i < 3; ++i){
              dynamixel.writePWM(i, convert.torque_to_PWM_unit(i, convert.speed_degrees_to_radians(convert.unit_to_speed_degree(dynamixel.read_current_velocity(i))), torque[i]));
              Serial.println("write PWM okay");
            }

            prev_time = millis();
          }

          if (abs(path_planning.get_goal_position(0) - convert.unit_to_degree(dynamixel.read_current_position(0))) < ALLOWED_ERROR){
            first_ok = true;
            Serial.println("Breaking 1st okay");
          }
          if (abs(path_planning.get_goal_position(1) - convert.unit_to_degree(dynamixel.read_current_position(1))) < ALLOWED_ERROR){
            second_ok = true;
            Serial.println("Breaking 2nd okay");
          }
          if (abs(path_planning.get_goal_position(2) - convert.unit_to_degree(dynamixel.read_current_position(2))) < ALLOWED_ERROR){
            third_ok = true;
            Serial.println("Breaking 3rd okay");
          }
          if (first_ok && second_ok && third_ok){
            dynamixel.switching_operating_mode(dynamixel.MANUAL_MODE);
            Serial.println("Breaking the loop");
            break;
          }
          /*endTime = millis();
          Serial.print("Execution time = ");
          Serial.print(endTime - startTime);
          Serial.println(" milli seconds");
          delay(1000);*/
        }
        job_status = JOB_SUCCESS;
        send_json();
        job_status = JOB_FAILED;
        path_status = false;
        first_ok = false;
        second_ok = false;
        third_ok = false;

      } else{
        job_status = JOB_FAILED;
        send_json();
      }

    } else if (instruction == dynamixel.GRIPPER){
      dynamixel.use_gripper();
      instruction = "";
      delay(GRIPPER_DELAY);
      job_status = JOB_SUCCESS;
      send_json();
    } else{
      job_status = JOB_FAILED;
      send_json();
    }
    operation_id++;
    mode = 0;

  } else if (mode == dynamixel.MANUAL_MODE && !input_empty){

    if (instruction == dynamixel.DOWN){
      dynamixel.move_down();
      delay(overwrite_delay);
    } else if (instruction == dynamixel.UP){
      dynamixel.move_up();
      delay(overwrite_delay);
    } else if (instruction == dynamixel.LEFT){
      dynamixel.move_left();
      delay(overwrite_delay);
    } else if (instruction == dynamixel.RIGHT){
      dynamixel.move_right();
      delay(overwrite_delay);
    } else{
      mode = 0;
      delay(overwrite_delay);
    }   
  }
}

void get_servo_positions(){
  for (int i = 0; i < 3; ++i){
    servo_position [i] = convert.unit_to_degree(dynamixel.read_current_position(i));
  }
}

String serialInput(){
  delay(100);
  String input = "";
  char c;
  bool open_brace = false;
  while (Serial.available() > 0){
    c = Serial.read();
    if ((c == '{') || (open_brace == true)){
      open_brace = true;
      input += c;
    }
    if (c == '}'){
      open_brace = false;
    }
  }
 
  return input;
}

void clear_pc_buffer(){
  while (Serial.available() > 0){
    Serial.read();
  }
}

void applyInstructions(String instructions){
  StaticJsonBuffer<200> jsonBuffer;
  JsonObject& jsonInstructions = jsonBuffer.parseObject(instructions);

  if (jsonInstructions.success()){
    String string_mode = jsonInstructions["mode"];
    mode = string_mode.toInt();
    String string_gesture = jsonInstructions["gesture"];
    gesture = string_gesture;

    if (mode == dynamixel.MANUAL_MODE){
      instruction = dynamixel.get_instruction(dynamixel.MANUAL_MODE, gesture);
    }else if (mode == dynamixel.PRE_SET_MODE){
      instruction = dynamixel.get_instruction(dynamixel.PRE_SET_MODE, gesture);
    }else{
      instruction = "";
    }
  }else{
    clear_pc_buffer();
  }
}

void send_json(){
  get_servo_positions();
  StaticJsonBuffer<200> jsonBuffer;
  JsonObject& jsonMessage = jsonBuffer.createObject();
  jsonMessage ["operation id"] = operation_id;      // int
  jsonMessage ["mode"] = mode;              // int
  jsonMessage ["gesture"] = gesture;            // String
  jsonMessage ["job status"] = job_status;        // String
  jsonMessage ["instruction"] = instruction;        // String
  jsonMessage ["path status"] = path_status;        // bool
  jsonMessage ["servo 1 position"] = servo_position [0];  // float
  jsonMessage ["servo 2 position"] = servo_position [1];  // float
  jsonMessage ["servo 3 position"] = servo_position [2];  // float
  clear_pc_buffer();
  jsonMessage.printTo(Serial);
  Serial.println();
  Serial.flush();
}
