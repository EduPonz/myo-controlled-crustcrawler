/*
Name:    PathPlanning.cpp
Created:  12/1/2017 11:42:29 AM
Author: Steffan Svendsen, Jesper Bro Kirstein Rosenberg, Vincent Joly, Simone Jensen, David Michalik, Eduardo Ponz Segrelles, Ivelin Krasimirov Penchev
Editor: http://www.visualmicro.com
  This libraries functionality is to take in servos' current and goal positions, and based on those and the acceleration constrients (placed inside the library) 
  compute the amount of acceleration(deg/s^2) to be applied during the acceleration periode(placed inside the library), and if constant velocity is needed then calculate
  for how long to keep a constant velocity(deg/s) before beginning deceleration. Furthermore, it can find the direction for the acceleration to be applied in and also find a servos' 
  position(deg) at any given time after any given movement.  
*/

#include "Arduino.h"
#include "PathPlanning.h"

PathPlanning::PathPlanning(){}

// this function finds and returns the biggest duration of the Constant Velocity  
void PathPlanning::time_constant_velocity(){

    if(this->totalTime >= 2 * this->_TIME_OF_ACCELERATION){
      this->constantVelocityTime = (this->totalTime - (2 * this->_TIME_OF_ACCELERATION));
     }else{
      this->constantVelocityTime = 0; 
     }
}


// This function finds and returns the acceleration and the direction(+/-) in which it need to be applied for a specific servo, for it to reach its position 
//simultaneously with the other servos.
// It takes an argument from the specific servo, its delta theta. 
void PathPlanning::_calculate_acceleration() {

  float deltaTheta [3];

  for (int i = 0; i < 3; i++) {
    deltaTheta[i] = fabs(this->_servo_goal_pos[i] - this->_servo_current_pos[i]);

    if (this->totalTime == 2 * this->_TIME_OF_ACCELERATION) {
      this->servo_acceleration[i] = (deltaTheta[i] / (this->_TIME_OF_ACCELERATION * this->_TIME_OF_ACCELERATION));
    }else {
      this->servo_acceleration[i] = (deltaTheta[i] / (this->totalTime - this->_TIME_OF_ACCELERATION)) / this->_TIME_OF_ACCELERATION;
    }

    if (this->_servo_current_pos[i] > this->_servo_goal_pos[i]) {
      this->servo_acceleration[i] = -this->servo_acceleration[i];
    }else {
      this->servo_acceleration[i] = this->servo_acceleration[i];
    }
  }
}


// This function determines and returns the direction(+/-) that the deceleration needs to have.
// It takes the arguments theta_0 and theta_f from the servo and the deceleration necessary from the individualAcceleration() function. 
void PathPlanning::_calculate_deceleration() {

  for (int i = 0; i < 3; i++) {
    this->servo_deceleration[i] = -this->servo_acceleration[i];
  }
}


// This function finds and returns the biggest delta theta value of three servos.
// It takes three arguments from the servos' (their current position) and three arguments which are the servos' desired position.
float PathPlanning::dominant_servo_deltaTheta(){

  float servo_deltaTheta [3];

  for (int i = 0; i < 3; i++) {
    servo_deltaTheta[i] = fabs(this->_servo_goal_pos[i] - this->_servo_current_pos[i]);
  }
      
    if(servo_deltaTheta[0] >= servo_deltaTheta[1] && servo_deltaTheta[0] > servo_deltaTheta[2]){
      return(servo_deltaTheta[0]);
     }
        
    if(servo_deltaTheta[1] > servo_deltaTheta[0] && servo_deltaTheta[1] >= servo_deltaTheta[2]){
      return(servo_deltaTheta[1]);
     }
          
    if(servo_deltaTheta[2] > servo_deltaTheta[1] && servo_deltaTheta[2] >= servo_deltaTheta[0]){
      return(servo_deltaTheta[2]);
     }  
}


// This function finds and returns the duration for the movement based on the acceleration constraints.
// It takes an argument in the form of delta theta of a servo.
float PathPlanning::time_total(float delta_theta){
  
  if ((delta_theta / (this->_THETA_2_DOT * this->_TIME_OF_ACCELERATION) + this->_TIME_OF_ACCELERATION) > 2 * this->_TIME_OF_ACCELERATION ){
    this->totalTime = (delta_theta / (this->_THETA_2_DOT * this->_TIME_OF_ACCELERATION) + this->_TIME_OF_ACCELERATION);
    return(this->totalTime);
  }else{
    this->totalTime = 2 * this->_TIME_OF_ACCELERATION;
     return(this->totalTime);
  }
}


// This function calculates and returns a servos' angular position at a specific sample time, by looking at how far it has traveled from where it started.
// It takes the arguments individualAcceleration, totalTime, sampleTime, timeConstantVelocity, startPosition and accelerationDirection. 
float PathPlanning::get_position_sample(int servo_id, float sampleTime_milliSec) {

   float accelerationTravel; 
   float velocityTravel;
   float decelerationTravel;
     float sampleTime = sampleTime_milliSec / 1000;
   
     if(sampleTime <= this->_TIME_OF_ACCELERATION){
          accelerationTravel = 0.5 * (this->servo_acceleration[servo_id] * (sampleTime * sampleTime));
      }else{
          accelerationTravel = 0.5 * (this->servo_acceleration[servo_id] * (this->_TIME_OF_ACCELERATION * this->_TIME_OF_ACCELERATION));
      }
  
     if(this->constantVelocityTime > 0 && sampleTime <= (this->constantVelocityTime + this->_TIME_OF_ACCELERATION) && sampleTime > this->_TIME_OF_ACCELERATION) {
          velocityTravel = (this->servo_acceleration[servo_id] * this->_TIME_OF_ACCELERATION) * (sampleTime - this->_TIME_OF_ACCELERATION);
     }else if(this->constantVelocityTime > 0 && sampleTime > (this->constantVelocityTime + this->_TIME_OF_ACCELERATION)){
          velocityTravel = ((this->servo_acceleration[servo_id] * this->_TIME_OF_ACCELERATION) * (this->constantVelocityTime));
     }else{
          velocityTravel = 0;
     }
  
     if(sampleTime > (this->totalTime - this->_TIME_OF_ACCELERATION)){
          decelerationTravel = 0.5 * ((this->servo_acceleration[servo_id]) * ((sampleTime - (this->totalTime - this->_TIME_OF_ACCELERATION)) * (sampleTime - (this->totalTime - this->_TIME_OF_ACCELERATION))));
    //}else if(this->constantVelocityTime = 0 && sampleTime > this->_TIME_OF_ACCELERATION){
        // decelerationTravel = 0.5 * ((this->servo_acceleration[servo_id]) * ((sampleTime - this->_TIME_OF_ACCELERATION) * (sampleTime - this->_TIME_OF_ACCELERATION)));
     }else{
          decelerationTravel = 0;
     }
  
     if(this->servo_acceleration[servo_id] >= 0){
          return(accelerationTravel + velocityTravel + decelerationTravel + this->_servo_current_pos[servo_id]);
     }else{
          return(this->_servo_current_pos[servo_id] - accelerationTravel - velocityTravel - decelerationTravel);
     }
}


// This function calculates and returns a servos' angular velocity at a specific sample time.
// It takes the arguments totalTime, sampleTime_milliSec, timeConstantVelocity, acceleration. 
float PathPlanning::get_velocity_sample(int servo_id, float sampleTime_milliSec){

  float velocity;
    float sampleTime = sampleTime_milliSec / 1000;

    if(sampleTime <= this->_TIME_OF_ACCELERATION){
          velocity = 0.5 * this->servo_acceleration[servo_id] * sampleTime;
          return(velocity);
    }else if(this->constantVelocityTime > 0 && sampleTime <= (this->constantVelocityTime + this->_TIME_OF_ACCELERATION) && sampleTime > this->_TIME_OF_ACCELERATION) {
          velocity = (this->servo_acceleration[servo_id] * this->_TIME_OF_ACCELERATION);
          return(velocity);
    }else if(sampleTime > (this->totalTime - this->_TIME_OF_ACCELERATION) && sampleTime > this->_TIME_OF_ACCELERATION){
          velocity = (0.5 * this->servo_acceleration[servo_id] * this->_TIME_OF_ACCELERATION) - 0.5 * this->servo_acceleration[servo_id] * (sampleTime - (this->totalTime - this->_TIME_OF_ACCELERATION));
          return(velocity);
    }
  
}


// This function calculates and returns a servos' angular acceleration at a specific sample time.
// It takes the arguments totalTime, sampleTime_milliSec, timeConstantVelocity, acceleration. 
float PathPlanning::get_acceleration_sample(int servo_id, float sampleTime_milliSec){
  
     float sampleTime = sampleTime_milliSec / 1000;  

    if(sampleTime <= this->_TIME_OF_ACCELERATION){
          return(this->servo_acceleration[servo_id]);
    }else if(this->constantVelocityTime > 0 && sampleTime <= (this->constantVelocityTime + this->_TIME_OF_ACCELERATION) && sampleTime > this->_TIME_OF_ACCELERATION){
          return(0);
    }else if(sampleTime > (this->totalTime - this->_TIME_OF_ACCELERATION) && sampleTime > this->_TIME_OF_ACCELERATION){
          return(-this->servo_acceleration[servo_id]);
    }
}


// This function calculates and returns a servos' angular position, it has to have when the Acceleration period has ended.
void PathPlanning::_calculate_acceleration_segment_end_theta() {

  float segmentEndTheta [3];
  for (int i = 0; i < 3; i++) {
    segmentEndTheta[i] = fabs(0.5 * (this->servo_acceleration[i] * (this->_TIME_OF_ACCELERATION * this->_TIME_OF_ACCELERATION)));

    if (this->servo_acceleration[i] > 0) {
      this->servo_accelerationSegmentEnd[i] = this->_servo_current_pos[i] + segmentEndTheta[i];
    }else {
      this->servo_accelerationSegmentEnd[i] = this->_servo_current_pos[i] - segmentEndTheta[i];
    }
  }
}


// This function calculates and returns a servos' angular position, it has to have when the Acceleration and Constant Velocity period has ended.
void PathPlanning::_calculate_deceleration_segment_start_theta() {

  float segmentStartTheta[3];

  for (int i = 0; i < 3; i++) {
    if (this->constantVelocityTime > 0) {
      segmentStartTheta[i] = fabs(this->servo_acceleration[i] * this->_TIME_OF_ACCELERATION * this->constantVelocityTime);
      if (this->servo_acceleration[i] > 0) {
        this->servo_decelerationSegmentStart[i] = this->servo_accelerationSegmentEnd[i] + segmentStartTheta[i];
      }else {
        this->servo_decelerationSegmentStart[i] = this->servo_accelerationSegmentEnd[i] - segmentStartTheta[i];
      }
    }else {
      segmentStartTheta[i] = this->servo_accelerationSegmentEnd[i];
      this->servo_decelerationSegmentStart[i] = segmentStartTheta[i];
    }
  }
}


// This function returns the goal position of the last calculated path, based on the called servo/servo_id. 
float PathPlanning::get_goal_position(int servo_id){
  
  return(this->_servo_goal_pos[servo_id]);
}




// Question here!!! This function is not relevant!!!
//The function checks if the goal position of Servo 1, 2 and 3 is within the allowed angle range. Returns false if the robot mustn't move! 
bool PathPlanning::final_angle_restriction_check(float knownServo1_f, float knownServo2_f, float knownServo3_f) {
  if (knownServo1_f < this->_theta_restricted_servo1[0] || knownServo1_f > this->_theta_restricted_servo1[1]) {
    // Final position of servo 1 is not allowed.
    return false;
  }
  else if (knownServo2_f < this->_theta_restricted_servo2[0] || knownServo2_f > this->_theta_restricted_servo2[1]) {
    // Final position of servo 2 is not allowed.
    return false;
  }
  else if (knownServo3_f < this->_theta_restricted_servo3[0] || knownServo3_f > this->_theta_restricted_servo3[1]) {
    // Final position of servo 3 is not allowed.
    return false;
  }
  else {
    // Movement of the three servos is allowed.
    return true;
  }
}




//This function checks if the starting position of the servo before going to a specific position is within the allowed area.
//If it is not, it will send a warnining and will give a delay before executing the program. Retunrns false if gives a warning.
bool PathPlanning::starting_angle_warning(float servo1_current_pos, float servo2_current_pos, float servo3_current_pos) {
  if (servo1_current_pos < this->_theta_restricted_servo1[0] || servo1_current_pos > this->_theta_restricted_servo1[1]) {
    // WARNING: Starting position of servo 1 is not safe.
    if (servo2_current_pos < this->_theta_restricted_servo2[0] || servo2_current_pos > this->_theta_restricted_servo2[1]) {
      // WARNING: Starting position of servo 2 is not safe.
    }
    if (servo3_current_pos < this->_theta_restricted_servo3[0] || servo3_current_pos > this->_theta_restricted_servo3[1]) {
      // WARNING: Starting position of servo 3 is not safe.  
    }
    return false;
  }
  else if (servo2_current_pos < this->_theta_restricted_servo2[0] || servo2_current_pos > this->_theta_restricted_servo2[1]) {
    // WARNING: Starting position of servo 2 is not safe.
    if (servo3_current_pos < this->_theta_restricted_servo3[0] || servo3_current_pos > this->_theta_restricted_servo3[1]) {
      // WARNING: Starting position of servo 3 is not safe. 
    }
    return false;
  }
  else if (servo3_current_pos < this->_theta_restricted_servo3[0] || servo3_current_pos > this->_theta_restricted_servo3[1]) {
    // WARNING: Starting position of servo 3 is not safe.
    return false;
  }
  else {
    // Starting position is safe.
    return true;
  }
}


// This function calculates and changes the servo_acceleration_direction[], servo_accelerationSegmentEnd[], servo_decelerationSegmentStart[], constantVelocityTime and totalTime
// which are public variables that then can be accessed. 
bool PathPlanning::calculate_path(float servo1_current_pos, float servo2_current_pos, float servo3_current_pos, String instruction){

  if (this->final_angle_restriction_check(servo1_current_pos, servo2_current_pos, servo3_current_pos) && this->starting_angle_warning(servo1_current_pos, servo1_current_pos, servo1_current_pos)) {

    if (instruction == this->_HOME) {
      for (int i = 0; i < 3; i++) {
        this->_servo_goal_pos[i] = this->home_position[i];
      }
    }else if (instruction == this->_EXTENDED) {
      for (int i = 0; i < 3; i++) {
        this->_servo_goal_pos[i] = this->extended_position[i];
      }
    }else if (instruction == this->_TO_USER) {
      for (int i = 0; i < 3; i++) {
        this->_servo_goal_pos[i] = this->to_user_position[i];
      }
    }

    this->_servo_current_pos[0] = servo1_current_pos;
    this->_servo_current_pos[1] = servo2_current_pos;
    this->_servo_current_pos[2] = servo3_current_pos;

    float dominant_deltaTheta = this->dominant_servo_deltaTheta();
    this->totalTime = time_total(dominant_deltaTheta);
    this->time_constant_velocity();

    this->_calculate_acceleration();
    this->_calculate_deceleration();
    this->_calculate_acceleration_segment_end_theta();
    this->_calculate_deceleration_segment_start_theta();

    return(true);

  }else {
    return(false);
  }
}
