/*  
  Name:   PathPlanning.h
  Created : 12 / 1 / 2017 11 : 42 : 29 AM
  Author : Steffan Svendsen, Jesper Bro Kirstein Rosenberg, Vincent Joly, Simone Jensen, David Michalik, Eduardo Ponz Segrelles, Ivelin Krasimirov Penchev
  Editor : http://www.visualmicro.com
*/

#ifndef PathPlanning_h
#define PathPlanning_h

#include "Arduino.h"

class PathPlanning{
  public:
      void time_constant_velocity();
      float dominant_servo_deltaTheta();
      float time_total(float delta_theta);
      float get_position_sample(int servo_id, float sampleTime_milliSec);
      float get_velocity_sample(int servo_id, float sampleTime_milliSec);
      float get_acceleration_sample(int servo_id, float sampleTime_milliSec);
      float get_goal_position(int servo_id);
      bool starting_angle_warning(float servo1_current_pos, float servo2_current_pos, float servo3_current_pos);
      bool calculate_path(float servo1_current_pos, float servo2_current_pos, float servo3_current_pos, String instruction);

      // Public variables 
	  float servo_velocity[3] = { 0, 0, 0 };
      float servo_acceleration [3] = {0, 0, 0};
      float servo_deceleration [3] = { 0, 0, 0};
      float servo_accelerationSegmentEnd [3] = {0, 0, 0};
      float servo_decelerationSegmentStart [3] = {0, 0, 0};
      float constantVelocityTime = 0;
      float totalTime = 0;

      // CrustCrawler preset positions
      const float home_position[3] = {220, 90, 180};	      //{193.6, 67.32, 293.04}		 
      const float extended_position[3] = {193.6, 88, 180.224};      //{193.6, 88, 180.224}
      const float to_user_position[3] = {45, 90, 180};       //{105.6, 180.224, 88}
      
    
  private:

      // Private functions
      void _calculate_acceleration();
      void _calculate_deceleration();
      void _calculate_acceleration_segment_end_theta();
      void _calculate_deceleration_segment_start_theta();
	  void _calculate_velocity();
        
      // Private Constant variables
      const float _TIME_OF_ACCELERATION = 0.50;   // This is the preset acceleration duration time.
      const float _THETA_2_DOT = 140;         // This is the preset maximum acceleration. 
      const float _PI = 3.1415927;
      const String _HOME = "home";
      const String _EXTENDED = "extended";
      const String _TO_USER = "to_user";
      
      // Private changing variables 
      float _servo_goal_pos[3];
      float _servo_current_pos[3];
      
      //Bottom and top angle limits for each servo. TO BE EDITED!!
      int _theta_restricted_servo1 [2] = {86, 356};
	  int _theta_restricted_servo2 [2] = {200, 180};
      int _theta_restricted_servo3 = 128;
    
    
};

#endif
