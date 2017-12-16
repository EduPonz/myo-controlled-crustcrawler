/*
Name:    DynamixelPro2.h
Created:  12/5/2017 3:25:09 PM
Author:   Steffan Svendsen, Vincent Joly, Simone Jensen, David Michalik, Eduardo Ponz Segrelles, Ivelin Krasimirov Penchev, Jesper Bro Kirstein Rosenberg
*/

#ifndef DynamixelPro2_h
#define DynamixelPro2_h

#include "Arduino.h"

#define NONE                            0x00
#define READ                            0x01
#define ALL                             0x02
#define SERVO_CONTROL_PIN 0x02       // Control pin of buffer chip, NOTE: this does not matter becasue we are not using a half to full contorl buffer.
#define SERVO_SET_BAUDRATE 57600    // Baud rate speed which the Dynamixel will be set too (57600)
#define LED13 0x0D
#define CW_LIMIT_ANGLE 0x001        // lowest clockwise angle is 1, as when set to 0 it set servo to wheel mode
#define CCW_LIMIT_ANGLE 0xFFF       // Highest anit-clockwise angle is 0XFFF, as when set to 0 it set servo to wheel mode

class DynamixelPro2{
public:
  const int PRE_SET_MODE = 2;
  const int MANUAL_MODE = 1;
  const String GRIPPER = "gripper";
  const String EXTENDED = "extended";
  const String TO_USER = "to_user";
  const String HOME = "home";
  const String UNKNOWN_GESTURE = "unknown gesture";
  const String DOWN = "home";
  const String UP = "up";
  const String LEFT = "left";
  const String RIGHT = "right";
  const String UNKNOWN_MODE = "unknown mode";


  void begin(Stream&);
  void end(void);
  void initialization();
  void move_left();
  void move_right();
  void move_up();
  void move_down();
  void use_gripper();
  void write_holding_torque(bool state);
  void write_goal_position(int servo_id, unsigned int pos);
  int read_current_position(int servo_id);
  int read_current_velocity(int servo_id);
  String get_instruction(int mode, String gesture);
  void switching_operating_mode(int modeChoice);
  void write_operation_mode(unsigned char ID, unsigned short modeSel);
  float readPWM(unsigned char ID);
  void writePWM(int servo_id, int PWM);


private:

  Stream * _software_serial;
  
  void _write_to_servo_id(unsigned char ID, unsigned short addr, unsigned char *arr, int n);
  void _write_profile_acceleration(int servo_id, unsigned int pac);
  void _write_profile_velocity(int servo_id, unsigned int pvl);
  void _read_from_servo_id(unsigned char ID, unsigned short addr, int n);
  void _read_return_packet(void);
  void _get_parameters(void);
  void _clear_RX_buffer(void);
  void _transmit_instruction_packet(int transLen);
  void _mode_switch(int mode);
  void _change_parameters(int vel, int acc);
  void _set_direction_pin(unsigned char D_Pin);
  unsigned short _update_crc(unsigned char *data_blk_ptr, unsigned short data_blk_size);

  const int _GRIPPER_OPEN [2] = {3072, 1024};       // 3072, 1024
  const int _GRIPPER_CLOSE [2] = {2048, 2048};
  const String _FIST = "fist";
  const String _FINGERS_SPREAD = "fingersSpread";
  const String _WAVE_IN = "waveIn";
  const String _WAVE_OUT = "waveOut";
  const unsigned char _SERVO_HEX_ID [5] = {0x01, 0x02, 0x03, 0x04, 0x05}; // Array to hold servo id's in HEX form

  unsigned int _data [15];                  // Data from ReturnPacket
  unsigned int _return_packet [100];            // Array to hold returned status packet data
  unsigned char _instruction_packet_array [64];			// Array to hold instruction packet data
  unsigned char _status_return_value;					// Status packet return states ( NON , READ , ALL )
  const float _home_position [3] = {2200, 765, 3330}; 
  char            Direction_Pin;						// Pin to control TX/RX buffer chip
};

#endif
