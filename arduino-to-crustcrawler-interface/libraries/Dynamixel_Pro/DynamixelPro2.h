/*
Name:		Dynamixel_Demo.h
Created:	12/5/2017 3:25:09 PM
Author:		Steffan Svendsen, Vincent Joly, Simone Jensen, David Michalik, Eduardo Ponz Segrelles, Ivelin Krasimirov Penchev
*/

#ifndef DynamixelPro2_h
#define DynamixelPro2_h

#include "Arduino.h"

#define NONE                            0x00
#define READ                            0x01
#define ALL                             0x02

class DynamixelPro2 {
				public:
						DynamixelPro2() : _direction_pin(-1), _status_return_value(READ) { }
						void begin(long);
						void end(void);
						void move_left();
						void move_right();
						void move_up();
						void move_down();
						void write_holding_torque();
						void write_torque(int servo_id, float torque, float time_of_torque);
						void write_goal_position(int servo_id, unsigned int pos);
						int read_current_position(int servo_id);
						int read_current_velocity(int servo_id);

				private:
						Stream *_serial;

						void _write_to_servo_id(int servo_id, unsigned short addr, unsigned char *arr, int n);
						void _read_from_servo_id(unsigned char ID, unsigned short addr, int n);
						void _read_return_packet(void);
						void _read_parameters(void);
						void _clear_RX_buffer(void);
						void _transmit_instruction_packet(int transLen);
						unsigned short _update_crc(unsigned char *data_blk_ptr, unsigned short data_blk_size);
						

						char _direction_pin;									// Pin to control TX/RX buffer chip
						unsigned int _data[15];									// Data from ReturnPacket
						unsigned int _return_packet[100];						// Array to hold returned status packet data
						unsigned char _instruction_packet_array[64];			// Array to hold instruction packet data
						unsigned char _status_return_value;						// Status packet return states ( NON , READ , ALL )	
						unsigned char _servo_HEX_id[5] = {0x01, 0x02, 0x03, 0x04, 0x05};	// Array to hold servo id's in HEX form

};

#endif
