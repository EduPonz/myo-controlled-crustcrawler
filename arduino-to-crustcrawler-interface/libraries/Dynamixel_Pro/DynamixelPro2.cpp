/*
Name:    DynamixelPro2.cpp
Created:  12/5/2017 3:25:09 PM
Author:   Steffan Svendsen, Vincent Joly, Simone Jensen, David Michalik, Eduardo Ponz Segrelles, Ivelin Krasimirov Penchev, Jesper Bro Kirstein Rosenberg
*/

#include "DynamixelPro2.h"

void DynamixelPro2::begin(Stream &serial){

  _software_serial = &serial;  // Set a reference to a specified Stream object (Hard or Soft Serial)
}

void DynamixelPro2::end(){

#if defined(__AVR_ATmega32U4__) || defined(__MK20DX128__) || defined(__AVR_ATmega2560__)
  Serial1.end();
#else
  Serial.end();
#endif
}

void DynamixelPro2::initialization(){
  this->_status_return_value = READ;
  this->_set_direction_pin(SERVO_CONTROL_PIN);
  this->switching_operating_mode(this->MANUAL_MODE);
  for (int i = 0; i < 3; i++){
    this->write_goal_position(i, this->_home_position [i]);
  }

  this->write_goal_position(3, this->_GRIPPER_OPEN [0]);
  this->write_goal_position(4, this->_GRIPPER_OPEN [1]);
}

void DynamixelPro2::_set_direction_pin(unsigned char D_Pin){

  Direction_Pin = D_Pin;
  pinMode(Direction_Pin, OUTPUT);

}

void DynamixelPro2::move_left(){
  int current_pos = this->read_current_position(0);

  this->write_goal_position(0, current_pos + 45);

}

void DynamixelPro2::move_right(){
  int current_pos = this->read_current_position(0);

  this->write_goal_position(0, current_pos - 45);
}

void DynamixelPro2::move_up(){
  int Pos = this->read_current_position(1);
  if (Pos < 3072){
    this->write_goal_position(1, Pos + 45);
  } else{
    this->write_goal_position(1, Pos - 45);
  }
}

void DynamixelPro2::move_down(){

  int Pos = this->read_current_position(1);

  if (Pos > 1024){
    this->write_goal_position(1, Pos - 50);
  } else{
    this->write_goal_position(1, Pos + 50);
  }
}

void DynamixelPro2::use_gripper(){
  int pos [2];

  pos [0] = this->read_current_position(3);   // Reads the position of servo 4 
  pos [1] = this->read_current_position(4);   // Reads the position of servo 5

  if ((abs(pos [0] - _GRIPPER_OPEN [0]) < 10) || (abs(pos [1] - _GRIPPER_OPEN [1]) < 10)){
    this->write_goal_position(3, this->_GRIPPER_CLOSE [0]);
    this->write_goal_position(4, this->_GRIPPER_CLOSE [1]);
  } else{
    this->write_goal_position(3, this->_GRIPPER_OPEN [0]);
    this->write_goal_position(4, this->_GRIPPER_OPEN [1]);
  }
}

void DynamixelPro2::write_holding_torque(bool state){

  unsigned char arr [1] = {state};
  for (int i = 0; i < 5; i++){
    this->_write_to_servo_id(_SERVO_HEX_ID [i], 0x40, arr, 1);
  }
}

void DynamixelPro2::write_goal_position(int servo_id, unsigned int pos){

  pos %= 4096;
  unsigned char ID = this->_SERVO_HEX_ID [servo_id];

  unsigned char arr[] = {
    (pos & 0xFF),
    (pos & 0xFF00) >> 8,
    (pos & 0xFF0000) >> 16,
    (pos & 0xFF000000) >> 24
  };

  this->_write_to_servo_id(ID, 0x74, arr, 4);
}

void DynamixelPro2::_write_profile_acceleration(int servo_id, unsigned int pac){

  pac %= 32767;

  unsigned char arr[] = {
    (pac & 0xFF),
    (pac & 0xFF00) >> 8,
    (pac & 0xFF0000) >> 16,
    (pac & 0xFF000000) >> 24
  };

  this->_write_to_servo_id(servo_id, 0x6C, arr, 4);
}

void DynamixelPro2::_write_profile_velocity(int servo_id, unsigned int pvl){

  pvl %= 1023;

  unsigned char arr[] = {
    (pvl & 0xFF),
    (pvl & 0xFF00) >> 8,
    (pvl & 0xFF0000) >> 16,
    (pvl & 0xFF000000) >> 24
  };

  this->_write_to_servo_id(servo_id, 0x70, arr, 4);
}

void DynamixelPro2::_write_to_servo_id(unsigned char ID, unsigned short addr, unsigned char *arr, int n){

  n += 5;

  _instruction_packet_array [0] = ID;
  _instruction_packet_array [1] = (n & 0xFF); //length
  _instruction_packet_array [2] = (n & 0xFF00) >> 8; //length
  _instruction_packet_array [3] = 0x03; //Instruction
  _instruction_packet_array [4] = (addr & 0xFF); //address
  _instruction_packet_array [5] = (addr & 0xFF00) >> 8; //address

  for (int i = 0; i < n - 5; i++){
    _instruction_packet_array [i + 6] = arr [i];
  }

  _transmit_instruction_packet(n);

}

void DynamixelPro2::write_operation_mode(unsigned char ID, unsigned short modeSel){

  unsigned char arr [1] = {modeSel};

  _write_to_servo_id(ID, 0xB, arr, 1);

}

void DynamixelPro2::writePWM(int servo_id, int PWM){

  unsigned char ID = _SERVO_HEX_ID[servo_id];

  if(PWM > 855){
    PWM = 855;
  }else if(PWM < -855){
    PWM = -855;
    }
  unsigned char arr[] = {
    (PWM & 0xFF),
    (PWM & 0xFF00) >> 8,
    (PWM & 0xFF0000) >> 16,
    (PWM & 0xFF000000) >> 24
  };

  _write_to_servo_id(ID, 0x64, arr, 4);
  
  Serial.println(PWM);

}

int DynamixelPro2::read_current_position(int servo_id){

  unsigned char ID = this->_SERVO_HEX_ID [servo_id];

  this->_read_from_servo_id(ID, 0x84, 4);     //Read from adress 0x84 (Present Position), byte size 4
  this->_get_parameters();           //Filters parameters from ReturnPacket

  int sum = (_data [2] << 8) | _data [1];     //Converting two information bytes into a integer (position data)
  return (sum);
}

int DynamixelPro2::read_current_velocity(int servo_id){

  unsigned char ID = this->_SERVO_HEX_ID [servo_id];

  this->_read_from_servo_id(ID, 0x80, 4);       //Read from adress 0x80 (Present Velocity), byte size 4
  this->_get_parameters();             //Filters parameters from ReturnPacket

  int sum = (_data [2] << 8) | _data [1];       //Converting two information bytes into a integer (position data)
  return (sum);
}

void DynamixelPro2::switching_operating_mode(int modeChoice){

  for (int i = 0; i < 5; ++i){

    if (modeChoice == 1){ // mode of selection (Posistion)
      unsigned int current_pos = read_current_position(_SERVO_HEX_ID [i]);
      write_holding_torque(false);
      write_operation_mode(_SERVO_HEX_ID [i], 0x03);
      write_holding_torque(true);
      this->_change_parameters(5, 5);
      write_goal_position(_SERVO_HEX_ID [i], current_pos + 25);

    } else if ((modeChoice == 2) && (i < 3)){ // mode of selection (PWM)
      int pwm = readPWM(_SERVO_HEX_ID [i]);
      Serial.println("Switching modes");
      write_holding_torque(false);
      Serial.println("Modes Switched");
      write_operation_mode(_SERVO_HEX_ID [i], 0x10);
      write_holding_torque(true);
      writePWM(_SERVO_HEX_ID [i], pwm + 3);

    }
  }
  for (int i = 3; i < 5; i++) {
	  this->_write_profile_acceleration(_SERVO_HEX_ID[i], 25);
	  this->_write_profile_velocity(_SERVO_HEX_ID[i], 25);
  }
}

float DynamixelPro2::readPWM(unsigned char ID){

  _read_from_servo_id(ID, 0x7C, 4);                   //Read from adress 0x7E (Present Load), byte size 4 (should be 2?)
  _get_parameters();                      //Filters parameters from ReturnPacket

  float sum;
  sum = (_data [2] << 8) | _data [1];       //Converting two information bytes into a float (load data)

  return sum;
}

void DynamixelPro2::_read_from_servo_id(unsigned char ID, unsigned short addr, int n){

  n += 3;
  this->_instruction_packet_array [0] = ID;
  this->_instruction_packet_array [1] = (n & 0xFF);         //length of packet
  this->_instruction_packet_array [2] = (n & 0xFF00) >> 8;      //length of packet
  this->_instruction_packet_array [3] = 0x02;           //Instruction
  this->_instruction_packet_array [4] = (addr & 0xFF);        //address
  this->_instruction_packet_array [5] = (addr & 0xFF00) >> 8;   //address
  this->_instruction_packet_array [6] = ((n - 3) & 0xFF);     //data length
  this->_instruction_packet_array [7] = ((n - 3) & 0xFF00) >> 8;    // data length

  this->_transmit_instruction_packet(n);
  this->_read_return_packet();
}

void DynamixelPro2::_get_parameters(void){

  int j = 0;
  for (int i = 0; i < 100; i++){

    if (this->_return_packet [i] == 0x55 && this->_return_packet [i - 1] == 0 && this->_return_packet [i + 1] == 0){  //Filtering the parameters from the returnpacket, by searching for the instruction (0x00, 0x55, 0x00)

      this->_data [j] = this->_return_packet [i - 3];             //Saving ID

      this->_data [j + 1] = this->_return_packet [i + 2];           //Saving parameter bytes
      this->_data [j + 2] = this->_return_packet [i + 3];           //Saving parameter bytes

      j += 3;
    }
  }
}

void DynamixelPro2::_read_return_packet(void){

  int i = 0;
  while (this->_software_serial->available() > 0){         //Read information when available
    int incomingbyte = this->_software_serial->read();     //Save incomingbyte

    this->_return_packet [i] = incomingbyte;        //Save data in ReturnPacket array
    i++;
  }
}

void DynamixelPro2::_change_parameters(int vel, int acc){
  
  for (int i = 0; i < 5; i++){
    this->_write_profile_acceleration(this->_SERVO_HEX_ID [i], acc);		// Set the Profile acceleration.(MAX 32767)
    this->_write_profile_velocity(this->_SERVO_HEX_ID [i], vel);			// Set the Profile velocity. (MAX 1023)
  }
}

String DynamixelPro2::get_instruction(int mode, String gesture){

  if (mode == this->PRE_SET_MODE){
    if (gesture == this->_FIST){
      return this->GRIPPER;
    } else if (gesture == this->_FINGERS_SPREAD){
      return this->EXTENDED;
    } else if (gesture == this->_WAVE_IN){
      return this->TO_USER;
    } else if (gesture == this->_WAVE_OUT){
      return this->HOME;
    } else{
      return this->UNKNOWN_GESTURE;
    }
  } else if (mode == this->MANUAL_MODE){
    if (gesture == this->_FIST){
      return this->DOWN;
    } else if (gesture == this->_FINGERS_SPREAD){
      return this->UP;
    } else if (gesture == this->_WAVE_IN){
      return this->LEFT;
    } else if (gesture == this->_WAVE_OUT){
      return this->RIGHT;
    } else{
      return this->UNKNOWN_GESTURE;
    }
  } else{
    return this->UNKNOWN_MODE;
  }
}

void DynamixelPro2::_clear_RX_buffer(void){

  while (this->_software_serial->read() != -1);      // Clears the RX buffer;
}

void DynamixelPro2::_transmit_instruction_packet(int transLen){						// Transmit instruction packet to Dynamixel

  _clear_RX_buffer();
  if (Direction_Pin > -1){
    digitalWrite(Direction_Pin, HIGH);                                               // Set TX Buffer pin to HIGH
  }

  int arrLen = transLen + 7;

  unsigned char pt[7 + transLen];

  pt [0] = 0xFF;
  pt [1] = 0xFF;
  pt [2] = 0xFD;
  pt [3] = 0x00;
  int i;
  for (i = 0; i <= transLen; i++){
    pt [i + 4] = _instruction_packet_array [i];
  }

  unsigned short crc = _update_crc(pt, arrLen - 2);

  unsigned char CRC_L = (crc & 0x00FF);
  unsigned char CRC_H = (crc >> 8) & 0x00FF;

  i += 4;

  pt [i++] = CRC_L;
  pt [i] = CRC_H;

  for (i = 0; i < arrLen; i++){
    _software_serial->write(pt [i]);
  }

  noInterrupts();

#if defined(__AVR_ATmega32U4__) || defined(__MK20DX128__) || defined(__AVR_ATmega2560__) // Leonardo and Mega use Serial1
  if ((UCSR1A & B01100000) != B01100000){                                             // Wait for TX data to be sent
    _software_serial->flush();
  }

#elif defined(__SAM3X8E__)

  //if(USART_GetFlagStatus(USART1, USART_FLAG_TC) != RESET)
  _software_serial->flush();
  //}

#else
  if ((UCSR0A & B01100000) != B01100000){        // Wait for TX data to be sent
    _software_serial->flush();
  }

#endif

  if (Direction_Pin > -1){
    digitalWrite(Direction_Pin, LOW);           //Set TX Buffer pin to LOW after data has been sent
  }

  interrupts();

  delay(10);                    // This delay value was orginally 20
}

unsigned short DynamixelPro2::_update_crc(unsigned char *data_blk_ptr, unsigned short data_blk_size){
  unsigned short crc_accum = 0;
  unsigned short i, j;
  unsigned short crc_table [256] = {
    0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
    0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
    0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
    0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
    0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
    0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
    0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
    0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
    0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
    0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
    0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
    0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
    0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
    0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
    0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
    0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
    0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
    0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
    0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
    0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
    0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
    0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
    0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
    0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
    0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
    0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
    0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
    0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
    0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
    0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
    0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
    0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
  };

  for (j = 0; j < data_blk_size; j++){
    i = ((unsigned short)(crc_accum >> 8) ^ data_blk_ptr [j]) & 0xFF;
    crc_accum = (crc_accum << 8) ^ crc_table [i];
  }

  return crc_accum;
}
