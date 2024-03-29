#pragma once

#include <string>
#include <string.h>
#include <cstring>
#include <fstream>
#include <iostream>
#include <ctime>

#include "json.hpp"
#include "SerialPort.h"
#include <myo/myo.hpp>

using namespace std;


class MyoData : public myo::DeviceListener
{
public:

	MyoData();	
	int connectToMyo();
	~MyoData();

private:
	
	int returnGestureNumber(string);

	int manualMode();
	int presetMode();
	int developerMode();
	int switchModes();

	void populateJson(int, string);
	void saveJson(string);
	void saveIncJson(string);
	void jsonHandler(string, int);
	void sendToSerial(string);

	string recieveFromSerial();
	void onPose(myo::Myo*, uint64_t, myo::Pose);

	myo::Pose currentPose;
	string prev_pose;
	
	bool isUnlocked;

	int mode_type_;	
	int gesture_number_;
	int json_id_;
	bool manual_mode_state_;
	
	string output_json_;
	string current_time_;

	char* port_name_;	
	SerialPort *arduino_obj_;
	
	ofstream output_json_file;	
	ofstream inc_json_file;
	
	time_t now_construct;
	time_t now_serial_read;
};
