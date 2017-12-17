#include "stdafx.h"
#include "MyoData.h"

#include <array>
#include <sstream>
#include <stdexcept>
#include <Windows.h>

#include <chrono>
#include <thread>
#include <iomanip>
#include <ratio>

#define DATA_LENGTH 255
#define DATA_LENGTH_OF_1 1

#define MODE_MANUAL 1
#define MODE_PRESET 2
#define MODE_DEVEL 3
#define MODE_EXIT 4
#define MODE_QUIT 5
#define COM6 "\\\\.\\COM6"		
#define DELAY_OF_ONE_SEC std::this_thread::sleep_for(std::chrono::milliseconds(1000))
#define DELAY_OF_1500_MS std::this_thread::sleep_for(std::chrono::milliseconds(1500))

using namespace std;
using json = nlohmann::json;
using namespace std::chrono;

MyoData::MyoData() {
	manual_mode_state_ = false;
	json_id_ = 0;
	output_json_file.open("output_json_file.txt");
	inc_json_file.open("inc_json_file.txt");
	arduino_obj_ = new SerialPort(COM6);
	prev_pose = "rest";

	now_construct = time(nullptr);

	/*string mode;
	cout << "Press 'd' and 'Enter' to go into developer mode or 'n' and 'Enter' to go to normal. \n";
	cin >> mode;

	if (mode == "d") {
		int myo_mode;
		cout << "myo mode: ";
		cin >> myo_mode;

		string gesture;
		cout << "gesture: ";
		cin >> gesture;

		populateJson(myo_mode, gesture);
	}
	else
		connectToMyo();*/
}

int MyoData::connectToMyo()
{
	currentPose = myo::Pose::rest;
	myo::Hub hub("com.project.myo_project");

	cout << "\r";
	cout << "Attempting to find a Myo..." << string(65, ' ');

	int atemps = 0;
	myo::Myo* myo = hub.waitForMyo(10000);
	while (!myo && atemps < 5) {
		if (!myo) {
			throw runtime_error("Unable to find a Myo!");
			cout << atemps << "Unable to find a Myo! \n";
		}
		atemps++;
	}

	cout << "\r";
	cout << "Connected to a Myo armband!" << string(65, ' ');

	hub.addListener(this);

	while (1) {
		DELAY_OF_ONE_SEC;
		DELAY_OF_ONE_SEC;
		while (1) {
			hub.run(1000 / 25);
			mode_type_ = switchModes();

			if (mode_type_ == MODE_MANUAL || mode_type_ == MODE_PRESET || mode_type_ == MODE_DEVEL || mode_type_ == MODE_QUIT)
				break;
		}

		DELAY_OF_ONE_SEC;
		DELAY_OF_ONE_SEC;

		if (mode_type_ == MODE_MANUAL || mode_type_ == MODE_PRESET || mode_type_ == MODE_DEVEL || mode_type_ == MODE_QUIT) {

			if (mode_type_ == MODE_MANUAL) {

				while (true) {

					hub.run(1000 / 20);
					mode_type_ = manualMode();

					if (mode_type_ == MODE_EXIT)
						break;
				}
			}
			else if (mode_type_ == MODE_PRESET) {

				while (true) {
					hub.run(1000 / 20);
					mode_type_ = presetMode();

					if (mode_type_ == MODE_EXIT)
						break;
				}
			}
			else if (mode_type_ == MODE_DEVEL) {

				while (true) {
					hub.run(1000 / 20);
					mode_type_ = developerMode();

					if (mode_type_ == MODE_EXIT)
						break;
				}
			}
			else if (mode_type_ == MODE_QUIT) {
				exit(0);
			}
		}
	}
}

void MyoData::onPose(myo::Myo* myo, uint64_t timestamp, myo::Pose pose) {
	currentPose = pose;
	isUnlocked = false;

	myo->unlock(myo::Myo::unlockHold);

	myo->notifyUserAction();
}

int MyoData::returnGestureNumber(string p_inc_gesture) {

	int gesture_number_ = 0;

	if (p_inc_gesture == "fist") {
		gesture_number_ = 1;
		return gesture_number_;
	}
	else if (p_inc_gesture == "fingersSpread") {
		gesture_number_ = 2;
		return gesture_number_;
	}
	else if (p_inc_gesture == "waveIn") {
		gesture_number_ = 3;
		return gesture_number_;
	}
	else if (p_inc_gesture == "waveOut") {
		gesture_number_ = 4;
		return gesture_number_;
	}
	else if (p_inc_gesture == "doubleTap") {
		gesture_number_ = 5;
		return gesture_number_;
	}
	else if (p_inc_gesture == "rest") {
		gesture_number_ = 6;
		return gesture_number_;
	}
	else {
		gesture_number_ = 0;
		return gesture_number_;
	}
}

int MyoData::switchModes() {

	gesture_number_ = 0;

	switch (gesture_number_ = returnGestureNumber(currentPose.toString())) {
	case 1:
		std::cout << '\r';
		cout << "You choose: 'Manual Mode'" << string(65, ' ');
		return MODE_MANUAL;
		break;

	case 2:
		std::cout << '\r';
		cout << "You choose: 'Preset Mode'" << string(65, ' ');
		return MODE_PRESET;
		break;

		/*case 3:
		std::cout << '\r';
		cout << "You choose: 'Developer Mode'" << string(55, ' ');
		return MODE_DEVEL;
		break;*/

	case 5:
		std::cout << '\r';
		cout << "Quitting the program!" << string(100, ' ');
		return MODE_QUIT;
		break;

	default:
		std::cout << '\r';
		cout << "FIST: Manual Mode, " << "SPREAD: Preset Mode, " << "WAVEIN: Developer Mode, " << "DOUBLETAP: Quit." << string(10, ' ');
		break;
	}
}

int MyoData::manualMode() {

	if (prev_pose != currentPose.toString() || manual_mode_state_ == false) {
		manual_mode_state_ = true;
		prev_pose = currentPose.toString();
		gesture_number_ = 0;
		mode_type_ = MODE_MANUAL;
		switch (gesture_number_ = returnGestureNumber(currentPose.toString())) {
		case 1:
			std::cout << '\r';
			cout << "FIST: Moving end effector: DOWN" << string(65, ' ');
			populateJson(mode_type_, currentPose.toString());
			break;

		case 2:
			std::cout << '\r';
			cout << "SPREAD: Moving end effector: UP" << string(65, ' ');
			populateJson(mode_type_, currentPose.toString());
			break;

		case 3:
			std::cout << '\r';
			cout << "WAVEIN: Moving end effector: LEFT" << string(65, ' ');
			populateJson(mode_type_, currentPose.toString());
			break;

		case 4:
			std::cout << '\r';
			cout << "WAVEOUT: Moving end effector: RIGHT" << string(65, ' ');
			populateJson(mode_type_, currentPose.toString());
			break;

		case 5:
			std::cout << '\r';
			cout << "DOUBLE TAP, you are QUITTING this mode" << string(75, ' ') << "\n";
			gesture_number_ = 0;
			return MODE_EXIT;
			break;

		case 6:
			std::cout << '\r';
			cout << "You are in REST. FIST: down, SPREAD: up, WAVE IN: left, WAVE OUT: right, DOUBLE TAP: exit mode" << string(10, ' ');
			populateJson(mode_type_, currentPose.toString());
			break;

		default:
			std::cout << '\r';
			cout << "Uninterpretable gesture." << string(65, ' ');
			break;
		}
	}
}

int MyoData::presetMode() {

	gesture_number_ = 0;
	mode_type_ = MODE_PRESET;

	switch (gesture_number_ = returnGestureNumber(currentPose.toString())) {
	case 1:
		std::cout << '\r';
		cout << "Opening/Closing the gripper." << string(100, ' ');
		populateJson(mode_type_, currentPose.toString());
		break;

	case 2:
		std::cout << '\r';
		cout << "Moving to extend." << string(100, ' ');
		populateJson(mode_type_, currentPose.toString());
		break;

	case 3:
		std::cout << '\r';
		cout << "Moving to user." << string(100, ' ');
		populateJson(mode_type_, currentPose.toString());
		break;

	case 4:
		std::cout << '\r';
		cout << "Moving home." << string(100, ' ');
		populateJson(mode_type_, currentPose.toString());
		break;

	case 5:
		std::cout << '\r';
		cout << "DOUBLE TAP, you are QUITTING this mode" << string(75, ' ') << "\n";
		gesture_number_ = 0;
		return MODE_EXIT;
		break;

	case 6:
		std::cout << '\r';
		cout << "FIST: Open/Close the gripper, " << "SPREAD: Extend, " << "WAVEOUT: Home, " << "WAVEIN: Move to user, DOUBLE TAP: exit mode" << string(15, ' ');
		break;

	default:
		std::cout << '\r';
		cout << "Uninterpretable gesture." << string(65, ' ');
		break;
	}
}

int MyoData::developerMode() {
	cout << "yey you made it to devel mode";
	return MODE_EXIT;
}

void MyoData::populateJson(int p_mode, string p_gesture) {

	json_id_++;
	json pose_json_;

	pose_json_ = {
		{ "id", json_id_ },
		{ "mode", p_mode },
		{ "gesture", p_gesture }
	};
	output_json_ = pose_json_.dump();
	jsonHandler(output_json_, p_mode);

}

void MyoData::saveJson(string p_json) {

	time_t time_stamp = returnCurrTime();
	output_json_file << time_stamp << " -> " << p_json << "\n";
}

void MyoData::saveIncJson(string p_json) {

	time_t time_stamp = returnCurrTime();
	inc_json_file << time_stamp << " -> " << p_json << "\n";
}

void MyoData::jsonHandler(string output_json, int mode_type) {

	string response_from_robot = "";
	if (mode_type == MODE_MANUAL) {

		saveJson(output_json);
		sendToSerial(output_json);

	}
	else if (mode_type == MODE_PRESET) {

		saveJson(output_json);
		sendToSerial(output_json);
		DELAY_OF_1500_MS;

		bool empty_response = false;
		unsigned long start_time = std::time(nullptr);

		while ((response_from_robot == "") && ((std::time(nullptr) - start_time) < 7)) {
			cout << '\r';
			cout << "Waiting for response from the robot.." << string(65, ' ');

			response_from_robot = recieveFromSerial();

			if (response_from_robot != "") {
				stringstream response;
				response << "MyoData::jsonHandler -> " << response_from_robot << "\n";
				OutputDebugStringA(response.str().c_str());

				stringstream to_json;
				to_json << response_from_robot;

				try {
					json incoming_json = json::parse(to_json);
					OutputDebugString(L"MyoData::jsonHandler -> Successful parsing to Json \n");

					/*if (incoming_json["job status"] == "success") {
						cout << "\r";
						cout << "Response is ok! Job succeeded! \n";
						empty_response = true;
						OutputDebugString(L"MyoData::jsonHandler -> job status = success \n");
					}
					else {
						cout << '\r';
						cout << "Response is ok! Job failed! \n";
						OutputDebugString(L"MyoData::jsonHandler -> job status = failed \n");
					}*/

				}
				catch (const std::exception& e) {
					cout << "\r";
					cout << "Incomprehensible robot response \n";
					stringstream exception;
					exception << e.what() << "\n";
					OutputDebugStringA(exception.str().c_str());
					OutputDebugString(L"MyoData::jsonHandler -> Failed parsing to Json \n");
				}
				break;
			}
		}
	}
}

void MyoData::sendToSerial(string p_output_json) {

	char *output_serial_ = &p_output_json[0];

	if (arduino_obj_->isConnected()) {

		bool has_written = arduino_obj_->writeSerialPort(output_serial_, DATA_LENGTH);

		if (has_written) {

			now_serial_read = time(nullptr);

			stringstream msg;
			msg << "MyoData::sendToSerial -> " << output_serial_ << "\n";
			OutputDebugStringA(msg.str().c_str());
			//cout << "\nMyoData:sendToSerial -> " << output_serial_ << "\n";

		}
		else {
			OutputDebugString(L"MyoData::sendToSerial -> Data not sent \n");
		}
	}
}

string MyoData::recieveFromSerial() {

	char recieved_char[DATA_LENGTH_OF_1];
	string received_string = "";
	string complete_string = "";
	string complete_answer = "";

	if (arduino_obj_->isConnected()) {

		int has_read = arduino_obj_->readSerialPort(recieved_char, DATA_LENGTH_OF_1);
		received_string.assign(recieved_char, DATA_LENGTH_OF_1);

		if (received_string != "{") {
			do {
				has_read = arduino_obj_->readSerialPort(recieved_char, DATA_LENGTH_OF_1);

				if (has_read) {
					received_string.assign(recieved_char, DATA_LENGTH_OF_1);
					complete_answer.append(received_string);
				}
			} while ((recieved_char[0] != '{') && has_read);
		}

		if (received_string == "{") {

			complete_string.append(received_string);
			complete_answer.append(received_string);

			do {
				has_read = arduino_obj_->readSerialPort(recieved_char, DATA_LENGTH_OF_1);

				if (has_read) {
					received_string.assign(recieved_char, DATA_LENGTH_OF_1);
					complete_string.append(received_string);
					complete_answer.append(received_string);
				}

			} while (recieved_char[0] != '}');

			saveIncJson(complete_string);
			stringstream msg;
			msg << "MyoData::recieveFromSerial JSON -> " << complete_string << "\n";
			OutputDebugStringA(msg.str().c_str());
			//cout << "\r";
			//cout << "MyoData::recieveFromSerial JSON -> " << complete_string << "\n";
		}

		if (received_string == "}") {
			do {
				has_read = arduino_obj_->readSerialPort(recieved_char, DATA_LENGTH_OF_1);

				if (has_read) {
					received_string.assign(recieved_char, DATA_LENGTH_OF_1);
					complete_answer.append(received_string);
				}
			} while (has_read);
		}

		if (complete_answer != "" && complete_string == "") {
			stringstream msg;
			msg << "MyoData::recieveFromSerial ANSWER -> " << complete_answer << "\n";
			OutputDebugStringA(msg.str().c_str());
			//cout << "\r";
			//cout << "MyoData::recieveFromSerial ANSWER -> " << complete_answer << "\n";
		}
	}
	return complete_string;
}

time_t MyoData::returnCurrTime() {

	auto curr_time = std::chrono::system_clock::now();

	std::time_t timestamp = std::chrono::system_clock::to_time_t(curr_time);

	return timestamp;
}

MyoData::~MyoData() {
	inc_json_file.close();
	output_json_file.close();
	//myo->lock(myo::Myo::lock);
}
