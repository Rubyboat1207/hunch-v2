#include <iostream>
#include <cstdint>
#include <cstring>
#include <string>
#include "adafruitdcmotor.h"
#include "adafruitmotorhat.h"
#include "sockpp/tcp_connector.h"
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <vector>
#include <optional>
#include "packet.h"

using namespace std::chrono;

enum class RobotState {
    LOADING,
    AWAITING_CONNECTION,
    ACQUIRED_CONNECTION,
    READ_MESSAGES,
    UPDATING_MOTORS,
    SENDING_IMAGE,
    HOUSEKEEPING,
    HANDLE_MESSAGE
};

std::string state_to_string(RobotState state) {
    switch (state) {
        case RobotState::LOADING: return "LOADING";
        case RobotState::AWAITING_CONNECTION: return "AWAITING_CONNECTION";
        case RobotState::ACQUIRED_CONNECTION: return "ACQUIRED_CONNECTION";
        case RobotState::READ_MESSAGES: return "READ_MESSAGES";
        case RobotState::UPDATING_MOTORS: return "UPDATING_MOTORS";
        case RobotState::SENDING_IMAGE: return "SENDING_IMAGE";
        case RobotState::HOUSEKEEPING: return "HOUSEKEEPING";
        case RobotState::HANDLE_MESSAGE: return "HANDLE_MESSAGE";
        default: return "UNKNOWN_STATE";
    }
}

struct SendableData {
    std::optional<HunchPacket*> packet;
    std::optional<std::pair<uint8_t*, int>> extra_data;

    SendableData(HunchPacket* hp) {
        packet = std::optional<HunchPacket*>(hp);
    }
    SendableData(std::pair<uint8_t*, int> data) {
        extra_data = std::optional<std::pair<uint8_t*, int>>(data);
    }
    SendableData(HunchPacket* hp, std::pair<uint8_t*, int> data) {
        packet = std::optional<HunchPacket*>(hp);
        extra_data = std::optional<std::pair<uint8_t*, int>>(data);
    }
};

const RobotState defaultState = RobotState::READ_MESSAGES;

static RobotState state = RobotState::LOADING;
static std::vector<SendableData> write_queue = std::vector<SendableData>();
static HunchPacket* processing_packet = nullptr;
static sockpp::tcp_connector connection;
AdafruitMotorHAT hat;
std::string host = "10.9.11.26";
int port = 5000;
int left_motor_port;
int right_motor_port;

void tick_state_machine();

float mapValue(float value, float inputMin, float inputMax, float outputMin, float outputMax) {
    // Scale the value from input range to output range
    float scaledValue = (value - inputMin) / (inputMax - inputMin);
    return outputMin + scaledValue * (outputMax - outputMin);
}

void change_state(RobotState new_state) {
    state = new_state;
    std::cout << "setting state to: " << state_to_string(state) << std::endl;
}

void sm_attempt_connection() {
    auto result = connection.connect(host, port, 10s);

    if(result) {
        connection.read_timeout(5s);
        change_state(RobotState::ACQUIRED_CONNECTION);
    }
}

void sm_load() {
    if(processing_packet != nullptr) {
        delete processing_packet;
        processing_packet = nullptr;
    }
    sockpp::initialize();

    change_state(RobotState::AWAITING_CONNECTION);
}

void sm_on_connected() {
    write_queue.push_back(SendableData(HunchPacket::ofMessage("Connected to server successfully!")));

    change_state(RobotState::READ_MESSAGES);
}

void sm_read_messages() {
    uint8_t buffer[sizeof(HunchPacket)];
    std::memset(&buffer, 0, sizeof(buffer));
    
	auto res = connection.read_n(buffer, sizeof(HunchPacket));

	if(!res) {
		return;
	}

   change_state(RobotState::HANDLE_MESSAGE);
   processing_packet = new HunchPacket(buffer);
   std::cout << *processing_packet << std::endl;
}

std::optional<cv::Mat> take_image() {
    cv::VideoCapture cap(0);
    if(!cap.isOpened()) {
        write_queue.push_back(SendableData(HunchPacket::ofMessage("Camera failed to open.")));
        return std::nullopt;
    }

    cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));

    double width  = cap.get(cv::CAP_PROP_FRAME_WIDTH);
    double height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);

    cv::Mat img;
    cap >> img;
    if (img.empty()) {
        write_queue.push_back(SendableData(HunchPacket::ofMessage("Frame was empty!")));
        return std::nullopt;
    }

    return std::optional<cv::Mat>(img);
}

void sm_send_image() {
    auto mat = take_image();

    if(!mat.has_value()) {
        return;
    }
    
    std::vector<uchar> jpg;
    try {
        bool success = cv::imencode(".jpg", mat.value(), jpg);
        if(!success) {
            write_queue.push_back(SendableData(HunchPacket::ofMessage("Encoding failed.")));
            return;
        }
    }catch(std::exception ex) {
        write_queue.push_back(SendableData(HunchPacket::ofMessage("Encoding failed catastrophically.")));
    }
    
    HunchPacket* packet = new HunchPacket();
    packet->u = jpg.size();
    packet->flags = ClientFlags::SENDING_PICTURE;

    change_state(RobotState::HANDLE_MESSAGE);
}

void updateMotor(int slot, int speed) {
    auto motor = hat.getMotor(1);

    if(speed != 0) {
		motor->setSpeed(abs(speed));
		motor->run(speed > 0 ? 
				AdafruitDCMotor::kForward : 
				AdafruitDCMotor::kBackward
		);
	}else {
		motor->run(AdafruitDCMotor::kBrake);
	}
}

void sm_update_motors() {
	float left_speed = mapValue(processing_packet->x, -1, 1, -255, 255);
	float right_speed = mapValue(processing_packet->y, -1, 1, -255, 255);
	
	updateMotor(left_motor_port, left_speed);
	updateMotor(right_motor_port, right_speed);

    change_state(RobotState::HANDLE_MESSAGE);
}

void sm_housekeep() {
    for(auto packet : write_queue) {
        if(packet.packet.has_value()) {
            connection.write_n(reinterpret_cast<const char*>(packet.packet.value()), sizeof(HunchPacket));
            if(packet.packet.value()->message[0] != '\0') {
                std::cout << packet.packet.value()->message << std::endl;
            }
            delete packet.packet.value();
        }
        if(packet.extra_data.has_value()) {
            auto v = packet.extra_data.value();
            connection.write_n(reinterpret_cast<const char*>(v.first), v.second);
            delete v.first;
        }
    }

    write_queue.clear();
    change_state(RobotState::READ_MESSAGES);
}

void sm_handle_message() {
    if((processing_packet->flags && ServerFlags::REQUEST_IMAGE) == ServerFlags::REQUEST_IMAGE) {
        change_state(RobotState::SENDING_IMAGE);
        while(state != RobotState::HANDLE_MESSAGE) {
            tick_state_machine();
        }
    }

    if((processing_packet->flags && ServerFlags::DONT_INTERPRET_MOTORS) != ServerFlags::DONT_INTERPRET_MOTORS) {
        change_state(RobotState::UPDATING_MOTORS);
        while(state != RobotState::HANDLE_MESSAGE) {
            tick_state_machine();
        }
    }

    delete processing_packet;
    processing_packet = nullptr;
    change_state(RobotState::HOUSEKEEPING);
}

void tick_state_machine() {
    try {
        switch (state) {
            case(RobotState::LOADING): sm_load(); break;
            case(RobotState::AWAITING_CONNECTION): sm_attempt_connection(); break;
            case(RobotState::ACQUIRED_CONNECTION): sm_on_connected(); break;
            case(RobotState::READ_MESSAGES): sm_read_messages(); break;
            case(RobotState::HANDLE_MESSAGE): sm_handle_message(); break;
            case(RobotState::UPDATING_MOTORS): sm_update_motors(); break;
            case(RobotState::SENDING_IMAGE): sm_send_image(); break;
            case(RobotState::HOUSEKEEPING): sm_housekeep(); break;
        }
    }catch(...) {
        // some horrible exception just occurred. restart.
        write_queue.push_back(SendableData(HunchPacket::ofMessage("Horrible crash just occurred, but not a segfault. Good luck. Restarting.")));
        state = RobotState::LOADING;
    }
}


int main() {
    while(true) {
        tick_state_machine();
    }
}
