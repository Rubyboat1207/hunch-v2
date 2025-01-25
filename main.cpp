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

const RobotState defaultState = RobotState::READ_MESSAGES;

static RobotState state = RobotState::LOADING;
static std::vector<HunchPacket*> write_queue = std::vector<HunchPacket*>();
static HunchPacket* processing_packet = nullptr;
static sockpp::tcp_connector connection;
AdafruitMotorHAT hat;
std::string host;
int port;
int left_motor_port;
int right_motor_port;

float mapValue(float value, float inputMin, float inputMax, float outputMin, float outputMax) {
    // Scale the value from input range to output range
    float scaledValue = (value - inputMin) / (inputMax - inputMin);
    return outputMin + scaledValue * (outputMax - outputMin);
}

void attempt_connection() {
    auto result = connection.connect(host, port, 10s);

    if(result) {
        connection.read_timeout(5s);
        state = RobotState::ACQUIRED_CONNECTION;
    }
}

void load() {
    if(processing_packet != nullptr) {
        delete processing_packet;
        processing_packet = nullptr;
    }
    sockpp::initialize();

    state = RobotState::AWAITING_CONNECTION;
}

void on_connected() {
    write_queue.push_back(HunchPacket::ofMessage("Connected to server successfully!"));

    state = RobotState::READ_MESSAGES;
}

void read_messages() {
    uint8_t buffer[sizeof(HunchPacket)];
    std::memset(&buffer, 0, sizeof(buffer));
    
	auto res = connection.read_n(buffer, sizeof(HunchPacket));

	if(!res) {
		return;
	}
}

std::optional<cv::Mat> take_image() {
    cv::VideoCapture cap(0);
    if(!cap.isOpened()) {
        write_queue.push_back(HunchPacket::ofMessage("Camera failed to open."));
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
        write_queue.push_back(HunchPacket::ofMessage("Frame was empty!"));
        return std::nullopt;
    }

    return std::optional<cv::Mat>(img);
}

void send_image() {
    auto mat = take_image();

    if(!mat.has_value()) {
        return;
    }
    
    std::vector<uchar> jpg;
    try {
        bool success = cv::imencode(".jpg", mat.value(), jpg);
        if(!success) {
            write_queue.push_back(HunchPacket::ofMessage("Encoding failed."));
            return;
        }
    }catch(std::exception ex) {
        write_queue.push_back(HunchPacket::ofMessage("Encoding failed catastrophically."));
    }
    
    HunchPacket* packet = new HunchPacket();
    packet->u = jpg.size();
    packet->flags = ClientFlags::SENDING_PICTURE;
}

void update_motors() {
	float left_speed = mapValue(processing_packet->x, -1, 1, -255, 255);
	float right_speed = mapValue(processing_packet->y, -1, 1, -255, 255);
	
	updateMotor(left_motor_port, left_speed);
	updateMotor(right_motor_port, right_speed);
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

void send_queued_packets() {
    for(auto packet : write_queue) {
        connection.write_n(reinterpret_cast<const char*>(&packet), sizeof(packet));
        delete packet;
    }

    write_queue.clear();
    state = RobotState::READ_MESSAGES;
}

void handle_message() {
    if((processing_packet->flags && ServerFlags::REQUEST_IMAGE) == ServerFlags::REQUEST_IMAGE) {
        state = RobotState::SENDING_IMAGE;
        while(state != RobotState::HANDLE_MESSAGE) {
            tick_state_machine();
        }
    }

    if((processing_packet->flags && ServerFlags::DONT_INTERPRET_MOTORS) != ServerFlags::DONT_INTERPRET_MOTORS) {
        state = RobotState::UPDATING_MOTORS;
        while(state != RobotState::HANDLE_MESSAGE) {
            tick_state_machine();
        }
    }
    
    processing_packet = nullptr;
    state = RobotState::HOUSEKEEPING;
}

void tick_state_machine() {
    try {
        switch (state) {
            case(RobotState::LOADING): load(); break;
            case(RobotState::AWAITING_CONNECTION): attempt_connection(); break;
            case(RobotState::ACQUIRED_CONNECTION): on_connected(); break;
            case(RobotState::READ_MESSAGES): read_messages(); break;
            case(RobotState::HANDLE_MESSAGE): handle_message(); break;
            case(RobotState::UPDATING_MOTORS): update_motors(); break;
            case(RobotState::SENDING_IMAGE): send_image(); break;
            case(RobotState::HOUSEKEEPING): send_queued_packets(); break;
        }
    }catch(...) {
        // some horrible exception just occurred. restart.
        write_queue.push_back(HunchPacket::ofMessage("Horrible crash just occurred, but not a segfault. Good luck"));
        state = RobotState::LOADING;
    }
}


int main() {
    while(true) {
        tick_state_machine();
    }
}