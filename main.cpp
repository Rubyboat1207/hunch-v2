#include <iostream>
#include <cstdint>
#include <cstring>
#include <string>
#define DBG
#include "adafruitdcmotor.h"
#include "adafruitmotorhat.h"
#include "sockpp/tcp_connector.h"
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <vector>
#include <optional>
#include "packet.h"
#include "state_machine.h"
#include <thread>
#include <mutex>

using namespace std::chrono;

enum class LogLevel {
    INFO,
    WARNING,
    ERR
};

const RobotState defaultState = RobotState::READ_MESSAGES;

static RobotState state = RobotState::LOADING;
static std::vector<SendableData> write_queue = std::vector<SendableData>();
static std::vector<std::string> log_queue = std::vector<std::string>();
static HunchPacket processing_packet;
static sockpp::tcp_connector connection;
AdafruitMotorHAT hat;
std::string host = "10.9.11.26";
int port = 5000;
int left_motor_port = 2;
int right_motor_port = 1;
float heartbeat_freq = 2;
float heartbeat_timeout = 15;
long last_sent_heartbeat = 0;
long last_received_heartbeat = 0;

std::mutex write_queue_mutex;
std::mutex connection_write_mutex;

void update_last_heartbeat_time() {
    last_sent_heartbeat = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

void add_to_write_queue(const SendableData& data) {
    std::lock_guard<std::mutex> guard(write_queue_mutex);
    write_queue.push_back(data);
}

void log(LogLevel level, std::string message) {
    std::string prefix = "";
    switch (level) {
        case LogLevel::INFO: prefix = "[INFO] "; break;
        case LogLevel::WARNING: prefix = "[WARNING] "; break;
        case LogLevel::ERR: prefix = "[ERROR] "; break;
    }
    log_queue.push_back(prefix + message);
    std::cout << prefix << message << std::endl;
}

float mapValue(float value, float inputMin, float inputMax, float outputMin, float outputMax) {
    // Scale the value from input range to output range
    float scaledValue = (value - inputMin) / (inputMax - inputMin);
    return outputMin + scaledValue * (outputMax - outputMin);
}

void change_state(RobotState new_state, bool should_log=true) {
    state = new_state;
    // std::cout << "setting state to: " << state_to_string(state) << std::endl;
    if(should_log) {
        log(LogLevel::INFO, "Setting state to: " + state_to_string(state));
    }
}

void sm_attempt_connection() {
    auto result = connection.connect(host, port, 10s);

    if(result) {
        connection.read_timeout(5s);
        change_state(RobotState::ACQUIRED_CONNECTION);
    }
}

void sm_load() {
    sockpp::initialize();

    change_state(RobotState::AWAITING_CONNECTION, false);
}

void sm_on_connected() {
    log(LogLevel::INFO, "Connected to server successfully!");

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
   processing_packet = HunchPacket::decode(buffer);
   std::cout << processing_packet << std::endl;
}

std::optional<cv::Mat> take_image() {
    cv::VideoCapture cap(0);
    if(!cap.isOpened()) {
        log(LogLevel::ERR, "Camera failed to open.");
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
        log(LogLevel::ERR, "Frame was empty!");
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
            log(LogLevel::ERR, "Encoding failed.");
            return;
        }
    }catch(std::exception ex) {
        log(LogLevel::ERR, "Encoding failed catastrophically.");
        return;
    }
    
    HunchPacket* packet = new HunchPacket();
    packet->u = jpg.size();
    packet->flags = ClientFlags::SENDING_PICTURE;
    uint8_t* data = new uint8_t[jpg.size()];
    memcpy(data, reinterpret_cast<uint8_t*>(jpg.data()), jpg.size());
    add_to_write_queue(SendableData(packet, std::make_pair(data, jpg.size())));
    change_state(RobotState::HANDLE_MESSAGE);
}

void updateMotor(int slot, int speed) {
    auto motor = hat.getMotor(slot);

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
	float left_speed = mapValue(processing_packet.x, -1, 1, -255, 255);
	float right_speed = mapValue(processing_packet.y, -1, 1, -255, 255);
	
	updateMotor(left_motor_port, left_speed);
	updateMotor(right_motor_port, right_speed);

    change_state(RobotState::HANDLE_MESSAGE);
}

void process_logs() {
    std::optional<std::string> message = std::nullopt;
    if(log_queue.size() > 0) {
        std::string log_message = "";
        for(auto queued_message : log_queue) {
            log_message += queued_message + "\n";
        }
        log_queue.clear();
        message = std::optional<std::string>(log_message);
    }
    if(message.has_value()) {
        auto msg = message.value();
        int req_packets = (msg.size() / 1024) + 1;
        for(int i = 0; i < req_packets; i++) {
            int start = i * 1024;
            int end = (i + 1) * 1024;
            if(end > msg.size()) {
                end = msg.size();
            }
            std::string sub = msg.substr(start, end - start);
            // check if a packet is able to be hijacked for this message
            bool found = false;
            for(auto packet : write_queue) {
                if(packet.packet.has_value() && packet.packet.value()->message[0] == '\0') {
                    std::memcpy(packet.packet.value()->message, sub.c_str(), sub.size());
                    found = true;
                    break;
                }
            }
            if(!found) {
                std::lock_guard<std::mutex> guard(write_queue_mutex);
                add_to_write_queue(SendableData(HunchPacket::ofMessage(sub)));
            }
        }
    }
}

void sm_housekeep() {
    long time_since_last_heartbeat = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count() - last_sent_heartbeat;
    if(!connection.is_open() || time_since_last_heartbeat > (heartbeat_timeout * 1000)) {
        change_state(RobotState::AWAITING_CONNECTION, false);
        return;
    }
    
    process_logs();
    std::lock_guard<std::mutex> guard(connection_write_mutex);
    for(auto packet : write_queue) {
        update_last_heartbeat_time();
        if(packet.packet.has_value()) {
            connection.write_n(reinterpret_cast<const char*>(packet.packet.value()), sizeof(HunchPacket));
            std::cout << "sending" << *packet.packet.value() << std::endl;
        }
        if(packet.extra_data.has_value()) {
            auto v = packet.extra_data.value();
            std::cout << "Sending " << v.second << " extra bytes of extra data." << std::endl;
            connection.write_n(reinterpret_cast<const char*>(v.first), v.second);
        }

        packet.clean();
    }

    write_queue.clear();
}

void sm_handle_message(int depth) {
    last_received_heartbeat = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    if((processing_packet.flags && ServerFlags::HEARTBEAT) == ServerFlags::HEARTBEAT) {
        change_state(RobotState::READ_MESSAGES);
        return;
    }
    if((processing_packet.flags && ServerFlags::PANIC_RESET) == ServerFlags::PANIC_RESET) {
        change_state(RobotState::LOADING);
        return;
    }
    if((processing_packet.flags && ServerFlags::REQUEST_IMAGE) == ServerFlags::REQUEST_IMAGE) {
        change_state(RobotState::SENDING_IMAGE);
        tick_until(RobotState::HANDLE_MESSAGE, depth);
    }

    if((processing_packet.flags && ServerFlags::DONT_INTERPRET_MOTORS) != ServerFlags::DONT_INTERPRET_MOTORS) {
        change_state(RobotState::UPDATING_MOTORS);
        tick_until(RobotState::HANDLE_MESSAGE, depth);
    }

    change_state(RobotState::READ_MESSAGES);
}

void tick_until(RobotState target, int depth) {
    if(depth > 10) {
        throw std::runtime_error("Recursion depth exceeded.");
    }
    while(state != target) {
        tick_state_machine(depth + 1);
    }
}

void tick_state_machine(int depth) {
    sm_housekeep();
    try {
        switch (state) {
            case(RobotState::LOADING): sm_load(); break;
            case(RobotState::AWAITING_CONNECTION): sm_attempt_connection(); break;
            case(RobotState::ACQUIRED_CONNECTION): sm_on_connected(); break;
            case(RobotState::READ_MESSAGES): sm_read_messages(); break;
            case(RobotState::HANDLE_MESSAGE): sm_handle_message(depth); break;
            case(RobotState::UPDATING_MOTORS): sm_update_motors(); break;
            case(RobotState::SENDING_IMAGE): sm_send_image(); break;
        }
    }catch(std::exception& ex) {
        // some horrible exception just occurred. restart.
        add_to_write_queue(SendableData(HunchPacket::ofMessage(std::string("Horrible crash just occurred, but not a segfault. ex: ") + ex.what())));
        state = RobotState::LOADING;
    }
}

void maintain_heartbeat() {
    long time_since_last_heartbeat = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count() - last_received_heartbeat;

    if(time_since_last_heartbeat > (heartbeat_freq * 1000)) {
        add_to_write_queue(SendableData(HunchPacket::ofMessage(std::string("Heartbeat missed! Sending!"))));
    }

    std::lock_guard<std::mutex> guard(connection_write_mutex);
    HunchPacket* packet = new HunchPacket();
    packet->flags = ClientFlags::HEARTBEAT;
    connection.write_n(reinterpret_cast<const char*>(packet), sizeof(HunchPacket));
}

void keep_up_heartbeat() {
    while(true) {
        if(state == RobotState::AWAITING_CONNECTION) {
            std::this_thread::sleep_for(5s);
        }
        maintain_heartbeat();
        std::this_thread::sleep_for(1s);
    }
}

int main() {
    std::thread heartbeat_thread(keep_up_heartbeat);
    while(true) {
        tick_state_machine(0);
    }
    heartbeat_thread.join();
    return 0;
}
