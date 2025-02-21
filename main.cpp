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
#include "utils.h"
#include "lua_commands.hpp"

using namespace std::chrono;

const RobotState defaultState = RobotState::READ_MESSAGES;

static RobotState state = RobotState::LOADING;
static std::vector<SendableData> write_queue = std::vector<SendableData>();
static std::vector<std::string> log_queue = std::vector<std::string>();
static HunchPacket processing_packet;
static sockpp::tcp_connector connection;
AdafruitMotorHAT hat;
static std::string lua;
#ifdef HOST
std::string host = HOST;
#else
std::string host = "10.9.11.26";
#endif
int port = 5000;
int lf_port = 1;
int lb_port = 3;
int rf_port = 2;
int rb_port = 4;
float heartbeat_freq = 2;
float heartbeat_timeout = 5;
long last_sent_heartbeat = 0;
long last_received_heartbeat = 0;
long connected_time = 0;
int loglevel = 2;
int outputToConsole = 1;

std::mutex write_queue_mutex;
std::mutex connection_write_mutex;

#define CONFIG_HOST 0
#define CONFIG_PORT 1
#define CONFIG_LF_MOTOR_PORT 2
#define CONFIG_LB_MOTOR_PORT 8
#define CONFIG_RF_MOTOR_PORT 3
#define CONFIG_RB_MOTOR_PORT 9
#define CONFIG_HEARTBEAT_FREQ 4
#define CONFIG_HEARTBEAT_TIMEOUT 5
#define CONFIG_STDOUT 6
#define CONFIG_LOG_LEVEL 7

void set_config(int config, std::string value);

void update_last_heartbeat_time() {
    last_sent_heartbeat = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

void add_to_write_queue(const SendableData& data) {
    std::lock_guard<std::mutex> guard(write_queue_mutex);
    write_queue.push_back(data);
}

void log(LogLevel level, std::string message) {
    if(loglevel < (int) level) {
        return;
    }
    std::string prefix = "";
    switch (level) {
        case LogLevel::INFO: prefix = "[INFO] "; break;
        case LogLevel::WARNING: prefix = "[WARNING] "; break;
        case LogLevel::ERR: prefix = "[ERROR] "; break;
    }
    log_queue.push_back(prefix + message);
    if(outputToConsole) {
        std::cout << prefix << message << std::endl;
    }
}

void change_state(RobotState new_state, std::string reason, bool should_log) {
    state = new_state;
    // std::cout << "setting state to: " << state_to_string(state) << std::endl;
    if(should_log) {
        log(LogLevel::INFO, "Setting state to: " + state_to_string(state) + " Reason: " + reason);
    }
}

void sm_attempt_connection() {
    auto result = connection.connect(host, port, 10s);

    if(result || result.error().value() == 10035) {
        change_state(RobotState::ACQUIRED_CONNECTION, "Connected to server.");
        connected_time = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    }
}

void sm_load() {
    sockpp::initialize();

    change_state(RobotState::AWAITING_CONNECTION, "Loaded successfully.");
}

void sm_on_connected() {
    log(LogLevel::INFO, "Connected to server successfully!");

    change_state(RobotState::READ_MESSAGES, "Initial connection successful.");
    update_last_heartbeat_time();
}

void sm_read_messages() {
    uint8_t buffer[sizeof(HunchPacket)];
    std::memset(&buffer, 0, sizeof(buffer));
    
	auto res = connection.read_n(buffer, sizeof(HunchPacket));

	if(!res) {
		return;
	}

   change_state(RobotState::HANDLE_MESSAGE, "Read message from server.");
   processing_packet = HunchPacket::decode(buffer);
//    std::cout << processing_packet << std::endl;
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
    change_state(RobotState::HANDLE_MESSAGE, "Sent image to server.");
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
	
	updateMotor(lb_port, left_speed);
	updateMotor(lf_port, -left_speed);
	updateMotor(rb_port, right_speed);
	updateMotor(rf_port, -right_speed);

    change_state(RobotState::HANDLE_MESSAGE, "Updated motors.");
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
                add_to_write_queue(SendableData(HunchPacket::ofMessage(sub)));
            }
        }
    }
}

void sm_housekeep() {
    if(state == RobotState::LOADING || state == RobotState::AWAITING_CONNECTION) {
        return;
    }
    long time_since_last_heartbeat = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count() - last_sent_heartbeat;
    long time_since_connected = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count() - connected_time;
    if(!connection.is_open() || time_since_last_heartbeat > (heartbeat_timeout * 1000) && time_since_connected > 1000) {
        change_state(RobotState::AWAITING_CONNECTION, "Connection lost.");
        return;
    }
    
    process_logs();
    std::lock_guard<std::mutex> guard(connection_write_mutex);
    for(auto packet : write_queue) {
        update_last_heartbeat_time();
        if(packet.packet.has_value()) {
            connection.write_n(reinterpret_cast<const char*>(packet.packet.value()), sizeof(HunchPacket));
            // std::cout << "sending" << *packet.packet.value() << std::endl;
        }
        if(packet.extra_data.has_value()) {
            auto v = packet.extra_data.value();
            // std::cout << "Sending " << v.second << " extra bytes of extra data." << std::endl;
            connection.write_n(reinterpret_cast<const char*>(v.first), v.second);
        }

        packet.clean();
    }

    write_queue.clear();
}

void sm_handle_message(int depth) {
    last_received_heartbeat = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    if((processing_packet.flags & ServerFlags::HEARTBEAT) == ServerFlags::HEARTBEAT) {
        change_state(RobotState::READ_MESSAGES, "Got heartbeat from server.");
        // std::cout << "got heartbeat from server" << std::endl;
        return;
    }
    if((processing_packet.flags & ServerFlags::PANIC_RESET) == ServerFlags::PANIC_RESET) {
        change_state(RobotState::LOADING, "Server requested a panic reset.");
        return;
    }
    if((processing_packet.flags & ServerFlags::REQUEST_IMAGE) == ServerFlags::REQUEST_IMAGE) {
        change_state(RobotState::SENDING_IMAGE, "Server requested an image.");
        tick_until(RobotState::HANDLE_MESSAGE, depth);
    }

    if((processing_packet.flags & ServerFlags::DONT_INTERPRET_MOTORS) != ServerFlags::DONT_INTERPRET_MOTORS) {
        change_state(RobotState::UPDATING_MOTORS, "Server requested motor updates.");
        tick_until(RobotState::HANDLE_MESSAGE, depth);
    }

    if((processing_packet.flags & ServerFlags::ADJUST_CONFIG) == ServerFlags::ADJUST_CONFIG) {
        std::string value = std::string(processing_packet.message);
        
        try {
            set_config(processing_packet.u, value);

        }catch(std::exception ex) {
            log(LogLevel::ERR, "Failed to adjust config");
        }
        change_state(RobotState::HANDLE_MESSAGE, "Config adjusted. Returning to message handling.");
    }
    // this needs to run before lua_done check. If re-ordering, make sure they stay in this order.
    if((processing_packet.flags & ServerFlags::CONTAINS_LUA) == ServerFlags::CONTAINS_LUA) {
        lua += std::string(processing_packet.message);
    }
    if((processing_packet.flags & ServerFlags::LUA_DONE) == ServerFlags::LUA_DONE) {
        log(LogLevel::INFO, "Running Lua code.");
        run_lua_string(lua);
        lua = "";
    }

    change_state(RobotState::READ_MESSAGES, "message handled.");
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
    long time = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    long last_sent_hb_time = time - last_sent_heartbeat;
    long last_received_hb_time = time - last_received_heartbeat;

    if(last_received_hb_time > (heartbeat_timeout * 1000)) {
        // std::cout << "Heartbeat timeout!" << std::endl;
        change_state(RobotState::AWAITING_CONNECTION, "Heartbeat timeout.");
        return;
    }

    if(last_sent_hb_time > (heartbeat_freq * 1000)) {
        add_to_write_queue(SendableData(HunchPacket::ofMessage(std::string("Heartbeat missed! Sending!"))));
    }

    std::lock_guard<std::mutex> guard(connection_write_mutex);
    HunchPacket* packet = new HunchPacket();
    packet->flags = ClientFlags::HEARTBEAT;
    connection.write_n(reinterpret_cast<const char*>(packet), sizeof(HunchPacket));
    delete packet;
}

void keep_up_heartbeat() {
    while(true) {
        if(state == RobotState::AWAITING_CONNECTION || state == RobotState::LOADING) {
            std::this_thread::sleep_for(5s);
            continue;
        }
        maintain_heartbeat();
        std::this_thread::sleep_for(5s);
    }
}

void parse_arguments(int argc, char** argv) {
    for(int i = 0; i < argc; i++) {
        std::string arg = argv[i];
        if(arg == "--host" && i + 1 < argc) {
            set_config(CONFIG_HOST, argv[++i]);
        } else if(arg == "--port" && i + 1 < argc) {
            set_config(CONFIG_PORT, argv[++i]);
        } else if(arg == "--lf_motor_port" && i + 1 < argc) {
            set_config(CONFIG_LF_MOTOR_PORT, argv[++i]);
        } else if(arg == "--rf_motor_port" && i + 1 < argc) {
            set_config(CONFIG_RF_MOTOR_PORT, argv[++i]);
        } else if(arg == "--lb_motor_port" && i + 1 < argc) {
            set_config(CONFIG_LF_MOTOR_PORT, argv[++i]);
        } else if(arg == "--rb_motor_port" && i + 1 < argc) {
            set_config(CONFIG_RF_MOTOR_PORT, argv[++i]);
        } else if(arg == "--heartbeat_freq" && i + 1 < argc) {
            set_config(CONFIG_HEARTBEAT_FREQ, argv[++i]);
        } else if(arg == "--heartbeat_timeout" && i + 1 < argc) {
            set_config(CONFIG_HEARTBEAT_TIMEOUT, argv[++i]);
        } else if(arg == "--stdout" && i + 1 < argc) {
            set_config(CONFIG_STDOUT, argv[++i]);
        } else if(arg == "--log-level" && i + 1 < argc) {
            set_config(CONFIG_LOG_LEVEL, argv[++i]);
        }
    }
}

void set_config(int config, std::string value) {
    switch(config) {
        case 0: host = value; break;
        case 1: port = std::stoi(value); break;
        case 2: lf_port = std::stoi(value); break;
        case 3: rf_port = std::stoi(value); break;
        case 4: heartbeat_freq = std::stof(value); break;
        case 5: heartbeat_timeout = std::stof(value); break;
        case 6: outputToConsole = std::stoi(value); break;
        case 7: loglevel = std::stoi(value); break;
        case 8: lb_port = std::stoi(value); break;
        case 9: rb_port = std::stoi(value); break;
    }
}

int main(int arc, char** argv) {
    parse_arguments(arc, argv);
    init_lua();
    std::cout << "Hello Hunch! Connecting to '" << host << "'" << std::endl;
    std::thread heartbeat_thread(keep_up_heartbeat);
    cv::VideoCapture cap(0);
    if(!cap.isOpened()) {
        log(LogLevel::WARNING, "Couldn't find a camera. Continuing without.");
    }
    cap.release();
    while(true) {
        tick_state_machine(0);
    }
    heartbeat_thread.join();
    return 0;
}
