#include <iostream>
#include <cstdint>
#include <cstring>
#include "adafruitdcmotor.h"
#include "adafruitmotorhat.h"
#include "sockpp/tcp_connector.h"
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <vector>
#include <optional>
#include "packet.h"

static int PORT = 5000;
#define HOST "10.9.11.26"

using namespace std::chrono;

// FLAGS:
#define S_REQUEST_PICTURE 1
#define C_SENDING_PICTURE 1
#define S_DONT_INTERPRET_MOTORS 2
#define F_LOG 4

float mapValue(float value, float inputMin, float inputMax, float outputMin, float outputMax) {
    // Scale the value from input range to output range
    float scaledValue = (value - inputMin) / (inputMax - inputMin);
    return outputMin + scaledValue * (outputMax - outputMin);
}

static uint8_t incoming_tcp_data[sizeof(HunchPacket)];

std::optional<HunchPacket> update_tcp(sockpp::tcp_connector* conn) {
	std::cout << "reading..." << std::endl;
	auto res = conn->read_n(incoming_tcp_data, sizeof(HunchPacket));

	if(!res) {
		std::cout << "res returned null" << std::endl;
		return std::nullopt;
	}
	

	auto packet = HunchPacket::decode(incoming_tcp_data);

	return std::optional<HunchPacket>{packet};
}

void on_take_picture(sockpp::tcp_connector* conn);
void run_motors(HunchPacket* packet);

int main() {
	std::cout << "expecting " << sizeof(HunchPacket) << " bytes.";

	sockpp::initialize();
	sockpp::tcp_connector conn;
	
	auto res = conn.connect(HOST, PORT, 10s);

	while(!res) {
		res = conn.connect(HOST, PORT, 10s);	
	}

	conn.read_timeout(5s);

	while(true) {
		auto res = update_tcp(&conn);

		if(res.has_value()) {
			std::cout << "Packet Recieved" << std::endl;
			auto packet = res.value();

			if((packet.flags & S_REQUEST_PICTURE) == S_REQUEST_PICTURE) {
				std::cout << "Taking Picture!" << std::endl;
				on_take_picture(&conn);
			}

			if((packet.flags & S_DONT_INTERPRET_MOTORS) != S_DONT_INTERPRET_MOTORS) {
				run_motors(&packet);
			}
		}else {
			std::cout << "None packet, left beef" << std::endl;
		}
	}
}

void on_take_picture(sockpp::tcp_connector* conn)
{
    cv::VideoCapture cap(0);
    if(!cap.isOpened()) {
        std::cerr << "Camera Failed To Open" << std::endl;
        return;
    }

    // Optional: set desired resolution & MJPG
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));

    double width  = cap.get(cv::CAP_PROP_FRAME_WIDTH);
    double height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    std::cout << "Resolution: " << width << " x " << height << std::endl;

    cv::Mat img;
    cap >> img;  // capture one frame
    if (img.empty()) {
        std::cerr << "Captured empty frame!\n";
        return;
    }
    std::cout << "Frame is: " << img.cols << " x " << img.rows << std::endl;

    // Encode the frame as JPEG
    std::vector<uchar> jpg;
    bool success = cv::imencode(".jpg", img, jpg);
    if(!success) {
        std::cerr << "cv::imencode failed!\n";
        return;
    }
    std::cout << "JPEG size: " << jpg.size() << " bytes\n";

    // Prepare the HunchPacket (same layout as in C#)
    HunchPacket packet;
    std::memset(&packet, 0, sizeof(packet));   // zero all fields
    packet.version = 1;                        // fill what you want
    packet.x       = 0.0f;                     // not used, but set to 0
    packet.y       = 0.0f;                     
    packet.u       = static_cast<int32_t>(jpg.size()); // store JPEG size
    packet.v       = 0;                         
    packet.flags   = C_SENDING_PICTURE;

    // Send the HunchPacket header
    if(!conn->write_n(reinterpret_cast<const char*>(&packet), sizeof(packet))) {
        std::cerr << "Failed to send HunchPacket\n";
        return;
    }

    // Then send the raw JPEG data
    if(!conn->write_n(jpg.data(), jpg.size())) {
        std::cerr << "Failed to send JPEG data\n";
        return;
    }

    std::cout << "Sent image successfully.\n";
}

void run_motors(HunchPacket* packet) {
	AdafruitMotorHAT hat;

	float left_motor = mapValue(packet->x, -1, 1, -255, 255);
	float right_motor = mapValue(packet->y, -1, 1, -255, 255);
	
	auto left = hat.getMotor(1);
	auto right = hat.getMotor(2);
	
	if(left_motor != 0) {
		left->setSpeed(abs((int) left_motor));
		left->run(left_motor > 0 ? 
				AdafruitDCMotor::kForward : 
				AdafruitDCMotor::kBackward
		);
	}else {
		left->run(AdafruitDCMotor::kBrake);
	}
	if(right_motor != 0) {
		right->setSpeed(abs((int) right_motor));
		right->run(right_motor > 0 ? 
				AdafruitDCMotor::kForward : 
				AdafruitDCMotor::kBackward
		);
	}else {
		right->run(AdafruitDCMotor::kBrake);
	}
	std::cout << "lm: " << left_motor << " rm: " << right_motor << std::endl;
}
