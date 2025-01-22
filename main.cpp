#include <iostream>
#include <cstdint>
#include <cstring>
#include "adafruitdcmotor.h"
#include "adafruitmotorhat.h"
#include "sockpp/tcp_connector.h"
#include <opencv2/opencv.hpp>
#include <vector>
#include <optional>
#include "packet.h"

static int PORT = 5000;
#define HOST "10.9.11.159"

using namespace std::chrono;

// FLAGS:
#define SEND_PICTURE 1
#define DONT_INTERPRET_MOTORS 2

AdafruitMotorHAT hat;

float mapValue(float value, float inputMin, float inputMax, float outputMin, float outputMax) {
    // Scale the value from input range to output range
    float scaledValue = (value - inputMin) / (inputMax - inputMin);
    return outputMin + scaledValue * (outputMax - outputMin);
}

static uint8_t* incoming_tcp_data;

std::optional<HunchPacket> update_tcp(sockpp::tcp_connector* conn) {
	incoming_tcp_data = nullptr;
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


int main() {
	std::cout << "expecting " << sizeof(HunchPacket) << " bytes.";

	sockpp::initialize();
	sockpp::tcp_connector conn;

	auto res = conn.connect(HOST, PORT, 10s);

	if(!res) {
		std::cerr << "Error Connecting to Host PC! (TODO: Add retrys)" << std::endl;
		return 1;
	}
	
	conn.read_timeout(5s);

	while(true) {
		auto res = update_tcp(&conn);

		if(res.has_value()) {
			std::cout << "PACKET YAY!!!" << std::endl;
			auto packet = res.value();

			if((packet.flags & SEND_PICTURE) == SEND_PICTURE) {
				std::cout << "Taking Picture!" << std::endl;
				on_take_picture(&conn);
				return 0;
			}
		}else {
			std::cout << "None packet, left beef" << std::endl;
		}

		return 0;
	}
}

void on_take_picture(sockpp::tcp_connector* conn) {
    // GStreamer pipeline for libcamera
    std::string pipeline = "libcamera-vid --width 640 --height 480 --framerate 30 --output - | decodebin ! videoconvert ! appsink";

    // Open the camera using the GStreamer pipeline
    cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);

    if (!cap.isOpened()) {
        std::cerr << "Camera Failed To Open" << std::endl;
        exit(1);
    }

    cv::Mat img;
    cap >> img; // Capture a single frame

    if (img.empty()) {
        std::cerr << "Failed to capture an image from the camera" << std::endl;
        return;
    }

    std::vector<uchar> jpg;
    bool success = cv::imencode(".jpg", img, jpg);

    if (!success) {
        std::cerr << "Failed to encode the image as JPEG" << std::endl;
        return;
    }

    // Create and populate the packet
    HunchPacket* packet = new HunchPacket();
    packet->flags = SEND_PICTURE;
    packet->x = jpg.size();

    // Ensure the size of the image is manageable
    if (jpg.size() > 8388608) { // 8MB limit
        std::cerr << "Image is too large to fit inside a float" << std::endl;
        delete packet; // Clean up memory
        return;
    }

    // Send the packet and the image data
    if (success) {
        conn->write_n(reinterpret_cast<void*>(packet), sizeof(HunchPacket));
        conn->write_n(reinterpret_cast<void*>(jpg.data()), jpg.size());
    }

    delete packet; // Clean up memory
}

void run_motors(HunchPacket* packet) {
	float left_motor = mapValue(packet->x, -1, 1, -255, 255);
	float right_motor = mapValue(packet->y, -1, 1, -255, 255);

	hat.getMotor(1)->setSpeed((int) left_motor);
	hat.getMotor(2)->setSpeed((int) right_motor);
}
