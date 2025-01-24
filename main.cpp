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
#define SEND_PICTURE 1
#define DONT_INTERPRET_MOTORS 2

AdafruitMotorHAT hat;

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
	cv::VideoCapture cap(0);

	if(!cap.isOpened()) {
		std::cerr << "Camera Failed To Open" << std::endl;
		exit(1);
	}

	cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
	cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
	cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));

	double width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
  double height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
  std::cout << "Resolution: " << width << " x " << height << std::endl;

	cv::Mat img;

	cap >> img;

	std::cout << "frame is: " << img.cols << " x " << img.rows << std::endl;

	std::vector<uchar> jpg;
	
	bool success = cv::imencode(".jpg", img, jpg);
	
	HunchPacket* packet = new HunchPacket();
	packet->flags = SEND_PICTURE;
	packet->u = jpg.size();
	if(jpg.size() > 8388608) {
		std::cout << "Image is too large to fit inside a float" << std::endl;
	}

  std::cout << "jpg size is:" << jpg.size() << std::endl

	if(success) {
		conn->write_n(reinterpret_cast<void*>(packet), sizeof(HunchPacket));
		conn->write_n(static_cast<uchar*>(jpg.data()), jpg.size());
	}

  delete packet;
  cap.release();
}

void run_motors(HunchPacket* packet) {
	float left_motor = mapValue(packet->x, -1, 1, -255, 255);
	float right_motor = mapValue(packet->y, -1, 1, -255, 255);

	hat.getMotor(1)->setSpeed((int) left_motor);
	hat.getMotor(2)->setSpeed((int) right_motor);
}
