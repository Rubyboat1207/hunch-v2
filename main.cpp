#include <iostream>
#include "sockpp/tcp_connector.h"

static int PORT = 5000;
#define HOST "10.9.11.159"

using namespace std::chrono;

struct HunchPacket {
	uint16_t version;
	float x, y, u, v;
	uint64_t flags;
	char message[1024];

	static HunchPacket decode(char* data) {
		auto ver = (uint16_t*) data;
		auto x = (float*) data + sizeof(uint16_t);
		auto y = (float*) data + sizeof(uint16_t) + sizeof(float) * 1;
		auto u = (float*) data + sizeof(uint16_t) + sizeof(float) * 2;
		auto v = (float*) data + sizeof(uint16_t) + sizeof(float) * 3;
		auto flags = (uint64_t*) data + sizeof(uint16_t) + sizeof(float) * 4;	
		auto message = data + sizeof(uint16_t) + sizeof(float) * 4 + sizeof(uint64_t);


		auto packet = HunchPacket { 
			.version = *ver, 
			.x = *x, 
			.y = *y,
			.u = *u,
			.v = *v,
			.flags = *flags
		};

		strcpy(packet.message, message);

		return packet;
	}
};

static char* incoming_tcp_data;

bool update_tcp(sockpp::tcp_connector conn) {
	auto res = conn.read_n(incoming_tcp_data, sizeof(HunchPacket));


	if(!res) {
		return false;
	}

	auto packet = HunchPacket::decode(incoming_tcp_data);

	std::cout << packet.message << std::endl;

	return true;
}

int main() {
	sockpp::initialize();
	sockpp::tcp_connector conn;

	incoming_tcp_data = new char[sizeof(HunchPacket)];

	auto res = conn.connect(HOST, PORT, 10s);

	if(!res) {
		std::cerr << "Error Connecting to Host PC! (TODO: Add retrys)" << std::endl;
		return 1;
	}
	
	conn.read_timeout(5s);

	while(true) {
		update_tcp(conn);
	}
}
