#include "packet.h"
#include <cstring>


HunchPacket HunchPacket::decode(const uint8_t* data) {
		HunchPacket packet;

		// Read version
		packet.version = *reinterpret_cast<const uint16_t*>(data);

		// Read x, y, u, v
		packet.x = *reinterpret_cast<const float*>(data + sizeof(uint16_t));
		packet.y = *reinterpret_cast<const float*>(data + sizeof(uint16_t) + sizeof(float) * 1);
		packet.u = *reinterpret_cast<const float*>(data + sizeof(uint16_t) + sizeof(float) * 2);
		packet.v = *reinterpret_cast<const float*>(data + sizeof(uint16_t) + sizeof(float) * 3);

		// Read flags
		packet.flags = *reinterpret_cast<const uint64_t*>(data + sizeof(uint16_t) + sizeof(float) * 4);

		// Read message
		const char* messageStart = reinterpret_cast<const char*>(data + sizeof(uint16_t) + sizeof(float) * 4 + sizeof(uint64_t));
		std::strncpy(packet.message, messageStart, sizeof(packet.message) - 1);

		// Ensure null termination
		packet.message[sizeof(packet.message) - 1] = '\0';

		return packet;
}

