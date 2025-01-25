#include "packet.h"
#include <cstring>


HunchPacket HunchPacket::decode(const uint8_t* data) {
	HunchPacket packet;

	decodeTo(data, &packet);

	return packet;
}

void HunchPacket::decodeTo(const uint8_t *data, HunchPacket *dest)
{
	dest->version = *reinterpret_cast<const uint16_t*>(data);

	dest->x = *reinterpret_cast<const float*>(data + sizeof(uint16_t));
	dest->y = *reinterpret_cast<const float*>(data + sizeof(uint16_t) + sizeof(float) * 1);
	dest->u = *reinterpret_cast<const int32_t*>(data + sizeof(uint16_t) + sizeof(float) * 2);
	dest->v = *reinterpret_cast<const int32_t*>(data + sizeof(uint16_t) + sizeof(float) * 2 + sizeof(int32_t));

	dest->flags = *reinterpret_cast<const uint64_t*>(data + sizeof(uint16_t) + sizeof(float) * 2 + sizeof(int32_t) * 2);

	const char* messageStart = reinterpret_cast<const char*>(data + sizeof(uint16_t) + sizeof(float) * 2 + sizeof(int32_t) * 2 + sizeof(uint64_t));
	std::strncpy(dest->message, messageStart, sizeof(dest->message) - 1);

	dest->message[sizeof(dest->message) - 1] = '\0';
}

HunchPacket *HunchPacket::ofMessage(const char* message)
{
  auto packet = new HunchPacket();

  std::strncpy(packet->message, message, 1);
  packet->flags = ClientFlags::LOG;

  return packet;
}
