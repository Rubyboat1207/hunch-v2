#include "packet.h"
#include <cstring>


HunchPacket HunchPacket::decode(const uint8_t* data) {
	HunchPacket packet;

	// Read version
	packet.version = *reinterpret_cast<const uint16_t*>(data);

	// Read x, y, u, v
	packet.x = *reinterpret_cast<const float*>(data + sizeof(uint16_t));
	packet.y = *reinterpret_cast<const float*>(data + sizeof(uint16_t) + sizeof(float) * 1);
	packet.u = *reinterpret_cast<const int32_t*>(data + sizeof(uint16_t) + sizeof(float) * 2);
	packet.v = *reinterpret_cast<const int32_t*>(data + sizeof(uint16_t) + sizeof(float) * 2 + sizeof(int32_t));

	// Read flags
	packet.flags = *reinterpret_cast<const uint64_t*>(data + sizeof(uint16_t) + sizeof(float) * 2 + sizeof(int32_t) * 2);

	// Read message
	const char* messageStart = reinterpret_cast<const char*>(data + sizeof(uint16_t) + sizeof(float) * 2 + sizeof(int32_t) * 2 + sizeof(uint64_t));
	std::strncpy(packet.message, messageStart, sizeof(packet.message) - 1);

	// Ensure null termination
	packet.message[sizeof(packet.message) - 1] = '\0';

	return packet;
}

HunchPacket *HunchPacket::ofMessage(std::string message)
{
  auto packet = new HunchPacket();

  std::strncpy(packet->message, message.c_str(), message.size());
  packet->flags = ClientFlags::LOG;

  return packet;
}

SendableData::SendableData(HunchPacket *hp)
{
  packet = std::optional<HunchPacket *>(hp);
}

SendableData::SendableData(std::pair<uint8_t *, int> data)
{
  extra_data = std::optional<std::pair<uint8_t *, int>>(data);
}

SendableData::SendableData(HunchPacket* hp, std::pair<uint8_t*, int> data)
{
	packet = std::optional<HunchPacket*>(hp);
	extra_data = std::optional<std::pair<uint8_t*, int>>(data);
}

void SendableData::clean()
{
  if (packet.has_value())
  {
	delete packet.value();
  }
  if (extra_data.has_value())
  {
	delete extra_data.value().first;
  }
}