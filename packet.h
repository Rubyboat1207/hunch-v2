#include<iostream>


#pragma pack(push, 1)
struct HunchPacket {
    uint16_t version;
    float x, y;
    int32_t u, v;
    uint64_t flags;
    char message[1024];

    static HunchPacket decode(const uint8_t* data);
    static HunchPacket* ofMessage(const char* message);
};
#pragma pack(pop)

struct ClientFlags {
    const static uint64_t SENDING_PICTURE = 1;
    const static uint64_t LOG = 4;
};

struct ServerFlags {
    const static uint64_t REQUEST_IMAGE = 1;
    const static uint64_t DONT_INTERPRET_MOTORS = 2;
    const static uint64_t LOG = 4;
};