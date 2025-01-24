#include<iostream>


#pragma pack(push, 1)
struct HunchPacket {
    uint16_t version;
    float x, y;
    int32_t u, v;
    uint64_t flags;
    char message[1024];

    static HunchPacket decode(const uint8_t* data);
};
#pragma pack(pop)

