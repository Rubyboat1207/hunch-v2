#include<iostream>
#include<optional>
#include<cstdint>
#include<cstring>

#pragma pack(push, 1)
struct HunchPacket {
    uint16_t version;
    float x, y;
    int32_t u, v;
    uint64_t flags;
    char message[1024];

    static HunchPacket decode(const uint8_t* data);
    static HunchPacket* ofMessage(std::string message);
    inline HunchPacket() {
        this->version = 1;
        this->x = 0;
        this->y = 0;
        this->u = 0;
        this->v = 0;
        this->flags = 0;
        memset(this->message, 0, sizeof(this->message));
    }

    friend std::ostream& operator<<(std::ostream& os, const HunchPacket& packet) {
        os << "Version: " << packet.version << "\n"
           << "X: " << packet.x << "\n"
           << "Y: " << packet.y << "\n"
           << "U: " << packet.u << "\n"
           << "V: " << packet.v << "\n"
           << "Flags: " << packet.flags << "\n"
           << "Message: " << packet.message << "\n";
        return os;
    }
};
#pragma pack(pop)

struct ClientFlags {
    const static uint64_t SENDING_PICTURE = 1;
    const static uint64_t LOG = 4;
    const static uint64_t HEARTBEAT = 16;
    const static uint64_t DISABLE_HEARTBEAT = 32;
};

struct ServerFlags {
    const static uint64_t REQUEST_IMAGE = 1;
    const static uint64_t DONT_INTERPRET_MOTORS = 2;
    const static uint64_t LOG = 4;
    const static uint64_t PANIC_RESET = 8;
    const static uint64_t HEARTBEAT = 16;
    const static uint64_t DISABLE_HEARTBEAT = 32;
    const static uint64_t ADJUST_CONFIG = 64;
    const static uint64_t CONTAINS_LUA = 128;
    const static uint64_t LUA_DONE = 256;
};

struct SendableData {
    std::optional<HunchPacket*> packet;
    std::optional<std::pair<uint8_t*, int>> extra_data;

    SendableData(HunchPacket* hp);
    SendableData(std::pair<uint8_t*, int> data);
    SendableData(HunchPacket* hp, std::pair<uint8_t*, int> data);
    void clean();
};