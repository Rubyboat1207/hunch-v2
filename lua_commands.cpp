extern "C" {
    #include "lua.h"
    #include "lauxlib.h"
    #include "lualib.h"
}
#include <string>
#include <unordered_map>
#include "utils.h"
#include "state_machine.h"
#include <chrono>
#include <thread>
#include "packet.h"

static lua_State *L;
using LuaCFunction = int (*)(lua_State* L);

static int run_motor(lua_State* L) {
    int motorPort = lua_tonumber(L, 1);
    float speed = lua_tonumber(L, 2);

    int mappedSpeed = map_value(speed, -1, 1, -255, 255);
    
    update_motor(motorPort, mappedSpeed);

    return 0;
}

static int send_image(lua_State* L) {
    change_state(RobotState::SENDING_IMAGE, "Lua requested an image.");

    return 0;
}

static int log_message(lua_State* L) {
    std::string message = lua_tostring(L, 1);
    LogLevel level = (LogLevel) lua_tonumber(L, 2);

    log(level, message);

    return 0;
}

static int sleep_ms(lua_State* L) {
    int ms = lua_tonumber(L, 1);
    float heartbeatFreq = get_heartbeat_freq() * 1000;
    int elapsed = 0;
    while (elapsed < ms) {
        int sleepDuration = std::min((int)heartbeatFreq, ms - elapsed);
        std::cout << sleepDuration << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(sleepDuration));
        auto hp = new HunchPacket();
        hp->flags = ClientFlags::HEARTBEAT;
        add_to_write_queue(SendableData(hp));
        send_enqueued_messages();
        elapsed += sleepDuration;
    }

    return 0;
}

static int run_motors_side(lua_State* L) {
    bool isLeft = lua_toboolean(L, 1);
    float speed = lua_tonumber(L, 2);

    run_side(isLeft, speed);

    return 0;
}

static int run_motors_both(lua_State* L) {
    float leftSpeed = lua_tonumber(L, 1);
    float rightSpeed = lua_tonumber(L, 2);

    run_motors(leftSpeed, rightSpeed);

    return 0;
}


void init_lua() {
    L = luaL_newstate();
    luaL_openlibs(L);

    std::unordered_map<std::string, LuaCFunction> functionMap;

    functionMap["run_motor"] = run_motor;
    functionMap["run_side"] = run_motors_side;
    functionMap["run_motors"] = run_motors_both;
    functionMap["send_image"] = send_image;
    functionMap["log_message"] = log_message;
    functionMap["sleep_ms"] = sleep_ms;

    for (const auto& pair : functionMap) {
		lua_register(L, pair.first.c_str(), pair.second);
	}
    

    lua_pushnumber(L, (int)LogLevel::INFO);
    lua_setglobal(L, "LOG_INFO");

    lua_pushnumber(L, (int)LogLevel::WARNING);
    lua_setglobal(L, "LOG_WARN");

    lua_pushnumber(L, (int)LogLevel::ERR);
    lua_setglobal(L, "LOG_ERROR");

    lua_pushboolean(L, true);
    lua_setglobal(L, "MOTORS_LEFT");

    lua_pushboolean(L, false);
    lua_setglobal(L, "MOTORS_RIGHT");
}

void run_lua_string(std::string code) {
    // std::thread luaThread([code]() {
        luaL_dostring(L, code.c_str());
    // });
    // luaThread.detach();
}