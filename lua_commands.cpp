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

static lua_State *L;
using LuaCFunction = int (*)(lua_State* L);

static int run_motor(lua_State* L) {
    int motorPort = lua_tonumber(L, 1);
    float speed = lua_tonumber(L, 2);

    int mappedSpeed = mapValue(speed, -1, 1, -255, 255);
    
    updateMotor(motorPort, mappedSpeed);

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
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));

    return 0;
}

void init_lua() {
    L = luaL_newstate();
    luaL_openlibs(L);

    std::unordered_map<std::string, LuaCFunction> functionMap;

    functionMap["run_motor"] = run_motor;
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
}

void run_lua_string(std::string code) {
    luaL_dostring(L, code.c_str());
}