#ifndef UTILS
#define UTILS
#include <string>

enum class LogLevel {
    VERBOSE=3,
    INFO=2,
    WARNING=1,
    ERR=0
};

float map_value(float value, float inputMin, float inputMax, float outputMin, float outputMax);
void log(LogLevel level, std::string message);
void update_motor(int slot, int speed);
void run_motors(int leftSpeed, int rightSpeed);
void run_side(bool isLeft, int speed);
int get_motor_port(bool isLeft, bool isBack);
float get_heartbeat_freq();

#endif