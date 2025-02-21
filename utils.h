#ifndef UTILS
#define UTILS
#include <string>

enum class LogLevel {
    INFO=2,
    WARNING=1,
    ERR=0
};

float mapValue(float value, float inputMin, float inputMax, float outputMin, float outputMax);
void log(LogLevel level, std::string message);
void updateMotor(int slot, int speed);

#endif