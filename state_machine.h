#ifndef STATE_MACHINE
#define STATE_MACHINE
#include <string>

enum class RobotState {
    LOADING,
    AWAITING_CONNECTION,
    ACQUIRED_CONNECTION,
    READ_MESSAGES,
    UPDATING_MOTORS,
    SENDING_IMAGE,
    HANDLE_MESSAGE
};

std::string state_to_string(RobotState state);

void tick_state_machine(int depth=0);
void tick_until(RobotState target, int depth);
void change_state(RobotState new_state,  std::string reason, bool should_log=true);

#endif