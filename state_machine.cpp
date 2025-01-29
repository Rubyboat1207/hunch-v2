#include "state_machine.h"

std::string state_to_string(RobotState state) {
    switch (state) {
        case RobotState::LOADING: return "LOADING";
        case RobotState::AWAITING_CONNECTION: return "AWAITING_CONNECTION";
        case RobotState::ACQUIRED_CONNECTION: return "ACQUIRED_CONNECTION";
        case RobotState::READ_MESSAGES: return "READ_MESSAGES";
        case RobotState::UPDATING_MOTORS: return "UPDATING_MOTORS";
        case RobotState::SENDING_IMAGE: return "SENDING_IMAGE";
        case RobotState::HANDLE_MESSAGE: return "HANDLE_MESSAGE";
        default: return "UNKNOWN_STATE";
    }
}