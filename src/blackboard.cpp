/*
    Authors:
        Pedro Deniz
*/

#include "movement_pkg/blackboard.h"


Blackboard::Blackboard() {
    reset();
}

Blackboard::~Blackboard() {
    // Optional: Log destruction if needed
}

void Blackboard::reset() {
    ball.reset();
    left_goalpost.reset();
    right_goalpost.reset();
}

void Blackboard::setTarget(const std::string& name, const TargetInfo& info) {
    if (name == "ball") {
        ball = info;
    } else if (name == "left_goalpost") {
        left_goalpost = info;
    } else if (name == "right_goalpost") {
        right_goalpost = info;
    }
}

const TargetInfo* Blackboard::getTarget(const std::string& name) const {
    if (name == "ball") {
        return &ball;
    } else if (name == "left_goalpost") {
        return &left_goalpost;
    } else if (name == "right_goalpost") {
        return &right_goalpost;
    }
    return nullptr;
}
