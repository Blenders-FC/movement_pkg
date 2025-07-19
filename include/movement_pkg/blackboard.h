/*
    Authors:
        Pedro Deniz
*/

#ifndef BLACKBOARD_H
#define BLACKBOARD_H

#include <iostream>
#include <string>
#include <unordered_map>


// Structure to store information about any tracked object (ball, goalposts)
struct TargetInfo {
    double distance = 0.0;  // Distance from the robot
    double tilt = 0.0;      // Tilt angle
    double pan_angle = 0.0;       // Pan angle
    int turncnt =0;
    int refereeStatus=0; //still as default value
    bool middle_field_placement=false;

    void reset() {
        distance = 0.0;
        tilt = 0.0;
        pan_angle = 0.0;
        turncnt =0;
        refereeStatus =0;
        middle_field_placement=false;
    }
};

// Blackboard class to store shared information between BT nodes
class Blackboard {
public:
    Blackboard();
    ~Blackboard();

    // Reset all stored data
    void reset();

    // Setter function to update values safely
    void setTarget(const std::string& name, const TargetInfo& info);

    // Getter function to retrieve values (returns nullptr if not found)
    const TargetInfo* getTarget(const std::string& name) const;

private:
    // Main tracked targets
    TargetInfo ball;
    TargetInfo left_goalpost;
    TargetInfo right_goalpost;
    TargetInfo m_turncnt;
    TargetInfo m_refereeStatus;
    TargetInfo middle_field_placement;
};

#endif // BLACKBOARD_H
