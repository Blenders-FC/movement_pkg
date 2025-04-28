/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#ifndef FOV_CALCULATION_H
#define FOV_CALCULATION_H

#include "movement_pkg/utils.h"


class FOVCalculation : public virtual utils
{
public:
    // Constructor
    explicit FOVCalculation(double ball_x = 0, double ball_y = 0, double head_pan = 0, double head_tilt = 0);
    // Virtual destructor for inheritance
    virtual ~FOVCalculation();

private:
    // Internal functions
    double getVerticalAngle();

    //////////// Variables ////////////
    // Default resolution: 640x480
    const double HALF_HFOV_DEF_RES = 32.935;        // 65.87°
    const double HALF_VFOV_DEF_RES = 25.915;        // 51.83°
    const double ASP_RAT_DEF_RES = 1.33333;         // 640/480
    const double INV_ASP_RAT_DEF_RES = 0.75;        // 1 / ASP_RAT
    const int CAM_HEIGHT_DEF_RES = 480;             // pixels
    const int CAM_WIDTH_DEF_RES = 640;              // pixels
    const int H_CENTER_DEF_RES = 240;               // CAM_HEIGHT / 2
    const int W_CENTER_DEF_RES = 320;               // CAM_WIDTH / 2
    // Original resolution: 1920x1080
    const double HALF_HFOV_ORIG_RES = 35.215;       // 70.43°
    const double HALF_VFOV_ORIG_RES = 21.655;       // 43.31°
    const double ASP_RAT_ORIG_RES = 1.77777;        // 1920/1080
    const double INV_ASP_RAT_ORIG_RES = 0.5626;     // 1 / ASP_RAT
    const int CAM_HEIGHT_ORIG_RES = 1080;           // pixels
    const int CAM_WIDTH_ORIG_RES = 1920;            // pixels
    const int H_CENTER_ORIG_RES = 540;              // CAM_HEIGHT / 2
    const int W_CENTER_ORIG_RES = 960;              // CAM_WIDTH / 2
    // Smallest resolution: 426x240
    const double HALF_HFOV_SMALL_RES = 35.205;      // 70.41°
    const double HALF_VFOV_SMALL_RES = 21.675;      // 43.35°
    const double ASP_RAT_SMALL_RES = 1.775;         // 426/240
    const double INV_ASP_RAT_SMALL_RES = 0.56338;   // 1 / ASP_RAT
    const int CAM_HEIGHT_SMALL_RES = 240;           // pixels
    const int CAM_WIDTH_SMALL_RES = 426;            // pixels
    const int H_CENTER_SMALL_RES = 120;              // CAM_HEIGHT / 2
    const int W_CENTER_SMALL_RES = 213;              // CAM_WIDTH / 2
    
    // Common variables
    const double CAMERA_HEIGHT_ = 0.46;
    const double HIP_PITCH_OFFSET_ = 0.12217305;    //7°

    // Head angles
    double tilt_angle;

    // Angles calculation
    double diff_x;
    double diff_y;
    double phi;
    double betha;

    // Focal lengths
    double focal_length_x;
    double focal_length_y;

protected:
    // External functions
    void calcFocalLength_y();
    void calcFocalLength_x();
    void calcPanAngle();
    void calcDistanceToTarget();

    // Write-only variables
    float ball_position_x;
    float ball_position_y;
    double current_head_pan;
    double current_head_tilt;

    // Read-only variables
    double pan_angle;
    double distance_to_target;
    
    // Blackboard
    TargetInfo ball_info;
};

#endif  // FOV_CALCULATION_H
