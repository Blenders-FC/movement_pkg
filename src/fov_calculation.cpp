/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#include <movement_pkg/fov_calculation.h>

FOVCalculation::FOVCalculation(float ball_x, float ball_y, double head_pan, double head_tilt) 
    : utils(),
    focal_length_x(0),
    focal_length_y(0),
    tilt_angle(0),
    pan_angle(0),
    distance_to_target(0),
    ball_position_x(ball_x),
    ball_position_y(ball_y),
    current_head_pan(head_pan),
    current_head_tilt(head_tilt) {}

FOVCalculation::~FOVCalculation() {}
  
double FOVCalculation::calcFocalLength_y() {
    /*
    Focal length determines how much of the FOV is captured by the camera and is responsible for the zooming effect.
    It represents the distance from the camera's optical center (lens) to the image plane.
    The higher the focal length, the smaller the FOV.

    This functions returns focal length in pixels.
    */
    focal_length_y = CAM_HEIGHT_DEF_RES / (2 * tan(HALF_VFOV_DEF_RES));

}

double FOVCalculation::calcFocalLength_x() {
    /*
    Focal length determines how much of the FOV is captured by the camera and is responsible for the zooming effect.
    It represents the distance from the camera's optical center (lens) to the image plane.
    The higher the focal length, the smaller the FOV.

    This functions returns focal length in pixels.
    */
    focal_length_x = CAM_WIDTH_DEF_RES / (2 * tan(HALF_VHOV_DEF_RES));
}

double FOVCalculation::getVerticalAngle(){
    /*
    ball_y - center_y gives the vertical distance of the object from the center of the image.
    if ball_y > Cy -> ball is below th center (img coord are from top to bottom)
    if ball_y < Cy -> ball is above th center 
    */
    diff_y = ball_position_y - H_CENTER_DEF_RES;
    betha = atan(diff_y / focal_length_y);  // Converting ratio into an angle
    tilt_angle = current_head_tilt + betha;  // Adding θ + β (head tilt + betha)
    return tilt_angle;
}

void FOVCalculation::calcPanAngle(){
    diff_x = ball_position_x - W_CENTER_DEF_RES;
    phi = atan(diff_x / focal_length_x);  // Converting ratio into an angle
    pan_angle = current_head_pan + phi;  // Adding θ + β (head tilt + betha)
}

void FOVCalculation::calcDistanceToTarget(){
    distance_to_target = CAMERA_HEIGHT_ / tan(getVerticalAngle());  // cot(x) = tan(π/2 - x) = 1/tan(x)
}
