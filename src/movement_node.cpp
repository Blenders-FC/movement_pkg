#include <iostream>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>

// Assume robotis_framework's methods
namespace robotis_framework {
    Eigen::Vector3d getTransitionXYZ(double x, double y, double z) {
        return Eigen::Vector3d(x, y, z);
    }

    Eigen::Matrix3d getInertiaXYZ(double xx, double xy, double xz, double yy, double yz, double zz) {
        Eigen::Matrix3d inertia;
        inertia << xx, xy, xz,
                   xy, yy, yz,
                   xz, yz, zz;
        return inertia;
    }
}

// LinkData structure
class LinkData {
public:
    std::string name_;              // Name of the link
    int parent_;                    // Parent link ID
    int sibling_;                   // Sibling link ID
    int child_;                     // Child link ID
    double mass_;                   // Mass of the link
    Eigen::Vector3d center_of_mass_; // Center of mass of the link (relative to the parent)
    Eigen::Vector3d relative_position_; // Position relative to the parent
    Eigen::Vector3d joint_axis_;    // Axis of rotation for the joint
    double joint_limit_max_;        // Maximum joint limit (in radians)
    double joint_limit_min_;        // Minimum joint limit (in radians)
    Eigen::Matrix3d inertia_;       // Inertia matrix

    // Constructor to initialize the link data
    LinkData(std::string name, double mass, Eigen::Vector3d com, Eigen::Vector3d pos)
        : name_(name), mass_(mass), center_of_mass_(com), relative_position_(pos) {}

    // Add more constructors if needed
};

// List of links
std::vector<LinkData> op3_link_data_;

Eigen::Vector3d calculateCoM() {
    Eigen::Vector3d com(0.0, 0.0, 0.0);  // Center of Mass
    double total_mass = 0.0;

    // Iterate over all links and compute the weighted center of mass
    for (size_t i = 0; i < op3_link_data_.size(); ++i) {
        double mass = op3_link_data_[i].mass_;
        Eigen::Vector3d link_com = op3_link_data_[i].center_of_mass_;

        // Weighted sum for CoM
        com += link_com * mass;
        total_mass += mass;
    }

    // Normalize by total mass to get the final CoM
    if (total_mass > 0.0) {
        com /= total_mass;
    }

    return com;
}

int main() {
    // Populate the link data with some links (you can populate as many as needed)
    op3_link_data_.push_back(LinkData("l_leg_end", 0.0, robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0), robotis_framework::getTransitionXYZ(0.024, 0.0, -0.0305)));
    // Add more links...

    // Calculate CoM
    Eigen::Vector3d com = calculateCoM();

    // Output the result
    std::cout << "Center of Mass: " << com.transpose() << std::endl;

    return 0;
}


OP3KinematicsDynamics::OP3KinematicsDynamics(TreeSelect tree)
{
  for (int id = 0; id <= ALL_JOINT_ID; id++)
    op3_link_data_[id] = new LinkData();

  if (tree == WholeBody)
  {
    op3_link_data_[0]->name_ = "base";
    op3_link_data_[0]->parent_ = -1;
    op3_link_data_[0]->sibling_ = -1;
    op3_link_data_[0]->child_ = 23;
    op3_link_data_[0]->mass_ = 0.0;
    op3_link_data_[0]->relative_position_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    op3_link_data_[0]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    op3_link_data_[0]->center_of_mass_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    op3_link_data_[0]->joint_limit_max_ = 100.0;
    op3_link_data_[0]->joint_limit_min_ = -100.0;
    op3_link_data_[0]->inertia_ = robotis_framework::getInertiaXYZ(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    /* ----- passive joint -----*/

    op3_link_data_[23]->name_ = "passive_x";
    op3_link_data_[23]->parent_ = 0;
    op3_link_data_[23]->sibling_ = -1;
    op3_link_data_[23]->child_ = 24;
    op3_link_data_[23]->mass_ = 0.0;
    op3_link_data_[23]->relative_position_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    op3_link_data_[23]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    op3_link_data_[23]->center_of_mass_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    op3_link_data_[23]->joint_limit_max_ = 100.0;
    op3_link_data_[23]->joint_limit_min_ = -100.0;
    op3_link_data_[23]->inertia_ = robotis_framework::getInertiaXYZ(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    op3_link_data_[24]->name_ = "passive_y";
    op3_link_data_[24]->parent_ = 23;
    op3_link_data_[24]->sibling_ = -1;
    op3_link_data_[24]->child_ = 25;
    op3_link_data_[24]->mass_ = 0.0;
    op3_link_data_[24]->relative_position_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    op3_link_data_[24]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    op3_link_data_[24]->center_of_mass_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    op3_link_data_[24]->joint_limit_max_ = 100.0;
    op3_link_data_[24]->joint_limit_min_ = -100.0;
    op3_link_data_[24]->inertia_ = robotis_framework::getInertiaXYZ(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    op3_link_data_[25]->name_ = "passive_z";
    op3_link_data_[25]->parent_ = 24;
    op3_link_data_[25]->sibling_ = -1;
    op3_link_data_[25]->child_ = 26;
    op3_link_data_[25]->mass_ = 0.0;
    op3_link_data_[25]->relative_position_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.385);
    op3_link_data_[25]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    op3_link_data_[25]->center_of_mass_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    op3_link_data_[25]->joint_limit_max_ = 100.0;
    op3_link_data_[25]->joint_limit_min_ = -100.0;
    op3_link_data_[25]->inertia_ = robotis_framework::getInertiaXYZ(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    op3_link_data_[26]->name_ = "passive_yaw";
    op3_link_data_[26]->parent_ = 25;
    op3_link_data_[26]->sibling_ = -1;
    op3_link_data_[26]->child_ = 27;
    op3_link_data_[26]->mass_ = 0.0;
    op3_link_data_[26]->relative_position_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    op3_link_data_[26]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 1.0);
    op3_link_data_[26]->center_of_mass_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    op3_link_data_[26]->joint_limit_max_ = 100.0;
    op3_link_data_[26]->joint_limit_min_ = -100.0;
    op3_link_data_[26]->inertia_ = robotis_framework::getInertiaXYZ(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    op3_link_data_[27]->name_ = "passive_pitch";
    op3_link_data_[27]->parent_ = 26;
    op3_link_data_[27]->sibling_ = -1;
    op3_link_data_[27]->child_ = 28;
    op3_link_data_[27]->mass_ = 0.0;
    op3_link_data_[27]->relative_position_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    op3_link_data_[27]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, 1.0, 0.0);
    op3_link_data_[27]->center_of_mass_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    op3_link_data_[27]->joint_limit_max_ = 100.0;
    op3_link_data_[27]->joint_limit_min_ = -100.0;
    op3_link_data_[27]->inertia_ = robotis_framework::getInertiaXYZ(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    op3_link_data_[28]->name_ = "passive_roll";
    op3_link_data_[28]->parent_ = 27;
    op3_link_data_[28]->sibling_ = -1;
    op3_link_data_[28]->child_ = 29;
    op3_link_data_[28]->mass_ = 0.0;
    op3_link_data_[28]->relative_position_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    op3_link_data_[28]->joint_axis_ = robotis_framework::getTransitionXYZ(1.0, 0.0, 0.0);
    op3_link_data_[28]->center_of_mass_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    op3_link_data_[28]->joint_limit_max_ = 100.0;
    op3_link_data_[28]->joint_limit_min_ = -100.0;
    op3_link_data_[28]->inertia_ = robotis_framework::getInertiaXYZ(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    /* ----- body -----*/

    // pelvis_link
    op3_link_data_[29]->name_ = "pelvis";
    op3_link_data_[29]->parent_ = 28;
    op3_link_data_[29]->sibling_ = -1;
    op3_link_data_[29]->child_ = 19;
    op3_link_data_[29]->mass_ = 1.3492787;
    op3_link_data_[29]->relative_position_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    op3_link_data_[29]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    op3_link_data_[29]->center_of_mass_ = robotis_framework::getTransitionXYZ(-0.015014868, 0.00013099, 0.065815797);
    op3_link_data_[29]->joint_limit_max_ = 100.0;
    op3_link_data_[29]->joint_limit_min_ = -100.0;
    op3_link_data_[29]->inertia_ = robotis_framework::getInertiaXYZ(0.03603, 0.00000, 0.00016, 0.02210, 0.00000,
                                                                    0.03830);

    /* ----- head -----*/

    // head_pan
    op3_link_data_[19]->name_ = "head_pan";
    op3_link_data_[19]->parent_ = 29;
    op3_link_data_[19]->sibling_ = 1;
    op3_link_data_[19]->child_ = 20;
    op3_link_data_[19]->mass_ = 0.011759436;
    op3_link_data_[19]->relative_position_ = robotis_framework::getTransitionXYZ(-0.001, 0.0, 0.1365);
    op3_link_data_[19]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 1.0);
    op3_link_data_[19]->center_of_mass_ = robotis_framework::getTransitionXYZ(0.002327479, 0, 0.008227847);
    op3_link_data_[19]->joint_limit_max_ = 0.5 * M_PI;
    op3_link_data_[19]->joint_limit_min_ = -0.5 * M_PI;
    op3_link_data_[19]->inertia_ = robotis_framework::getInertiaXYZ(0.00011, 0.00000, 0.00000, 0.00003, 0.00000,
                                                                    0.00012);

    // head_tilt
    op3_link_data_[20]->name_ = "head_tilt";
    op3_link_data_[20]->parent_ = 19;
    op3_link_data_[20]->sibling_ = -1;
    op3_link_data_[20]->child_ = -1;
    op3_link_data_[20]->mass_ = 0.13630649;
    op3_link_data_[20]->relative_position_ = robotis_framework::getTransitionXYZ(0.01, 0.019, 0.0285);
    op3_link_data_[20]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, -1.0, 0.0);
    op3_link_data_[20]->center_of_mass_ = robotis_framework::getTransitionXYZ(0.002298411, -0.018634079, 0.027696734);
    op3_link_data_[20]->joint_limit_max_ = 0.5 * M_PI;
    op3_link_data_[20]->joint_limit_min_ = -0.5 * M_PI;
    op3_link_data_[20]->inertia_ = robotis_framework::getInertiaXYZ(0.00113, 0.00001, -0.00005, 0.00114, 0.00002,
                                                                    0.00084);

    /*----- right arm -----*/

    // right arm shoulder pitch
    op3_link_data_[1]->name_ = "r_sho_pitch";
    op3_link_data_[1]->parent_ = 29;
    op3_link_data_[1]->sibling_ = 2;
    op3_link_data_[1]->child_ = 3;
    op3_link_data_[1]->mass_ = 0.011759436;
    op3_link_data_[1]->relative_position_ = robotis_framework::getTransitionXYZ(-0.001, -0.06, 0.111);
    op3_link_data_[1]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, -1.0, 0.0);
    op3_link_data_[1]->center_of_mass_ = robotis_framework::getTransitionXYZ(0, -0.008227847, -0.002327479);
    op3_link_data_[1]->joint_limit_max_ = 0.5 * M_PI;
    op3_link_data_[1]->joint_limit_min_ = -0.5 * M_PI;
    op3_link_data_[1]->inertia_ = robotis_framework::getInertiaXYZ(0.00018, 0.0, 0.0, 0.00058, -0.00004, 0.00057);

    // right arm shoulder roll
    op3_link_data_[3]->name_ = "r_sho_roll";
    op3_link_data_[3]->parent_ = 1;
    op3_link_data_[3]->sibling_ = -1;
    op3_link_data_[3]->child_ = 5;
    op3_link_data_[3]->mass_ = 0.1775763;
    op3_link_data_[3]->relative_position_ = robotis_framework::getTransitionXYZ(0.019, -0.0285, -0.01);
    op3_link_data_[3]->joint_axis_ = robotis_framework::getTransitionXYZ(-1.0, 0.0, 0.0);
    op3_link_data_[3]->center_of_mass_ = robotis_framework::getTransitionXYZ(-0.018438243, -0.045143767, 0.000281212);
    op3_link_data_[3]->joint_limit_max_ = 0.3 * M_PI;
    op3_link_data_[3]->joint_limit_min_ = -0.5 * M_PI;
    op3_link_data_[3]->inertia_ = robotis_framework::getInertiaXYZ(0.00043, 0.00000, 0.00000, 0.00112, 0.00000,
                                                                   0.00113);

    // right arm elbow
    op3_link_data_[5]->name_ = "r_el";
    op3_link_data_[5]->parent_ = 3;
    op3_link_data_[5]->sibling_ = -1;
    op3_link_data_[5]->child_ = 21;
    op3_link_data_[5]->mass_ = 0.041267974;
    op3_link_data_[5]->relative_position_ = robotis_framework::getTransitionXYZ(0, -0.0904, -0.0001);
    op3_link_data_[5]->joint_axis_ = robotis_framework::getTransitionXYZ(1.0, 0.0, 0.0);
    op3_link_data_[5]->center_of_mass_ = robotis_framework::getTransitionXYZ(-0.019000003, -0.070330391, 0.00380012);
    op3_link_data_[5]->joint_limit_max_ = 0.5 * M_PI;
    op3_link_data_[5]->joint_limit_min_ = -0.5 * M_PI;
    op3_link_data_[5]->inertia_ = robotis_framework::getInertiaXYZ(0.00277, 0.00002, -0.00001, 0.00090, 0.00004,
                                                                   0.00255);

    // right arm end effector
    op3_link_data_[21]->name_ = "r_arm_end";
    op3_link_data_[21]->parent_ = 5;
    op3_link_data_[21]->sibling_ = -1;
    op3_link_data_[21]->child_ = -1;
    op3_link_data_[21]->mass_ = 0.0;
    op3_link_data_[21]->relative_position_ = robotis_framework::getTransitionXYZ(0.0, -0.15, 0.0);
    op3_link_data_[21]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    op3_link_data_[21]->center_of_mass_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    op3_link_data_[21]->joint_limit_max_ = 100.0;
    op3_link_data_[21]->joint_limit_min_ = -100.0;
    op3_link_data_[21]->inertia_ = robotis_framework::getInertiaXYZ(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    /*----- left arm -----*/

    // left arm shoulder pitch
    op3_link_data_[2]->name_ = "l_sho_pitch";
    op3_link_data_[2]->parent_ = 29;
    op3_link_data_[2]->sibling_ = 7;
    op3_link_data_[2]->child_ = 4;
    op3_link_data_[2]->mass_ = 0.011759436;
    op3_link_data_[2]->relative_position_ = robotis_framework::getTransitionXYZ(-0.001, 0.06, 0.111);
    op3_link_data_[2]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, 1.0, 0.0);
    op3_link_data_[2]->center_of_mass_ = robotis_framework::getTransitionXYZ(0, 0.008227847, -0.002327479);
    op3_link_data_[2]->joint_limit_max_ = 0.5 * M_PI;
    op3_link_data_[2]->joint_limit_min_ = -0.5 * M_PI;
    op3_link_data_[2]->inertia_ = robotis_framework::getInertiaXYZ(0.00018, 0.00000, 0.00000, 0.00058, 0.00004,
                                                                   0.00057);

    // left arm shoulder roll
    op3_link_data_[4]->name_ = "l_sho_roll";
    op3_link_data_[4]->parent_ = 2;
    op3_link_data_[4]->sibling_ = -1;
    op3_link_data_[4]->child_ = 6;
    op3_link_data_[4]->mass_ = 0.1775763;
    op3_link_data_[4]->relative_position_ = robotis_framework::getTransitionXYZ(0.019, 0.0285, -0.01);
    op3_link_data_[4]->joint_axis_ = robotis_framework::getTransitionXYZ(-1.0, 0.0, 0.0);
    op3_link_data_[4]->center_of_mass_ = robotis_framework::getTransitionXYZ(-0.018438243, 0.045143767, 0.000281212);
    op3_link_data_[4]->joint_limit_max_ = 0.5 * M_PI;
    op3_link_data_[4]->joint_limit_min_ = -0.3 * M_PI;
    op3_link_data_[4]->inertia_ = robotis_framework::getInertiaXYZ(0.00043, 0.00000, 0.00000, 0.00112, 0.00000,
                                                                   0.00113);

    // left arm elbow
    op3_link_data_[6]->name_ = "l_el";
    op3_link_data_[6]->parent_ = 4;
    op3_link_data_[6]->sibling_ = -1;
    op3_link_data_[6]->child_ = 22;
    op3_link_data_[6]->mass_ = 0.041267974;
    op3_link_data_[6]->relative_position_ = robotis_framework::getTransitionXYZ(0, 0.0904, -0.0001);
    op3_link_data_[6]->joint_axis_ = robotis_framework::getTransitionXYZ(1.0, 0.0, 0.0);
    op3_link_data_[6]->center_of_mass_ = robotis_framework::getTransitionXYZ(-0.018999997, 0.070330391, 0.003800117);
    op3_link_data_[6]->joint_limit_max_ = 0.5 * M_PI;
    op3_link_data_[6]->joint_limit_min_ = -0.5 * M_PI;
    op3_link_data_[6]->inertia_ = robotis_framework::getInertiaXYZ(0.00277, -0.00002, -0.00001, 0.00090, -0.00004,
                                                                   0.00255);

    // left arm end effector
    op3_link_data_[22]->name_ = "l_arm_end";
    op3_link_data_[22]->parent_ = 6;
    op3_link_data_[22]->sibling_ = -1;
    op3_link_data_[22]->child_ = -1;
    op3_link_data_[22]->mass_ = 0.0;
    op3_link_data_[22]->relative_position_ = robotis_framework::getTransitionXYZ(0.0, 0.15, 0.0);
    op3_link_data_[22]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    op3_link_data_[22]->center_of_mass_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    op3_link_data_[22]->joint_limit_max_ = 100.0;
    op3_link_data_[22]->joint_limit_min_ = -100.0;
    op3_link_data_[22]->inertia_ = robotis_framework::getInertiaXYZ(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    /* ----- right leg -----*/

    // right leg hip yaw
    op3_link_data_[7]->name_ = "r_hip_yaw";
    op3_link_data_[7]->parent_ = 29;
    op3_link_data_[7]->sibling_ = 8;
    op3_link_data_[7]->child_ = 9;
    op3_link_data_[7]->mass_ = 0.011813898;
    op3_link_data_[7]->relative_position_ = robotis_framework::getTransitionXYZ(0, -0.035, 0);
    op3_link_data_[7]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, 0.0, -1.0);
    op3_link_data_[7]->center_of_mass_ = robotis_framework::getTransitionXYZ(-0.001566062, 0, -0.00774017);
    op3_link_data_[7]->joint_limit_max_ = 0.45 * M_PI;
    op3_link_data_[7]->joint_limit_min_ = -0.45 * M_PI;
    op3_link_data_[7]->inertia_ = robotis_framework::getInertiaXYZ(0.00024, 0.00000, 0.00000, 0.00101, 0.00000,
                                                                   0.00092);

    // right leg hip roll
    op3_link_data_[9]->name_ = "r_hip_roll";
    op3_link_data_[9]->parent_ = 7;
    op3_link_data_[9]->sibling_ = -1;
    op3_link_data_[9]->child_ = 11;
    op3_link_data_[9]->mass_ = 0.17885985;
    op3_link_data_[9]->relative_position_ = robotis_framework::getTransitionXYZ(-0.024, 0, -0.0285);
    op3_link_data_[9]->joint_axis_ = robotis_framework::getTransitionXYZ(-1.0, 0.0, 0.0);
    op3_link_data_[9]->center_of_mass_ = robotis_framework::getTransitionXYZ(0.00388263, -0.000278863, -0.012138713);
    op3_link_data_[9]->joint_limit_max_ = 0.3 * M_PI;
    op3_link_data_[9]->joint_limit_min_ = -0.3 * M_PI;
    op3_link_data_[9]->inertia_ = robotis_framework::getInertiaXYZ(0.00056, 0.00000, 0.00000, 0.00168, 0.00000,
                                                                   0.00171);

    // right leg hip pitch
    op3_link_data_[11]->name_ = "r_hip_pitch";
    op3_link_data_[11]->parent_ = 9;
    op3_link_data_[11]->sibling_ = -1;
    op3_link_data_[11]->child_ = 13;
    op3_link_data_[11]->mass_ = 0.11543381;
    op3_link_data_[11]->relative_position_ = robotis_framework::getTransitionXYZ(0.0241, -0.019, 0);
    op3_link_data_[11]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, -1.0, 0.0);
    op3_link_data_[11]->center_of_mass_ = robotis_framework::getTransitionXYZ(0.000590366, 0.019005093, -0.084075186);
    op3_link_data_[11]->joint_limit_max_ = 0.4 * M_PI;
    op3_link_data_[11]->joint_limit_min_ = -0.4 * M_PI;
    op3_link_data_[11]->inertia_ = robotis_framework::getInertiaXYZ(0.04329, -0.00027, 0.00286, 0.04042, 0.00203,
                                                                    0.00560);

    // right leg knee
    op3_link_data_[13]->name_ = "r_knee";
    op3_link_data_[13]->parent_ = 11;
    op3_link_data_[13]->sibling_ = -1;
    op3_link_data_[13]->child_ = 15;
    op3_link_data_[13]->mass_ = 0.040146918;
    op3_link_data_[13]->relative_position_ = robotis_framework::getTransitionXYZ(0.0001, 0,	-0.11015);
    op3_link_data_[13]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, -1.0, 0.0);
    op3_link_data_[13]->center_of_mass_ = robotis_framework::getTransitionXYZ(0, 0.021514031, -0.055);
    op3_link_data_[13]->joint_limit_max_ = 0.1 * M_PI;
    op3_link_data_[13]->joint_limit_min_ = -0.7 * M_PI;
    op3_link_data_[13]->inertia_ = robotis_framework::getInertiaXYZ(0.01971, -0.00031, -0.00294, 0.01687, -0.00140,
                                                                    0.00574);

    // right leg ankle pitch
    op3_link_data_[15]->name_ = "r_ank_pitch";
    op3_link_data_[15]->parent_ = 13;
    op3_link_data_[15]->sibling_ = -1;
    op3_link_data_[15]->child_ = 17;
    op3_link_data_[15]->mass_ = 0.17885985;
    op3_link_data_[15]->relative_position_ = robotis_framework::getTransitionXYZ(0, 0, -0.11);
    op3_link_data_[15]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, 1.0, 0.0);
    op3_link_data_[15]->center_of_mass_ = robotis_framework::getTransitionXYZ(-0.02021737, 0.018721137, 0.012138713);
    op3_link_data_[15]->joint_limit_max_ = 0.45 * M_PI;
    op3_link_data_[15]->joint_limit_min_ = -0.45 * M_PI;
    op3_link_data_[15]->inertia_ = robotis_framework::getInertiaXYZ(0.00056, 0.00000, 0.00000, 0.00168, 0.00000,
                                                                    0.00171);

    // right leg ankle roll
    op3_link_data_[17]->name_ = "r_ank_roll";
    op3_link_data_[17]->parent_ = 15;
    op3_link_data_[17]->sibling_ = -1;
    op3_link_data_[17]->child_ = 31;
    op3_link_data_[17]->mass_ = 0.069344849;
    op3_link_data_[17]->relative_position_ = robotis_framework::getTransitionXYZ(-0.0241, 0.019, 0);
    op3_link_data_[17]->joint_axis_ = robotis_framework::getTransitionXYZ(1.0, 0.0, 0.0);
    op3_link_data_[17]->center_of_mass_ = robotis_framework::getTransitionXYZ(0.023733194, -0.010370444, -0.027601507);
    op3_link_data_[17]->joint_limit_max_ = 0.45 * M_PI;
    op3_link_data_[17]->joint_limit_min_ = -0.45 * M_PI;
    op3_link_data_[17]->inertia_ = robotis_framework::getInertiaXYZ(0.00022, 0.00000, -0.00001, 0.00099, 0.00000,
                                                                    0.00091);

    // right leg end
    op3_link_data_[31]->name_ = "r_leg_end";
    op3_link_data_[31]->parent_ = 17;
    op3_link_data_[31]->sibling_ = -1;
    op3_link_data_[31]->child_ = -1;
    op3_link_data_[31]->mass_ = 0.0;
    op3_link_data_[31]->relative_position_ = robotis_framework::getTransitionXYZ(0.024, 0.0, -0.0305);
    op3_link_data_[31]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    op3_link_data_[31]->center_of_mass_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    op3_link_data_[31]->joint_limit_max_ = 100.0;
    op3_link_data_[31]->joint_limit_min_ = -100.0;
    op3_link_data_[31]->inertia_ = robotis_framework::getInertiaXYZ(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    /* ----- left leg -----*/

    // left leg hip yaw
    op3_link_data_[8]->name_ = "l_hip_yaw";
    op3_link_data_[8]->parent_ = 29;
    op3_link_data_[8]->sibling_ = -1;
    op3_link_data_[8]->child_ = 10;
    op3_link_data_[8]->mass_ = 0.011813898;
    op3_link_data_[8]->relative_position_ = robotis_framework::getTransitionXYZ(0, 0.035, 0);
    op3_link_data_[8]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, 0.0, -1.0);
    op3_link_data_[8]->center_of_mass_ = robotis_framework::getTransitionXYZ(-0.001566062, 0, -0.00774017);
    op3_link_data_[8]->joint_limit_max_ = 0.45 * M_PI;
    op3_link_data_[8]->joint_limit_min_ = -0.45 * M_PI;
    op3_link_data_[8]->inertia_ = robotis_framework::getInertiaXYZ(0.00024, 0.00000, 0.00000, 0.00101, 0.00000,
                                                                   0.00092);

    // left leg hip roll
    op3_link_data_[10]->name_ = "l_hip_roll";
    op3_link_data_[10]->parent_ = 8;
    op3_link_data_[10]->sibling_ = -1;
    op3_link_data_[10]->child_ = 12;
    op3_link_data_[10]->mass_ = 0.17885985;
    op3_link_data_[10]->relative_position_ = robotis_framework::getTransitionXYZ(-0.024, 0, -0.0285);
    op3_link_data_[10]->joint_axis_ = robotis_framework::getTransitionXYZ(-1.0, 0.0, 0.0);
    op3_link_data_[10]->center_of_mass_ = robotis_framework::getTransitionXYZ(0.00388263, 0.000278863, -0.012138713);
    op3_link_data_[10]->joint_limit_max_ = 0.3 * M_PI;
    op3_link_data_[10]->joint_limit_min_ = -0.3 * M_PI;
    op3_link_data_[10]->inertia_ = robotis_framework::getInertiaXYZ(0.00056, 0.00000, 0.00000, 0.00168, 0.00000,
                                                                    0.00171);

    // left leg hip pitch
    op3_link_data_[12]->name_ = "l_hip_pitch";
    op3_link_data_[12]->parent_ = 10;
    op3_link_data_[12]->sibling_ = -1;
    op3_link_data_[12]->child_ = 14;
    op3_link_data_[12]->mass_ = 0.11543381;
    op3_link_data_[12]->relative_position_ = robotis_framework::getTransitionXYZ(0.0241, 0.019, 0);
    op3_link_data_[12]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, 1.0, 0.0);
    op3_link_data_[12]->center_of_mass_ = robotis_framework::getTransitionXYZ(0.000590367, -0.019005093, -0.084075186);
    op3_link_data_[12]->joint_limit_max_ = 0.4 * M_PI;
    op3_link_data_[12]->joint_limit_min_ = -0.4 * M_PI;
    op3_link_data_[12]->inertia_ = robotis_framework::getInertiaXYZ(0.04328, 0.00028, 0.00288, 0.04042, -0.00202,
                                                                    0.00560);

    // left leg knee pitch
    op3_link_data_[14]->name_ = "l_knee";
    op3_link_data_[14]->parent_ = 12;
    op3_link_data_[14]->sibling_ = -1;
    op3_link_data_[14]->child_ = 16;
    op3_link_data_[14]->mass_ = 0.040146918;
    op3_link_data_[14]->relative_position_ = robotis_framework::getTransitionXYZ(0.0001, 0, -0.11015);
    op3_link_data_[14]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, 1.0, 0.0);
    op3_link_data_[14]->center_of_mass_ = robotis_framework::getTransitionXYZ(0, -0.021514031, -0.055);
    op3_link_data_[14]->joint_limit_max_ = 0.7 * M_PI;
    op3_link_data_[14]->joint_limit_min_ = -0.1 * M_PI;
    op3_link_data_[14]->inertia_ = robotis_framework::getInertiaXYZ(0.01971, 0.00031, -0.00294, 0.01687, 0.00140,
                                                                    0.00574);

    // left leg ankle pitch
    op3_link_data_[16]->name_ = "l_ank_pitch";
    op3_link_data_[16]->parent_ = 14;
    op3_link_data_[16]->sibling_ = -1;
    op3_link_data_[16]->child_ = 18;
    op3_link_data_[16]->mass_ = 0.17885985;
    op3_link_data_[16]->relative_position_ = robotis_framework::getTransitionXYZ(0.000, 0.000, -0.110);
    op3_link_data_[16]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, -1.0, 0.0);
    op3_link_data_[16]->center_of_mass_ = robotis_framework::getTransitionXYZ(-0.02021825, -0.018721131, 0.012138988);
    op3_link_data_[16]->joint_limit_max_ = 0.45 * M_PI;
    op3_link_data_[16]->joint_limit_min_ = -0.45 * M_PI;
    op3_link_data_[16]->inertia_ = robotis_framework::getInertiaXYZ(0.00056, 0.00000, 0.00000, 0.00168, 0.00000,
                                                                    0.00171);

    // left leg ankle roll
    op3_link_data_[18]->name_ = "l_ank_roll";
    op3_link_data_[18]->parent_ = 16;
    op3_link_data_[18]->sibling_ = -1;
    op3_link_data_[18]->child_ = 30;
    op3_link_data_[18]->mass_ = 0.069344849;
    op3_link_data_[18]->relative_position_ = robotis_framework::getTransitionXYZ(-0.0241, -0.019, 0);
    op3_link_data_[18]->joint_axis_ = robotis_framework::getTransitionXYZ(1.0, 0.0, 0.0);
    op3_link_data_[18]->center_of_mass_ = robotis_framework::getTransitionXYZ(0.023733194, 0.010370444, -0.027601507);
    op3_link_data_[18]->joint_limit_max_ = 0.45 * M_PI;
    op3_link_data_[18]->joint_limit_min_ = -0.45 * M_PI;
    op3_link_data_[18]->inertia_ = robotis_framework::getInertiaXYZ(0.00022, 0.00000, -0.00001, 0.00099, 0.00000,
                                                                    0.00091);

    // left leg end
    op3_link_data_[30]->name_ = "l_leg_end";
    op3_link_data_[30]->parent_ = 18;
    op3_link_data_[30]->sibling_ = -1;
    op3_link_data_[30]->child_ = -1;
    op3_link_data_[30]->mass_ = 0.0;
    op3_link_data_[30]->relative_position_ = robotis_framework::getTransitionXYZ(0.024, 0.0, -0.0305);
    op3_link_data_[30]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    op3_link_data_[30]->center_of_mass_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    op3_link_data_[30]->joint_limit_max_ = 100.0;
    op3_link_data_[30]->joint_limit_min_ = -100.0;
    op3_link_data_[30]->inertia_ = robotis_framework::getInertiaXYZ(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  }
  
  hip_offset_angle_rad_ = atan2(0.0001, 0.11015);
  hip_pitch_offset_m_ = 0.0001;
  thigh_length_m_ = sqrt(op3_link_data_[ID_R_LEG_START + 2 * 3]->relative_position_.coeff(0, 0)*op3_link_data_[ID_R_LEG_START + 2 * 3]->relative_position_.coeff(0, 0)
                               + op3_link_data_[ID_R_LEG_START + 2 * 3]->relative_position_.coeff(2, 0)*op3_link_data_[ID_R_LEG_START + 2 * 3]->relative_position_.coeff(2, 0));
  calf_length_m_ = std::fabs(op3_link_data_[ID_R_LEG_START + 2 * 4]->relative_position_.coeff(2, 0));
  ankle_length_m_ = std::fabs(op3_link_data_[ID_R_LEG_END]->relative_position_.coeff(2, 0));
  leg_side_offset_m_ = 2.0 * (std::fabs(op3_link_data_[ID_R_LEG_START]->relative_position_.coeff(1, 0)));
}

