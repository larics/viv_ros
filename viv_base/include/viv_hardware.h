#ifndef VIV_HARDWARE_H
#define VIV_HARDWARE_H

#include "hardware_interface/joint_state_interface.h"
#include "hardware_interface/joint_command_interface.h"
#include "hardware_interface/robot_hw.h"
#include "ros/ros.h"
#include <string>

#include "std_msgs/Float64MultiArray.h"

namespace viv_base {
    class VivHardware : public hardware_interface::RobotHW
    {
        public:

            VivHardware(ros::NodeHandle nh, ros::NodeHandle private_nh, double target_control_freq);

            void writeCommandsToHardware();

        private:

            void registerControlInterfaces();
            double radps_to_rpm(double vel_radps);

            ros::NodeHandle nh_, private_nh_;

            // ROS Control interfaces
            hardware_interface::VelocityJointInterface velocity_joint_interface_;

            // ROS Parameters
            double wheel_diameter_, max_accel_, max_speed_;

            double polling_timeout_;

            /*
             * Joint structure that is hooked to ros_control's InterfaceManager, to allow control via diff_drive_controller
             */
            struct Joint
            {
                double position;
                double position_offset;
                double velocity;
                double effort;
                double velocity_command;

                Joint() :
                position(0), velocity(0), effort(0), velocity_command(0)
                { }
            } joints_[4];

            ros::Publisher velocity_command_pub_;
    };

} // namespace viv_base
#endif  // VIV_HARDWARE_H