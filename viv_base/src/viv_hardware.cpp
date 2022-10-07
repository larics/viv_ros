#include "viv_hardware.h"

namespace viv_base
{
    /*
     * Initialize ViV hardware
     */
    VivHardware::VivHardware(ros::NodeHandle nh, ros::NodeHandle private_nh, double target_control_freq)
    : nh_(nh), private_nh_(private_nh)
    {
        registerControlInterfaces();
        velocity_command_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("viv_epos_driver/motor_velocities", 1);
    }

    /*
     * Register interfaces with the RobotHW interface manager, allowing ros_control operation
     */
    void VivHardware::registerControlInterfaces()
    {
        std::cout << "Registeramo control interface\n";
        std::vector<std::string> joint_names {  "viv_wheel_front_left_joint", 
                                                "viv_wheel_front_right_joint",
                                                "viv_wheel_rear_left_joint",
                                                "viv_wheel_rear_right_joint" }; 

        for (unsigned int i = 0; i < joint_names.size(); i++)
        {
            hardware_interface::JointStateHandle joint_state_handle(joint_names[i],
                                                                    &joints_[i].position, &joints_[i].velocity,
                                                                    &joints_[i].effort);

            hardware_interface::JointHandle joint_handle( joint_state_handle, &joints_[i].velocity_command);
            velocity_joint_interface_.registerHandle(joint_handle);
        }
        registerInterface(&velocity_joint_interface_);
    }

    // rad/s to rpm
    double VivHardware::radps_to_rpm(double vel_radps) { 
        return vel_radps * 30 / 3.14159;
    }

    /*
     * Get latest velocity commands from ros_control via joint structure, and publish to epos_ros hardware
     */
    void VivHardware::writeCommandsToHardware()
    {
        // Print commanded velocities
        /*
        std::cout << "Commanded velocities :\n";
        for (auto joint  : joints_) {
            std::cout << radps_to_rpm(joint.velocity_command) << " rpm\n";
        }
        std::cout << "\n";
        */
        std::vector<double> joint_vel_commands {    radps_to_rpm(joints_[2].velocity_command), 
                                                    -radps_to_rpm(joints_[3].velocity_command),
                                                    -radps_to_rpm(joints_[1].velocity_command),
                                                    radps_to_rpm(joints_[0].velocity_command) };

        std_msgs::Float64MultiArray vel_command_msg;
        vel_command_msg.data = joint_vel_commands;
        velocity_command_pub_.publish(vel_command_msg);
    }


}  // namespace viv_base


