#include "viv_hardware.h"

namespace viv_base
{
/*
  * Initialize ViV hardware
  */
VivHardware::VivHardware(ros::NodeHandle nh, ros::NodeHandle private_nh)
: nh_(nh), private_nh_(private_nh)
{
  private_nh_.param<double>("wheel_diameter", wheel_diameter_, 0.08 * 2.0);
  registerControlInterfaces();
  velocity_command_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("viv_epos_driver/motor_velocities", 1);
  epos_enc_sub_ = nh_.subscribe( "viv_epos_driver/joint_state", 1, &VivHardware::encoderCallback, this);
}

/*
  * Register interfaces with the RobotHW interface manager, allowing ros_control operation
  */
void VivHardware::registerControlInterfaces()
{
  std::cout << "Registering control interface\n";
  std::vector<std::string> joint_names {  "viv_wheel_front_left_joint", 
                                          "viv_wheel_front_right_joint",
                                          "viv_wheel_rear_left_joint",
                                          "viv_wheel_rear_right_joint" }; 

  for (uint32_t i = 0; i < joint_names.size(); i++)
  {
      hardware_interface::JointStateHandle joint_state_handle(joint_names[i],
                                                              &joints_[i].position, 
                                                              &joints_[i].velocity,
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

void VivHardware::updateJointsFromHardware()
{
  //horizon_legacy::Channel<clearpath::DataEncoders>::Ptr enc;
  //enc = horizon_legacy::Channel<clearpath::DataEncoders>::requestData(
  //  polling_timeout_);
  /*if (enc)
  {
    ROS_DEBUG_STREAM("Received travel information (L:" << enc->getTravel(LEFT)
        << " R:" << enc->getTravel(RIGHT) << ")");
        */
  //for (int i = 0; i < 4; i++)
  //{
    //for(int i = 0; i < 9999; i++)
    //  std::cout << "POZICIJEEE\n";

    joints_[0].position = linearToAngular(enc_in_meters_[0]);
    joints_[1].position = linearToAngular(enc_in_meters_[1]);
    joints_[2].position = linearToAngular(enc_in_meters_[2]);
    joints_[3].position = linearToAngular(enc_in_meters_[3]);
    
    /*double delta = linearToAngular(enc->getTravel(i % 2)) - joints_[i].position - joints_[i].position_offset;

    // detect suspiciously large readings, possibly from encoder rollover
    if (std::abs(delta) < 1.0)
    {
      joints_[i].position += delta;
    }
    else
    {
      // suspicious! drop this measurement and update the offset for subsequent readings
      joints_[i].position_offset += delta;
      ROS_DEBUG("Dropping overflow measurement from encoder");
    }
    */
  //}
  /*
  }

  horizon_legacy::Channel<clearpath::DataDifferentialSpeed>::Ptr speed;
  speed = horizon_legacy::Channel<clearpath::DataDifferentialSpeed>::requestData(polling_timeout_);
  if (speed)
  {
    ROS_DEBUG_STREAM("Received linear speed information (L:" << speed->getLeftSpeed()
        << " R:" << speed->getRightSpeed() << ")");
    for (int i = 0; i < 4; i++)
    {
      if (i % 2 == LEFT)
      {
        joints_[i].velocity = linearToAngular(speed->getLeftSpeed());
      }
      else
      { // assume RIGHT
        joints_[i].velocity = linearToAngular(speed->getRightSpeed());
      }
    }
  }
  */
}

/*
  * Get latest velocity commands from ros_control via joint structure, and publish to epos_ros hardware
  */
void VivHardware::writeCommandsToHardware()
{
  std::vector<double> joint_vel_commands {radps_to_rpm(joints_[2].velocity_command), 
                                          -radps_to_rpm(joints_[3].velocity_command),
                                          -radps_to_rpm(joints_[1].velocity_command),
                                          radps_to_rpm(joints_[0].velocity_command) };

  std_msgs::Float64MultiArray vel_command_msg;
  vel_command_msg.data = joint_vel_commands;
  velocity_command_pub_.publish(vel_command_msg);
}

/**
* Viv reports travel in encoder beats, translate to meters, then to rad
*/
double VivHardware::linearToAngular(const double &data) const
{
  return data / wheel_diameter_ * 2;
}

void VivHardware::encoderCallback(const sensor_msgs::JointStatePtr& msg)
{

  auto coeff = 1.315538133e-6; //num of meters per one encoder beat
  enc_in_meters_[0] = coeff * msg->position[0];
  enc_in_meters_[1] = -coeff * msg->position[2];
  enc_in_meters_[2] = coeff * msg->position[3];
  enc_in_meters_[3] = -coeff * msg->position[1];
}

}  // namespace viv_base

