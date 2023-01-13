#include "viv_hardware.h"
#include "controller_manager/controller_manager.h"
#include "ros/callback_queue.h"

#include <boost/chrono.hpp>

typedef boost::chrono::steady_clock time_source;

void controlLoop(viv_base::VivHardware &viv,
                 controller_manager::ControllerManager &cm,
                 time_source::time_point &last_time)
{
  // Calculate monotonic time difference
  time_source::time_point this_time = time_source::now();
  boost::chrono::duration<double> elapsed_duration = this_time - last_time;
  ros::Duration elapsed(elapsed_duration.count());
  last_time = this_time;

  // Process control loop
  viv.updateJointsFromHardware();
  cm.update(ros::Time::now(), elapsed);
  viv.writeCommandsToHardware();
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "viv_base");
  ros::NodeHandle nh, private_nh("~");

  double control_frequency;
  private_nh.param<double>("control_frequency", control_frequency, 10.0);

  // Initialize robot hardware and link to controller manager
  viv_base::VivHardware viv(nh, private_nh);
  controller_manager::ControllerManager cm(&viv, nh);

  // Setup separate queue and single-threaded spinner to process timer callbacks
  // that interface with Husky hardware - libhorizon_legacy not threadsafe. This
  // avoids having to lock around hardware access, but precludes realtime safety
  // in the control loop. -> this is copied from Husky
  ros::CallbackQueue viv_queue;
  ros::AsyncSpinner viv_spinner(1, &viv_queue);

  time_source::time_point last_time = time_source::now();
  ros::TimerOptions control_timer( ros::Duration(1 / control_frequency),
                                  boost::bind(controlLoop, boost::ref(viv), 
                                  boost::ref(cm), boost::ref(last_time)),
                                  &viv_queue);
  ros::Timer control_loop = nh.createTimer(control_timer);

  viv_spinner.start();

  // Process remainder of ROS callbacks separately, mainly ControlManager related
  ros::spin();

  return 0;
}
