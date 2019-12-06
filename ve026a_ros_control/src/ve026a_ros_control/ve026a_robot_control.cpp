/**
 * @file ve026a_robot_control.cpp
 */

#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <controller_manager/controller_manager.h>
#include "ve026a_ros_control/ve026a_hardware_interface.h"

ros::Duration getPeriod()
{
  return ros::Duration(0.008); // 125Hz // sync? async?
}

ros::Time getTime()
{
  return ros::Time::now();
}

bool setMotor(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res,
              VE026ARobotHW *robo)
{
  bool on = req.data;
  robo->turnOnMotor(on);
  res.success = true;
  res.message = "Tryed to set motor " + std::string(on ? "ON" : "OFF");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ve026a_robot_control");
  ros::NodeHandle nh;


  VE026ARobotHW robo;
  bool is_connect = robo.initialize();

  auto service = nh.advertiseService<std_srvs::SetBool::Request, std_srvs::SetBool::Response>("set_motor", boost::bind(&setMotor, _1, _2, &robo));

  ROS_INFO("start controller loop");
  if (is_connect)
  {
    controller_manager::ControllerManager cm(&robo, nh);

    ros::Rate rate(1.0 / getPeriod().toSec());
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ROS_INFO("started spinner");

    ros::Time start = getTime();
    ros::Duration dt = getPeriod();
    while(ros::ok())
    {
      auto now = getTime();

      robo.read(now, dt);
      cm.update(now, dt);
      robo.write(now, dt);
      rate.sleep();
    }
    spinner.stop();
  }

  return 0;
}
