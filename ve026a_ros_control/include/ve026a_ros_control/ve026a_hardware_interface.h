/**
 * @file ve026a_hardware_interface.h
 */

#ifndef INCLUDE_VE026A_ROS_CONTROL_VE026a_ROBOT_HW_H
#define INCLUDE_VE026A_ROS_CONTROL_VE026a_ROBOT_HW_H

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <boost/thread.hpp>

#include "ve026a_driver/bcap/bcap_serial.h"

#define JOINT_MAX 7

class VE026ARobotHW : public hardware_interface::RobotHW
{
public:
  VE026ARobotHW();
  virtual ~VE026ARobotHW();

  bool initialize();
  void read(const ros::Time& time, const ros::Duration& period);
  void write(const ros::Time& time, const ros::Duration& period);

  void turnOnMotor(bool on);
private:
  void abort_bcap(BCAP_HRESULT hr); // throw exception?
  void getJointFeedback(std::vector<float>& joint_angle);
  void shutdown();

private:
  hardware_interface::JointStateInterface m_JntStInterface;
  hardware_interface::PositionJointInterface m_PosJntInterface;
  //joint_limits_interface::PositionJointSoftLimitsInterface m_jnt_limits_interface;
  double m_cmd[JOINT_MAX];
  double m_pos[JOINT_MAX];
  double m_vel[JOINT_MAX];
  double m_eff[JOINT_MAX];

  // handle for bcap
  unsigned int h_robot_;
  unsigned int h_joint_angle_variable_;
  unsigned int h_controller_;

  std::string port_;
  int baud_;

  boost::shared_ptr<BCapSerial> bcap_;
  boost::mutex m_mtxMode_;
};

#endif
