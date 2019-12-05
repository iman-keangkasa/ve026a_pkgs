/**
 * @file connection_test_node.cpp
 */

#include <ros/ros.h>
#include <iostream>
#include <vector>
#include "ve026a_driver/bcap/bcap_serial.h"
#include <boost/smart_ptr.hpp>
#include <boost/thread.hpp>

using namespace std;

boost::shared_ptr<BCapSerial> bcap;
boost::mutex G_control_mutex;

void abort_m(BCAP_HRESULT hr)
{
  printf("BCAP_HRESULT %x\n", hr);
  if (FAILED(hr))
  {
    cout << "abort!" << endl;
    abort();
  }
}

void turnOnMotor(bool on, unsigned int& h_robot)
{
  int mode = on ? 1 : 0;
  int result = 0;
  boost::unique_lock<boost::mutex> lock(G_control_mutex);
  BCAP_HRESULT hr = bcap->RobotExecute2(h_robot, "Motor", VT_I2, 1, &mode, &result);
  abort_m(hr);
}

// Return position angle as degree
void getJointFeedback(vector<float>& joint_angle, unsigned int& h_joint_angle_variable)
{
  if (joint_angle.size() != 8) joint_angle.resize(8);
  BCAP_HRESULT hr;
  boost::unique_lock<boost::mutex> lock(G_control_mutex);
  hr = bcap->VariableGetValue(h_joint_angle_variable, joint_angle.data());
  abort_m(hr);
}

void shutdown(unsigned int& h_robot, unsigned int& h_controller)
{
  //control_thread.join();

  BCAP_HRESULT hr;
  turnOnMotor(false, h_robot);

  hr = bcap->RobotRelease(h_robot);
  abort_m(hr);

  hr = bcap->ControllerDisconnect(h_controller);
  abort_m(hr);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ve026a_connection_test_node");

  string port;
  if (argc > 1) port = string(argv[1]);
  else port = "/dev/ttyUSB0";
  ROS_INFO("Port is %s, and Baudrate is 115200", port.c_str());

  unsigned int baud = 115200;
  bcap = boost::shared_ptr<BCapSerial>(new BCapSerial(port, baud));

  BCAP_HRESULT hr;
  unsigned int h_controller;
  unsigned int h_robot;
  unsigned int h_joint_angle_variable;

  hr = bcap->ControllerConnect("", "", "", "", &h_controller);
  abort_m(hr);

  hr = bcap->ControllerGetRobot(h_controller, "VE026A", "$IsIDHandle$", &h_robot);
  abort_m(hr);

  turnOnMotor(false, h_robot);

  int mode = 258;
  long result = 0;
  hr = bcap->RobotExecute2(h_robot, "slvChangeMode", VT_I4, 1, &mode, &result);

  hr = bcap->RobotGetVariable(h_robot, "@CURRENT_ANGLE", "", &h_joint_angle_variable);
  abort_m(hr);

  vector<float> joint_angle;
  getJointFeedback(joint_angle, h_joint_angle_variable);
  int count = 0;
  for (auto&& var : joint_angle) {
    cout << count << ", " << var << "[deg]" << endl;
  }


  // sample code
  turnOnMotor(true, h_robot);
  float command_result[7];
  float command_angle[7] = {0, 0, 0, 0, 0, 0, joint_angle[6]};
  hr = bcap->RobotExecute2(h_robot, "slvMode", VT_R4 | VT_ARRAY, 7, command_angle, command_result);
  cout << "pose1" << endl;

  sleep(5);
  for (int i = 0; i < 7; i++)
  {
    cout << i << " th joint : " << command_result[i] << " deg" << endl;
  }

  float command_angle2[7] = {30, 0, 0, 0, 0, 30, joint_angle[6]};
  hr = bcap->RobotExecute2(h_robot, "slvMode", VT_R4 | VT_ARRAY, 7, command_angle2, command_result);
  cout << "pose2" << endl;

  sleep(5);
  for (int i = 0; i < 7; i++)
  {
    cout << i << " th joint : " << command_result[i] << " deg" << endl;
  }

  hr = bcap->RobotExecute2(h_robot, "slvMode", VT_R4 | VT_ARRAY, 7, command_angle, command_result);
  cout << "pose1" << endl;

  sleep(5);
  for (int i = 0; i < 7; i++)
  {
    cout << i << " th joint : " << command_result[i] << " deg" << endl;
  }

  ROS_INFO("VE026A Shut down...");

  // Shut down
  shutdown(h_robot, h_controller);

  return 0;
}
