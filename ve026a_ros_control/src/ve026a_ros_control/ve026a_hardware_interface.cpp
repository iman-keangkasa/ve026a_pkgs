/**
 * @file ve026a_hardware_interface.cpp
 */

#include "ve026a_ros_control/ve026a_hardware_interface.h"
#include <ros/ros.h>
#include <urdf/model.h>

using namespace std;

#define RAD2DEG(x) ((x) *180. / M_PI)
#define DEG2RAD(x) ((x) /180. * M_PI)

VE026ARobotHW::VE026ARobotHW()
{
  memset(m_cmd, 0, sizeof(m_cmd));
  memset(m_pos, 0, sizeof(m_pos));

  // default value
  port_ = "/dev/ttyUSB0";
  baud_ = 115200;
}

VE026ARobotHW::~VE026ARobotHW()
{
  shutdown();
}

bool VE026ARobotHW::initialize()
{
  //ros::NodeHandle nh("~");
  ros::NodeHandle nh;

  // Get parameters
  if (!nh.getParam("port", port_))
  {
    ROS_WARN("Default Port is used");
  }
  ROS_INFO("Port is %s", port_.c_str());

  if (!nh.getParam("baud", baud_))
  {
    ROS_WARN("Default baudrate is used");
  }
  ROS_INFO("Baudrate is %d",baud_); 

  // Get Urdf
  string urdf_string;
  //if (!ros::param::get("robot_description", urdf_string))
  if (!nh.getParam("/robot_description", urdf_string))
  {
    ROS_ERROR("Failed to get robot_description");
    return false;
  }

  urdf::Model model;
  if (!model.initString(urdf_string))
  {
    ROS_ERROR("Failed to parse URDF");
    return false;
  }

  // Set control interface
  /// @note Should read joint-name from ros-parameter
  //for (int i = 0; i < 7; i++) /// @todo add 7th joint(gripper)
  for (int i = 0; i < 6; i++)
  {
    string joint_name = "joint_" + to_string(i+1);

    hardware_interface::JointStateHandle state_handle(
        joint_name,
        &m_pos[i],
        &m_vel[i],
        &m_eff[i]);
    m_JntStInterface.registerHandle(state_handle);

    hardware_interface::JointHandle pos_handle(
        m_JntStInterface.getHandle(joint_name), &m_cmd[i]);
    m_PosJntInterface.registerHandle(pos_handle);

//    // Register joint limits
//    joint_limits_interface::JointLimits limits;
//    joint_limits_interface::SoftJointLimits soft_limits;
//    const bool urdf_limits_ok =
//      joint_limits_interface::getJointLimits(model.getJoint(joint_name), limits);
//    if (!urdf_limits_ok) ROS_WARN("urdf limits: %s is not defined", joint_name.c_str());
//
//    const bool urdf_soft_limits_ok =
//      joint_limits_interface::getSoftJointLimits(model.getJoint(joint_name), soft_limits);
//    if (!urdf_soft_limits_ok) ROS_WARN("urdf soft limits: %s is not defined", joint_name.c_str());
//    joint_limits_interface::PositionJointSoftLimitsHandle limits_handle(
//        //pos_handle,
//        m_PosJntInterface.getHandle(joint_name),
//        limits,
//        soft_limits);
//
//    m_jnt_limits_interface.registerHandle(limits_handle);
  }

  registerInterface(&m_JntStInterface);
  registerInterface(&m_PosJntInterface);
//  registerInterface(&m_jnt_limits_interface);

  // Prepare bcap
  bcap_ = boost::shared_ptr<BCapSerial>(new BCapSerial(port_, baud_));

  BCAP_HRESULT hr;

  hr = bcap_->ControllerConnect("", "", "", "", &h_controller_);
  abort_bcap(hr);

  hr = bcap_->ControllerGetRobot(h_controller_, "VE026A", "$IsIDHandle$", &h_robot_);
  abort_bcap(hr);

  turnOnMotor(false);

  int mode = 258;
  long result = 0;
  hr = bcap_->RobotExecute2(h_robot_, "slvChangeMode", VT_I4, 1, &mode, &result);
  hr = bcap_->RobotGetVariable(h_robot_, "@CURRENT_ANGLE", "", &h_joint_angle_variable_);
  abort_bcap(hr);

  // show current angle
  vector<float> joint_angle;
  getJointFeedback(joint_angle);
  for (int i = 0; i < joint_angle.size(); i++)
  {
    cout << "joint_" << i+1 << ", " << joint_angle[i] << "[deg]" << endl;
  }

  //turnOnMotor(true); // set this by service
  ROS_INFO("Start to control VE026A. Turn on motors by ros-service");

  return true;
}

void VE026ARobotHW::read(const ros::Time& time, const ros::Duration& period)
{
  //boost::mutex::scoped_lock lockMode(m_mtxMode_); // Not necessary

  vector<float> joint_angle;
  getJointFeedback(joint_angle);

  for (int i = 0; i < JOINT_MAX; i++)
  {
    m_pos[i] = DEG2RAD(joint_angle[i]);
  }
}

void VE026ARobotHW::write(const ros::Time& time, const ros::Duration& period)
{
  boost::mutex::scoped_lock lockMode(m_mtxMode_); // Necessary
//  m_jnt_limits_interface.enforceLimits(period);
  //stringstream ss;
  //for (int i = 0; i < JOINT_MAX; i++)
  //{
  //  ss << RAD2DEG(m_cmd[i]) << "[deg], ";
  //}
  //ROS_DEBUG("cmdToHW: %s", ss.str().c_str());

  float command_result[JOINT_MAX];
  float command_angle[JOINT_MAX];
  for (int i = 0; i < JOINT_MAX; ++i) command_angle[i] = RAD2DEG(m_cmd[i]); // double to float
  BCAP_HRESULT hr = bcap_->RobotExecute2(h_robot_, "slvMode", VT_R4 | VT_ARRAY, JOINT_MAX, command_angle, command_result);

  // need error processing
  abort_bcap(hr);
}

void VE026ARobotHW::abort_bcap(BCAP_HRESULT hr)
{
  if (FAILED(hr))
  {
    printf("BCAP_HRESULT %x\n", hr);
    cout << "abort!" << endl;
    abort(); // throw exception?
  }
}

void VE026ARobotHW::turnOnMotor(bool on)
{
  int mode = on ? 1 : 0;
  int result = 0;
  boost::unique_lock<boost::mutex> lock(m_mtxMode_);
  BCAP_HRESULT hr = bcap_->RobotExecute2(h_robot_, "Motor", VT_I2, 1, &mode, &result);
  abort_bcap(hr);
}

void VE026ARobotHW::getJointFeedback(std::vector<float>& joint_angle)
{
  if (joint_angle.size() != 8) joint_angle.resize(8);
  BCAP_HRESULT hr;
  boost::unique_lock<boost::mutex> lock(m_mtxMode_);
  hr = bcap_->VariableGetValue(h_joint_angle_variable_, joint_angle.data());
  abort_bcap(hr);
}

void VE026ARobotHW::shutdown()
{
  BCAP_HRESULT hr;
  turnOnMotor(false);

  hr = bcap_->RobotRelease(h_robot_);
  abort_bcap(hr);

  hr = bcap_->ControllerDisconnect(h_controller_);
  abort_bcap(hr);
}
