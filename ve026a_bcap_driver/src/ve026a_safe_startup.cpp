/**
 * @file simple_driver_node.cpp
 * @brief Driver node without ROS_Control just to publish commands
 */

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <signal.h>
#include <bcap/bcap_serial.h>
#include <boost/smart_ptr.hpp>
#include <boost/thread.hpp>


using namespace std;

boost::shared_ptr<BCapSerial> bcap;
boost::mutex G_control_mutex;

// handler
unsigned int G_h_controller; //this will be the address of the controller
unsigned int G_h_robot; //this will be the address of the robot
unsigned int G_h_joint_angle_variable; //this will be the address of joint_angle

void set_initial_position(vector<float> initial_angle, ros::NodeHandle& nh_param)
{
  map <string, float> joint_zeros;
  vector <string> joint_names;
  unsigned iterator;

  if( nh_param.hasParam("zeros"))
  {
    nh_param.getParam("zeros", joint_zeros);
    for (map<string,float>::iterator element = joint_zeros.begin(); 
          element != joint_zeros.end(); 
            element++) //iterating through a map
    {
      joint_names.push_back(element->first);
    }

    for (iterator = 0; iterator < joint_names.size(); ++iterator)
    {
      cout<< joint_names[iterator] <<": "<<initial_angle[iterator]*M_PI/180<<" RAD"<<endl;
      joint_zeros[ joint_names[iterator] ] = initial_angle[iterator]*M_PI/180;
    }

    nh_param.setParam("zeros",joint_zeros);
  }
}

void abort_m(BCAP_HRESULT hr)
{
  if (FAILED(hr))
  {
    printf("BCAP_HRESULT %x\n", hr);
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

void sigintHandler(int sit)
{
  shutdown(G_h_robot, G_h_controller);
  ros::shutdown();
}

void jsCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  map<string, double> joint_to_num;
  joint_to_num.insert(make_pair("joint_1", 0));
  joint_to_num.insert(make_pair("joint_2", 1));
  joint_to_num.insert(make_pair("joint_3", 2));
  joint_to_num.insert(make_pair("joint_4", 3));
  joint_to_num.insert(make_pair("joint_5", 4));
  joint_to_num.insert(make_pair("joint_6", 5));
  joint_to_num.insert(make_pair("joint_7", 6));

  float command_angle[7] = {0, 0, 0, 0, 0, 0, 0};
  float command_result[7];

  try
  {
    for (int i = 0; i < msg->name.size(); i++)
    {
      string name = msg->name[i];
      int idx = joint_to_num.at(name);
      command_angle[idx] = msg->position[i]*180/M_PI; // degree
    }
    auto  hr = bcap->RobotExecute2(G_h_robot, "slvMode", VT_R4 | VT_ARRAY, 7, command_angle, command_result);
    abort_m(hr);
  }
  catch(out_of_range e)
  {
    ROS_ERROR("Received inproper jointstate");
    ROS_ERROR("Must send position for joint_1, ... , joint_6");
    return;
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "veo26a_safe_startup_node");
  ros::NodeHandle nh;

  string port;
  if (argc > 1) port = string(argv[1]);
  else port = "/dev/ttyUSB0";
  ROS_INFO("Port is %s, and Baudrate is 115200", port.c_str());

  unsigned int baud = 115200;
  bcap = boost::shared_ptr<BCapSerial>(new BCapSerial(port, baud));

  BCAP_HRESULT hr;

  hr = bcap->ControllerConnect("", "", "", "", &G_h_controller);
  abort_m(hr);

  hr = bcap->ControllerGetRobot(G_h_controller, "VE026A", "$IsIDHandle$", &G_h_robot);
  abort_m(hr);

  turnOnMotor(false, G_h_robot);

  int mode = 258;
  long result = 0;
  hr = bcap->RobotExecute2(G_h_robot, "slvChangeMode", VT_I4, 1, &mode, &result);

  hr = bcap->RobotGetVariable(G_h_robot, "@CURRENT_ANGLE", "", &G_h_joint_angle_variable);
  abort_m(hr);

  vector<float> joint_angle;
  getJointFeedback(joint_angle, G_h_joint_angle_variable);
  
  int count = 0; 
  for (auto&& var : joint_angle) {
    cout << count << ", " << var << "[deg]" << endl;
  }
  
  set_initial_position(joint_angle, nh);

  //shutdown(G_h_robot, G_h_controller);

  ROS_INFO("Start to subscribe joint state");
  turnOnMotor(true, G_h_robot);
  ros::Subscriber js_sub = nh.subscribe("joint_states", 10, jsCallback);

  // For shutdown of arm
  signal(SIGINT, sigintHandler);

  ros::spin();

  return 0;
}
