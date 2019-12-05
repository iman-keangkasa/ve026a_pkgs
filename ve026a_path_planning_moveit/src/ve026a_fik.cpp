#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>

int main( int argc, char ** argv)
{
  ros::init(argc, argv, "ve026a_fik_node");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  //robot and its definition is loaded into kinematic_model
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
  
  //robot_state will maintain the configuration of the robot
  robot_state::RobotStatePtr kinematic_state (new robot_state::RobotState(kinematic_model));
  kinematic_state->setToDefaultValues(); //what is the default value?
  const robot_state::JointModelGroup * joint_model_group = kinematic_model->getJointModelGroup("ve026a_arm");

  const std::vector <std::string> & joint_names = joint_model_group->getVariableNames();
  
  //Getting joint values
  //Use ve026a_driver for this part
  std::vector <double> joint_values;
  kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
  
  for (std::size_t i = 0; i < joint_names.size(); ++i)
  {
    ROS_INFO("Joint %s: %f",joint_names[i].c_str(), joint_values[i]);
  }

  //setJointGroupPosition() does not enforce joint limits so I need to enforce the limits
  //as an example 
  joint_values[0] = -15;
  kinematic_state -> setJointGroupPositions(joint_model_group, joint_values);
  //kinematic_state -> enforceBounds(joint_model_group); //as of now I suggest not using this. seems to invert the right joint limit configuration
  ROS_INFO_STREAM("Current state is "<< (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));
  //ROS_INFO_STREAM("Current state is "<< kinematic_state->satisfiesBounds());

  for (std::size_t i = 0; i < joint_names.size(); ++i)
  {
    ROS_INFO("Joint %s: %f",joint_names[i].c_str(), joint_values[i]);
  }
  //Computing forward kinematics
  //****************************
  //I have incorporated a dummy joint to get the state of the end effector called
  //joint effector. The frame attached to it is 
  //link_effector
  
  kinematic_state->setToRandomPositions(joint_model_group); //in configuration space
  const Eigen::Affine3d & end_effector_state = kinematic_state->getGlobalLinkTransform("link_effector");

  ROS_INFO_STREAM("Translation: \n" << end_effector_state.translation() << "\n");
  ROS_INFO_STREAM("Rotation: \n" << end_effector_state.rotation() << "\n");

  //Inverse Kinemaitcs
  //""""""""""""""""""
  //Now to get the inverse kinematics for the robot
  //To solve IK, we need to:
  //
  //(a) get the desired pose of the end effector (we already set above) setToRandomPositions()
  //(b) The number of attemp to solve IK: 10
  //(c) The timeout for each attempt: 0.1s

  std::size_t attempts = 10;
  double timeout = 0.1;
  bool found_ik = kinematic_state->setFromIK(joint_model_group, end_effector_state, attempts,timeout);
  
  if (found_ik)
  {
    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    for (std::size_t i = 0; i < joint_names.size(); ++i)
    {
      ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }
  }
  else
  {
    ROS_INFO("Did not find IK solution");
  }

  //Getting the Jacobian
  //
  Eigen::Vector3d reference_point_position (0.0, 0.0, 0.0);
  Eigen::MatrixXd jacobian;
  kinematic_state -> getJacobian(joint_model_group,
                                  kinematic_state ->getLinkModel(joint_model_group->getLinkModelNames().back()),
                                    reference_point_position, jacobian);
  ROS_INFO_STREAM("Jacobian: \n" << jacobian << "\n");
  ros::shutdown();
  return 0;
}





















