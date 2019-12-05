#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <Eigen/Geometry>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
using namespace std;
int main(int argc, char **argv)
{
  ros::init(argc, argv, "ve026a_move_group_node");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  //get ve026a_arm
  static const std::string PLANNING_GROUP = "ve026a_arm";

  //We will use move_group_interface:'MoveGroup' class 
  //we set it up using ve026a_arm name 
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  //We will use planning_scene_interface: 'PlanningSceneInterface'
  //to add and remove objects into the cell of the robot
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  //We use pointers to publishe the robot state for performance
  //KIV how can we use this with our hardware (robot_state is part of moveit_core)
  const robot_state::JointModelGroup * joint_model_group = 
    move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  //VISUALIZATION
  namespace rvt = rviz_visual_tools;

  //Use the virtual joint for intialization
  moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
  visual_tools.deleteAllMarkers();

  //provide buttons and keyboard shortcut using loadRemoteControl()
  visual_tools.loadRemoteControl();

  //We will use RVIZ markers too
  Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
  text_pose.translation().z() = 0.5;
  visual_tools.publishText(text_pose, "MoveGroupInterface VE026A", rvt::WHITE, rvt::XLARGE);

  //Batch publishing is used to reduce number of message being sent to rvix
  //for large visualization
  visual_tools.trigger();

  //GETTING BASIC INFORMATION
  //
  //Print the name of the reference frame for ve026a
  ROS_INFO_NAMED("ve026a_moveit_attempt1", "Reference frame: %s", move_group.getPlanningFrame().c_str());

  //Uncomment this if I have set up my end effector
  ROS_INFO_NAMED("ve026a_moveit_attemp1", "End Effector Link: %s", move_group.getEndEffectorLink().c_str());

  visual_tools.prompt("Press next to start");

  //setting up target_pose
  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.w = 1.0;
  target_pose1.position.x = 0.15;
  target_pose1.position.y = 0;
  target_pose1.position.z = 0.15;
  move_group.setPoseTarget(target_pose1);

  //Now call the planner to compute the plan an visualize it

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if(success)
  {
    ROS_INFO_NAMED("ve026a_moveit_attempt1", "Visualizing plan 1 as trajectory line");
    visual_tools.publishAxisLabeled(target_pose1, "Pose1");
    visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in RvizVisualToolsGui windows to continue");
  }
  else
  {
    cout<<"Planning failed"<<endl;
  }

   
  ros::shutdown();
  return 0;
  
}









