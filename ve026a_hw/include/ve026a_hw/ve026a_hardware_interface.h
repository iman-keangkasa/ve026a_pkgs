#ifndef ROS_CONTROL__VE026A_HARDWARE_INTERFACE_H
#define ROS_CONTROL__VE026A_HARDWARE_INTERFACE_H

#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <controller_manager/controller_manager.h>
#include <boost/scoped_ptr.hpp>
#include <ros/ros.h>
#include <bcap/bcap_serial.h>
//#include <tr1cpp/tr1.h>
//#include <tr1cpp/arm.h>
//#include <tr1cpp/joint.h>
#include <ve026a_hw/ve026a_hardware.h>
using namespace std;
using namespace hardware_interface;
using joint_limits_interface::JointLimits;
using joint_limits_interface::SoftJointLimits;
using joint_limits_interface::PositionJointSoftLimitsHandle;
using joint_limits_interface::PositionJointSoftLimitsInterface;


namespace ve026a_hardware_interface
{
	static const double POSITION_STEP_FACTOR = 10; //should I use this?
	static const double VELOCITY_STEP_FACTOR = 10; //should I use this?

	class ve026a_HardwareInterface: public ve026a_hardware_interface::ve026a_Hardware
	{
		public:
                        //translate main() into this 
			ve026a_HardwareInterface(ros::NodeHandle& nh,unsigned int baud, std::string port);
			~ve026a_HardwareInterface();
			void init();
			void update(const ros::TimerEvent& e);
			void read();
			//void write(ros::Duration elapsed_time);
                        void write();
		protected:
			ros::NodeHandle nh_;
			ros::Timer non_realtime_loop_;
			ros::Duration control_period_;
			ros::Duration elapsed_time_;
			PositionJointInterface positionJointInterface;
			PositionJointSoftLimitsInterface positionJointSoftLimitsInterface;
			double loop_hz_;
			boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;
			//variables
                        double p_error_, v_error_, e_error_;
			std::string _logInfo;
                        //variables
                        boost::shared_ptr <BCapSerial> bcap_;
                        boost::mutex G_control_mutex_;

                        unsigned int h_controller_;
                        unsigned int h_robot_;
                        unsigned int h_joint_angle_variable_;

                        std::string port_;
                        int baud_;
                        //BCAP_HRESULT hr;
                
                        //methods
                        //[IMAN]void degree2rad(vector<double>& degrees);
                        //[IMAN]void rad2degree(vector<double> & degrees);
                        //[IMAN]choose only one (functions from ve026a_states_publisher.cpp)
                        //[IMAN]void set_initial_position(vector<float> initial_angle, ros::NodeHandle & nh_param); //not sure if I should use this
                        //[IMAN]void set_initial_position(vector<float> initial_angle);

                        
                        void abort_m(BCAP_HRESULT hr);
                        void turnOnMotor(bool on);
                        void getJointFeedback(vector<float>& joint_angle);
                        void shutdown();
                        //void signintHandler(int sit);
                        void init_position();
                        //[IMAN] void moveJoint(const sensor_msgs::JointState::ConstPtr& msg);


	};
              
                

}

#endif
