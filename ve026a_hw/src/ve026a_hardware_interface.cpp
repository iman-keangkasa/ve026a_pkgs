#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <ve026a_hw/ve026a_hardware_interface.h>
#include <bcap/bcap_serial.h>

#define RAD2DEG(x) ( (x)*180./M_PI)
#define DEG2RAD(x) ( (x)/180./M_PI)

using namespace std;
using namespace hardware_interface;
using joint_limits_interface::JointLimits;
using joint_limits_interface::SoftJointLimits;
using joint_limits_interface::PositionJointSoftLimitsHandle;
using joint_limits_interface::PositionJointSoftLimitsInterface;

namespace ve026a_hardware_interface
{
  ve026a_HardwareInterface::ve026a_HardwareInterface(ros::NodeHandle& nh, unsigned int baud, std::string port) \
    : nh_(nh)
    {
        init();
        controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
        nh_.param("/ve026a/hardware_interface/loop_hz", loop_hz_, 0.1);
        ROS_DEBUG_STREAM_NAMED("contructor", "Using loop frequency of "<< loop_hz_ << " hz");
        ros::Duration update_freq = ros::Duration(1.0/loop_hz_);
        non_realtime_loop_ = nh_.createTimer(update_freq, &ve026a_HardwareInterface::update, this);
        //[IMAN]initialize baud and port. Find where I can put this into rosparam
        if(port != "")
        {
        port_ = port;
        }
        else
        { 
          ROS_INFO("Port is not set");
          abort();
        }
        
        if(baud != 115200)
        {
          baud_ = 115200;
        }
        

        ROS_INFO_NAMED("hardware_interface", "Loaded generic_hardware_interface.");
    }

    ve026a_HardwareInterface::~ve026a_HardwareInterface()
    {
      shutdown();
    }

    void ve026a_HardwareInterface::init()
    {
      nh_.getParam("/ve026a/hardware_interface/joints", joint_names_);
      if (joint_names_.size() == 0)
      {
        ROS_FATAL_STREAM_NAMED("init","No joints found on parameter server for controller. Did you load the proper yaml file?");
      }
      num_joints_ = joint_names_.size();
      joint_position_.resize(num_joints_);
      joint_velocity_.resize(num_joints_);
      joint_effort_.resize(num_joints_);
      joint_position_command_.resize(num_joints_);
      joint_velocity_command_.resize(num_joints_);
      joint_effort_command_.resize(num_joints_);
    
    //prepares bcap
      turnOnMotor(true);
      bcap_ = boost::shared_ptr<BCapSerial>(new BCapSerial(port_, baud_));
    //start bcaps, get controller handle, get robot handle, get joint handle
      BCAP_HRESULT hr;
      int mode = 258;
      long result = 0;
      hr = bcap_->RobotExecute2(h_robot_, "slvChangeMode", VT_I4, 1, &mode, &result);
      hr = bcap_->RobotGetVariable(h_robot_, "@CURRENT_ANGLE", "", &h_joint_angle_variable_);
      abort_m(hr);

      //initialize current position
      //to joint command so that robot
      //will not jump to arbitrary position

      init_position();
      for (int i = 0; i < num_joints_; ++i)
      {
      //create joint_state_interface
        JointStateHandle jointStateHandle(joint_names_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]);
        joint_state_interface_.registerHandle(jointStateHandle);

      //create position joint interface
        JointHandle jointPositionHandle(jointStateHandle, &joint_position_command_[i]);
      //[IMAN] Should I skip JointLimits limits and soft joint limits?
        position_joint_interface_.registerHandle(jointPositionHandle);

      //create velocity joint interface
        JointHandle jointVelocityHandle(jointStateHandle, &joint_velocity_command_[i]);
        velocity_joint_interface_.registerHandle(jointVelocityHandle);

      //create effort joint interface
        JointHandle jointEffortHandle(jointStateHandle, &joint_effort_command_[i]);
        effort_joint_interface_.registerHandle(jointEffortHandle);
      } 

      registerInterface(&joint_state_interface_);
      registerInterface(&position_joint_interface_);
      registerInterface(&velocity_joint_interface_);
      registerInterface(&effort_joint_interface_);
      read(); //setting initial position
      //init_position();
    }

    void ve026a_HardwareInterface::update(const ros::TimerEvent& e)
    {
      _logInfo = "\n";
      _logInfo += "Joint Position Command:\n";
      for (int i = 0; i < num_joints_; i++)
      {
        std::ostringstream jointPositionStr;
        jointPositionStr << joint_position_command_[i];
        _logInfo += " " + joint_names_[i] +": " + jointPositionStr.str() + "\n";
      }
      elapsed_time_ = ros::Duration(e.current_real - e.last_real);

      read();
      controller_manager_->update(ros::Time::now(), elapsed_time_);
      //write(elapsed_time_);
      write();
      
    }

    void ve026a_HardwareInterface::read()
    {
      vector<float>joint_angle;

      //_logInfo += "Joint State:\n";
      getJointFeedback(joint_angle);
    
      for(int i = 0; i < num_joints_; i++)
      {
        joint_position_[i] = joint_angle[i]* M_PI / 180.; //double type radian
      }
    } 

 
    void ve026a_HardwareInterface::getJointFeedback(vector<float>& joint_angle)
    {
    if (joint_angle.size() != num_joints_) joint_angle.resize(num_joints_); 
    //if (joint_angle.size() != 8) joint_angle.resize(8); 
    //boost::unique_lock<boost::mutex> lock(G_control_mutex);
      BCAP_HRESULT hr;
      hr = bcap_->VariableGetValue(h_joint_angle_variable_, joint_angle.data());
      abort_m(hr);
    //[IMAN] TRY ... CATCH HERE?
    }
    
    void ve026a_HardwareInterface::abort_m(BCAP_HRESULT hr)
    {
      if (FAILED(hr))
      {
        ROS_INFO("BCAP_HRESULT %x\n", hr);
        cout << "abort!" << endl;
      //shutdown();
        abort();
      }
    }
  
    void ve026a_HardwareInterface::shutdown()
    {
      BCAP_HRESULT hr;
      turnOnMotor(false);
      hr = bcap_->RobotRelease(h_robot_);
      abort_m(hr);

      hr = bcap_->ControllerDisconnect(h_controller_);
      abort_m(hr);
    }

    void ve026a_HardwareInterface::turnOnMotor(bool on)
    {
      int mode = on ? 1 : 0;
      int result = 0;
      boost::unique_lock<boost::mutex> lock(G_control_mutex_);
      BCAP_HRESULT hr = bcap_->RobotExecute2(h_robot_, "Motor", VT_I2, 1, &mode, &result);
      abort_m(hr);
    }

    //void ve026a_HardwareInterface::write(ros::Duration elapsed_time)
    void ve026a_HardwareInterface::write()
    {
      boost::mutex::scoped_lock lockMode(G_control_mutex_);
      float command_result[num_joints_];
      float command_angle[num_joints_];
      for (int i = 0; i < num_joints_; ++i)
      {
        command_angle[i] = joint_position_command_[i]* 180. / M_PI; //
      }
      BCAP_HRESULT hr = bcap_->RobotExecute2(h_robot_, "slvMode", VT_R4 | VT_ARRAY, num_joints_, command_angle, command_result);
      abort_m(hr);
    }
  
    void ve026a_HardwareInterface::init_position()
    {
      read();
      joint_position_ = joint_position_command_;
      write();

    }
} 
