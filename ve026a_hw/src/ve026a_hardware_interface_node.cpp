#include <ve026a_hw/ve026a_hardware_interface.h>

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "ve026a_hardware_interface_node");
  ros::NodeHandle nh;
  
  //We runt the ros loop in a separate thread because 
  //external calls such as service callbacks to 
  //load controllers can block the (main) control loop

  //ros::AsyncSpinner spinner(1);
  //spinner.start();
  
  //initialize the class with port name and baud value
  ve026a_hardware_interface::ve026a_HardwareInterface ve026a(nh,115200,"/dev/ttyUSB0");
  ros::spin();
  //ros::waitForShutdown();
  return 0;
}
