#include "avt_vimba_camera/trigger.h"

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "trigger_node");

  // ros::NodeHandle nh;
  // ros::NodeHandle nhp("~");

  // avt_vimba_camera::MonoCamera mc(nh,nhp);

  trigger::Trigger trigger;

  ros::spin();
  return 0;
}
