#include "avt_vimba_camera/trigger.h"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "trigger_node");

  trigger::Trigger trigger;
  trigger.Init();

  ros::spin();
  return 0;
}
