#ifndef AVT_VIMBA_CAMERA_TRIGGER_H
#define AVT_VIMBA_CAMERA_TRIGGER_H

#include "VimbaCPP/Include/VimbaCPP.h"

#include <arpa/inet.h>

#include <ros/ros.h>

namespace trigger
{

class Trigger
{
public:
  Trigger();
  ~Trigger();

private:
  void LoadParams();
  void InitializeAddress();
  void PrepareActionCommand();
  bool SetIntFeatureValue(const std::string& name, int64_t value);
  void SendActionCommand();

  ros::NodeHandle pnh_;
  ros::NodeHandle nh_;

  AVT::VmbAPI::VimbaSystem& vimba_system_;
  AVT::VmbAPI::InterfacePtr interface_ptr_;

  // uint32_t action_device_key_;
  // uint32_t action_group_key_;
  // uint32_t action_group_mask_;
  int action_device_key_;
  int action_group_key_;
  int action_group_mask_;

  struct in_addr destination_ip_;
};

}  // namespace trigger

#endif  // AVT_VIMBA_CAMERA_TRIGGER_H
