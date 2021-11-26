#ifndef AVT_VIMBA_CAMERA_TRIGGER_H
#define AVT_VIMBA_CAMERA_TRIGGER_H

#include "VimbaCPP/Include/VimbaCPP.h"

#include <arpa/inet.h>

#include <ros/ros.h>

#include <std_msgs/Bool.h>

namespace trigger
{
class Trigger
{
public:
  Trigger();
  ~Trigger();

  void Init();

private:
  void LoadParams();
  void InitializeAddress();
  bool PrepareActionCommand();
  bool SetIntFeatureValue(const std::string& name, int64_t value);

  void TimerCb(const ros::TimerEvent& event);
  void TriggerCb(const std_msgs::Bool::ConstPtr& msg);
  void SendActionCommand();

  AVT::VmbAPI::VimbaSystem& vimba_system_;
  AVT::VmbAPI::InterfacePtr interface_ptr_;

  ros::NodeHandle pnh_;
  ros::NodeHandle nh_;

  ros::Timer trigger_timer_;
  ros::Subscriber trigger_sub_;

  // Params
  struct in_addr destination_ip_;
  std::string trigger_src_;
  float timer_period_;
  int action_device_key_;
  int action_group_key_;
  int action_group_mask_;
};

}  // namespace trigger

#endif  // AVT_VIMBA_CAMERA_TRIGGER_H
