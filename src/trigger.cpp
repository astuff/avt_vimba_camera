#include "avt_vimba_camera/trigger.h"

namespace trigger
{
Trigger::Trigger() : vimba_system_(AVT::VmbAPI::VimbaSystem::GetInstance()), pnh_("~")
{
}

Trigger::~Trigger()
{
  vimba_system_.Shutdown();
}

void Trigger::Init()
{
  VmbErrorType return_value = vimba_system_.Startup();

  if (return_value != VmbErrorSuccess)
  {
    ROS_ERROR_STREAM("Failed to start Vimba system, vimba error code: " << return_value);
    ros::shutdown();
  }

  LoadParams();
  InitializeAddress();

  if (trigger_src_ == "timer")
  {
    trigger_timer_ = nh_.createTimer(ros::Duration(timer_period_), &Trigger::TimerCb, this);
  }
  else if (trigger_src_ == "subscriber")
  {
    trigger_sub_ = nh_.subscribe("trigger_input", 10, &Trigger::TriggerCb, this);
  }
  else
  {
    ROS_ERROR("Unknown trigger_src %s", trigger_src_.c_str());
    ros::shutdown();
  }
}

void Trigger::LoadParams()
{
  std::string destination_ip;
  pnh_.param<std::string>("destination_ip", destination_ip, "192.168.3.40");
  pnh_.param<std::string>("trigger_src", trigger_src_, "timer");
  pnh_.param<float>("timer_period", timer_period_, 0.1);
  pnh_.param<int>("action_device_key", action_device_key_, 1);
  pnh_.param<int>("action_group_key", action_group_key_, 1);
  pnh_.param<int>("action_group_mask", action_group_mask_, 1);

  if (inet_aton(destination_ip.c_str(), &destination_ip_) == 0)
  {
    ROS_ERROR("Unable to parse desination_ip: %s", destination_ip.c_str());
    ros::shutdown();
  }
}

void Trigger::InitializeAddress()
{
  VmbErrorType return_value = VmbErrorSuccess;

  if (!SetIntFeatureValue("GevActionDestinationIPAddress", destination_ip_.s_addr))
  {
    ROS_ERROR("Could not set destination address");
  }

  ROS_INFO("Destination address set");
}

bool Trigger::PrepareActionCommand()
{
  return (SetIntFeatureValue("ActionDeviceKey", 1) && SetIntFeatureValue("ActionGroupKey", 1) &&
          SetIntFeatureValue("ActionGroupMask", 1));
}

// Sets an integer feature value on the vimba system
bool Trigger::SetIntFeatureValue(const std::string& name, int64_t value)
{
  VmbErrorType return_value = VmbErrorSuccess;

  AVT::VmbAPI::FeaturePtr feature_ptr;
  return_value = vimba_system_.GetFeatureByName(name.c_str(), feature_ptr);

  if (return_value != VmbErrorSuccess)
  {
    ROS_ERROR_STREAM("Failed to get feature, vimba error code: " << return_value);
    return false;
  }
  else
  {
    return_value = feature_ptr->SetValue((VmbInt64_t)value);
  }

  return (return_value == VmbErrorSuccess);
}

void Trigger::TimerCb(const ros::TimerEvent& event)
{
  SendActionCommand();
}

void Trigger::TriggerCb(const std_msgs::Bool::ConstPtr& msg)
{
  SendActionCommand();
}

void Trigger::SendActionCommand()
{
  if (!PrepareActionCommand())
  {
    ROS_ERROR_THROTTLE(1.0, "Failed to prepare action command");
    return;
  }

  VmbErrorType return_value = VmbErrorSuccess;

  AVT::VmbAPI::FeaturePtr lFeature;
  return_value = vimba_system_.GetFeatureByName("ActionCommand", lFeature);

  if (return_value == VmbErrorSuccess)
  {
    return_value = lFeature->RunCommand();
  }

  if (return_value == VmbErrorSuccess)
  {
    ROS_DEBUG("Action command sent");
  }
  else
  {
    ROS_ERROR_THROTTLE(1.0, "Failed to send action command");
  }
}

}  // namespace trigger
