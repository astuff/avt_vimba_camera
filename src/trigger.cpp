#include "avt_vimba_camera/trigger.h"

namespace trigger
{

Trigger::Trigger() : vimba_system_(AVT::VmbAPI::VimbaSystem::GetInstance()), pnh_("~")
{
  vimba_system_.Startup();

  // Load ROS params
  LoadParams();

  InitializeAddress();
  PrepareActionCommand();

  SendActionCommand();
}

Trigger::~Trigger()
{
  vimba_system_.Shutdown();
}

void Trigger::LoadParams()
{
  std::string destination_ip;
  pnh_.param<int>("action_device_key", action_device_key_, 1);
  pnh_.param<int>("action_group_key", action_group_key_, 1);
  pnh_.param<int>("action_group_mask", action_group_mask_, 1);
  pnh_.param<std::string>("destination_ip", destination_ip, "192.168.3.40");

  if(inet_aton(destination_ip.c_str(), &destination_ip_) == 0)
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

void Trigger::PrepareActionCommand()
{
  VmbErrorType return_value = VmbErrorSuccess;

  if (SetIntFeatureValue("ActionDeviceKey", 1) &&
      SetIntFeatureValue("ActionGroupKey", 1) &&
      SetIntFeatureValue("ActionGroupMask", 1))
  {
    ROS_INFO("Action command ready");
  }
  else
  {
    ROS_ERROR("Failed to prepare action command");
    ros::shutdown();
  }
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

void Trigger::SendActionCommand()
{
  VmbErrorType return_value = VmbErrorSuccess;

  AVT::VmbAPI::FeaturePtr lFeature;
  return_value = vimba_system_.GetFeatureByName("ActionCommand", lFeature );

  if(return_value == VmbErrorSuccess)
  {
    return_value = lFeature->RunCommand();
  }

  if (return_value == VmbErrorSuccess)
  {
    ROS_DEBUG("Action command sent");
  }
  else
  {
    ROS_ERROR("Failed to send action command");
  }
}


}  // namespace trigger





