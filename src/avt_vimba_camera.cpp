/// Copyright (c) 2014,
/// Systems, Robotics and Vision Group
/// University of the Balearican Islands
/// All rights reserved.
///
/// Redistribution and use in source and binary forms, with or without
/// modification, are permitted provided that the following conditions are met:
///     * Redistributions of source code must retain the above copyright
///       notice, this list of conditions and the following disclaimer.
///     * Redistributions in binary form must reproduce the above copyright
///       notice, this list of conditions and the following disclaimer in the
///       documentation and/or other materials provided with the distribution.
///     * All advertising materials mentioning features or use of this software
///       must display the following acknowledgement:
///       This product includes software developed by
///       Systems, Robotics and Vision Group, Univ. of the Balearican Islands
///     * Neither the name of Systems, Robotics and Vision Group, University of
///       the Balearican Islands nor the names of its contributors may be used
///       to endorse or promote products derived from this software without
///       specific prior written permission.
///
/// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
/// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
/// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
/// ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
/// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
/// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
/// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
/// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
/// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
/// THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <avt_vimba_camera/avt_vimba_camera.h>
#include <avt_vimba_camera/avt_vimba_api.h>

#include <ros/ros.h>

#include <signal.h>
#include <thread>

namespace avt_vimba_camera
{
static const char* AutoMode[] = { "Off", "Once", "Continuous" };
static const char* TriggerMode[] = { "Invalid", "Freerun", "FixedRate", "Software", "Line0",  "Line1",
                                     "Line2",   "Line3",   "Line4",     "Action0",  "Action1" };
static const char* AcquisitionMode[] = { "Continuous", "SingleFrame", "MultiFrame", "Recorder" };
static const char* PixelFormatMode[] = { "Mono8",           "Mono10",          "Mono10Packed",    "Mono12",
                                         "Mono12Packed",    "BayerGR8",        "BayerRG8",        "BayerGB8",
                                         "BayerBG8",        "BayerGR10",       "BayerRG10",       "BayerGB10",
                                         "BayerBG10",       "BayerGR12",       "BayerRG12",       "BayerGB12",
                                         "BayerBG12",       "BayerGR10Packed", "BayerRG10Packed", "BayerGB10Packed",
                                         "BayerBG10Packed", "BayerGR12Packed", "BayerRG12Packed", "BayerGB12Packed",
                                         "BayerBG12Packed", "RGB8Packed",      "BGR8Packed" };
static const char* BalanceRatioMode[] = { "Red", "Blue" };
static const char* FeatureDataType[] = { "Unknown", "int", "float", "enum", "string", "bool" };

static const char* State[] = { "Opening", "Idle", "Camera not found", "Format error", "Error", "Ok" };

static volatile int keepRunning = 1;

void intHandler(int dummy)
{
  keepRunning = 0;
}

AvtVimbaCamera::AvtVimbaCamera() : AvtVimbaCamera(ros::this_node::getName().c_str())
{
}

AvtVimbaCamera::AvtVimbaCamera(const std::string& name)
{
  // Init global variables
  opened_ = false;     // camera connected to the api
  streaming_ = false;  // capturing frames
  on_init_ = true;     // on initialization phase
  name_ = name;

  camera_state_ = OPENING;

  updater_.setHardwareID("unknown");
  updater_.add(name_, this, &AvtVimbaCamera::getCurrentState);
  updater_.update();
}

void AvtVimbaCamera::start(const std::string& ip_str, const std::string& guid_str, const std::string& frame_id,
                           bool print_all_features)
{
  if (opened_)
    return;

  frame_id_ = frame_id;
  updater_.broadcast(0, "Starting device with IP:" + ip_str + " or GUID:" + guid_str);

  // Determine which camera to use. Try IP first
  if (!ip_str.empty())
  {
    diagnostic_msg_ = "Trying to open camera by IP: " + ip_str;
    ROS_INFO_STREAM("Trying to open camera by IP: " << ip_str);
    vimba_camera_ptr_ = openCamera(ip_str, print_all_features);
    if (!vimba_camera_ptr_)
    {
      ROS_WARN("Camera pointer is empty. Returning...");
      return;
    }
    updater_.setHardwareID(ip_str);
    guid_ = ip_str;
    // If both guid and IP are available, open by IP and check guid
    if (!guid_str.empty())
    {
      std::string cam_guid_str;
      vimba_camera_ptr_->GetSerialNumber(cam_guid_str);
      if (!vimba_camera_ptr_)
      {
        ROS_WARN("Camera pointer is empty. Returning...");
        return;
      }
      assert(cam_guid_str == guid_str);
      updater_.setHardwareID(guid_str);
      guid_ = guid_str;
      diagnostic_msg_ = "GUID " + cam_guid_str + " matches for camera with IP: " + ip_str;
      ROS_INFO_STREAM("GUID " << cam_guid_str << " matches for camera with IP: " << ip_str);
    }
  }
  else if (!guid_str.empty())
  {
    // Only guid available
    diagnostic_msg_ = "Trying to open camera by ID: " + guid_str;
    ROS_INFO_STREAM("Trying to open camera by ID: " << guid_str);
    vimba_camera_ptr_ = openCamera(guid_str, print_all_features);
    updater_.setHardwareID(guid_str);
    guid_ = guid_str;
  }
  else
  {
    // No identifying info (GUID and IP) are available
    diagnostic_msg_ = "Can't connect to the camera: at least GUID or IP need to be set.";
    ROS_ERROR("Can't connect to the camera: at least GUID or IP need to be set.");
    camera_state_ = ERROR;
  }
  updater_.update();

  getFeatureValue("GevTimestampTickFrequency", vimba_timestamp_tick_freq_);

  // From the SynchronousGrab API example:
  // TODO Set the GeV packet size to the highest possible value
  VmbInterfaceType cam_int_type;
  vimba_camera_ptr_->GetInterfaceType(cam_int_type);
  if (cam_int_type == VmbInterfaceEthernet)
  {
    runCommand("GVSPAdjustPacketSize");
  }

  // Create a frame observer for this camera
  SP_SET(frame_obs_ptr_,
         new FrameObserver(vimba_camera_ptr_,
                           std::bind(&avt_vimba_camera::AvtVimbaCamera::frameCallback, this, std::placeholders::_1)));
  camera_state_ = IDLE;

  updater_.update();
}

void AvtVimbaCamera::stop()
{
  if (!opened_)
    return;
  vimba_camera_ptr_->Close();
  opened_ = false;
}

void AvtVimbaCamera::startImaging()
{
  if (!streaming_)
  {
    // Start streaming
    VmbErrorType err = vimba_camera_ptr_->StartContinuousImageAcquisition(3, IFrameObserverPtr(frame_obs_ptr_));
    if (err == VmbErrorSuccess)
    {
      diagnostic_msg_ = "Starting continuous image acquisition";
      ROS_INFO_STREAM("Starting continuous image acquisition ...");
      streaming_ = true;
      camera_state_ = OK;
    }
    else
    {
      diagnostic_msg_ = "Could not start continuous image acquisition. Error: " + api_.errorCodeToMessage(err);
      ROS_ERROR_STREAM("Could not start continuous image acquisition. "
                       << "\n Error: " << api_.errorCodeToMessage(err));
      camera_state_ = ERROR;
    }
  }
  else
  {
    ROS_WARN_STREAM("Start imaging called, but the camera is already imaging.");
  }
  updater_.update();
}

void AvtVimbaCamera::stopImaging()
{
  if (streaming_ || on_init_)
  {
    VmbErrorType err = vimba_camera_ptr_->StopContinuousImageAcquisition();
    if (err == VmbErrorSuccess)
    {
      diagnostic_msg_ = "Acquisition stopped";
      ROS_INFO_STREAM("Acquisition stoppped ...");
      streaming_ = false;
      camera_state_ = IDLE;
    }
    else
    {
      diagnostic_msg_ = "Could not stop image acquisition. Error: " + api_.errorCodeToMessage(err);
      ROS_ERROR_STREAM("Could not stop image acquisition."
                       << "\n Error: " << api_.errorCodeToMessage(err));
      camera_state_ = ERROR;
    }
  }
  else
  {
    ROS_WARN_STREAM("Stop imaging called, but the camera is already stopped.");
  }
  updater_.update();
}

CameraPtr AvtVimbaCamera::openCamera(const std::string& id_str, bool print_all_features)
{
  // Details:   The ID might be one of the following:
  //            "IP:169.254.12.13",
  //            "MAC:000f31000001",
  //            or a plain serial number: "1234567890".

  CameraPtr camera;
  VimbaSystem& vimba_system(VimbaSystem::GetInstance());

  // set handler to catch ctrl+c presses
  sighandler_t oldHandler = signal(SIGINT, intHandler);

  // get camera
  VmbErrorType err = vimba_system.GetCameraByID(id_str.c_str(), camera);
  while (err != VmbErrorSuccess)
  {
    if (keepRunning)
    {
      ROS_WARN_STREAM("Could not find camera using " << id_str << ". Retrying every two seconds ...");
      ros::Duration(2.0).sleep();
      err = vimba_system.GetCameraByID(id_str.c_str(), camera);
    }
    else
    {
      ROS_ERROR_STREAM("Could not find camera using " << id_str << "\n Error: " << api_.errorCodeToMessage(err));
      camera_state_ = CAMERA_NOT_FOUND;
      return camera;
    }
  }

  // open camera
  err = camera->Open(VmbAccessModeFull);
  while (err != VmbErrorSuccess && keepRunning)
  {
    if (keepRunning)
    {
      ROS_WARN_STREAM("Could not open camera. Retrying every two seconds ...");
      err = camera->Open(VmbAccessModeFull);
      ros::Duration(2.0).sleep();
    }
    else
    {
      ROS_ERROR_STREAM("Could not open camera " << id_str << "\n Error: " << api_.errorCodeToMessage(err));
      camera_state_ = CAMERA_NOT_FOUND;
      return camera;
    }
  }

  // set previous handler back
  signal(SIGINT, oldHandler);

  std::string cam_id, cam_name;
  camera->GetID(cam_id);
  camera->GetName(cam_name);
  ROS_INFO_STREAM("Opened connection to camera named " << cam_name << " with ID " << cam_id);

  ros::Duration(2.0).sleep();

  if (print_all_features)
  {
    printAllCameraFeatures(camera);
  }
  opened_ = true;
  camera_state_ = IDLE;
  return camera;
}

void AvtVimbaCamera::frameCallback(const FramePtr vimba_frame_ptr)
{
  std::unique_lock<std::mutex> lock(config_mutex_);
  camera_state_ = OK;
  diagnostic_msg_ = "Camera operating normally";

  // Call the callback implemented by other classes
  std::thread thread_callback = std::thread(userFrameCallback, vimba_frame_ptr);
  thread_callback.join();

  updater_.update();
}

double AvtVimbaCamera::getTimestamp()
{
  double timestamp = -1.0;
  if (runCommand("GevTimestampControlLatch"))
  {
    VmbInt64_t freq, ticks;
    getFeatureValue("GevTimestampTickFrequency", freq);
    getFeatureValue("GevTimestampValue", ticks);
    timestamp = static_cast<double>(ticks) / static_cast<double>(freq);
  }
  return timestamp;
}

double AvtVimbaCamera::getDeviceTemp()
{
  double temp = -1.0;
  if (setFeatureValue("DeviceTemperatureSelector", "Main") == VmbErrorSuccess)
  {
    getFeatureValue("DeviceTemperature", temp);
  }
  return temp;
}

int AvtVimbaCamera::getSensorWidth()
{
  int sensor_width;
  if (getFeatureValue("SensorWidth", sensor_width))
  {
    return sensor_width;
  }
  else
  {
    return -1;
  }
}

int AvtVimbaCamera::getSensorHeight()
{
  int sensor_height;
  if (getFeatureValue("SensorHeight", sensor_height))
  {
    return sensor_height;
  }
  else
  {
    return -1;
  }
}

double AvtVimbaCamera::getTimestampRealTime(VmbUint64_t timestamp_ticks)
{
  return (static_cast<double>(timestamp_ticks)) / (static_cast<double>(vimba_timestamp_tick_freq_));
}

// Template function to SET a feature value from the camera
template <typename T>
VmbErrorType AvtVimbaCamera::setFeatureValue(const std::string& feature_str, const T& val)
{
  VmbErrorType err;
  FeaturePtr vimba_feature_ptr;
  err = vimba_camera_ptr_->GetFeatureByName(feature_str.c_str(), vimba_feature_ptr);
  if (err == VmbErrorSuccess)
  {
    bool writable;
    err = vimba_feature_ptr->IsWritable(writable);
    if (err == VmbErrorSuccess)
    {
      if (writable)
      {
        ROS_DEBUG_STREAM("Setting feature " << feature_str << " value " << val);
        VmbFeatureDataType data_type;
        err = vimba_feature_ptr->GetDataType(data_type);
        if (err == VmbErrorSuccess)
        {
          if (data_type == VmbFeatureDataEnum)
          {
            bool available;
            err = vimba_feature_ptr->IsValueAvailable(val, available);
            if (err == VmbErrorSuccess)
            {
              if (available)
              {
                err = vimba_feature_ptr->SetValue(val);
              }
              else
              {
                ROS_WARN_STREAM("Feature " << feature_str << " is available now.");
              }
            }
            else
            {
              ROS_WARN_STREAM("Feature " << feature_str << ": value unavailable\n\tERROR "
                                         << api_.errorCodeToMessage(err));
            }
          }
          else
          {
            err = vimba_feature_ptr->SetValue(val);
          }
        }
        else
        {
          ROS_WARN_STREAM("Feature " << feature_str << ": Bad data type\n\tERROR " << api_.errorCodeToMessage(err));
        }
      }
      else
      {
        ROS_WARN_STREAM("Feature " << feature_str << " is not writable.");
      }
    }
    else
    {
      ROS_WARN_STREAM("Feature " << feature_str << ": ERROR " << api_.errorCodeToMessage(err));
    }
  }
  else
  {
    ROS_WARN_STREAM("Could not get feature " << feature_str << ", your camera probably doesn't support it.");
  }
  return err;
}

// Template function to GET a feature value from the camera
template <typename T>
bool AvtVimbaCamera::getFeatureValue(const std::string& feature_str, T& val)
{
  ROS_DEBUG_STREAM("Asking for feature " << feature_str);
  VmbErrorType err;
  FeaturePtr vimba_feature_ptr;
  VmbFeatureDataType data_type;
  err = vimba_camera_ptr_->GetFeatureByName(feature_str.c_str(), vimba_feature_ptr);
  if (err == VmbErrorSuccess)
  {
    bool readable;
    vimba_feature_ptr->IsReadable(readable);
    if (readable)
    {
      vimba_feature_ptr->GetDataType(data_type);
      if (err != VmbErrorSuccess)
      {
        std::cout << "[Could not get feature Data Type. Error code: " << err << "]" << std::endl;
      }
      else
      {
        switch (data_type)
        {
          case VmbFeatureDataBool:
            bool bValue;
            err = vimba_feature_ptr->GetValue(bValue);
            if (err == VmbErrorSuccess)
            {
              val = static_cast<T>(bValue);
            }
            break;
          case VmbFeatureDataFloat:
            double fValue;
            err = vimba_feature_ptr->GetValue(fValue);
            if (err == VmbErrorSuccess)
            {
              val = static_cast<T>(fValue);
            }
            break;
          case VmbFeatureDataInt:
            VmbInt64_t nValue;
            err = vimba_feature_ptr->GetValue(nValue);
            if (err == VmbErrorSuccess)
            {
              val = static_cast<T>(nValue);
            }
            break;
          default:
            err = VmbErrorNotFound;
            break;
        }
        if (err != VmbErrorSuccess)
        {
          ROS_WARN_STREAM("Could not get feature value. Error code: " << api_.errorCodeToMessage(err));
        }
      }
    }
    else
    {
      ROS_WARN_STREAM("Feature " << feature_str << " is not readable.");
    }
  }
  else
  {
    ROS_WARN_STREAM("Could not get feature " << feature_str);
  }
  return (err == VmbErrorSuccess);
}

// Function to GET a feature value from the camera, overloaded to strings
bool AvtVimbaCamera::getFeatureValue(const std::string& feature_str, std::string& val)
{
  ROS_DEBUG_STREAM("Asking for feature " << feature_str);
  VmbErrorType err;
  FeaturePtr vimba_feature_ptr;
  VmbFeatureDataType data_type;
  err = vimba_camera_ptr_->GetFeatureByName(feature_str.c_str(), vimba_feature_ptr);
  if (err == VmbErrorSuccess)
  {
    bool readable;
    vimba_feature_ptr->IsReadable(readable);
    if (readable)
    {
      vimba_feature_ptr->GetDataType(data_type);
      if (err != VmbErrorSuccess)
      {
        ROS_ERROR_STREAM("[Could not get feature Data Type. Error code: " << err << "]");
      }
      else
      {
        std::string strValue;
        switch (data_type)
        {
          case VmbFeatureDataEnum:
            err = vimba_feature_ptr->GetValue(strValue);
            if (err == VmbErrorSuccess)
            {
              val = strValue;
            }
            break;
          case VmbFeatureDataString:
            err = vimba_feature_ptr->GetValue(strValue);
            if (err == VmbErrorSuccess)
            {
              val = strValue;
            }
            break;
          default:
            err = VmbErrorNotFound;
            break;
        }
        if (err != VmbErrorSuccess)
        {
          ROS_WARN_STREAM("Could not get feature value. Error code: " << api_.errorCodeToMessage(err));
        }
      }
    }
    else
    {
      ROS_WARN_STREAM("Feature " << feature_str << " is not readable.");
    }
  }
  else
  {
    ROS_WARN_STREAM("Could not get feature " << feature_str);
  }
  return (err == VmbErrorSuccess);
}

// Tries to configure a camera feature.
// Updates the config value passed in if a different config is in use by the camera.
template <typename Vimba_Type, typename Std_Type>
void AvtVimbaCamera::configureFeature(const std::string& feature_str, const Vimba_Type& val_in, Std_Type& val_out)
{
  Vimba_Type actual_value;

  VmbErrorType return_value = setFeatureValue(feature_str, val_in);
  if (return_value == VmbErrorSuccess || return_value == VmbErrorInvalidValue)
  {
    getFeatureValue(feature_str, actual_value);
    if (val_in == actual_value)
    {
      ROS_INFO_STREAM(" - " << feature_str << " set to " << actual_value);
    }
    else
    {
      ROS_WARN_STREAM(" - Tried to set " << feature_str << " to " << val_in << " but the camera used " << actual_value
                                         << " instead");
      val_out = static_cast<Std_Type>(actual_value);
    }
  }
  else
  {
    val_out = static_cast<Std_Type>(val_in);
  }
}

// Overloaded for strings, template specialization doesn't currently compile with GCC
void AvtVimbaCamera::configureFeature(const std::string& feature_str, const std::string& val_in, std::string& val_out)
{
  std::string actual_value;

  VmbErrorType return_value = setFeatureValue(feature_str, val_in.c_str());
  if (return_value == VmbErrorSuccess || return_value == VmbErrorInvalidValue)
  {
    getFeatureValue(feature_str, actual_value);
    if (val_in == actual_value)
    {
      ROS_INFO_STREAM(" - " << feature_str << " set to " << actual_value);
    }
    else
    {
      ROS_WARN_STREAM(" - Tried to set " << feature_str << " to " << val_in << " but the camera used " << actual_value
                                         << " instead");
      val_out = actual_value;
    }
  }
  else
  {
    val_out = val_in;
  }
}

// Template function to RUN a command
bool AvtVimbaCamera::runCommand(const std::string& command_str)
{
  FeaturePtr feature_ptr;
  VmbErrorType err = vimba_camera_ptr_->GetFeatureByName(command_str.c_str(), feature_ptr);
  if (err == VmbErrorSuccess)
  {
    err = feature_ptr->RunCommand();
    if (err == VmbErrorSuccess)
    {
      bool is_command_done = false;
      do
      {
        err = feature_ptr->IsCommandDone(is_command_done);
        if (err != VmbErrorSuccess)
        {
          break;
        }
        ROS_DEBUG_STREAM_THROTTLE(1, "Waiting for command " << command_str.c_str() << "...");
      } while (false == is_command_done);
      ROS_DEBUG_STREAM("Command " << command_str.c_str() << " done!");
      return true;
    }
    else
    {
      ROS_WARN_STREAM("Could not run command " << command_str << ". Error: " << api_.errorCodeToMessage(err));
      return false;
    }
  }
  else
  {
    ROS_WARN_STREAM("Could not get feature command " << command_str << ". Error: " << api_.errorCodeToMessage(err));
    return false;
  }
}

void AvtVimbaCamera::printAllCameraFeatures(const CameraPtr& camera)
{
  VmbErrorType err;
  FeaturePtrVector features;

  // The static details of a feature
  std::string strName;           // The name of the feature
  std::string strDisplayName;    // The display name of the feature
  std::string strDescription;    // A long description of the feature
  std::string strCategory;       // A category to group features
  std::string strSFNCNamespace;  // The Std Feature Naming Convention namespace
  std::string strUnit;           // The measurement unit of the value
  VmbFeatureDataType eType;      // The data type of the feature

  // The changeable value of a feature
  VmbInt64_t nValue;
  std::string strValue;

  std::stringstream strError;

  // Fetch all features of our cam
  err = camera->GetFeatures(features);
  if (err == VmbErrorSuccess)
  {
    // Query all static details as well as the value of
    // all fetched features and print them out.
    for (FeaturePtrVector::const_iterator iter = features.begin(); features.end() != iter; ++iter)
    {
      err = (*iter)->GetName(strName);
      if (err != VmbErrorSuccess)
      {
        strError << "[Could not get feature Name. Error code: " << err << "]";
        strName.assign(strError.str());
      }

      err = (*iter)->GetDisplayName(strDisplayName);
      if (err != VmbErrorSuccess)
      {
        strError << "[Could not get feature Display Name. Error code: " << err << "]";
        strDisplayName.assign(strError.str());
      }

      err = (*iter)->GetDescription(strDescription);
      if (err != VmbErrorSuccess)
      {
        strError << "[Could not get feature Description. Error code: " << err << "]";
        strDescription.assign(strError.str());
      }

      err = (*iter)->GetCategory(strCategory);
      if (err != VmbErrorSuccess)
      {
        strError << "[Could not get feature Category. Error code: " << err << "]";
        strCategory.assign(strError.str());
      }

      err = (*iter)->GetSFNCNamespace(strSFNCNamespace);
      if (err != VmbErrorSuccess)
      {
        strError << "[Could not get feature SNFC Namespace. Error code: " << err << "]";
        strSFNCNamespace.assign(strError.str());
      }

      err = (*iter)->GetUnit(strUnit);
      if (err != VmbErrorSuccess)
      {
        strError << "[Could not get feature Unit. Error code: " << err << "]";
        strUnit.assign(strError.str());
      }

      std::cout << "/// Feature Name: " << strName << std::endl;
      std::cout << "/// Display Name: " << strDisplayName << std::endl;
      std::cout << "/// Description: " << strDescription << std::endl;
      std::cout << "/// SNFC Namespace: " << strSFNCNamespace << std::endl;
      std::cout << "/// Unit: " << strUnit << std::endl;
      std::cout << "/// Value: ";

      err = (*iter)->GetDataType(eType);
      if (err != VmbErrorSuccess)
      {
        std::cout << "[Could not get feature Data Type. Error code: " << err << "]" << std::endl;
      }
      else
      {
        switch (eType)
        {
          case VmbFeatureDataBool:
            bool bValue;
            err = (*iter)->GetValue(bValue);
            if (err == VmbErrorSuccess)
            {
              std::cout << bValue << " (bool)" << std::endl;
            }
            break;
          case VmbFeatureDataEnum:
            err = (*iter)->GetValue(strValue);
            if (err == VmbErrorSuccess)
            {
              std::cout << strValue << " (string enum)" << std::endl;
            }
            break;
          case VmbFeatureDataFloat:
            double fValue;
            err = (*iter)->GetValue(fValue);
            {
              std::cout << fValue << " (float)" << std::endl;
            }
            break;
          case VmbFeatureDataInt:
            err = (*iter)->GetValue(nValue);
            {
              std::cout << nValue << " (int)" << std::endl;
            }
            break;
          case VmbFeatureDataString:
            err = (*iter)->GetValue(strValue);
            {
              std::cout << strValue << " (string)" << std::endl;
            }
            break;
          case VmbFeatureDataCommand:
          default:
            std::cout << "[None]" << std::endl;
            break;
        }
        if (err != VmbErrorSuccess)
        {
          std::cout << "Could not get feature value. Error code: " << err << std::endl;
        }
      }

      std::cout << std::endl;
    }
  }
  else
  {
    std::cout << "Could not get features. Error code: " << api_.errorCodeToMessage(err) << std::endl;
  }
}

void AvtVimbaCamera::updateConfig(Config& config)
{
  std::unique_lock<std::mutex> lock(config_mutex_);

  if (streaming_)
  {
    stopImaging();
    ros::Duration(0.5).sleep();  // sleep for half a second
  }

  if (on_init_)
  {
    config_ = config;
  }
  diagnostic_msg_ = "Updating configuration";

  updateExposureConfig(config);
  updateGainConfig(config);
  updateWhiteBalanceConfig(config);
  updateImageModeConfig(config);
  updateROIConfig(config);
  updateBandwidthConfig(config);
  updateGPIOConfig(config);
  updateUSBGPIOConfig(config);
  updatePtpModeConfig(config);
  updatePixelFormatConfig(config);
  updateAcquisitionConfig(config);
  updateIrisConfig(config);
  config_ = config;

  if (on_init_)
  {
    on_init_ = false;
  }

  startImaging();
}

/** Change the Trigger configuration */
void AvtVimbaCamera::updateAcquisitionConfig(Config& config)
{
  if (on_init_)
  {
    ROS_INFO("Updating Acquisition and Trigger config:");
  }

  if (config.acquisition_mode != config_.acquisition_mode || on_init_)
  {
    configureFeature("AcquisitionMode", config.acquisition_mode, config.acquisition_mode);
  }
  if (config.acquisition_rate != config_.acquisition_rate || on_init_)
  {
    configureFeature("AcquisitionFrameRateAbs", static_cast<float>(config.acquisition_rate), config.acquisition_rate);
  }
  if (config.trigger_mode != config_.trigger_mode || on_init_)
  {
    configureFeature("TriggerMode", config.trigger_mode, config.trigger_mode);
  }
  if (config.trigger_selector != config_.trigger_selector || on_init_)
  {
    configureFeature("TriggerSelector", config.trigger_selector, config.trigger_selector);
  }
  if (config.trigger_source != config_.trigger_source || on_init_)
  {
    configureFeature("TriggerSource", config.trigger_source, config.trigger_source);
  }
  if (config.trigger_activation != config_.trigger_activation || on_init_)
  {
    configureFeature("TriggerActivation", config.trigger_activation, config.trigger_activation);
  }
  if (config.trigger_delay != config_.trigger_delay || on_init_)
  {
    configureFeature("TriggerDelayAbs", static_cast<float>(config.trigger_delay), config.trigger_delay);
  }
  if (config.action_device_key != config_.action_device_key || on_init_)
  {
    configureFeature("ActionDeviceKey", static_cast<VmbInt64_t>(config.action_device_key), config.action_device_key);
  }
  if (config.action_group_key != config_.action_group_key || on_init_)
  {
    configureFeature("ActionGroupKey", static_cast<VmbInt64_t>(config.action_group_key), config.action_group_key);
  }
  if (config.action_group_mask != config_.action_group_mask || on_init_)
  {
    configureFeature("ActionGroupMask", static_cast<VmbInt64_t>(config.action_group_mask), config.action_group_mask);
  }
}

/* Update the Iris config */
void AvtVimbaCamera::updateIrisConfig(Config& config)
{
  if (on_init_)
  {
    ROS_INFO("Updating Iris config:");
  }

  if (config.iris_auto_target != config_.iris_auto_target || on_init_)
  {
    configureFeature("IrisAutoTarget", static_cast<VmbInt64_t>(config.iris_auto_target), config.iris_auto_target);
  }
  if (config.iris_mode != config_.iris_mode || on_init_)
  {
    configureFeature("IrisMode", config.iris_mode, config.iris_mode);
  }
  if (config.iris_video_level_max != config_.iris_video_level_max || on_init_)
  {
    configureFeature("IrisVideoLevelMax", static_cast<VmbInt64_t>(config.iris_video_level_max),
                     config.iris_video_level_max);
  }
  if (config.iris_video_level_min != config_.iris_video_level_min || on_init_)
  {
    configureFeature("IrisVideoLevelMin", static_cast<VmbInt64_t>(config.iris_video_level_min),
                     config.iris_video_level_min);
  }
}

/** Change the Exposure configuration */
void AvtVimbaCamera::updateExposureConfig(Config& config)
{
  if (on_init_)
  {
    ROS_INFO("Updating Exposure config:");
  }

  if (config.exposure != config_.exposure || on_init_)
  {
    configureFeature("ExposureTimeAbs", static_cast<float>(config.exposure), config.exposure);
  }
  if (config.exposure_auto != config_.exposure_auto || on_init_)
  {
    configureFeature("ExposureAuto", config.exposure_auto, config.exposure_auto);
  }
  if (config.exposure_auto_alg != config_.exposure_auto_alg || on_init_)
  {
    configureFeature("ExposureAutoAlg", config.exposure_auto_alg, config.exposure_auto_alg);
  }
  if (config.exposure_auto_tol != config_.exposure_auto_tol || on_init_)
  {
    configureFeature("ExposureAutoAdjustTol", static_cast<VmbInt64_t>(config.exposure_auto_tol),
                     config.exposure_auto_tol);
  }
  if (config.exposure_auto_max != config_.exposure_auto_max || on_init_)
  {
    configureFeature("ExposureAutoMax", static_cast<VmbInt64_t>(config.exposure_auto_max), config.exposure_auto_max);
  }
  if (config.exposure_auto_min != config_.exposure_auto_min || on_init_)
  {
    configureFeature("ExposureAutoMin", static_cast<VmbInt64_t>(config.exposure_auto_min), config.exposure_auto_min);
  }
  if (config.exposure_auto_outliers != config_.exposure_auto_outliers || on_init_)
  {
    configureFeature("ExposureAutoOutliers", static_cast<VmbInt64_t>(config.exposure_auto_outliers),
                     config.exposure_auto_outliers);
  }
  if (config.exposure_auto_rate != config_.exposure_auto_rate || on_init_)
  {
    configureFeature("ExposureAutoRate", static_cast<VmbInt64_t>(config.exposure_auto_rate), config.exposure_auto_rate);
  }
  if (config.exposure_auto_target != config_.exposure_auto_target || on_init_)
  {
    configureFeature("ExposureAutoTarget", static_cast<VmbInt64_t>(config.exposure_auto_target),
                     config.exposure_auto_target);
  }
}

/** Change the Gain configuration */
void AvtVimbaCamera::updateGainConfig(Config& config)
{
  if (on_init_)
  {
    ROS_INFO("Updating Gain config:");
  }

  if (config.gain != config_.gain || on_init_)
  {
    configureFeature("Gain", static_cast<float>(config.gain), config.gain);
  }
  if (config.gain_auto != config_.gain_auto || on_init_)
  {
    configureFeature("GainAuto", config.gain_auto, config.gain_auto);
  }
  if (config.gain_auto_adjust_tol != config_.gain_auto_adjust_tol || on_init_)
  {
    configureFeature("GainAutoAdjustTol", static_cast<VmbInt64_t>(config.gain_auto_adjust_tol),
                     config.gain_auto_adjust_tol);
  }
  if (config.gain_auto_max != config_.gain_auto_max || on_init_)
  {
    configureFeature("GainAutoMax", static_cast<float>(config.gain_auto_max), config.gain_auto_max);
  }
  if (config.gain_auto_min != config_.gain_auto_min || on_init_)
  {
    configureFeature("GainAutoMin", static_cast<float>(config.gain_auto_min), config.gain_auto_min);
  }
  if (config.gain_auto_outliers != config_.gain_auto_outliers || on_init_)
  {
    configureFeature("GainAutoOutliers", static_cast<VmbInt64_t>(config.gain_auto_outliers), config.gain_auto_outliers);
  }
  if (config.gain_auto_rate != config_.gain_auto_rate || on_init_)
  {
    configureFeature("GainAutoRate", static_cast<VmbInt64_t>(config.gain_auto_rate), config.gain_auto_rate);
  }
  if (config.gain_auto_target != config_.gain_auto_target || on_init_)
  {
    configureFeature("GainAutoTarget", static_cast<VmbInt64_t>(config.gain_auto_target), config.gain_auto_target);
  }
}

/** Change the White Balance configuration */
void AvtVimbaCamera::updateWhiteBalanceConfig(Config& config)
{
  if (on_init_)
  {
    ROS_INFO("Updating White Balance config:");
  }

  if (config.balance_ratio_abs != config_.balance_ratio_abs || on_init_)
  {
    configureFeature("BalanceRatioAbs", static_cast<float>(config.balance_ratio_abs), config.balance_ratio_abs);
  }
  if (config.balance_ratio_selector != config_.balance_ratio_selector || on_init_)
  {
    configureFeature("BalanceRatioSelector", config.balance_ratio_selector, config.balance_ratio_selector);
  }
  if (config.whitebalance_auto != config_.whitebalance_auto || on_init_)
  {
    configureFeature("BalanceWhiteAuto", config.whitebalance_auto, config.whitebalance_auto);
  }
  if (config.whitebalance_auto_tol != config_.whitebalance_auto_tol || on_init_)
  {
    configureFeature("BalanceWhiteAutoAdjustTol", static_cast<VmbInt64_t>(config.whitebalance_auto_tol),
                     config.whitebalance_auto_tol);
  }
  if (config.whitebalance_auto_rate != config_.whitebalance_auto_rate || on_init_)
  {
    configureFeature("BalanceWhiteAutoRate", static_cast<VmbInt64_t>(config.whitebalance_auto_rate),
                     config.whitebalance_auto_rate);
  }
}

/** Change the Binning and Decimation configuration */
void AvtVimbaCamera::updatePtpModeConfig(Config& config)
{
  if (on_init_)
  {
    ROS_INFO("Updating PTP config:");
  }

  if (config.ptp_mode != config_.ptp_mode || on_init_)
  {
    // configureFeature("PtpMode", config.ptp_mode, config.ptp_mode);
    configureFeature("PtpMode", config.ptp_mode, config.ptp_mode);
  }
}

/** Change the Binning and Decimation configuration */
void AvtVimbaCamera::updateImageModeConfig(Config& config)
{
  if (on_init_)
  {
    ROS_INFO("Updating Image Mode config:");
  }

  if (config.decimation_x != config_.decimation_x || on_init_)
  {
    configureFeature("DecimationHorizontal", static_cast<VmbInt64_t>(config.decimation_x), config.decimation_x);
  }
  if (config.decimation_y != config_.decimation_y || on_init_)
  {
    configureFeature("DecimationVertical", static_cast<VmbInt64_t>(config.decimation_y), config.decimation_y);
  }
  if (config.binning_x != config_.binning_x || on_init_)
  {
    configureFeature("BinningHorizontal", static_cast<VmbInt64_t>(config.binning_x), config.binning_x);
  }
  if (config.binning_y != config_.binning_y || on_init_)
  {
    configureFeature("BinningVertical", static_cast<VmbInt64_t>(config.binning_y), config.binning_y);
  }
}

/** Change the ROI configuration */
void AvtVimbaCamera::updateROIConfig(Config& config)
{
  if (on_init_)
  {
    ROS_INFO("Updating ROI config:");
  }

  if (config.width != config_.width || on_init_)
  {
    configureFeature("Width", static_cast<VmbInt64_t>(config.width), config.width);
  }
  if (config.height != config_.height || on_init_)
  {
    configureFeature("Height", static_cast<VmbInt64_t>(config.height), config.height);
  }
  if (config.offset_x != config_.offset_x || on_init_)
  {
    configureFeature("OffsetX", static_cast<VmbInt64_t>(config.offset_x), config.offset_x);
  }
  if (config.offset_y != config_.offset_y || on_init_)
  {
    configureFeature("OffsetY", static_cast<VmbInt64_t>(config.offset_y), config.offset_y);
  }
}

/** Change the Bandwidth configuration */
void AvtVimbaCamera::updateBandwidthConfig(Config& config)
{
  if (on_init_)
  {
    ROS_INFO("Updating Bandwidth config:");
  }

  if (config.stream_bytes_per_second != config_.stream_bytes_per_second || on_init_)
  {
    configureFeature("StreamBytesPerSecond", static_cast<VmbInt64_t>(config.stream_bytes_per_second),
                     config.stream_bytes_per_second);
  }
}

/** Change the Pixel Format configuration */
void AvtVimbaCamera::updatePixelFormatConfig(Config& config)
{
  if (on_init_)
  {
    ROS_INFO("Updating PixelFormat config:");
  }

  if (config.pixel_format != config_.pixel_format || on_init_)
  {
    configureFeature("PixelFormat", config.pixel_format, config.pixel_format);
  }
}

/** Change the Gige GPIO configuration */
void AvtVimbaCamera::updateGPIOConfig(Config& config)
{
  if (on_init_)
  {
    ROS_INFO("Updating GPIO config:");
  }
  if (config.sync_in_selector != config_.sync_in_selector || on_init_)
  {
    configureFeature("SyncInSelector", config.sync_in_selector, config.sync_in_selector);
  }
  if (config.sync_out_polarity != config_.sync_out_polarity || on_init_)
  {
    configureFeature("SyncOutPolarity", config.sync_out_polarity, config.sync_out_polarity);
  }
  if (config.sync_out_selector != config_.sync_out_selector || on_init_)
  {
    configureFeature("SyncOutSelector", config.sync_out_selector, config.sync_out_selector);
  }
  if (config.sync_out_source != config_.sync_out_source || on_init_)
  {
    configureFeature("SyncOutSource", config.sync_out_source, config.sync_out_source);
  }
}

/** Change the USB GPIO configuration */
void AvtVimbaCamera::updateUSBGPIOConfig(Config& config)
{
  if (on_init_)
  {
    ROS_INFO("Updating USB GPIO config:");
  }
  if (config.line_selector != config_.line_selector || on_init_)
  {
    configureFeature("LineSelector", config.line_selector, config.line_selector);
  }
  if (config.line_mode != config_.line_mode || on_init_)
  {
    configureFeature("LineMode", config.line_mode, config.line_mode);
  }
}

void AvtVimbaCamera::getCurrentState(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  stat.add("ID", guid_);
  stat.add("Info", diagnostic_msg_);
  stat.add("Temperature", getDeviceTemp());

  switch (camera_state_)
  {
    case OPENING:
      stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Opening camera");
      break;
    case IDLE:
      stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Camera is idle");
      break;
    case OK:
      stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Camera is streaming");
      break;
    case CAMERA_NOT_FOUND:
      stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Cannot find requested camera %s", guid_.c_str());
      break;
    case FORMAT_ERROR:
      stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Problem retrieving frame");
      break;
    case ERROR:
      stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Camera has encountered an error");
      break;
    default:
      stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Camera is in unknown state");
      break;
  }
}
}  // namespace avt_vimba_camera
