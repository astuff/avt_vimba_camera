/// Copyright (c) 2014,
/// Systems, Robotics and Vision Group
/// University of the Balearic Islands
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
///       Systems, Robotics and Vision Group, Univ. of the Balearic Islands
///     * Neither the name of Systems, Robotics and Vision Group, University of
///       the Balearic Islands nor the names of its contributors may be used
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

#ifndef AVT_VIMBA_CAMERA_H
#define AVT_VIMBA_CAMERA_H

#include <VimbaCPP/Include/VimbaCPP.h>

#include <avt_vimba_camera/AvtVimbaCameraConfig.h>
#include <avt_vimba_camera/frame_observer.h>
#include <avt_vimba_camera/avt_vimba_api.h>

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

#include <string>
#include <mutex>

using AVT::VmbAPI::CameraPtr;
using AVT::VmbAPI::FramePtr;
using AVT::VmbAPI::IFrameObserverPtr;
using AVT::VmbAPI::VimbaSystem;

namespace avt_vimba_camera
{
enum CameraState
{
  OPENING,
  IDLE,
  CAMERA_NOT_FOUND,
  FORMAT_ERROR,
  ERROR,
  OK
};

class AvtVimbaCamera
{
public:
  typedef avt_vimba_camera::AvtVimbaCameraConfig Config;
  typedef std::function<void(const FramePtr)> frameCallbackFunc;

  AvtVimbaCamera();
  AvtVimbaCamera(const std::string& name);

  void start(const std::string& ip_str, const std::string& guid_str, const std::string& frame_id,
             bool print_all_features = false);
  void stop();

  void updateConfig(Config& config);
  void startImaging();
  void stopImaging();

  // Utility functions
  double getTimestampRealTime(VmbUint64_t timestamp_ticks);
  bool isOpened()
  {
    return opened_;
  }

  // Getters
  double getTimestamp();
  double getDeviceTemp();
  int getSensorWidth();
  int getSensorHeight();

  // Setters
  void setCallback(frameCallbackFunc callback)
  {
    userFrameCallback = callback;
  }

private:
  Config config_;

  AvtVimbaApi api_;
  // IFrame Observer
  SP_DECL(FrameObserver) frame_obs_ptr_;
  // The currently streaming camera
  CameraPtr vimba_camera_ptr_;
  // Current frame
  FramePtr vimba_frame_ptr_;
  // Tick frequency of the on-board clock. Equal to 1 GHz when PTP is in use.
  VmbInt64_t vimba_timestamp_tick_freq_ = 1;

  // Mutex
  std::mutex config_mutex_;

  CameraState camera_state_;
  bool opened_;
  bool streaming_;
  bool on_init_;
  std::string name_;
  std::string guid_;
  std::string frame_id_;

  diagnostic_updater::Updater updater_;
  std::string diagnostic_msg_;

  CameraPtr openCamera(const std::string& id_str, bool print_all_features);

  frameCallbackFunc userFrameCallback;
  void frameCallback(const FramePtr vimba_frame_ptr);

  template <typename T>
  VmbErrorType setFeatureValue(const std::string& feature_str, const T& val);
  template <typename T>
  bool getFeatureValue(const std::string& feature_str, T& val);
  bool getFeatureValue(const std::string& feature_str, std::string& val);
  template <typename Vimba_Type, typename Std_Type>
  void configureFeature(const std::string& feature_str, const Vimba_Type& val_in, Std_Type& val_out);
  void configureFeature(const std::string& feature_str, const std::string& val_in, std::string& val_out);
  bool runCommand(const std::string& command_str);
  void printAllCameraFeatures(const CameraPtr& camera);

  void updateAcquisitionConfig(Config& config);
  void updateExposureConfig(Config& config);
  void updateGainConfig(Config& config);
  void updateWhiteBalanceConfig(Config& config);
  void updateImageModeConfig(Config& config);
  void updateROIConfig(Config& config);
  void updateBandwidthConfig(Config& config);
  void updatePixelFormatConfig(Config& config);
  void updatePtpModeConfig(Config& config);
  void updateGPIOConfig(Config& config);
  void updateUSBGPIOConfig(Config& config);
  void updateIrisConfig(Config& config);

  void getCurrentState(diagnostic_updater::DiagnosticStatusWrapper& stat);
};
}  // namespace avt_vimba_camera
#endif
