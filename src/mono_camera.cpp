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

#include <avt_vimba_camera/mono_camera.h>

#define DEBUG_PRINTS 1

namespace avt_vimba_camera
{
MonoCamera::MonoCamera(ros::NodeHandle& nh, ros::NodeHandle& nhp)
  : nh_(nh), nhp_(nhp), it_(nhp), cam_(ros::this_node::getName())
{
  // Start Vimba & list all available cameras
  api_.start();

  // Set the image publisher before the streaming
  pub_ = it_.advertiseCamera("image_raw", 1);

  // Set the frame callback
  cam_.setCallback(std::bind(&avt_vimba_camera::MonoCamera::frameCallback, this, std::placeholders::_1));

  // Set the params
  nhp_.param("ip", ip_, std::string(""));
  nhp_.param("guid", guid_, std::string(""));
  nhp_.param("camera_info_url", camera_info_url_, std::string(""));
  nhp_.param("frame_id", frame_id_, std::string(""));
  nhp_.param("print_all_features", print_all_features_, false);
  nhp_.param("use_measurement_time", use_measurement_time_, false);
  nhp_.param("ptp_offset", ptp_offset_, 0);

  // Set camera info manager
  info_man_ = std::shared_ptr<camera_info_manager::CameraInfoManager>(
      new camera_info_manager::CameraInfoManager(nhp_, frame_id_, camera_info_url_));

  // Start dynamic_reconfigure & run configure()
  reconfigure_server_.setCallback(
      std::bind(&avt_vimba_camera::MonoCamera::configure, this, std::placeholders::_1, std::placeholders::_2));
}

MonoCamera::~MonoCamera(void)
{
  cam_.stop();
  pub_.shutdown();
}

void MonoCamera::frameCallback(const FramePtr& vimba_frame_ptr)
{
  ros::Time ros_time = ros::Time::now();
  if (pub_.getNumSubscribers() > 0)
  {
    sensor_msgs::Image img;
    if (api_.frameToImage(vimba_frame_ptr, img))
    {
      sensor_msgs::CameraInfo ci = info_man_->getCameraInfo();
      // Note: getCameraInfo() doesn't fill in header frame_id or stamp
      ci.header.frame_id = frame_id_;
      if (use_measurement_time_)
      {
        VmbUint64_t frame_timestamp;
        vimba_frame_ptr->GetTimestamp(frame_timestamp);
        ci.header.stamp = ros::Time(cam_.getTimestampRealTime(frame_timestamp)) + ros::Duration(ptp_offset_, 0);
      }
      else
      {
        ci.header.stamp = ros_time;
      }
      img.header.frame_id = ci.header.frame_id;
      img.header.stamp = ci.header.stamp;
      pub_.publish(img, ci);
    }
    else
    {
      ROS_WARN_STREAM("Function frameToImage returned 0. No image published.");
    }
  }
}

/** Dynamic reconfigure callback
 *
 *  Called immediately when callback first defined. Called again
 *  when dynamic reconfigure starts or changes a parameter value.
 *
 *  @param newconfig new Config values
 *  @param level bit-wise OR of reconfiguration levels for all
 *               changed parameters (0xffffffff on initial call)
 **/
void MonoCamera::configure(Config& newconfig, uint32_t level)
{
  try
  {
    // The camera already stops & starts acquisition
    // so there's no problem on changing any feature.
    if (!cam_.isOpened())
    {
      cam_.start(ip_, guid_, frame_id_, print_all_features_);
    }

    Config config = newconfig;
    cam_.updateConfig(config);
    updateCameraInfo(config);
  }
  catch (const std::exception& e)
  {
    ROS_ERROR_STREAM("Error reconfiguring mono_camera node : " << e.what());
  }
}

// See REP-104 for details regarding CameraInfo parameters
void MonoCamera::updateCameraInfo(const avt_vimba_camera::AvtVimbaCameraConfig& config)
{
  sensor_msgs::CameraInfo ci = info_man_->getCameraInfo();

  // Set the operational parameters in CameraInfo (binning, ROI)
  int binning_or_decimation_x = std::max(config.binning_x, config.decimation_x);
  int binning_or_decimation_y = std::max(config.binning_y, config.decimation_y);

  // Set the operational parameters in CameraInfo (binning, ROI)
  int sensor_width = cam_.getSensorWidth();
  int sensor_height = cam_.getSensorHeight();

  if (sensor_width == -1 || sensor_height == -1)
  {
    ROS_ERROR("Could not determine sensor pixel dimensions, camera_info will be wrong");
  }

  ci.width = sensor_width;
  ci.height = sensor_height;
  ci.binning_x = binning_or_decimation_x;
  ci.binning_y = binning_or_decimation_y;

  // ROI is in unbinned coordinates, need to scale up
  ci.roi.width = config.width * binning_or_decimation_x;
  ci.roi.height = config.height * binning_or_decimation_y;
  ci.roi.x_offset = config.offset_x * binning_or_decimation_x;
  ci.roi.y_offset = config.offset_y * binning_or_decimation_y;

  bool roi_is_full_image = (ci.roi.width == ci.width && ci.roi.height == ci.height);
  ci.roi.do_rectify = !roi_is_full_image;

  // push the changes to manager
  info_man_->setCameraInfo(ci);
}

};  // namespace avt_vimba_camera
