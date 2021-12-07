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

#include "avt_vimba_camera/mono_camera.h"

namespace avt_vimba_camera
{
// MonoCameraNode::MonoCamera(ros::NodeHandle& nh, ros::NodeHandle& nhp)
  // : nh_(nh), nhp_(nhp), it_(nhp), cam_(ros::this_node::getName())

/// TODO: replace "camera" with the name from params
MonoCameraNode::MonoCameraNode() : Node("mono_camera"), api_(this->get_logger()), cam_(this)
{
  RCLCPP_INFO(this->get_logger(), "Construct");

  // Start Vimba & list all available cameras
  api_.start();

  // Set the image publisher before the streaming
  // camera_info_pub_ = it_.advertiseCamera("image_raw", 1);

  // rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_sensor_data;
  rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_system_default;
  camera_info_pub_ = image_transport::create_camera_publisher(this, "image", custom_qos_profile);

  // Set the frame callback
  cam_.setCallback(std::bind(&avt_vimba_camera::MonoCameraNode::frameCallback, this, std::placeholders::_1));

  loadParams();

  // TODO: move this into the camera file, so that only camera-related params are configured
  // param_sub_ = this->add_on_set_parameters_callback(std::bind(&MonoCameraNode::parameterCallback, this, std::placeholders::_1));

  // next set avt params, so the callback is triggered?




  // Set camera info manager
  // info_man_ = std::shared_ptr<camera_info_manager::CameraInfoManager>(
  //     new camera_info_manager::CameraInfoManager(nhp_, frame_id_, camera_info_url_));
  // info_man_ = std::shared_ptr<camera_info_manager::CameraInfoManager>(
  //     new camera_info_manager::CameraInfoManager(this, frame_id_, camera_info_url_));

  // Start dynamic_reconfigure & run configure()
  // reconfigure_server_.setCallback(
  //     std::bind(&avt_vimba_camera::MonoCameraNode::configure, this, std::placeholders::_1, std::placeholders::_2));

  // Configure camera


  cam_.start(ip_, guid_, frame_id_, camera_info_url_);
  cam_.initConfig();
  // updateCameraInfo();

  cam_.startImaging();
}

MonoCameraNode::~MonoCameraNode(void)
{
  cam_.stop();
  camera_info_pub_.shutdown();
}

void MonoCameraNode::loadParams()
{
  ip_ = this->declare_parameter("ip", "");
  guid_ = this->declare_parameter("guid", "");
  camera_info_url_ = this->declare_parameter("camera_info_url", "");
  frame_id_ = this->declare_parameter("frame_id", "");
  use_measurement_time_ = this->declare_parameter("use_measurement_time", false);
  ptp_offset_ = this->declare_parameter("ptp_offset", 0);

  RCLCPP_INFO(this->get_logger(), "Parameters loaded");
}

void MonoCameraNode::frameCallback(const FramePtr& vimba_frame_ptr)
{
  // ros::Time ros_time = ros::Time::now();
  rclcpp::Time ros_time = this->get_clock()->now();



  // if (pub_.getNumSubscribers() > 0)
  // if (camera_info_pub_.getNumSubscribers() > 0)
  // {
    // RCLCPP_INFO(this->get_logger(), "subs");
    sensor_msgs::msg::Image img;
    if (api_.frameToImage(vimba_frame_ptr, img))
    {
      // sensor_msgs::msg::CameraInfo ci = info_man_->getCameraInfo();
      sensor_msgs::msg::CameraInfo ci = cam_.getCameraInfo();
      if (use_measurement_time_)
      {
        VmbUint64_t frame_timestamp;
        vimba_frame_ptr->GetTimestamp(frame_timestamp);
        ci.header.stamp = rclcpp::Time(cam_.getTimestampRealTime(frame_timestamp)) + rclcpp::Duration(ptp_offset_, 0);
      }
      else
      {
        ci.header.stamp = ros_time;
      }
      img.header.frame_id = ci.header.frame_id;
      // pub_.publish(img, ci);
      camera_info_pub_.publish(img, ci);
    }
    else
    {
      RCLCPP_WARN_STREAM(this->get_logger(), "Function frameToImage returned 0. No image published.");
    }
  // }
  // else
  // {
  //   RCLCPP_INFO(this->get_logger(), "No subs");
  // }
}

// rcl_interfaces::msg::SetParametersResult MonoCameraNode::parameterCallback (
//   const std::vector<rclcpp::Parameter> &parameters)
// {

// }

/** Dynamic reconfigure callback
 *
 *  Called immediately when callback first defined. Called again
 *  when dynamic reconfigure starts or changes a parameter value.
 *
 *  @param newconfig new Config values
 *  @param level bit-wise OR of reconfiguration levels for all
 *               changed parameters (0xffffffff on initial call)
 **/
// void MonoCameraNode::configure(Config& newconfig, uint32_t level)
// {
//   try
//   {
//     // The camera already stops & starts acquisition
//     // so there's no problem on changing any feature.
//     if (!cam_.isOpened())
//     {
//       cam_.start(ip_, guid_, frame_id_, print_all_features_);
//     }

//     Config config = newconfig;
//     cam_.updateConfig(config);
//     updateCameraInfo(config);
//   }
//   catch (const std::exception& e)
//   {
//     RCLCPP_ERROR_STREAM(this->get_logger(), "Error reconfiguring mono_camera node : " << e.what());
//   }
// }

// See REP-104 for details regarding CameraInfo parameters
// void MonoCameraNode::updateCameraInfo(const avt_vimba_camera::AvtVimbaCameraConfig& config)
// {
//   sensor_msgs::CameraInfo ci = info_man_->getCameraInfo();

//   // Set the operational parameters in CameraInfo (binning, ROI)
//   int binning_or_decimation_x = std::max(config.binning_x, config.decimation_x);
//   int binning_or_decimation_y = std::max(config.binning_y, config.decimation_y);

//   // Set the operational parameters in CameraInfo (binning, ROI)
//   int sensor_width = cam_.getSensorWidth();
//   int sensor_height = cam_.getSensorHeight();

//   if (sensor_width == -1 || sensor_height == -1)
//   {
//     RCLCPP_ERROR(this->get_logger(), "Could not determine sensor pixel dimensions, camera_info will be wrong");
//   }

//   ci.width = sensor_width;
//   ci.height = sensor_height;
//   ci.binning_x = binning_or_decimation_x;
//   ci.binning_y = binning_or_decimation_y;

//   // ROI is in unbinned coordinates, need to scale up
//   ci.roi.width = config.width * binning_or_decimation_x;
//   ci.roi.height = config.height * binning_or_decimation_y;
//   ci.roi.x_offset = config.offset_x * binning_or_decimation_x;
//   ci.roi.y_offset = config.offset_y * binning_or_decimation_y;

//   bool roi_is_full_image = (ci.roi.width == ci.width && ci.roi.height == ci.height);
//   ci.roi.do_rectify = !roi_is_full_image;

//   // push the changes to manager
//   info_man_->setCameraInfo(ci);
// }



// void MonoCameraNode::updateCameraInfo()
// {
//   sensor_msgs::msg::CameraInfo ci = info_man_->getCameraInfo();

//   // Set the operational parameters in CameraInfo (binning, ROI)
//   int binning_or_decimation_x = 1;
//   int binning_or_decimation_y = 1;

//   // Set the operational parameters in CameraInfo (binning, ROI)
//   int sensor_width = cam_.getSensorWidth();
//   int sensor_height = cam_.getSensorHeight();

//   if (sensor_width == -1 || sensor_height == -1)
//   {
//     RCLCPP_ERROR(this->get_logger(), "Could not determine sensor pixel dimensions, camera_info will be wrong");
//   }

//   ci.width = sensor_width;
//   ci.height = sensor_height;
//   ci.binning_x = binning_or_decimation_x;
//   ci.binning_y = binning_or_decimation_y;

//   // ROI is in unbinned coordinates, need to scale up
//   ci.roi.width = 100 * binning_or_decimation_x;
//   ci.roi.height = 100 * binning_or_decimation_y;
//   ci.roi.x_offset = 0 * binning_or_decimation_x;
//   ci.roi.y_offset = 0 * binning_or_decimation_y;

//   bool roi_is_full_image = (ci.roi.width == ci.width && ci.roi.height == ci.height);
//   ci.roi.do_rectify = !roi_is_full_image;

//   // push the changes to manager
//   info_man_->setCameraInfo(ci);
// }


}  // namespace avt_vimba_camera
