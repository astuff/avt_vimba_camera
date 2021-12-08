#include "avt_vimba_camera/mono_camera.h"

#include <memory>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<avt_vimba_camera::MonoCameraNode>();
  rclcpp::spin(node);

  return 0;
}
