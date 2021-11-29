#include "avt_vimba_camera/mono_camera.h"

#include <nodelet/nodelet.h>

namespace avt_vimba_camera
{
class MonoCameraNodelet : public nodelet::Nodelet
{
public:
  virtual void onInit();
  virtual ~MonoCameraNodelet();

private:
  MonoCamera* camera_;
};

}  // namespace avt_vimba_camera
