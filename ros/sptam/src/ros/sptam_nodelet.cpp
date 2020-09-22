#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include "stereo_driver.hpp"

namespace sptam
{
  class sptam_nodelet : public nodelet::Nodelet
  {
    public:

      void onInit()
      {
        NODELET_DEBUG("Initializing sptam nodelet...");
        sptam_interface_.reset( new sptam::stereo_driver( getNodeHandle(), getPrivateNodeHandle() ) );
      }

    private:

      std::unique_ptr<sptam::stereo_driver> sptam_interface_;
  };
}

PLUGINLIB_EXPORT_CLASS(sptam::sptam_nodelet, nodelet::Nodelet)
