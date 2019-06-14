#include "opencv2/opencv.hpp"
#include <opencv2/core/persistence.hpp>
#include "opencv2/xfeatures2d.hpp"
using namespace cv::xfeatures2d;


int main () {

  cv::Ptr<cv::Feature2D> surf = SURF::create();
  cv::FileStorage fs("../surf_params.yaml", cv::FileStorage::WRITE);
  cv::Ptr<cv::xfeatures2d::SURF> aux_ptr;
  aux_ptr = surf.dynamicCast<cv::xfeatures2d::SURF>();

  if( fs.isOpened() ) // if we have file with parameters, read them
  {
      std::cout << "reading parameters" << std::endl;

      surf->read(fs["surf_params"]);
  }
  else // else modify the parameters and store them; user can later edit the file to use different parameters
  {
      std::cout << "writing parameters" << std::endl;
      aux_ptr->setNOctaves(3); // lower the contrast threshold, compared to the default value
      {
          cv::internal::WriteStructContext ws(fs, "surf_params", CV_NODE_MAP);
          aux_ptr->write(fs);
      }
  }
  fs.release();

  cv::Mat image = cv::imread("/home/taihu/datasets/KITTI/00/image_0/000000.png"), descriptors;
  std::vector<cv::KeyPoint> keypoints;
//  surf->detectAndCompute(image, cv::noArray(), keypoints, descriptors);
  surf->detect(image, keypoints);
  std::cout << "keypoints: " << keypoints.size() << std::endl;


return 0;

}

