#include "DLDBriefLoopDetector.hpp"

#ifdef SHOW_PROFILING
  #include "../../utils/log/Profiler.hpp"
#endif // SHOW_PROFILING

using namespace std;

DLDBriefLoopDetector::DLDBriefLoopDetector(const string& voc_file_path, const Parameters& params)
: voc(voc_file_path), detector(voc, params)
{}

DetectionMatch DLDBriefLoopDetector::detectloop(const sptam::Map::SharedKeyFrame& stereo_frame)
{
  // Process image
  vector<cv::KeyPoint> keys;
  vector<FBrief::TDescriptor> descriptors;

  /* DLD expects brief descriptors as boost::dynamic_bitset<>, we need
   * to translate openCV descriptors. */

  /* cv::Mat descriptor with 8bits uchar's as data (there might be 16,32 or 64 uchar for each row)
   * TODO: Look for a "good" way of translating dynamic_bitset<uchar> to dynamic_bitset<ulong>
   *       as they use diferent block storages, we have to "copy" each bit all over again */
  const cv::Mat image_descriptors = stereo_frame->GetFrameLeft().GetFeatures().GetDescriptors();
  const vector<cv::KeyPoint>& image_kpts = stereo_frame->GetFrameLeft().GetFeatures().GetKeypoints();

  for(int i = 0; i < image_descriptors.rows; i++){

    const uchar* char_desc = image_descriptors.row(i).ptr(0);

    boost::dynamic_bitset<uchar> char_bset_desc; // openCV brief descriptors are in uchars!
    FBrief::TDescriptor bset_desc; // by default dynamic_biset uses ulong as block storage

    for(int i = 0; i < image_descriptors.cols; i++)
      char_bset_desc.append(char_desc[i]); // append each uchar

    bset_desc.resize(char_bset_desc.size());

    for(unsigned int i = 0; i < char_bset_desc.size(); i++)
      bset_desc[i] = char_bset_desc[i]; // copying each bit

    keys.push_back(image_kpts[i]);
    descriptors.push_back(bset_desc);
  }

  // add image to the collection and check if there is some loop
  DLoopDetector::DetectionResult result;
  detector.detectLoop(keys, descriptors, result);

  #ifdef SHOW_PROFILING
    WriteToLog(" lc Detection Result: ", result.status);
  #endif

  DetectionMatch dm = {result.detection(), result.query, result.match, -1};

  return dm;
}
