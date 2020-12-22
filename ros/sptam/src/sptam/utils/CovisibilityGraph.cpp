#include "../MapPoint.hpp"
#include "../StereoFrame.hpp"
#include "../Measurement.hpp"

#include "CovisibilityGraph.hpp"

/******
 * Template instantiation
 */

template class CovisibilityGraph<StereoFrame, MapPoint, Measurement>; /* TODO: move this to independent file which is included here */

/* Definitions */

template<typename KEYFRAME_T, typename MAP_POINT_T, typename MEAS_T>
typename CovisibilityGraph<KEYFRAME_T, MAP_POINT_T, MEAS_T>::SharedKeyFrame CovisibilityGraph<KEYFRAME_T, MAP_POINT_T, MEAS_T>::addKeyFrame( const KEYFRAME_T& keyFrame )
{
  SharedKeyFrame sKF(new KeyFrame( keyFrame ));

  keyframes_.push_back(sKF);

  sKF->setIteratorToContainer(std::prev(keyframes_.end()));

  return sKF;
}

template<typename KEYFRAME_T, typename MAP_POINT_T, typename MEAS_T>
void CovisibilityGraph<KEYFRAME_T, MAP_POINT_T, MEAS_T>::removeKeyFrame( const CovisibilityGraph<KEYFRAME_T, MAP_POINT_T, MEAS_T>::SharedKeyFrame& keyFrame )
{
  while ( not keyFrame->measurements_.empty() )
    removeMeasurement( keyFrame->measurements_.front() );

  keyframes_.erase(keyFrame->to_container_);
  keyFrame->to_container_ = keyframes_.end();
}

template<typename KEYFRAME_T, typename MAP_POINT_T, typename MEAS_T>
typename CovisibilityGraph<KEYFRAME_T, MAP_POINT_T, MEAS_T>::SharedMapPoint CovisibilityGraph<KEYFRAME_T, MAP_POINT_T, MEAS_T>::addMapPoint( const MAP_POINT_T& mapPoint )
{
  SharedMapPoint sMP(new MapPoint( mapPoint ));

  mappoints_.push_back(sMP);

  sMP->setIteratorToContainer(std::prev(mappoints_.end()));

  return sMP;
}

template<typename KEYFRAME_T, typename MAP_POINT_T, typename MEAS_T>
void CovisibilityGraph<KEYFRAME_T, MAP_POINT_T, MEAS_T>::removeMapPoint( const CovisibilityGraph<KEYFRAME_T, MAP_POINT_T, MEAS_T>::SharedMapPoint& mapPoint )
{
  while ( not mapPoint->measurements_.empty() )
    removeMeasurement( mapPoint->measurements_.front() );

  mappoints_.erase(mapPoint->to_container_);
  mapPoint->to_container_ = mappoints_.end();
}

template<typename KEYFRAME_T, typename MAP_POINT_T, typename MEAS_T>
void CovisibilityGraph<KEYFRAME_T, MAP_POINT_T, MEAS_T>::addMeasurement( SharedKeyFrame& keyFrame, SharedMapPoint& mapPoint, const MEAS_T& edge )
{

  /* Adding measurements to keyframes or mappoints that aren't in the graph, are discarted */
  if(keyFrame->to_container_ == keyframes_.end() or mapPoint->to_container_ == mappoints_.end())
    return;

  // Check if the point is in the graph

  // Get Keyframes which measure the point

  // This is a copy
  std::list< SharedMeasurement > point_measurements = mapPoint->measurements();

  for ( SharedMeasurement& meas_point : point_measurements )
  {
    // Omit current keyframe
    if (meas_point->keyFrame().get() == keyFrame.get())
      continue;

    CovisibilityGraph<KEYFRAME_T, MAP_POINT_T, MEAS_T>::SharedKeyFrame covisibilityKeyFrame ( meas_point->keyFrame() );

    // TODO: Gaston: La operacion no es atomica, por lo que un keyframe podria ser covisible a otro y no viceversa.
    keyFrame->addCovisibilityKeyframe( covisibilityKeyFrame );
    covisibilityKeyFrame->addCovisibilityKeyframe( keyFrame );


//    std::cout << "Keyframe: " << ( meas_point->keyFrame() ) << " nivel de covisibilidad: " << keyFrame->covisibilityKeyFrames_.at( covisibilityKeyFrame )  << std::endl;
//    std::cout << "covisibilityKeyFrames size: " << keyFrame->covisibilityKeyFrames_.size() << std::endl;
  }

  // add Keyframe-Point Measurement

  SharedMeasurement meas( new Measurement(edge, keyFrame, mapPoint) );
  // TODO: make atomic this behavior otherwise a keyframe may be connected to the mappoint but not yet the mappoint to the keyframe (concurrency)
  keyFrame->addMeasurement(meas);
  mapPoint->addMeasurement(meas);
}

template<typename KEYFRAME_T, typename MAP_POINT_T, typename MEAS_T>
void CovisibilityGraph<KEYFRAME_T, MAP_POINT_T, MEAS_T>::removeMeasurement( const SharedMeasurement& edge )
{
  /* TODO: Gaston: Covisibility information of the keyframe involved must be updated on removal */
  edge->keyFrame_->measurements_.erase( edge->it_keyFrame_ );
  edge->mapPoint_->measurements_.erase( edge->it_mapPoint_ );
}

#define MAX_COVISIBLES 100
#define MAX_PUNTOS 1000
#define MIN_COVISIBILITY 0

#define POLITICA_LOCAL_MAP_ORB_SLAM             1
#define POLITICA_LOCAL_MAP_ACOTADO              2


#define POLITICA_LOCAL_MAP POLITICA_LOCAL_MAP_ACOTADO


template<typename KEYFRAME_T, typename MAP_POINT_T, typename MEAS_T>
void CovisibilityGraph<KEYFRAME_T, MAP_POINT_T, MEAS_T>::getLocalMap(const SharedMapPointSet &mapPoints,
                                                                      CovisibilityGraph<KEYFRAME_T, MAP_POINT_T, MEAS_T>::SharedMapPointSet& localMap,
                                                                      CovisibilityGraph<KEYFRAME_T, MAP_POINT_T, MEAS_T>::SharedKeyFrameSet& localKeyFrames,
                                                                      CovisibilityGraph<KEYFRAME_T, MAP_POINT_T, MEAS_T>::SharedKeyFrame& referenceKeyFrame)
{
  //std::cout << "getting local map using " << mapPoints.size() << " input points" << std::endl;

  /* start from the set of input points (guarantees these points are in the localMap) */
  localMap = mapPoints;

#if (POLITICA_LOCAL_MAP == POLITICA_LOCAL_MAP_ACOTADO)
  /* this will store how many points of the tracked map the keyframe observes */
  std::map<SharedKeyFrame, size_t> covisibilityKeyframes;

  // Get the set of keyframes that observe the given mappoints, remember how many point each observes
  for (auto& mapPoint : mapPoints) {
    for (auto& meas : mapPoint->measurements() ) {

      auto it = covisibilityKeyframes.find(meas->keyFrame());
      if (it == covisibilityKeyframes.end()) covisibilityKeyframes.insert(std::make_pair(meas->keyFrame(), 1));
      else it->second++;
    }
  }

  /* find the reference keyframe (highest covisibility to input map) */
  auto it = std::max_element(covisibilityKeyframes.begin(), covisibilityKeyframes.end(),
                        [](const auto& a, const auto& b) -> bool { return a.second < b.second; });
  if (it == covisibilityKeyframes.end())
  {
    std::cout << "ERROR: no hay keyframes covisibles a los puntos trackeados" << std::endl;
    return;
  }
  else
  {
    referenceKeyFrame = it->first;
  }

  /* add points covisible to reference keyframe */
  auto covisibleToReference = referenceKeyFrame->covisibilityKeyFramesVector();

  /* find most covisibile keyframes up to a certain number of KFs */
  size_t nCovisibilityKeyframes = std::min((size_t)MAX_COVISIBLES, covisibleToReference.size());
  std::vector< std::pair<SharedKeyFrame, size_t> > orderedCovisibleToReference ( nCovisibilityKeyframes );
  std::partial_sort_copy(covisibleToReference.begin(), covisibleToReference.end(), orderedCovisibleToReference.begin(), orderedCovisibleToReference.end(), [](const auto& a, const auto& b) { return a.second > b.second; });
  //std::cout << "n.of. covisible keyframes to reference: " << nCovisibilityKeyframes << std::endl;

  /* add points from these KFs up to a certain number of points */
  for (const auto& pair : orderedCovisibleToReference)
  {
    //std::cout << "covisibile to reference keyframe: " << (uintptr_t)pair.first.get() << " with " << pair.second << " common measurements (total of " << pair.first->measurements().size() << " measurements)" << std::endl;
    if (pair.second < MIN_COVISIBILITY) continue;

    for (auto& meas : pair.first->measurements() )
    {
      localMap.insert( meas->mapPoint() );
    }

    localKeyFrames.insert( pair.first );
    if (localMap.size() > MAX_PUNTOS) break;
  }
#elif (POLITICA_LOCAL_MAP == POLITICA_LOCAL_MAP_ORB_SLAM)
  SharedKeyFrameSet K1;
  std::map<SharedKeyFrame, size_t> covisibilityKeyframes;

  // Go through the set of keyframes that observe the tracked points (K1) and remember covisibility degree for each
  for (const auto& mapPoint : mapPoints) {
    for (const auto& meas : mapPoint->measurements() ) {
      auto it = covisibilityKeyframes.find(meas->keyFrame());
      if (it == covisibilityKeyframes.end()) covisibilityKeyframes.insert(std::make_pair(meas->keyFrame(), 1));
      else it->second++;
    }
  }

  // Find the reference keyframe
  std::vector<std::pair<SharedKeyFrame, size_t> > vectorCovisibilityKeyframes;
  vectorCovisibilityKeyframes.reserve( covisibilityKeyframes.size() );
  for(const auto& kp : covisibilityKeyframes) { vectorCovisibilityKeyframes.push_back( kp ); }
  std::sort(vectorCovisibilityKeyframes.begin(), vectorCovisibilityKeyframes.end(), [](const auto& a, const auto& b) { return a.second > b.second; });
  referenceKeyFrame = vectorCovisibilityKeyframes.front().first;

  // Get the set of keyframes (K2) covisible to the ones in K1. Add all of K1 and K2 to output.
  for (const auto& pair : vectorCovisibilityKeyframes) {
    localKeyFrames.insert(pair.first);
    for (const auto& pair2 : pair.first->covisibilityKeyFrames())
    {
      localKeyFrames.insert(pair2.first);
    }
  }

  // Add points in K1 U K2 to output
  for (const SharedKeyFrame& kf : localKeyFrames)
  {
    for (const auto& meas : kf->measurements())
      localMap.insert(meas->mapPoint());
  }
#endif

  //std::cout << "localKeyFrames: " << localKeyFrames.size() << std::endl;
  //std::cout << "localMap: " << localMap.size() << std::endl;
}

