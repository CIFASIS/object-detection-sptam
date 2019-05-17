/**
 * This file is part of S-PTAM.
 *
 * Copyright (C) 2015 Taihú Pire and Thomas Fischer
 * For more information see <https://github.com/lrse/sptam>
 *
 * S-PTAM is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * S-PTAM is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with S-PTAM. If not, see <http://www.gnu.org/licenses/>.
 *
 * Authors:  Taihú Pire <tpire at dc dot uba dot ar>
 *           Thomas Fischer <tfischer at dc dot uba dot ar>
 *
 * Laboratory of Robotics and Embedded Systems
 * Department of Computer Science
 * Faculty of Exact and Natural Sciences
 * University of Buenos Aires
 */
#pragma once

#include "Frame.hpp"
#include "MapPoint.hpp"
#include "RowMatcher.hpp"

#include <memory>
#include <list>

class StereoFrame
{
  public:

    struct RectifiedCameraParameters
    {
      const double baseline;
      const Eigen::Vector2d focal_length;
      const Eigen::Vector2d principal_point;
    };

    typedef std::unique_ptr<StereoFrame> UniquePtr;

    StereoFrame(
      const size_t id,
      const CameraPose& cameraPose, const CameraParameters& calibrationLeft,
      const double stereo_baseline, const CameraParameters& calibrationRight,
      const ImageFeatures& imageFeaturesLeft, const ImageFeatures& imageFeaturesRight,
      bool bFixed = false
    );

    StereoFrame(const StereoFrame& stereoFrame);

    /**
     * Try to find correspondences between a cloud of mapPoints
     * and the features detected in the left and right frames
     * (independently).
     */
    void FindMatches(const std::vector<cv::Point3d>& points,
      const std::vector<cv::Mat>& descriptors,
      const cv::DescriptorMatcher& descriptorMatcher,
      const size_t matchingNeighborhoodThreshold, const double matchingDistanceThreshold,
      std::map<size_t, MEAS> &measurementsLeft,
      std::map<size_t, MEAS> &measurementsRight
    ) const;

    /**
     * Try to match and triangulate unmatched features from
     * the left and right frames.
     */
    // TODO this only modifies the internal cache of matched/unmatched
    // keypoints, which should be a mutable property.
    void TriangulatePoints(const RowMatcher& matcher,
      std::vector<cv::Point3d>& points,
      std::vector<cv::KeyPoint>& kpointsLeft, std::vector<cv::Mat>& descriptorsLeft,
      std::vector<cv::KeyPoint>& kpointsRight, std::vector<cv::Mat>& descriptorsRight
    )/* const*/;

    void UpdateCameraPose(const CameraPose& cameraPose);

    /**
     * @brief tell if a point should be visible by the keyframe.
     */
    bool canView(const MapPoint& mapPoint) const;

    size_t GetId() const
    {
      boost::shared_lock<boost::shared_mutex> lock(stereo_frames_mutex_);
      return id_;
    }

    inline const Eigen::Vector3d GetPosition() const
    {
      boost::shared_lock<boost::shared_mutex> lock(stereo_frames_mutex_);
      return frameLeft_.GetPosition();
    }

    inline const Eigen::Quaterniond GetOrientation() const
    {
      boost::shared_lock<boost::shared_mutex> lock(stereo_frames_mutex_);
      return frameLeft_.GetOrientation();
    }

    inline CameraPose GetCameraPose() const
    {
      boost::shared_lock<boost::shared_mutex> lock(stereo_frames_mutex_);
      return frameLeft_.GetCameraPose();
    }

    /**
     * True if Keyframe should remain fixed during bundle adjustment,
     * false if it should be adjusted.
     */
    inline bool isFixed() const
    {
      boost::shared_lock<boost::shared_mutex> lock(stereo_frames_mutex_);
      return bFixed_;
    }

    Frame GetFrameLeft()
    {
      boost::shared_lock<boost::shared_mutex> lock(stereo_frames_mutex_);
      return frameLeft_;
    }

    const Frame GetFrameLeft() const
    {
      boost::shared_lock<boost::shared_mutex> lock(stereo_frames_mutex_);
      return frameLeft_;
    }

    Frame GetFrameRight()
    {
      boost::shared_lock<boost::shared_mutex> lock(stereo_frames_mutex_);
      return frameRight_;
    }

    const Frame GetFrameRight() const
    {
      boost::shared_lock<boost::shared_mutex> lock(stereo_frames_mutex_);
      return frameRight_;
    }

    const RectifiedCameraParameters& getRectifiedCameraParameters() const
    { return rectified_camera_parameters_; }

  private:

  /*** private properties ***/

    mutable boost::shared_mutex stereo_frames_mutex_;

    Frame frameLeft_, frameRight_;

    const RectifiedCameraParameters rectified_camera_parameters_;

    // True if StereoFrame is not going to modified by BundleAdjustment
    const bool bFixed_;

    // KeyFrame Id
    const size_t id_;

    // Create 3D points
    void CreatePoints(const RowMatcher& matcher,
      const std::vector<cv::KeyPoint>& keypoints1, const cv::Mat& descriptors1,
      const std::vector<cv::KeyPoint>& keypoints2, const cv::Mat& descriptors2,
      std::vector<cv::Point3d>& points, std::list<std::pair<size_t, size_t> > &matches
    );

    static CameraPose ComputeRightCameraPose(const CameraPose& leftCameraPose, const double stereo_baseline);
};
