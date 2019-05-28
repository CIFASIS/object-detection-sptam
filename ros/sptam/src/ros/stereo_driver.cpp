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


#include <vector>
#include "stereo_driver.hpp"
#include "utils/opencv_parsers.hpp"
#include "utils/detection.hpp"
#include <stdlib.h>     /* abs */
#include "../sptam/FeatureExtractorThread.hpp"

#include "../sptam/Object.hpp"
#include "../sptam/ObjectMeasurement.hpp"
#include "../sptam/ObjectEdge.hpp"
//#include <ApproxMVBB/ComputeApproxMVBB.hpp>


#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_geometry/stereo_camera_model.h>
#include <image_geometry/pinhole_camera_model.h>

#ifdef SHOW_PROFILING
#include "../sptam/utils/log/Profiler.hpp"
#include "../sptam/utils/log/Logger.hpp"
#endif // SHOW_PROFILING

//Branch Includes
#include "../sptam/utils/cv2eigen.hpp"
#include <pcl_ros/point_cloud.h>
//



sptam::stereo_driver::stereo_driver(ros::NodeHandle& nh, ros::NodeHandle& nhp)
: base_driver(nh, nhp)
, cameraParametersLeft_( nullptr ), cameraParametersRight_( nullptr )
, lastImageSeq_( 0 ), imgTransport_( nhp )
{
    // Get node parameters
    
    bool use_approx_sync;
    nhp.param<bool>("approximate_sync", use_approx_sync, false);
    
    // Camera Calibration Parameters
    
    nhp.param<double>("FrustumNearPlaneDist", frustum_near_plane_distance_, 0.1);
    nhp.param<double>("FrustumFarPlaneDist", frustum_far_plane_distance_, 1000.0);
    
    // Load feature detectors and descriptors
    
    // Load salient point detector implementation from configuration.
    {
        std::string detector_name;
        nhp.param<std::string>("FeatureDetector/Name", detector_name, "GFTT");
        
        feature_detector_left_ = loadFeatureDetector( nhp, detector_name, "FeatureDetector" );
        feature_detector_right_ = loadFeatureDetector( nhp, detector_name, "FeatureDetector" );
    }
    
    // Load descriptor extractor implementation from configuration.
    {
        std::string descriptor_name;
        nhp.param<std::string>("DescriptorExtractor/Name", descriptor_name, "BRIEF");
        
        descriptor_extractor_left_ = loadDescriptorExtractor( nhp, descriptor_name, "DescriptorExtractor" );
        descriptor_extractor_right_ = loadDescriptorExtractor( nhp, descriptor_name, "DescriptorExtractor" );
    }
    
    // write configuration to log file
    #ifdef SHOW_PROFILING
    Logger::Write( "#   FrustumNearPlaneDist: " + std::to_string(  frustum_near_plane_distance_ ) + "\n" );
    Logger::Write( "#   FrustumFarPlaneDist: " + std::to_string(  frustum_far_plane_distance_ ) + "\n" );
    #endif
    
    // Subscribe to images messages
    
    sub_img_l_.subscribe(nhp, "/stereo/left/image_rect", 1);
    sub_info_l_.subscribe(nhp, "/stereo/left/camera_info", 1);
    sub_img_r_.subscribe(nhp, "/stereo/right/image_rect", 1);
    sub_info_r_.subscribe(nhp, "/stereo/right/camera_info", 1);
    
    if ( use_approx_sync )
    {
        approximate_sync_.reset( new ApproximateSync( ApproximatePolicy(10),
                                                      sub_img_l_, sub_info_l_,
                                                      sub_img_r_, sub_info_r_
        ) );
        
        approximate_sync_->registerCallback( boost::bind(
            &stereo_driver::onImages, this, _1, _2, _3, _4
        ) );
    }
    else
    {
        exact_sync_.reset( new ExactSync( ExactPolicy(1),
                                          sub_img_l_, sub_info_l_,
                                          sub_img_r_, sub_info_r_
        ) );
        
        exact_sync_->registerCallback( boost::bind(
            &stereo_driver::onImages, this, _1, _2, _3, _4
        ) );
    }
    
    //branch
    //Detection sub
    sub_detect_l_ = nhp.subscribe("/detection/stereo/left/det", 1,&stereo_driver::onDetection,this);
    //
    
    stereoFrame_Pub_ = imgTransport_.advertise("stereo_frame_before_refine", 100);
    stereoFrameAfter_Pub_ = imgTransport_.advertise("stereo_frame_after_refine", 100);
    leftFrame_Pub_ = imgTransport_.advertise("left_frame_before_refine", 100);
    rightFrame_Pub_ = imgTransport_.advertise("right_frame_before_refine", 100);
    leftFrameAfter_Pub_ = imgTransport_.advertise("left_frame_after_refine", 100);
    rightFrameAfter_Pub_ = imgTransport_.advertise("right_frame_after_refine", 100);
    
    ROS_INFO("S-PTAM stereo node initialized.");
}

// DetectionWithPoseList DetectionList
void sptam::stereo_driver::onDetection(const dl_node::DetectionWithPoseListConstPtr& detections_left)
{
    
    //-----------Recibo la data de ROS------------//
    
    // N�mero de Detecctiones
    int detsize = detections_left->data.size() ; 
    
    
    // Header info
    size_t currentSeq = detections_left->header.seq ;
    ros::Time currentTime = detections_left->header.stamp;
    
    // Camera Parameters
    float focal = cameraParametersLeft_->focalLength();
    size_t width = cameraParametersLeft_->imageWidth();
    size_t height = cameraParametersLeft_->imageHeight();
    //
    
    //Frame id 
    sptam::Map::SharedKeyFrame det_kf = sptam_->kf_list_[detections_left->kf_id] ;   
    CameraPose kf_pose = det_kf->GetCameraPose() ;
    CameraPose delta_kf = sptam_->delta_kf_list_[detections_left->kf_id] ;
    
    Eigen::Vector3d frame_position = det_kf->GetCameraPose().GetOrientationMatrix() * delta_kf.GetPosition() + det_kf->GetPosition() ;
    CameraPose frame_pose(frame_position
    ,det_kf->GetOrientation() * delta_kf.GetOrientationQuaternion()
    ,Eigen::Matrix6d::Identity()) ;
    
    Camera frame_camera(frame_pose,det_kf->GetFrameLeft().GetCamera().GetCalibration()) ;
    /*  
     * 
     *  Eigen::Matrix3d kf_orientation = kf_pose.GetOrientationMatrix() ;
     *  Eigen::Matrix3d Ot_rp = refined_pose.GetOrientationMatrix().transpose() ;
     *  Eigen::Matrix3d delta_O = kf_orientation * delta_O ;
     *  Eigen::Vector3d delta_P = delta_O * -refined_pose.GetPosition() + kf_pose.GetPosition() ;
     *  CameraPose kf_delta = CameraPose(delta_P,Eigen::Quaterniond(delta_O),refined_pose.covariance()) ;
     *  // Cuando este implementado descomentar. Por ahora el delta es la identidad
     *  //CameraPose kf_delta = CameraPose(Eigen::Vector3d(0.0,0.0,0.0),Eigen::Quaterniond::Identity(),Eigen::Matrix6d::Identity()) ;  
     * 
     * 
     *  Eigen::Vector3d kf_position = kf_pose.GetPosition() ; 
     *  Eigen::Quaterniond kf_quaternion = kf_pose.GetOrientationQuaternion() ; 
     *  Camera kf_cam = det_kf->GetFrameLeft().GetCamera() ;
     */
    
    
    
    // Detecciones
    for (int i = 0;i<detsize;i++) {
        
        
        std::cout << "Detection values : " ;
        std::cout << detections_left->data[i].cls  << ' '
        << detections_left->data[i].x1   << ' '
        << detections_left->data[i].y1   << ' '
        << detections_left->data[i].x2   << ' '
        << detections_left->data[i].y2   << ' '
        << detections_left->data[i].yaw  << ' '
        << detections_left->data[i].dimX << ' '
        << detections_left->data[i].dimY << ' '
        << detections_left->data[i].dimZ << std::endl ;
        
        
        
        //if (detections_left->data[i].cls != 3)
        //  continue ; 
        // Las coordenadas del centro del bbox respecto al sistema de referencia de la camara.
        // Tengo en cuenta que para el detector el 0,0 de la imagen esta en la esquina sup izq
        // S�, pero eso es intrinsico. El bbox tiene que ser desde el principal ray?
        //Eigen::Vector3d bbox_center((detections_left->data[i].x1+detections_left->data[i].x2)/2.0 - width/2.0,
        //                            (detections_left->data[i].y1+detections_left->data[i].y2)/2.0 - height/2.0,
        //
        //                                focal);
        
        double_t bbox_u = (detections_left->data[i].x1+detections_left->data[i].x2)/2.0 ;
        double_t bbox_v = (detections_left->data[i].y1+detections_left->data[i].y2)/2.0 ;
        Eigen::Vector3d bbox_center( bbox_u - width/2.0,
                                     bbox_v - height/2.0,
                                     focal);
        //
        size_t bbox_sizeX = detections_left->data[i].x2 - detections_left->data[i].x1 ;
        size_t bbox_sizeY =  detections_left->data[i].y2 - detections_left->data[i].y1 ;
        
        //Camera bbox_cam = getBBoxCam(kf_pose.ToWorld(bbox_center), kf_position, -kf_orientation.col(1), bbox_sizeX, bbox_sizeY, *cameraParametersLeft_) ;
        
        
        
        ObjectMeasurement detection = ObjectMeasurement(detections_left->data[i].cls,0.0,det_kf,delta_kf,
                                                        detections_left->data[i].x1,detections_left->data[i].y1,
                                                        detections_left->data[i].x2,detections_left->data[i].y2);
        
        SharedObjectMeasurement sDetection(new ObjectMeasurement( detection )); 
        
        /*
         *   Object Correspondance  
         *  *
         */
        bool new_object = false ;
        sptam::ObjectMap::ObjectList high_iou_list ;
        
        
        double_t iou_val ;
        sptam::ObjectMap::SharedObject iou_candidate = sptam_->omap.MaxIOUCandidate(sDetection,iou_val,high_iou_list) ;
        
        double_t image_l2_val ;
        sptam::ObjectMap::SharedObject iml2_candidate = sptam_->omap.MinL2Candidate(sDetection,image_l2_val) ;
        
        sptam::ObjectMap::SharedObject sObject;
        
        std::cout << "best iou: " << iou_val << ", " << high_iou_list.size() << "candidates > 0.5"  << std::endl ;
        
        if ((iou_val > 0.35) && (high_iou_list.size() > 0)) sptam_->omap.ReviseMerge(iou_candidate,high_iou_list) ;
        
        if ((iou_val > 0.35)) {
            sObject = iou_candidate ;
            if (sObject->GetHit()) {
                std::cout << "Algo anda mal, dos detecciones distintas mismo objeto" << std::endl ;
            } else {
                sObject->updateHit(true) ;
            }
            
        } else {
            std::cout << "No match" << std::endl ; 
            new_object = true ;
            sObject = NULL ;
        }
        
        //Checkeo si esta en el borde
        size_t inImageEdge = 0 ;
        if (detections_left->data[i].x1 <= 0) inImageEdge++ ;
        if (detections_left->data[i].y1 <= 0) inImageEdge++ ;
        if (detections_left->data[i].x2 >= width - 1) inImageEdge++ ;
        if (detections_left->data[i].y2 >= height - 1) inImageEdge++ ;
        
        if (inImageEdge >= 1) {
            std::cout << "Descartada por estar en el borde" << std::endl ;
            continue ;
        }
        
        
        
        Eigen::Matrix3d frame_O = sDetection->GetOrientation().toRotationMatrix() ;
        Eigen::Vector3d frame_P = sDetection->GetPosition() ;  
        
        // R D T
        Eigen::Vector3d objDim(detections_left->data[i].dimX,detections_left->data[i].dimY,detections_left->data[i].dimZ) ;
        double_t objAngle = detections_left->data[i].yaw ;
        
        Eigen::Vector3d up_world_from_frame = -frame_O.row(1) ; // Ac� es row, porque quiero ir de WF a CF
        Eigen::Vector3d up_world_from_world_frame(0.0,-1.0,0.0) ;
        Eigen::Vector3d up_kf_from_kf_frame(0.0,-1.0,0.0) ;
        
        
        
        Eigen::Matrix3d R ; 
        EigenPair T_pair ;
        
        
        std::cout << "Infering Object Translation: " << std::endl ;
        Eigen::Vector3d T = sDetection->GetObjPose(detections_left->data[i].yaw,up_world_from_frame,objDim,R,T_pair) ;
        std::cout << T(0) << ' ' << T(1) << ' ' << T(2) << std::endl ;
        std::cout << T_pair.first(0) << ' ' << T_pair.first(1) << ' ' << T_pair.first(2) << ' ' <<
        T_pair.second(0) << ' ' << T_pair.second(1) << ' ' << T_pair.second(2) << std::endl ;
        
        //
        const double_t MAX_OBJECT_DETECT_DISTANCE = 4.0 ;
        if (T.norm() > MAX_OBJECT_DETECT_DISTANCE+2.0) {
            std::cout << "Object too far. Skiping..." << std::endl ;   
        }
        
        
        Eigen::Vector3d faceCorrection = sDetection->GetFaceCorrection(objAngle,objDim) ;
        
        //Camera bbox_cam = sDetection->GetBBoxCamera(up_kf_from_kf_frame,*cameraParametersLeft_) ;
        
        
        
        size_t kf_counter = 0 ;
        size_t bbox_counter = 0 ;
        size_t bbox_debug = 0 ;
        size_t kf_debug = 0 ;
        
        sptam::Map::SharedMapPointSet filtered_set ;
        sptam::Map::SharedMapPointList filtered_points;
        //{
        /* Lock for reading a small segment of the Map, after getting the required points locking is no longer
         *	       required due to internal locking of MapPoints */
        
        
        //boost::shared_lock<boost::shared_mutex> lock(sptam_->GetMap().map_mutex_);
        
        //sptam::Map::SharedMapPointList pointList = sptam_->GetMap().getMapPoints();
        ROS_INFO("Entro al for");
        for (const auto& measurement : det_kf->measurements() ) {
            
            sptam::Map::SharedPoint sMapPoint = measurement->mapPoint() ;
            
            const MapPoint& mapPoint = *sMapPoint ;
            const Eigen::Vector3d& point = mapPoint.GetPosition();
           
            // kf_counter
            //if (kf_cam.CanView(point)) {
            cv::Point2d projection = project( sDetection->GetSrcCamera().GetProjection(), eigen2cv( point ) );
            ROS_INFO("Antes del if");
            if ((detections_left->data[i].x1 <= projection.x) &&
                (detections_left->data[i].y1 <= projection.y) &&
                (detections_left->data[i].x2 >= projection.x) &&
                (detections_left->data[i].y2 >= projection.y) &&
                (sMapPoint->IsBad() == false)) 
            {
                ROS_INFO("en el if");
                filtered_points.push_back( sMapPoint );
                //filtered_set.insert(sMapPoint) ;
                kf_counter++ ;
            }  
            
            //} 
            
            //if (bbox_cam.CanView(point))  bbox_counter++ ;                  
            
        }
        
        //}
        
        const size_t MIN_POINTS = 2;
        
        if (filtered_points.size() < MIN_POINTS) { 
            std::cout << "Not enough map points in BBox. Skipping..." << std::endl ;  
            continue ;
        } 
        
        cv::Matx44d transRt = computeTransformation44(frame_P,frame_O) ;
        
        filtered_points.sort(sortByZ(transRt)) ;
        
        filtered_points.remove_if(filterByDepth(transRt,T(2)+faceCorrection(2),T(2)*0.33)) ;
        if (filtered_points.size() < 1) {
            std::cout << "No map points left in object neighbourhood" << std::endl ; 
            continue ;
        } else {
            std::cout << filtered_points.size() << " points after 3D bbox filter" << std::endl ;
        }
        
        sptam::Map::SharedPoint depthPoint ;
        
        double_t obj_depth = depthFromMapPoints(sDetection->GetSrcCamera().GetProjection(),transRt,filtered_points,detections_left->data[i].x1+detections_left->data[i].x2,detections_left->data[i].y1+detections_left->data[i].y2,depthPoint) ;  
        
        sDetection->SetDepth(depthPoint) ;
        std::cout << "Obj depth : " << obj_depth << std::endl ;
        
        if (obj_depth > MAX_OBJECT_DETECT_DISTANCE) {
            std::cout << "Object too far. Skiping..." << std::endl ;
        }
        
        
        
        filtered_points.remove_if(filterByDepth(transRt,obj_depth,faceCorrection(2))) ;
        for (const auto& point : filtered_points) point->updateColor(COLOR_LIGHTCYAN) ;
        //std::cout << "Obj MapPoints : " << filtered_points.size() << std::endl ;
        
        //depthPoint->updateColor(COLOR_CHARTREUSE) ;
        
        
        
        // sObject = sptam_->omap.MatchDetection(sDetection,filtered_points) ;
        if (new_object) {
            Object nObject ;
            sObject = sptam_->omap.addObject(nObject) ;
            std::cout << "~~New Object" << std::endl ;
        }
        sObject->updateClass(detections_left->data[i].cls) ;
        
        
        
        
        sObject->AddMapPoints(filtered_points) ;
        sObject->AddMeasurement(sDetection) ;
        
        
        //Eigen::Vector3d obj_center_camera = back_project(kf_cam,bbox_u,bbox_v,obj_depth) ;
        //Eigen::Vector3d obj_center_world = sDetection->ToWorld(bbox_u,bbox_v) ;
        
        //Eigen::Vector3d test_center_cam(0.0,0.00,10.00) ;
        //Eigen::Vector3d test_center = kf_cam.GetPose().ToWorld(test_center_cam) ;
        
        //Eigen::Vector3d test2 = bbox_cam.GetPose().ToWorld(Eigen::Vector3d(0.0,0.0,obj_depth)) ;   
        
        
        //Eigen::Vector3d Tdepth(T(0)*obj_depth/T(2),T(1)*obj_depth/T(2),obj_depth) ;    
        
        Eigen::Vector3d Tdepth  = (abs(T_pair.first.norm() - obj_depth) < abs(T_pair.second.norm() - obj_depth))?T_pair.first:T_pair.second;
        
        //Tdepth = Tdepth + faceCorrection ;  
        
        Eigen::Matrix3d O_KF_O = delta_kf.GetOrientationMatrix() * R ;
        Eigen::Vector3d O_KF_P = delta_kf.GetOrientationMatrix() * Tdepth + delta_kf.GetPosition() ; 
        
        
        Eigen::Vector3d WF_normal = sDetection->ObjectNormalWorld(objAngle,up_world_from_frame) ;
        
        
        ObjectEdge edge(det_kf,O_KF_P,O_KF_O,objDim,depthPoint,WF_normal) ;
        SharedObjectEdge sEdge(new ObjectEdge(edge)) ;
        sObject->AddEdge(sEdge) ;   
        
        
        sObject->updateEstimateMedian(up_world_from_world_frame) ;
        
        //sObject->updateDimensions(objDim) ;
        //Eigen::Quaterniond WF_OQ(WF_O) ;
        //sObject->updateOrientation(WF_OQ) ;
        //sObject->updatePosition(WF_P) ;
        //sObject->updateColor(COLOR_PINK);   
        //
        
        //Object testO ;
        //testO.updatePosition(obj_center_world) ;
        //testO.updateColor(COLOR_PINK) ;
        //sptam_->omap.addObject(testO) ;
        
        //std::cout << "BBOX_CENTER_3D" << std::endl << test2 << std::endl ;
        //std::cout << "KF_CAM_CENTER_3D" << std::endl << obj_center_world << std::endl ;
        
        
        
        
        //outlier_filter(filtered_points) ;
        
        
        //Draw
        //cv::Mat left_out ;
        //const Camera cam(cameraPose_,*cameraParametersLeft_) ;
        //drawProjections(left_out, cam.GetProjection(), ConstSharedPtrListIterable<sptam::Map::Point>::from( filtered_points )) ;
        //
        
        /* 
         *	    if ( objectMapPub_.getNumSubscribers() >= 1 ) {
         *	    
         *	    //Create PointCloud message for visualization
         *	    pcl::PointCloud<pcl::PointXYZRGB> msg;
         *	    msg.header.frame_id = detections_left->header.frame_id ;
         *	    msg.height = 1;
         *	    msg.width = sptam_->omap.list_.size() ;
         *	    for(const auto& point : sptam_->omap.list_ )
         *	    {
         *	    // Get Point from Map
         *		const Eigen::Vector3d point3d = point.GetPosition();
         *		pcl::PointXYZRGB point_pcl;
         * 
         *		point_pcl.x = point3d.x();
         *		point_pcl.y = point3d.y();
         *		point_pcl.z = point3d.z();
         *		
         *	  
         *		msg.points.push_back ( point_pcl );
    }
    
    // Publish the PointCloud
    objectMapPub_.publish( msg );
    }
    
    */   
        std::cout << "-------" << std::endl ;                                  
        
    }
    
    std::cout << "~~~~~~~~~~~~~~~~~~~~ObjectMap:~~~~~~~~~~~~~~~~~~~" << std::endl ;
    sptam_->omap.Print() ;
    std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl ;
    
    
    
    
    sptam_->omap.AddOutliers( frame_camera) ;
    
    
    
    
}

void sptam::stereo_driver::onImages(
    const sensor_msgs::ImageConstPtr& img_msg_left, const sensor_msgs::CameraInfoConstPtr& left_info,
    const sensor_msgs::ImageConstPtr& img_msg_right, const sensor_msgs::CameraInfoConstPtr& right_info
)
{
    /////////////////////////////
    // Current tate estimation //
    /////////////////////////////
    
    size_t currentSeq = img_msg_left->header.seq;
    ros::Time currentTime = img_msg_left->header.stamp;
    //ros::Time currentTime = ros::Time::now(); // Gaston: level7_full has a bug on headers stamps (they jump into the future)
    
    #ifdef SHOW_PROFILING
    writeToLog("FRAME_TIMESTAMP", currentSeq, currentTime.sec, currentTime.nsec);
    #endif
    
    // Check if a image message was missed
    ROS_DEBUG_STREAM_COND((currentSeq - lastImageSeq_) > 1, "STETEREO FRAME WAS MISSED! current: " << currentSeq << " last: " << lastImageSeq_);
    lastImageSeq_ = currentSeq;
    
    const CameraPose estimated_camera_pose = estimateCameraPose( currentTime );
    
    //////////////////////
    // Image processing //
    //////////////////////
    
    // Extract camera parameters from the first cameraInfo messages.
    if ( not cameraParametersLeft_ )
        loadCameraCalibration(left_info, right_info);
    
    // convert image to OpenCv cv::Mat format (without modifying color channels)
    cv_bridge::CvImageConstPtr bridgeLeft_ptr = cv_bridge::toCvShare(img_msg_left, "");
    cv_bridge::CvImageConstPtr bridgeRight_ptr = cv_bridge::toCvShare(img_msg_right, "");
    
    // save images
    cv::Mat imageLeft(bridgeLeft_ptr->image, left_roi_);
    cv::Mat imageRight(bridgeRight_ptr->image, right_roi_);
    
    #ifdef SHOW_PROFILING
    sptam::Timer t_extraction;
    t_extraction.start();
    #endif
    
    // Extract features
    FeatureExtractorThread featureExtractorThreadLeft(imageLeft, *feature_detector_left_, *descriptor_extractor_left_, sptamParameters().nFeatures);
    FeatureExtractorThread featureExtractorThreadRight(imageRight, *feature_detector_right_, *descriptor_extractor_right_, sptamParameters().nFeatures);
    
    featureExtractorThreadLeft.WaitUntilFinished();
    const std::vector<cv::KeyPoint>& keyPointsLeft = featureExtractorThreadLeft.GetKeyPoints();
    const cv::Mat& descriptorsLeft = featureExtractorThreadLeft.GetDescriptors();
    
    featureExtractorThreadRight.WaitUntilFinished();
    const std::vector<cv::KeyPoint>& keyPointsRight = featureExtractorThreadRight.GetKeyPoints();
    const cv::Mat& descriptorsRight = featureExtractorThreadRight.GetDescriptors();
    
    #ifdef SHOW_PROFILING
    t_extraction.stop();
    WriteToLog(" tk extraction: ", t_extraction);
    sptam::Timer t_hashing;
    t_hashing.start();
    #endif
    
    ImageFeatures imageFeaturesLeft(cv::Size(left_info->width, left_info->height), keyPointsLeft, descriptorsLeft, sptamParameters().matchingCellSize);
    ImageFeatures imageFeaturesRight(cv::Size(right_info->width, right_info->height), keyPointsRight, descriptorsRight, sptamParameters().matchingCellSize);
    
    //  std::vector<cv::DMatch> cvMatches;
    //  rowMatcher_->match(keyPointsLeft, descriptorsLeft, keyPointsRight, descriptorsRight, cvMatches);
    
    //  cv::Mat imageMatches;
    //  cv::drawMatches(imageLeft,keyPointsLeft,imageRight,keyPointsRight,cvMatches, imageMatches);
    
    //  cv::imshow("Initial Matches",imageMatches);
    //  cv::waitKey(0);
    
    #ifdef SHOW_PROFILING
    t_hashing.stop();
    WriteToLog(" tk hashing: ", t_hashing);
    WriteToLog(" tk features_left: ", keyPointsLeft.size());
    WriteToLog(" tk features_right: ", keyPointsRight.size());
    #endif
    
    ///////////////////////
    // Process new frame //
    ///////////////////////
    
    StereoFrame frame(
        currentSeq,
        estimated_camera_pose, *cameraParametersLeft_,
        stereo_baseline_, *cameraParametersRight_,
        imageFeaturesLeft, imageFeaturesRight, not isMapInitialized()
    );
    
    // TODO horrible, no se puede independizar el dibujado stereo/rgbd del report?
    TrackingReport tracking_report(imageLeft, imageRight);
    
    #ifdef SHOW_TRACKED_FRAMES
    if (stereoFrame_Pub_.getNumSubscribers() > 0)
        tracking_report.enableDrawOutput(TrackingReport::DRAW_BEFORE_REFINE_STEREO);
    else
    {
        if (leftFrame_Pub_.getNumSubscribers() > 0)
            tracking_report.enableDrawOutput(TrackingReport::DRAW_BEFORE_REFINE_LEFT);
        if (rightFrame_Pub_.getNumSubscribers() > 0)
            tracking_report.enableDrawOutput(TrackingReport::DRAW_BEFORE_REFINE_RIGHT);
    }
    
    if (stereoFrameAfter_Pub_.getNumSubscribers() > 0)
        tracking_report.enableDrawOutput(TrackingReport::DRAW_AFTER_REFINE_STEREO);
    else
    {
        if (leftFrameAfter_Pub_.getNumSubscribers() > 0)
            tracking_report.enableDrawOutput(TrackingReport::DRAW_AFTER_REFINE_LEFT);
        if (rightFrameAfter_Pub_.getNumSubscribers() > 0)
            tracking_report.enableDrawOutput(TrackingReport::DRAW_AFTER_REFINE_RIGHT);
    }
    #endif
    
    
    //sptam::base_driver::publishKFwithPose( currentSeq, currentTime, estimated_camera_pose, img_msg_left) ;
    SetLeftImageMessage(img_msg_left) ;
    
    onFrame(currentSeq, currentTime, frame, tracking_report);
}

void sptam::stereo_driver::loadCameraCalibration(
    const sensor_msgs::CameraInfoConstPtr& left_info,
    const sensor_msgs::CameraInfoConstPtr& right_info
)
{
    ROS_INFO_STREAM("init calib");
    
    // Check if a valid calibration exists
    if (left_info->K[0] == 0.0) {
        ROS_ERROR("The camera is not calibrated");
        return;
    }
    
    // Ponemos que el frame id de las camara info sea el mismo
    sensor_msgs::CameraInfoPtr left_info_copy = boost::make_shared<sensor_msgs::CameraInfo>(*left_info);
    sensor_msgs::CameraInfoPtr right_info_copy = boost::make_shared<sensor_msgs::CameraInfo>(*right_info);
    left_info_copy->header.frame_id = "stereo";
    right_info_copy->header.frame_id = "stereo";
    
    // Get Stereo Camera Model from Camera Info message
    image_geometry::StereoCameraModel stereoCameraModel;
    stereoCameraModel.fromCameraInfo(left_info_copy, right_info_copy);
    
    // Get PinHole Camera Model from the Stereo Camera Model
    const image_geometry::PinholeCameraModel& cameraLeft = stereoCameraModel.left();
    const image_geometry::PinholeCameraModel& cameraRight = stereoCameraModel.right();
    
    // Get rectify intrinsic Matrix (is the same for both cameras because they are rectify)
    cv::Mat projectionLeft = cv::Mat( cameraLeft.projectionMatrix() );
    cv::Matx33d intrinsicLeft = projectionLeft( cv::Rect(0,0,3,3) );
    cv::Mat projectionRight = cv::Mat( cameraRight.projectionMatrix() );
    cv::Matx33d intrinsicRight = projectionRight( cv::Rect(0,0,3,3) );
    
    assert(intrinsicLeft == intrinsicRight);
    
    const cv::Matx33d& intrinsic = intrinsicLeft;
    
    // Save the baseline
    stereo_baseline_ = stereoCameraModel.baseline();
    ROS_INFO_STREAM("baseline: " << stereo_baseline_);
    assert( stereo_baseline_ > 0 );
    
    // get the Region Of Interes (If the images are already rectified but invalid pixels appear)
    left_roi_ = cameraLeft.rawRoi();
    right_roi_ = cameraRight.rawRoi();
    
    cameraParametersLeft_ = std::make_unique<CameraParameters>(intrinsic, left_roi_.width, left_roi_.height, frustum_near_plane_distance_,  frustum_far_plane_distance_, stereo_baseline_);
    cameraParametersRight_ = std::make_unique<CameraParameters>(intrinsic, right_roi_.width, right_roi_.height, frustum_near_plane_distance_,  frustum_far_plane_distance_, stereo_baseline_);
}

void sptam::stereo_driver::processTrackingReport(const uint32_t seq, const ros::Time& time, const TrackingReport& report) const
{
    cv_bridge::CvImage cv_img;
    cv_img.encoding = "bgr8";
    cv_img.header.seq = seq;
    cv_img.header.stamp = time;
    
    if (stereoFrame_Pub_.getNumSubscribers() > 0)
    {
        cv_img.image = report.stereoFrameBeforeRefine;
        stereoFrame_Pub_.publish(cv_img.toImageMsg());
    }
    
    if (leftFrame_Pub_.getNumSubscribers() > 0)
    {
        cv_img.image = report.leftFrameBeforeRefine;
        leftFrame_Pub_.publish(cv_img.toImageMsg());
    }
    
    if (rightFrame_Pub_.getNumSubscribers() > 0)
    {
        cv_img.image = report.rightFrameBeforeRefine;
        rightFrame_Pub_.publish(cv_img.toImageMsg());
    }
    
    if (stereoFrameAfter_Pub_.getNumSubscribers() > 0)
    {
        cv_img.image = report.stereoFrameAfterRefine;
        stereoFrameAfter_Pub_.publish(cv_img.toImageMsg());
    }
    
    if (leftFrameAfter_Pub_.getNumSubscribers() > 0)
    {
        cv_img.image = report.leftFrameAfterRefine;
        leftFrameAfter_Pub_.publish(cv_img.toImageMsg());
    }
    
    if (rightFrameAfter_Pub_.getNumSubscribers() > 0)
    {
        cv_img.image = report.rightFrameAfterRefine;
        rightFrameAfter_Pub_.publish(cv_img.toImageMsg());
    }
}


