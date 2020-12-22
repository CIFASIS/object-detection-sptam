#pragma once

#include <utility>      // std::pair, std::make_pair
#include <vector>
#include <list>
#include <map>
#include <array>
#include <set>
#include <algorithm>
#include <iostream>
#include "Map.hpp"
#include <eigen3/Eigen/Geometry>
#include "utils/cv2eigen.hpp"
#include "ObjectMeasurement.hpp"
#include "ObjectEdge.hpp"
#include "ObjectPoint.hpp"

#include <boost/thread/shared_mutex.hpp>
#include <boost/thread/locks.hpp>
#include <boost/thread/lock_types.hpp>

#include "opencv2/core/version.hpp"
#if CV_MAJOR_VERSION == 2
  #include <opencv2/core/core.hpp>
#elif CV_MAJOR_VERSION == 3
  #include <opencv2/core.hpp>
#endif

//namespace sptam{
  //class Map;
//}

typedef std::shared_ptr<ObjectPoint> SharedObjectPoint ;
typedef std::list<SharedObjectPoint> ObjectPointList;

const static  std::string names[] = { "bathtub", "bed", "chair", "desk", "dresser", "monitor", "night_stand", "sofa", "table", "toilet"};

class Object
{
  public:
  
   // typedef std::vector< sptam::ObjectMeasurement::PointPair > PointPairVector ;
  

    Object();


    Object(const Object& obj,const size_t id);

    inline size_t GetClass() const
    {
      boost::shared_lock<boost::shared_mutex> lock(object_mutex_);
      return class_;
    }

    inline std::string GetClassName() const
    {
      boost::shared_lock<boost::shared_mutex> lock(object_mutex_);
      return names[class_-1] + std::to_string(id_)  ;
    }


    inline void updateClass(const size_t new_class)
    {
      boost::unique_lock<boost::shared_mutex> lock(object_mutex_);
      class_ = new_class;
    }
 


    inline void AddMapPoint(const sptam::Map::SharedPoint& sharedPoint)
    {
      //boost::unique_lock<boost::shared_mutex> lock(object_mutex_);
     
      //supuestamente esto inicializa en 0 si no existe
      ++objectPoints_[sharedPoint] ;   
  
      //objectPoints_.insert(sptam::Map::SharedPointIntPair(sharedPoint, 1)) ;
    }
   
    inline void AddMapPoints(const sptam::Map::SharedMapPointList& list)
    {
      //boost::unique_lock<boost::shared_mutex> lock(object_mutex_);

      for (const auto& point : list) ++objectPoints_[point] ;
    }
    
    inline void AddMeasurement(const SharedObjectMeasurement& sharedMeasurement)
    {
       objectMeasurements_.insert(sharedMeasurement) ;
    }

    inline void AddEdge(const SharedObjectEdge& sharedEdge) 
    {
       boost::unique_lock<boost::shared_mutex> lock(object_mutex_);
       objectEdges_.insert(sharedEdge) ;
    }

 
    inline Eigen::Vector3d GetPosition() const
    {
      boost::shared_lock<boost::shared_mutex> lock(object_mutex_);
      return position_;
    }

    inline Eigen::Vector3d GetNormal() const
    {
      boost::shared_lock<boost::shared_mutex> lock(object_mutex_);
      return normal_;
    }

    inline void updateNormal(const Eigen::Vector3d& normal)
    {
      boost::unique_lock<boost::shared_mutex> lock(object_mutex_);
      normal_ = normal;
    }

    
    inline bool GetHit() const
    {
      boost::shared_lock<boost::shared_mutex> lock(object_mutex_);
      return hit_;
    }

    inline void updateHit(const bool& hit)
    {
      boost::unique_lock<boost::shared_mutex> lock(object_mutex_);
      hit_ = hit;
    }




    inline void updatePosition(const Eigen::Vector3d& position)
    {
      boost::unique_lock<boost::shared_mutex> lock(object_mutex_);
      position_ = position ;
    }
   
    inline Eigen::Quaterniond GetOrientation() const {
      boost::shared_lock<boost::shared_mutex> lock(object_mutex_);
      return orientation_ ;
    }

    inline Eigen::Matrix3d GetOrientationMatrix() const {
      boost::shared_lock<boost::shared_mutex> lock(object_mutex_);
      return orientationMatrix_ ;
    }



    inline void updateOrientation(const Eigen::Quaterniond& new_orientation)
    {
      boost::unique_lock<boost::shared_mutex> lock(object_mutex_);
      orientation_ = new_orientation ;
      orientationMatrix_ = orientation_.toRotationMatrix();
    }

    inline Eigen::Vector3d GetDimensions() const
    {
      boost::shared_lock<boost::shared_mutex> lock(object_mutex_);
      return dimensions_;
    }

    inline void updateDimensions(const Eigen::Vector3d& new_dimensions)
    {
      boost::unique_lock<boost::shared_mutex> lock(object_mutex_);
      dimensions_ = new_dimensions;
    }



    //inline void updateOrientationMatrix(const Eigen::Matrix3d& new_orientationMatrix)
    //{
    //  boost::unique_lock<boost::shared_mutex> lock(object_mutex_);
    //  orientationMatrix_ = new_orientationMatrix ;
    //  orientation_ = orientationMatrix_.toQuaternion();
    //}


    const Eigen::Matrix3d covariance() const
    {
      boost::shared_lock<boost::shared_mutex> lock(object_mutex_);
      return covariance_;
    }

    // Statistics update operations.

    // should this be considered a bad point?
    inline bool IsBad() const
    {
      boost::shared_lock<boost::shared_mutex> lock(object_mutex_);
      //std::cout << "nMeasurements: " << mapPoint.measurementCount_ << std::endl;
      //std::cout << "projectionCount: " << mapPoint.projectionCount_ << std::endl;
      //std::cout << "inliers: " << mapPoint.inlierCount_ << std::endl;
      //std::cout << "outliers: " << mapPoint.outlierCount_ << std::endl;

      return (inlierCount_ + outlierCount_ > 3) && (inlierCount_ < outlierCount_) ;

        // If it has no measurement
       // ( measurementCount_ == 0 )
        // If the outlier count is too high
       // or ( 20 < outlierCount_ and inlierCount_ < outlierCount_ )
        // If the measurement count is too low
       // or ( 20 < projectionCount_ and 10 * measurementCount_ < projectionCount_ );
    }

    inline bool IsDeletable() const
    {      
      boost::shared_lock<boost::shared_mutex> lock(object_mutex_);
      return (inlierCount_ + 4 < outlierCount_ ) ; 
    }


    inline size_t GetPointSize() 
    {
      boost::shared_lock<boost::shared_mutex> lock(object_mutex_);
      return objectPoints_.size() ;

    }  
 

    inline void IncreaseOutlierCount()/* const **/
    {
      boost::unique_lock<boost::shared_mutex> lock(object_mutex_);
      outlierCount_++;
    }

    inline void IncreaseInlierCount()/* const **/
    {
      boost::unique_lock<boost::shared_mutex> lock(object_mutex_);
      inlierCount_++;
    }

    inline void IncreaseProjectionCount()/* const **/
    {
      boost::unique_lock<boost::shared_mutex> lock(object_mutex_);
      projectionCount_++;
    }

    inline void IncreaseMeasurementCount()/* const **/
    {
      boost::unique_lock<boost::shared_mutex> lock(object_mutex_);
      measurementCount_++;
    }

    /**
     * @brief TODO give the color a meaning
     *
     * @return
     *   (r,g,b) components are in the range [0, 255).
     */
    inline cv::Vec3d getColor() const
    { 
      boost::shared_lock<boost::shared_mutex> lock(object_mutex_);
      return color_ ;
    }

    inline void updateColor(const cv::Vec3b& new_color)
    {
      boost::shared_lock<boost::shared_mutex> lock(object_mutex_);
      color_ = new_color ;
    }
    
    inline void addBoxMeasurement(const Eigen::Vector3d& normal,const double_t width,const double_t height) 
    {
    ;
    }
    
    

     
    const sptam::Map::SharedMapPointMap PointIntersectionSlow(const sptam::Map::SharedMapPointList& points) ;
     
     
    inline size_t PointIntersection(const sptam::Map::SharedMapPointSet& points) {
      
       size_t ret = 0 ;
     
       sptam::Map::SharedMapPointSet::iterator mapPoint_it ;
       sptam::Map::SharedMapPointMap::iterator objPoint_it ;
       
 
       for( mapPoint_it = points.begin(), objPoint_it = objectPoints_.begin();
         (mapPoint_it != points.end()) && (objPoint_it != objectPoints_.end()); ) 
        { 
           const sptam::Map::SharedPoint objPoint = objPoint_it->first ;
           const sptam::Map::SharedPoint mapPoint = *mapPoint_it ;           

           if (objPoint.get() < mapPoint.get()) {
             ++objPoint_it ;
           } else if (objPoint.get() == mapPoint.get()) {
             ++ret ;
             ++objPoint_it ;
             ++mapPoint_it ;
           } else {
             ++mapPoint_it ;
           }        

        
        }

        return ret ;
    
    }
    
    const sptam::Map::SharedMapPointList GetObjectMapPoints() ;
    
    const ObjectPointList GetObjectBBoxPoints();

    const sptam::Map::SharedMapPointList GetObjectDepthPoints() ; 


    void updateEstimate() 
    {
       boost::unique_lock<boost::shared_mutex> lock(object_mutex_);
       Eigen::Vector3d position(0.0,0.0,0.0) ;
       dimensions_ << 0.0 , 0.0 , 0.0 ;
       orientationMatrix_ << 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 ; 
       for(const auto& edge : objectEdges_ ) {
          position += edge->GetPosition_World() ;  
          dimensions_ += edge->GetDimensions() ;
          orientationMatrix_ += edge->GetOrientation_World() ;   
       }  
       position_ = position / objectEdges_.size() ;
       dimensions_ = dimensions_ / objectEdges_.size() ;
       orientationMatrix_ = orientationMatrix_ / objectEdges_.size() ;
       orientation_ = Eigen::Quaterniond(orientationMatrix_) ;
       std::cout << objectEdges_.size() << " EDGES" << std::endl ;    
    }

    void updateEstimateMedian(const Eigen::Vector3d& up)
    {
       boost::unique_lock<boost::shared_mutex> lock(object_mutex_);
       Eigen::Vector3d position(0.0,0.0,0.0) ;
       Eigen::Vector3d normal(0.0,0.0,0.0) ;
      
       bool first = true ;
 
       std::vector <double> p0,p1,p2,d0,d1,d2 ;
       
       //dimensions_ << 0.0 , 0.0 , 0.0 ;
       //orientationMatrix_ << 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 ;
       for(const auto& edge : objectEdges_ ) {
           p0.push_back(edge->GetPosition_World()(0));
           p1.push_back(edge->GetPosition_World()(1));
           p2.push_back(edge->GetPosition_World()(2));
           d0.push_back(edge->GetDimensions()(0));
           d1.push_back(edge->GetDimensions()(1));
           d2.push_back(edge->GetDimensions()(2));
           if (first) {
              normal = edge->GetNormal() ;
              first = false ;
           } else {
              normal = normal / 2.0 + edge->GetNormal() / 2.0 ;
              normal.normalize() ;
           } 
           //orientationMatrix_ += edge->GetOrientation_World() ;
       }

       std::sort(p0.begin(), p0.end());
       std::sort(p1.begin(), p1.end());
       std::sort(p2.begin(), p2.end());
       std::sort(d0.begin(), d0.end());
       std::sort(d1.begin(), d1.end());
       std::sort(d2.begin(), d2.end());



       position_(0) = p0[p0.size()/2] ;
       position_(1) = p1[p1.size()/2] ;
       position_(2) = p2[p2.size()/2] ;
       dimensions_(0) = d0[d0.size()/2] ;
       dimensions_(1) = d1[d1.size()/2] ;
       dimensions_(2) = d2[d2.size()/2] ;
 
       //normal = normal / objectEdges_.size() ;  
       //    
       Eigen::Vector3d y_dir = normal ;
       y_dir.normalize() ;
       // Z = up
       Eigen::Vector3d z_dir = up ;
       // X = Y x Z
       Eigen::Vector3d x_dir = y_dir.cross(z_dir) ;
       //z_dir = x_dir.cross(y_dir) ; 
       y_dir = z_dir.cross(x_dir) ;
       
       //Orientacion o transpuesta? probar
       //Seguro con columnas. Porque transformo una coordenada del sitema Objeto al sitema camara,
       //osea R * (1,0,0) me da col(0) que es la posicion del vector 1,0,0 en sistema Objeto pasado al sistema camara
       //OSEA COL SEGURO, pero terminologicamente es R o O?
       Eigen::Matrix3d R;
       R.col(0) = x_dir ; R.col(1) = y_dir ; R.col(2) = z_dir ;
       orientationMatrix_ = R ;



       //orientationMatrix_ = orientationMatrix_ / objectEdges_.size() ;
       orientation_ = Eigen::Quaterniond(orientationMatrix_) ;
       std::cout << objectEdges_.size() << " EDGES" << std::endl ;

    }
  

  private:

    //Map Points del objeto
    sptam::Map::SharedMapPointMap objectPoints_ ;
 
    SharedObjectMeasurementSet objectMeasurements_ ;

    SharedObjectEdgeSet objectEdges_ ; 

    mutable boost::shared_mutex object_mutex_;

    // position in world coordinates.
    Eigen::Vector3d position_;
 
    Eigen::Quaterniond orientation_;

    Eigen::Matrix3d orientationMatrix_;


    Eigen::Matrix3d covariance_;

    // unit vector represents direction with maximizes object width
    Eigen::Vector3d normal_;
    
    // Relativas al centro?
    Eigen::Vector3d dimensions_;
    
    //
    size_t class_ ;

    size_t id_ ;

    // This stats are used to evaluate the quality of the mapPoint.

    /**
     * @brief how many times was this point marked as an outlier
     */
    /*mutable */size_t outlierCount_;

    /**
     * @brief how many times was this point marked as an inlier
     */
    /*mutable */size_t inlierCount_;

    /**
     * @brief How many times was this point projected onto an image?
     * In other terms, its the number of views whose frustum contains
     * this point.
     */
    /*mutable */size_t projectionCount_;

    /**
     * @brief How many times was this point find in an image?
     * Note that this include the measurements from the tracking phase
     * and the keyframes.
     */
    /*mutable */size_t measurementCount_;
     
    /*
 *   How many times was this object expected to be seen, but was not measured  
 * */
  
    /*mutable */size_t missCount_;

    /*
 *   Keep track hit or miss in this frame
 * */
    /*mutable */ bool hit_ ;
 
    cv::Vec3b color_ ;
    //friend class sptam::Map;
};




