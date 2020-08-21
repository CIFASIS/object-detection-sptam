
#include "ObjectMeasurement.hpp"


cv::Matx44d ObjectMeasurement::computeTransformation()
{
  
  // R = O'
  const Eigen::Matrix3d rotationMatrix = src_cam_.GetPose().GetOrientationMatrix().transpose();

  // t = -R * C where C is the camera position
  const Eigen::Vector3d translation = -rotationMatrix * src_cam_.GetPose().GetPosition()  ;

  return cv::Matx44d(
    rotationMatrix(0, 0), rotationMatrix(0, 1), rotationMatrix(0, 2), translation[0],
    rotationMatrix(1, 0), rotationMatrix(1, 1), rotationMatrix(1, 2), translation[1],
    rotationMatrix(2, 0), rotationMatrix(2, 1), rotationMatrix(2, 2), translation[2],
    0, 0, 0, 1
  );
}



double_t depthFromMapPoints(const cv::Matx34d& projMat,const cv::Matx44d& transMat,const sptam::Map::SharedMapPointList& pointList,double_t x1x2, double_t y1y2,sptam::Map::SharedPoint& depth_point) {
       
   //boost::shared_lock<boost::shared_mutex> lock(object_mutex_); delete


   std::array < sptam::Map::SharedPoint,5 > keyMapPoint ;
   std::array < cv::Point2d,5 > foreground = {cv::Point2d(x1x2*0.5,y1y2*0.5),
                                              cv::Point2d(x1x2*0.33,y1y2*0.33),
                                              cv::Point2d(x1x2*0.33,y1y2*0.66),
                                              cv::Point2d(x1x2*0.66,y1y2*0.33),
                                              cv::Point2d(x1x2*0.66,y1y2*0.66)
                                             } ;
   std::array < double_t, 5 > dist =  {2000000.0,2000000.0,2000000.0,2000000.0,2000000.0} ;
   std::array < double_t, 5 > depth = {2000.0,2000.0,2000.0,2000.0,2000.0} ;
                   
   //sptam::Map::SharedMapPointMap ret ;

   std::cout << "Point Dephts" << std::endl ; 
   for ( const sptam::Map::SharedPoint& sMapPoint : pointList ) 
   {        
      const MapPoint& mapPoint = *sMapPoint ;
      const Eigen::Vector3d& point = mapPoint.GetPosition();
      cv::Point2d projection = project( projMat , eigen2cv( point ) );
     
      double_t auxaux = cv::norm(transform(transMat, eigen2cv( point ))) ;
      //std::cout << auxaux << std::endl ; 
 
      for (int i=0;i < 5;i++) {
         double_t difx = projection.x - foreground[i].x ;
         double_t dify = projection.y - foreground[i].y ;
         if (difx * difx + dify * dify < dist[i]) {
            dist[i] = difx * difx + dify * dify ;
            depth[i] = cv::norm(transform(transMat, eigen2cv( point ) )) ;
            keyMapPoint[i] = sMapPoint ;
         }
      }
   }

   std::array < std::pair<double_t, sptam::Map::SharedPoint > , 5 > depthmp ;

   for (int i=0; i< 5 ;i++) {
      //keyMapPoint[i]->updateColor(cv::Vec3b(0,0,128)) ; 
      depthmp[i] = std::make_pair(depth[i],keyMapPoint[i]) ; 
   }
      
   // Retorno la mediana
   //std::sort(depth.begin(),depth.end()) ;
   //std::sort(depthmp.begin(),depthmp.end()) ;
       
   //depthmp[0].second->updateColor(cv::Vec3b(0,255,127));//COLOR_CHARTREUSE);
   depth_point = depthmp[0].second ;   

   return depth[0] ;
}


double_t IoU(const size_t a_min_x, const size_t a_min_y, const size_t a_max_x, const size_t a_max_y,
             const size_t b_min_x, const size_t b_min_y, const size_t b_max_x, const size_t b_max_y)
{

    double_t epsilon = 0.001 ;

    size_t i_min_x = std::max(a_min_x,b_min_x) ;
    size_t i_min_y = std::max(a_min_y,b_min_y) ;
    size_t i_max_x = std::min(a_max_x,b_max_x) ;
    size_t i_max_y = std::min(a_max_y,b_max_y) ;

    double_t i_area = (i_max_x - i_min_x + 1) * (i_max_y - i_min_y + 1) ;
    double_t a_area = (a_max_x - a_min_x + 1) * (a_max_y - a_min_y + 1) ;
    double_t b_area = (b_max_x - b_min_x + 1) * (b_max_y - b_min_y + 1) ;

    if (i_area < epsilon) 
      return 0.0 ; 
    else
      return (i_area / double_t(a_area + b_area - i_area)) ;

}

ObjectMeasurement::ObjectMeasurement(const size_t obj_class, const double_t score, const sptam::Map::SharedKeyFrame& src_kf, const CameraPose& src_cp_delta,const size_t x1, const size_t y1, const size_t x2, const size_t y2) : obj_class_(obj_class), score_(score) ,src_kf_(src_kf),src_cp_delta_(src_cp_delta),src_cam_(src_kf->GetFrameLeft().GetCamera()), x1_(x1), y1_(y1), x2_(x2), y2_(y2)
{
   imagewidth_ = (double_t)src_cam_.GetCalibration().imageWidth() ;
   imageheight_ = (double_t)src_cam_.GetCalibration().imageHeight() ;
   focal_ = src_cam_.GetCalibration().focalLength() ;
   depthPoint_ = NULL ;
}


Camera ObjectMeasurement::GetBBoxCamera(const Eigen::Vector3d& up_vector, const CameraParameters& cp_in)
{
   //Repito código de GetNormal para obtner la orientacion Y la norma antes de normalizar
   // Hago todo en el World Frame
   // nueva orientacion                    
   Eigen::Vector3d z =  this->GetBBoxCenter_FocalPlane_World() - src_cam_.GetPosition() ;
   
   double_t f = z.norm() ;
   
   z.normalize() ;
   Eigen::Vector3d y = -up_vector ;
   y.normalize() ;
   Eigen::Vector3d x = y.cross(z) ;
   y = z.cross(x) ; 
  
   Eigen::Matrix3d o ;
   //Orientacion o transpuesta? probar
   //o.row(0) = x ; o.row(1) = y ; o.row(2) = z ;
   o.col(0) = x ; o.col(1) = y ; o.col(2) = z ;
   Eigen::Quaterniond q(o) ; 
   //
    
   cv::Matx33d intrin(f,   0.0,   0.0,
                       0.0,   f,   0.0,
                       0.0, 0.0,   1.0);
    
   CameraPose cameraPose(src_cam_.GetPosition(),q,Eigen::Matrix6d::Identity()) ;
   CameraParameters cameraParameters(intrin, x2_-x1_, y2_-y1_, cp_in.frustumNearPlaneDistance(), cp_in.frustumFarPlaneDistance(), cp_in.baseline()) ;
    
   return Camera(cameraPose,cameraParameters) ;
}

                                                                 // up_vector es respecto al frame de la camara
Eigen::Matrix3d ObjectMeasurement::GetObjRotationFromCamera(const double_t angle, Eigen::Vector3d& up_vector) 
{
   up_vector.normalize() ; 

   // bbox center in Camera Frame (0,0) in center (no offset)
   Eigen::Vector3d bbox_center_CF( (x1_+x2_)/2.0 - imagewidth_/2.0,
                                   (y1_+y2_)/2.0 - imageheight_/2.0,
                                    focal_) ;

   
   //Set up rotation to calculate Y object axis

   Eigen::Quaterniond t(Eigen::AngleAxisd(angle,up_vector)) ;
   Eigen::Vector3d y_dir = t.toRotationMatrix() * bbox_center_CF ;
   y_dir.normalize() ;
   
   // Z = up
   Eigen::Vector3d z_dir = up_vector ;
   // X = Y x Z
   Eigen::Vector3d x_dir = y_dir.cross(z_dir) ;
   y_dir = z_dir.cross(x_dir) ; 
   
   Eigen::Matrix3d R ;
   //Orientacion o transpuesta? probar
   //Seguro con columnas. Porque transformo una coordenada del sitema Objeto al sitema camara,
   //osea R * (1,0,0) me da col(0) que es la posicion del vector 1,0,0 en sistema Objeto pasado al sistema camara
   //OSEA COL SEGURO, pero terminologicamente es R o O?
   R.col(0) = x_dir ; R.col(1) = y_dir ; R.col(2) = z_dir ;
   //Eigen::Matrix3d R ;
   //R.row(0) = x_dir ; R.row(1) = y_dir ; R.row(2) = z_dir ;  
   /*
   std::cout << "R col 1 " <<  y_dir(0) << ' ' << y_dir(1) << ' ' << y_dir(2)  <<  std::endl ;
   std::cout << R(0,0) << ' '  << R(0,1) << ' ' << R(0,2) << std::endl
             << R(1,0) << ' '  << R(1,1) << ' ' << R(1,2) << std::endl
             << R(2,0) << ' '  << R(2,1) << ' ' << R(2,2) << std::endl ; 

   */ 
   return R ;
}

Eigen::Vector3d ObjectMeasurement::GetObjTranslationFromCamera(const Eigen::Matrix3d& rotation, 
                                                               const Eigen::Vector3d& obj_x1, 
                                                               const Eigen::Vector3d& obj_y1,
                                                               const Eigen::Vector3d& obj_x2,
                                                               const Eigen::Vector3d& obj_y2)
{

   double_t f = double_t(focal_) ;

   //Eigen::Matrix3d K ;
   //K << f,      0.0, 0.0,
   //     0.0,      f, 0.0,
   //     0.0,    0.0, 1.0;
   
   //Eigen::MatrixX3d KR = K * rotation ;
   
   Eigen::Vector3d A = rotation  * obj_x1 ;
   Eigen::Vector3d B = rotation  * obj_x2 ;
   Eigen::Vector3d E = rotation  * obj_y1 ;
   Eigen::Vector3d F = rotation  * obj_y2 ;
  
   // std::cout << "d "  << obj_x1(0) << ' ' <<  obj_x1(1) << ' ' <<  obj_x1(2) <<  std::endl ;
   // std::cout << "rd " << A(0) << ' ' <<  A(1) << ' ' <<  A(2) <<  std::endl ;
  
   double_t xmin = x1_ - imagewidth_/2.0  ;
   double_t xmax = x2_ - imagewidth_/2.0  ;
   double_t ymin = y1_ - imageheight_/2.0 ;
   double_t ymax = y2_ - imageheight_/2.0 ;
  
   double_t t3_width   = (f * B(0) - f * A(0) + xmin*A(2) - xmax*B(2))/(xmax - xmin) ;

   double_t t3_height =  (f * F(1) - f * E(1) + ymin*E(2) - ymax*F(2))/(ymax - ymin) ;
 

   double_t t1     = ((xmin/f) * (t3_width + A(2)))  - A(0) ;
   double_t t1_alt = ((xmax/f) * (t3_width + B(2)))  - B(0) ;
   double_t t2     = ((ymin/f) * (t3_width + E(2)))  - E(1) ;
   double_t t2_alt = ((ymax/f) * (t3_width + F(2)))  - F(1) ;

   double_t at1     = ((xmin/f) * (t3_height + A(2)))  - A(0) ;
   double_t at1_alt = ((xmax/f) * (t3_height + B(2)))  - B(0) ;
   double_t at2     = ((ymin/f) * (t3_height + E(2)))  - E(1) ;
   double_t at2_alt = ((ymax/f) * (t3_height + F(2)))  - F(1) ;

      
 

   //std::cout << t1 << "t1 alt " << t1_alt << '|' << t2 << "t2 alt " << t2_alt << '|' << t3_width << "t3 alt " << t3_height << std::endl ;
   //std::cout << at1 << "t1con t3 alt " << at1_alt << '|' << at2 << "t2  t3alt " << at2_alt << std::endl ;
   
   //Eigen::Vector3d T(t1,t2,t3) ;
 
   return Eigen::Vector3d((at1+at1_alt)/2.0,at2,t3_height) ;
   //return Eigen::Vector3d((t1+t1+at1+at1_alt)/4.0,(at2+at2+t2+t2_alt)/4.0,(t3_width+t3_height)/2.0) ;

}

Eigen::Vector3d ObjectMeasurement::GetObjPose(const double_t angle, Eigen::Vector3d& up, const Eigen::Vector3d& D, Eigen::Matrix3d& Rotation) {
  
   double_t f = focal_ ;

   Eigen::Matrix3d K ;
   K <<           f,    0.0,  imagewidth_/2.0,
                0.0,      f, imageheight_/2.0,
                0.0,    0.0,              1.0;	

  Eigen::Vector3d A( D(0)/2.0, D(1)/2.0, D(2)/2.0) ;
  Eigen::Vector3d C(-D(0)/2.0,-D(1)/2.0, D(2)/2.0) ;
  Eigen::Vector3d E( D(0)/2.0, D(1)/2.0,-D(2)/2.0) ;
  Eigen::Vector3d G(-D(0)/2.0,-D(1)/2.0,-D(2)/2.0) ;
  Eigen::Vector3d F(-D(0)/2.0, D(1)/2.0,-D(2)/2.0) ;
  Eigen::Vector3d H( D(0)/2.0,-D(1)/2.0,-D(2)/2.0) ;
  Eigen::Vector3d B(-D(0)/2.0, D(1)/2.0, D(2)/2.0) ;
  Eigen::Vector3d Q( D(0)/2.0,-D(1)/2.0, D(2)/2.0) ;



  std::array<Eigen::Vector3d, 4> YMAX = {E,G,F,H}  ; //F H al dope
  std::array<Eigen::Vector3d, 4> Ymin = {A,C,B,Q}  ; // B Q al dope
  std::array<Eigen::Vector3d, 4> XMAX = {E,F,G,H}  ;
  std::array<Eigen::Vector3d, 4> Xmin = {E,F,G,H}  ;
  std::array<Eigen::Vector3d, 8> OBB = {A,B,C,Q,E,F,G,H} ;

  Eigen::Matrix3d rotation = this->GetObjRotationFromCamera(angle,up) ;
  
  size_t errorT = 4000000 ;
  double_t maxIou = 0.0 ;
  Eigen::Vector3d minET ;
  Eigen::Vector3d iouET ;
  
  for (int iY=0; iY < 2 ;iY++) {
  for (int iy=0; iy < 2 ;iy++) {
  for (int iX=0; iX < 4 ;iX++) {
  for (int ix=0; ix < 4 ;ix++) {
    

     Eigen::Vector3d T = this->GetObjTranslationFromCamera(rotation,Xmin[ix],Ymin[iy],XMAX[iX],YMAX[iY]) ;
    // Eigen::Vector3d T = this->GetObjTranslationFromCamera(rotation,H,B,B,E) ;
     
     if (T(2) > 0.0) {
  
     size_t x_min = 10000 ;
     size_t x_max = 0 ;
     size_t y_min = 10000 ;
     size_t y_max = 0 ;

   
     for (int i=0; i < 8;i++) {
        Eigen::Vector3d homo =  K*(rotation * OBB[i] + T) ;
        size_t x = size_t((homo(0) / homo(2))+0.5) ;
        size_t y = size_t((homo(1) / homo(2))+0.5) ;
        if (x < -imagewidth_) x = -imagewidth_ ;
        if (x > 2*imagewidth_) x = 2*imagewidth_ ;
        if (y < -imageheight_) y = -imageheight_ ;
        if (y > 2*imageheight_) y = 2*imageheight_ ;

        if (x > x_max) x_max = x ;
        if (x < x_min) x_min = x ;
        if (y < y_min) y_min = y ;
        if (y > y_max) y_max = y ;
        
     }

     //std::cout << "Coord Pred: " << x_min << ' ' << y_min << ' ' << x_max << ' ' << y_max << std::endl ;
     size_t error = (x1_ - x_min)*(x1_ - x_min) + (x2_ - x_max)*(x2_ - x_max) + 
                    (y1_ - y_min)*(y1_ - y_min) + (y2_ - y_max)*(y2_ - y_max) ;
     if (error < errorT) {
      // std::cout << "DebugErrorT" << std::endl ;
       errorT = error ;
       minET = T ; 
     }

     //if (iou > maxIou) {
       //std::cout << "Iou debug T " << std::endl ;
     //  maxIou = iou ;
     //  iouET = T ;
    // }
  
     }
  }}}}
 
  // std::cout << minET(0) << ' ' <<  minET(1) << ' ' <<  minET(2) <<  std::endl ; 
  //std::cout << iouET(0) << ' ' <<  iouET(1) << ' ' <<  iouET(2) <<  std::endl ;

  Rotation = rotation ;


 return minET ;
}

Eigen::Vector3d ObjectMeasurement::ObjectNormalWorld(const double_t angle, Eigen::Vector3d& up_vector)
{
   up_vector.normalize() ;
  
   //bbox center in Camera Frame (0,0) in center (no offset)
   Eigen::Vector3d bbox_center_CF( (x1_+x2_)/2.0 - imagewidth_/2.0,
                                   (y1_+y2_)/2.0 - imageheight_/2.0,
                                   focal_) ;
   
  
   //Set up rotation to calculate Y object axis
   
   Eigen::Quaterniond t(Eigen::AngleAxisd(angle,up_vector)) ;
   Eigen::Vector3d y_dir = t.toRotationMatrix() * bbox_center_CF ;
   y_dir.normalize() ;
   //

   return src_cam_.GetPose().GetOrientationMatrix() * y_dir ;
}


Eigen::Vector3d ObjectMeasurement::GetFaceCorrection(const double_t objAngle, const Eigen::Vector3d& objDim) 
{
    double_t correction ;
    const double_t PI = 3.141592653589793;

    double_t alfa = std::atan2(objDim(1),objDim(0)) ;
    //Negativo del angulo, va desde Object Versor Y al vector Direcci�n
    double_t angle = 2.0*PI-objAngle ;

    if ((angle <= alfa) || (angle > 2.0*PI - alfa)) {
      //Interseca CD
      double_t slant = (objDim(1)/2.0) / std::cos(angle) ;
      correction = objDim(1) / 2.0 ;
      std::cout << "CD" << std::endl ;
    } else if (angle < PI-alfa)
    {
      //Interseca AD
      correction = objDim(0) / 2.0 ;
      std::cout << "AD" << std::endl ;
    } else if (angle < PI+alfa)
    {
      // Interseca AB
      correction = objDim(1) / 2.0 ;
      std::cout << "AB" << std::endl ;

    } else {
      // Interseca BC
      correction = objDim(0) / 2.0 ;
      std::cout << "BC" << std::endl ;
    }
 
    return Eigen::Vector3d(0.0,0.0,correction) ;
}
