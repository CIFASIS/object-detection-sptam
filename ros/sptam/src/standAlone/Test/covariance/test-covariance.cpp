/*
 * test-covariance.cpp
 * 
 * Copyright 2016 Unknown <tfischer@tostadora>
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 * 
 * 
 */

#include <list>
#include <random>
#include <iostream>
#include <Eigen/Core>

#include "Camera.hpp"
#include "CameraPose.hpp"
#include "tracker_g2o.hpp"

#include "utils/Draw.hpp"
#include "utils/macros.hpp"
#include "utils/Profiler.hpp"
#include "utils/cv2eigen.hpp"

#include "gui/PCLVisualizer.hpp"

#include "opencv2/core/version.hpp"
#if CV_MAJOR_VERSION == 2
  #include "configuration_parser_opencv2.hpp"
#elif CV_MAJOR_VERSION == 3
  #include "configuration_parser_opencv3.hpp"
#endif

#define N_MAP_POINTS 100
#define MIN_POINT_DEPTH 1
#define MAX_POINT_DEPTH 100

#define INITIAL_POSE_COVARIANCE Eigen::Matrix6d::Identity() * 1e-6

Eigen::Matrix3d setEulerYPR(double eulerZ, double eulerY, double eulerX)
{
	double ci ( cos(eulerX)); 
	double cj ( cos(eulerY)); 
	double ch ( cos(eulerZ)); 
	double si ( sin(eulerX)); 
	double sj ( sin(eulerY)); 
	double sh ( sin(eulerZ)); 
	double cc = ci * ch; 
	double cs = ci * sh; 
	double sc = si * ch; 
	double ss = si * sh;

	Eigen::Matrix3d R;

	R(0, 0) =  cj * ch; R(0, 1) = sj * sc - cs; R(0, 2) = sj * cc + ss;
	R(1, 0) =  cj * sh; R(1, 1) = sj * ss + cc; R(1, 2) = sj * cs - sc;
	R(2, 0) = -sj;      R(2, 1) = cj * si;      R(2, 2) = cj * ci;

	return R;
}

Eigen::Matrix3d setRPY(double roll, double pitch, double yaw)
{
	return setEulerYPR(yaw, pitch, roll);
}

Eigen::Quaterniond fromRPY(const Eigen::Vector3d& orientation_rpy)
{
	return Eigen::Quaterniond( setRPY(orientation_rpy[0], orientation_rpy[1], orientation_rpy[2]) );
}

void printPoint_(const Eigen::Vector3d& x)
{
  std::cout << "[" << x[0] << " " << x[1] << " " << x[2] << "]";
}

double toRad(double degree_value)
{
	return degree_value * M_PI / 180.0;
}

/**
 * @brief Build a camera pose object from 6d parameter vector.
 */
inline CameraPose getCameraPose(const Eigen::Vector3d& position, const Eigen::Vector3d& orientation_rpy)
{
	return CameraPose(position, fromRPY( orientation_rpy ), INITIAL_POSE_COVARIANCE);
}

void drawFeature(const cv::Mat& image, const cv::Point2d& feature, const cv::Scalar& color)
{
	cv::line(image, cv::Point(feature.x, feature.y - 4), cv::Point(feature.x, feature.y + 4), color);
	cv::line(image, cv::Point(feature.x + 4, feature.y), cv::Point(feature.x - 4, feature.y), color);
}

void runExperiment(size_t id, const Eigen::Vector3d& real_position, const Eigen::Vector3d& real_orientation, const CameraParameters& camera_calibration, bool debug = false)
{
	const CameraPose real_camera_pose = getCameraPose(real_position, real_orientation);

	std::random_device rd;
	std::mt19937 gen(rd());
	//~ gen.seed(std::random_device()());

	// Uniform distribution for generation of map point
	// uniformly distributed inside the camera frustum.
	std::uniform_real_distribution<> uniform(0, 1);

	// Normal distribution to simulate feature measurement noise.
	// 0 mean, 1 std fdev.
	std::normal_distribution<> feature_noise(0, 1);

	std::normal_distribution<> mu_x_noise(0, .1);
	std::normal_distribution<> mu_y_noise(0, .1);
	std::normal_distribution<> mu_z_noise(0, .1);

	// define actual point positions.
	//~ std::list<Eigen::Vector3d> map_points;
	std::list<sptam::Map::Point> map_points;
	forn(i, N_MAP_POINTS)
	{
		// They are defined in the camera frame because it is easier to
		// compute random points that fall inside the frustum.
		double xz = (MAX_POINT_DEPTH-MIN_POINT_DEPTH) * uniform( gen ) + MIN_POINT_DEPTH;

		double x_range_2 = xz * tan( toRad( camera_calibration.horizontalFov() ) / 2. );
		double xx = 2 * x_range_2 * uniform( gen ) - x_range_2;

		double y_range_2 = xz * tan( toRad( camera_calibration.verticalFov() ) / 2. );
		double xy = 2 * y_range_2 * uniform( gen ) - y_range_2;

		// point in homogenous camera frame coordinates.
		Eigen::Vector3d xc( xx, xy, xz );

		// Now we transform it to world coordinates and save it into the map.
		Eigen::Vector3d xw = real_camera_pose.ToWorld( xc );
		//~ map_points.push_back( xw );

		// Build SPTAM map point
		map_points.emplace_back( MapPoint(xw, Eigen::Vector3d(), cv::Mat(), Eigen::Matrix3d()) );
	}

	// ================================================================ //
	// Feature measurement
	// ================================================================ //

	// Now project onto the camera introducing some noise
	// to the camera parameters

	std::list<Match> measurements;

	for ( sptam::Map::Point& map_point : map_points )
	{
		// add some noise to the parameters
		// TODO ...
		Eigen::Vector3d noisy_position = real_position + Eigen::Vector3d( 0, 0, 0 );
		Eigen::Vector3d noisy_orientation = real_orientation + Eigen::Vector3d( 0, 0, 0 );

		// recompute the camera object
		const CameraPose noisy_camera_pose = getCameraPose(noisy_position, noisy_orientation);
		const Camera noisy_camera(noisy_camera_pose, camera_calibration);

		// project onto the new camera
		const cv::Matx34d& projection_matrix = noisy_camera.GetProjection();

		const cv::Point3d point = eigen2cv( map_point.GetPosition() );
		const cv::Point2d projection = project(projection_matrix, point);

		const cv::Point2d noisy_feature = projection + cv::Point2d( feature_noise( gen ), feature_noise( gen ) );

		// Build SPTAM meas and match
		Measurement meas(Measurement::Type::LEFT, Measurement::Source::SRC_TRIANGULATION, noisy_feature, cv::Mat());
		measurements.push_back( Match(map_point, meas) );
	}

	// ================================================================ //
	// Bundle Adjustment
	// ================================================================ //

	// Introduce some noise into the camera parameters
	// to be used as a camera prediction for BA.
	// TODO ...
	CameraPose predicted_camera_pose = real_camera_pose;

	// Now run BA using the camera prediction, the map points,
	// and the noisy measurements.
	tracker_g2o tracker(camera_calibration.focalLengths(), camera_calibration.principalPoint(), camera_calibration.baseline());

	CameraPose refined_camera_pose = tracker.RefineCameraPose(predicted_camera_pose, measurements);

	if ( debug )
	{
		// ================================================================ //
		// show some debug image
		// ================================================================ //

		cv::Mat image = cv::Mat::zeros(camera_calibration.imageHeight(), camera_calibration.imageWidth(), CV_8UC3);
		//~ cv::Mat image = cv::Mat::zeros(1000, 1000, CV_8UC3);
		{
			const Camera real_camera(real_camera_pose, camera_calibration);
			const cv::Matx34d& projection_matrix = real_camera.GetProjection();

			for ( const Match& match : measurements )
			{
				const cv::Point3d point = eigen2cv( match.mapPoint.GetPosition() );
				const cv::Point2d projection = project(projection_matrix, point);

				drawFeature(image, projection, COLOR_RED);

                const cv::Point2d measurement = match.measurement.GetKeypoints()[0].pt;
				drawFeature(image, measurement, COLOR_YELLOW);

				cv::line(image, projection, measurement, COLOR_GREEN);
			}
		}

		cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );
		cv::imshow( "Display window", image );
		cv::waitKey(0);

		// ================================================================ //
		// Show PCL cloud
		// ================================================================ //

		PCLVisualizer visualizer;
		visualizer.addCamera(real_camera_pose.GetPosition(), real_camera_pose.GetOrientationQuaternion(), camera_calibration.horizontalFov(), camera_calibration.verticalFov(), camera_calibration.frustumNearPlaneDistance(), camera_calibration.frustumFarPlaneDistance(), refined_camera_pose.covariance());
		for ( sptam::Map::Point& map_point : map_points )
			visualizer.addMapPoint( map_point.GetPosition() );
		visualizer.wait();
	}

	// ================================================================ //
	// Print pose and covariance for further analysis.
	// ================================================================ //

	std::cout << "BASE_LINK_POSE: " << id << " ";
	poseToStream(std::cout, refined_camera_pose.GetPosition(), refined_camera_pose.GetOrientationMatrix(), refined_camera_pose.covariance());
	std::cout << std::endl;
}

int main(int argc, char** argv)
{
	const CameraParameters camera_calibration = loadCameraCalibration("/home/tfischer/proyectos/sptam/configurationFiles/fantasy_cam.yaml", MIN_POINT_DEPTH, MAX_POINT_DEPTH);

	Eigen::Vector3d real_position( 0, 0, 0 );
	Eigen::Vector3d real_orientation( 0, -M_PI_4, 0 );

	runExperiment(0, real_position, real_orientation, camera_calibration, true);

	//~ forn(i, 1000)
		//~ runExperiment(i, real_position, real_orientation, camera_calibration);

	return 0;
}
