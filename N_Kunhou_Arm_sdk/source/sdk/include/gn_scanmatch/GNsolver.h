/*
 * GNsolver.h
 *
 *  Created on: Jul 13, 2020
 *      Author: wangrui
 */

#ifndef GNSOLVER_H_
#define GNSOLVER_H_
#include <cstdio>
#include <vector>
#include <cmath>
#include <iostream>

#include "gn_scanmatch/gridmap.h"
#include "RobotStruct.h"
#include "Eigen/Core"
#include "Eigen/Dense"
#include "Geometry.h"

class GN_solver {
public:
	GN_solver();
	virtual ~GN_solver();

	//
	//用激光雷达数据创建势场．
	map_t* CreateMapFromLaserPoints(Eigen::Vector3d map_origin_pt,
	                              std::vector<Eigen::Vector2d> laser_pts,
	                              double resolution,double sigma= 0.5);

	map_t* CreateMapFromOutline(Eigen::Vector3d map_origin_pt,
	                              std::vector<Eigen::Vector2d> laser_pts,
	                              double resolution);

	void matchData(Eigen::Vector3d& init_pose,std::vector<Eigen::Vector2d>& laser_pts);
	//用高斯牛顿的方法来进行优化
	Eigen::Vector3d GaussianNewtonOptimization(map_t*map,Eigen::Vector3d& init_pose,std::vector<Eigen::Vector2d>& laser_pts,int maxIterations =10);
	void ComputeHessianAndb(map_t* map, Eigen::Vector3d now_pose,
	                                  std::vector<Eigen::Vector2d>& laser_pts,
	                                  Eigen::Matrix3d& H, Eigen::Vector3d& b);
	Eigen::Vector2d GN_TransPoint(Eigen::Vector2d pt,Eigen::Matrix3d T);
	Eigen::Matrix3d GN_V2T(Eigen::Vector3d vec);
	Eigen::Vector3d InterpMapValueWithDerivatives(map_t* map,Eigen::Vector2d& coords);

	//
	void createOfficeMap();
	void createCarMap();
	void createLaserMap(const ScanData& msg);
	void scan2laser(ScanData &scan_data , SLaser & laser);
	void ConvertChampionLaserScanToEigenPointCloud(const SLaser& msg,
	                                               std::vector<Eigen::Vector2d>& eigen_pts);
	void ConvertChampionLaserScanToEigenPointCloud(const ScanData& msg,
	                                               std::vector<Eigen::Vector2d>& eigen_pts);
	void ConvertAllLaserScanToEigenPointCloud(const ScanData& msg,
	                                               std::vector<Eigen::Vector2d>& eigen_pts,int start = 0);

	void laser_scan_to_scan_match(const SLaser& scanmsg);
	void laser_scan_to_scan_match(const ScanData& scanmsg);

	void laser_scan_to_map_match(const SLaser& scanmsg);
	void laser_scan_to_map_match(const ScanData& scanmsg,Eigen::Vector3d& estimate_Pose);
	void calibrate_laser(const ScanData& scanmsg,Eigen::Vector3d& estimate_Pose);

	void setInitpose(Eigen::Vector3d p);
	void setTrans( Eigen::Vector3d);
	Eigen::Matrix3d get_R();

	Eigen::MatrixXd infomatrix;
    Eigen::Vector3d m_prevLaserPose;
    double laser_sigma;
    map_t* m_map;

    Eigen::Vector3d Trans_laser2robot;
    bool use_robot_center_;

    std::vector< map_t*> m_map_container; // 0 - N  , resolution high -> low
};

#endif /* GNSOLVER_H_ */
