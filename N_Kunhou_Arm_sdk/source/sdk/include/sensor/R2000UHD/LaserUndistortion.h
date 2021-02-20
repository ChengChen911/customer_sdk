/*
 * LaserUndistortion.h
 *
 *  Created on: Sep 30, 2019
 *      Author: neo
 *      Desc: An laser undistortion algorithm  by  chassis odometry;
 */

#ifndef LASERUNDISTORTION_H_
#define LASERUNDISTORTION_H_
#include "Comm.h"
#include "R2000UHD/R2000uhd.h"


class LaserUndistortion {
public:
	LaserUndistortion();
	virtual ~LaserUndistortion();

	void set_laser_para(F32 dx,F32 dy,F32 angle_diff);
	void put_odom(SOdomSpeed os);
	void put_scan_data(const scanData1& scandata);

	bool scan_undistortion();
	void get_adjust_scan(scanData1& scandata);

private:
	void get_robot_odom_diff(double vx,double vw,double dt,double last_ox,double last_oy,double last_oa,double&ox,double &oy,double &oa);
	void odom_diff2laser_diff(double rx,double ry,double ra/*rad*/,double laser_dx,double laser_dy,double laser_dth,double& lx,double& ly,double& la/*rad*/);
private:

	boost::mutex mu_odom_;
	SOdomSpeed last_odom_;

	boost::mutex mu_scan_data_;
	scanData1  scan_data_;

	scanData1  result_scan_;
private:
	F32 laser_dx_;
	F32 laser_dy_;
	F32 laser_angle_diff_;  //rad

	F32 scan_interval_;  //scan time ,s
	F32 bin_num_ ; // bin number
};

#endif /* LASERUNDISTORTION_H_ */
