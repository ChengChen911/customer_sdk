//
// Created by neo on 12/27/19.
//

#ifndef N_KUNHOU_ARM_R2000_REFLECTOR_H
#define N_KUNHOU_ARM_R2000_REFLECTOR_H

#include "RobotStruct.h"
#include <iostream>
#include <math.h>
#include <boost/thread.hpp>
#include "buffer_con.hpp"
#include "R2000Driver/r2000_driver.h"

#define CHANGLIANG_DIFF 0.004340383317396314  //Different resolution values are different
#define REFLECTOR_DIAMETER_  0.075
#define VEC_MAX 100

struct point{
    double dist;
    double orient;
    U32 amplitude;
    point(double _dist, double _orient, U32 _amplitude):dist(_dist), orient(_orient), amplitude(_amplitude){}
};

typedef struct xy{
    double x;
    double y;
    double orient;
    double dist;
}XY;
bool point_less_sort_distance(point,point);

class r2000_reflector {
public:
	r2000_reflector();
	r2000_reflector(ScanData &Laser_data);
    void Process();
    void Set_data(ScanData &Laser_data);
    void put_data(pepperl_fuchs::ScanData& r2000scan);
    void set_parameter(int sample,F32 start_angle);

	void getLM_Thread();
	F32 diff_angle;
    ~r2000_reflector();
    void th_stop();
private:
    U32 thres_amp;
    U32 max_amp;
    double norm(double dist1, double orient1, double dist2, double orient2);
    std::pair<U32, U32> get_m(std::vector<point > s);
    bool judge(std::vector<point > s, bool feg);
    bool process(ScanData &laser_data);
    //处理拖尾问题
    bool process(pepperl_fuchs::ScanData &laser_data);

    F32 get_stdev(std::vector<point > s);
    void get_close_points(std::vector<point > s,F32 target_dis,F32  dis_threshold,std::vector<point >& res);
    void recoder_laser_data(std::vector<point> vec_p,std::string fi_name);
    void recoder_laser_data(std::vector<std::vector<point>> vec_vec_p,std::string fi_name);
    void recoder_landmark_data(SLaser_L l,std::string fi_name);
    bool polynomialCurveFit(std::vector<std::pair<double,double>> input,int n,double x[VEC_MAX] );
    bool get_reflector_center(std::vector<point> s,XY& center);
private:
    SLaser_L landmark;
    ScanData laser_data;
    double number_function;
    double amplitude_a;
    double amplitude_b;
	boost::thread *th_;

	//add
	double form1_a;
	double form1_b;
	double form2_k;
	double form2_y;

	volatile bool is_getdata;
    CSBuffer<ScanData,1> laser_buffer;

    bool write_file_;
    F32 stdev_threshold_;       // greater thres,will filter again

    F32 max_detect_dis_;
    bool amplitude_fit_;
    bool distance_fit_;
    F32 distance_fit_range_;
    bool thread_run_;

    F32 r2000_start_angle_;
    F32 r2000_resolution_;
    S32 r2000_point_;

    CSBuffer<pepperl_fuchs::ScanData,1> pfscan_buf_;
};


#endif //N_KUNHOU_ARM_R2000_REFLECTOR_H
