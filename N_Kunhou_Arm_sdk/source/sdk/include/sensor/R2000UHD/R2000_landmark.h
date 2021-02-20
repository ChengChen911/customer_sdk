//
// Created by exbot on 10/12/18.
//

#ifndef N_KUNHOU_ARM_R2000_LANDMARK_H
#define N_KUNHOU_ARM_R2000_LANDMARK_H

#include "RobotStruct.h"
#include "getmatrix.h"
#include <iostream>
#include <math.h>
#include <boost/thread.hpp>
#include "buffer_con.hpp"

#define CHANGLIANG 0.004340383317396314  //Different resolution values are different
#define HUDU 0.017453292519943295
#define MAX_DETECT_POINT 8400


typedef struct _SLaser_R2000
{
	U32 stamp_;
	F32 start_angle_;
	F32 resolution_;	//
	F32 range_max_;
	F32 range_min_;

	F32 data_[8400];
	S32  echo_[8400];
	U32 used_;
	F32 angle_[8400];  //degree
} SLaser_R;




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
bool less_sort_distance(point,point);

class R2000_landmark {
public:
    R2000_landmark();

    void Process();
    void Set_data(SLaser_R &Laser_data);
    bool process(SLaser_R &laser_data);

	void getLM_Thread();
	F32 diff_angle;
    ~R2000_landmark();
    void th_stop();
    void get_landmarks(SLaser_L& lmks);

private:
    U32 thres_amp;
    U32 max_amp;
    double norm(double dist1, double orient1, double dist2, double orient2);
    std::pair<U32, U32> get_m(std::vector<point > s);
    bool judge(std::vector<point > s, bool feg);
    XY getcenter(double x, std::vector<point> s);
    double get_x(std::vector<point> s);
    void get_adjust(std::vector<std::pair<double, double> > &s);


    F32 get_stdev(std::vector<point > s);
    void get_close_points(std::vector<point > s,F32 target_dis,F32  dis_threshold,std::vector<point >& res);
    void recoder_laser_data(std::vector<point> vec_p,std::string fi_name);
    void recoder_laser_data(std::vector<std::vector<point>> vec_vec_p,std::string fi_name);
    void recoder_landmark_data(SLaser_L l,std::string fi_name);
    bool polynomialCurveFit(std::vector<std::pair<double,double>> input,int n,double x[MAX] );
    bool get_reflector_center(std::vector<point> s,XY& center);

    //plane reflector
    bool process_reflector_plane(std::vector<point>& vec_p);
    F32 plane_bin_num(F32  range ,F32 laser_bin_resolution_ , F32 plane_w);
private:
    SLaser_L landmark;
    SLaser_R laser_data;
    double number_function;
    double amplitude_a;
    double amplitude_b;
	boost::thread *th_;

	//add
	double form1_a; // fitting curve para of cylinder reflector
	double form1_b;
	double form2_k;
	double form2_y;

	volatile bool is_getdata;
    CSBuffer<SLaser_R,1> laser_buffer;

    bool write_file_;
    F32 stdev_threshold_;       // greater thres,will filter again

    F32 max_detect_dis_;
    bool amplitude_fit_;
    bool distance_fit_;
    F32 distance_fit_range_;
    bool thread_run_;


    F32 plane_width_;
    F32 plane_para_a_;
    F32 plane_para_b_;



public:
    int reflector_type_; //1:cylinder  2: plane
};


#endif //N_KUNHOU_ARM_R2000_LANDMARK_H
