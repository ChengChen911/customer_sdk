/*
 * reflector_struct.h
 *
 *  Created on: Nov 24, 2020
 *      Author: wangrui
 */

#ifndef REFLECTOR_STRUCT_H_
#define REFLECTOR_STRUCT_H_
#include "RobotStruct.h"
#include <vector>

struct beam_point{
    F32 distance_;  // m
    F32 orient_;    // rad
    U32 rssi_;
    F32 x_;
    F32 y_;

    beam_point(){};
    beam_point(F32 dist, F32 orie, U32 rssi, F32 x,F32 y):
    	distance_(dist), orient_(orie), rssi_(rssi),x_(x), y_(y){}

    static F32 beam_distance(beam_point &a , beam_point &b){
    	F32 dx = a.x_ - b.x_;
    	F32 dy = a.y_ - b.y_;
    	return sqrt(dx*dx+dy*dy);
    }
};

typedef std::vector<beam_point>  beamCluster;


typedef struct _Sreflector_parameters
{
	int type_;  //1:cylinder  2: plane
	F32 diameter_;   // m 反光柱半径
	F32 width_;      //m  反光条宽度
	F32 rssi_para_a_;
	F32 rssi_para_b_;
	F32 rssi_para_c_;
	F32 number_para_a_;
	F32 number_para_b_;
	F32 number_para_c_;
	F32 min_rssi_;
	F32 max_rssi_;
	F32 min_distance_;
	F32 max_distance_;
	F32 inliner_radius_;  //kdtree 半径 m
	F32 cluster_diff_;  //分组 容差  m

}reflector_parameters; /*							*/

//Reflector Parameter Set  0 - 9 : R2000  ; 10 -19 : nanscan3  ;  20- 29 : lms151
static const  reflector_parameters  Reflector_parameter[] =
{
		//R2000
	{1 , 0.075 , 0.075 , 2256.1 , -0.341 , 0.0 ,100.49 , -1.028 , 0.0 , 500, 2200 ,0.1 ,25, 0.07, 0.1}, // 不对应类型
	{1 , 0.075 , 0.075 , 2256.1 , -0.341 , 0.0 ,100.49 , -1.028 , 0.0 , 500, 2200 ,0.1 ,25, 0.05, 0.1}, //R2000, 圆柱75mm rssi  = 2256.1x^(-0.341) ; num = 100.49x^(-1.028)
	{1 , 0.050 , 0.050 ,      0 ,    0   , 0.0 , 0    ,    0   , 0.0 , 500, 2200  ,0.3 ,25, 0.05, 0.1 },//R2000, 圆柱50mm
	{2 , 0.070 , 0.075 ,     0 ,      0  , 0.0 ,  0   ,   0    , 0.0 , 500, 2200  ,0.3 ,25, 0.07, 0.1}, //R2000, 平面75mm
	{2 , 0.050 , 0.050 ,     0 ,      0  , 0.0 ,  0   ,   0    , 0.0 , 500, 2200  ,0.3 ,25, 0.05, 0.1}, //R2000, 平面50mm
	{1 , 0.075 , 0.075 ,     0 ,      0  , 0.0 ,  0   ,   0    , 0.0 , 500, 2200  ,0.3 ,25, 0.07, 0.1}, //预留
	{1 , 0.050 , 0.050 ,     0 ,      0  , 0.0 ,  0   ,   0    , 0.0 , 500, 2200  ,0.3 ,25, 0.05, 0.1}, //预留
	{1 , 0.075 , 0.075 ,     0 ,      0  , 0.0 ,  0   ,   0    , 0.0 , 500, 2200  ,0.3 ,25, 0.07, 0.1},
	{1 , 0.050 , 0.050 ,     0 ,      0  , 0.0 ,  0   ,   0    , 0.0 , 500, 2200  ,0.3 ,25, 0.05, 0.1},
	{1 , 0.050 , 0.050 ,     0 ,      0  , 0.0 ,  0   ,   0    , 0.0 , 500, 2200  ,0.3 ,25, 0.05, 0.1},

	//nanoscan3
	{1 , 0.075 , 0.075 , 2256.1 , -0.341 , 0.0 ,23.509 , -1.0 ,  0.0 , 160, 255 ,0.4 ,15, 0.07, 0.1},     //NANS3, 圆柱75mm
	{1 , 0.050 , 0.050 , 2256.1 , -0.341 , 0.0 ,15.5  , -1.0 , 0.0   , 160, 255 ,0.2 ,15, 0.06, 0.1},  //NANS3, 圆柱50mm
	{1 , 0.075 , 0.075 ,      0 ,    0   , 0.0 , 0    ,    0   , 0.0 , 500, 2200  ,0.3 ,15, 0.05, 0.1 }, //NANS3, 平面75mm
	{2 , 0.050 , 0.050 ,     0 ,      0  , 0.0 ,  0   ,   0    , 0.0 , 500, 2200  ,0.3 ,15, 0.07, 0.1},  //NANS3, 平面50mm
	{2 , 0.050 , 0.050 ,     0 ,      0  , 0.0 ,  0   ,   0    , 0.0 , 500, 2200  ,0.3 ,15, 0.05, 0.1},
	{1 , 0.075 , 0.075 ,     0 ,      0  , 0.0 ,  0   ,   0    , 0.0 , 500, 2200  ,0.3 ,15, 0.07, 0.1}, //预留
	{1 , 0.050 , 0.050 ,     0 ,      0  , 0.0 ,  0   ,   0    , 0.0 , 500, 2200  ,0.3 ,15, 0.05, 0.1}, //预留
	{1 , 0.075 , 0.075 ,     0 ,      0  , 0.0 ,  0   ,   0    , 0.0 , 500, 2200  ,0.3 ,15, 0.07, 0.1},
	{1 , 0.050 , 0.050 ,     0 ,      0  , 0.0 ,  0   ,   0    , 0.0 , 500, 2200  ,0.3 ,15, 0.05, 0.1},
	{1 , 0.050 , 0.050 ,     0 ,      0  , 0.0 ,  0   ,   0    , 0.0 , 500, 2200  ,0.3 ,15, 0.05, 0.1},

};





#endif /* REFLECTOR_STRUCT_H_ */
