//
// Created by neo on 11/24/2020.
//

#ifndef N_KUNHOU_ARM_R2000_REFLECTOR_EXTRACTOR_H
#define N_KUNHOU_ARM_R2000_REFLECTOR_EXTRACTOR_H

#include "RobotStruct.h"
#include <iostream>
#include <math.h>
#include <boost/thread.hpp>
#include "buffer_con.hpp"
#include "R2000Driver/r2000_driver.h"
#include "R2000Driver/reflector_structure.h"
#include "R2000Driver/kdtree.h"


class reflector_extractor {
public:
	reflector_extractor();
	~reflector_extractor();

	void set_parameter(int sample,F32 start_angle,F32 resolution,int reflector_para_id);
	void th_run();
    void th_stop();
	void Process();
    void put_data(pepperl_fuchs::ScanData& r2000scan);

private:

    bool extracting(pepperl_fuchs::ScanData& r2000scan);
    bool check_cluster( beamCluster& in_cluster , beamCluster& out_cluster);
    bool get_center(const beamCluster& cluster,beam_point&center_beam);

    kdtree* buildKdtree(beamCluster &all_points, int dim);
    int nearest_set(beam_point center_point,float radian,kdtree* _kdtree);
    void destroyKdtree(kdtree* _kdtree);

    void recoder_laser_data(std::vector<beam_point> vec_p,std::string fi_name);
    void recoder_scan_data(pepperl_fuchs::ScanData& r2000scan,std::string fi_name);
private:

    int reflector_para_id_;
    reflector_parameters reflector_para_;
    F32 max_detect_distance_;
	boost::thread *th_;
	bool thread_run_;

	SLaser_L landmark;
    CSBuffer<ScanData,1> laser_buffer;

    F32 r2000_start_angle_;  //rad
    F32 r2000_resolution_;  //rad
    S32 r2000_point_;

    CSBuffer<pepperl_fuchs::ScanData,1> pfscan_buf_;


    int write_file_;
    bool debug_;
};


#endif //N_KUNHOU_ARM_R2000_REFLECTOR_H
