#ifndef _POS_LASER_REF_WANGHONGTAO_2017_12_12_
#define _POS_LASER_REF_WANGHONGTAO_2017_12_12_

#include <map>

#include "Comm.h"
#include "RobotStruct.h"
#include "buffer_con.hpp"

#include "scanmatcher/psm_node.h"

#include "ref_feature_loc/pos_laser_map.h"

class pos_laser_ref{

public:

	pos_laser_ref();
	~pos_laser_ref();

	SBAG_ref init();

	void load_all();
	void save(const std::string file_name,const SBAG_ref &bag);
	void del(const std::string file_name);
	void del_all();

	void test(SBAG &show_bag,SBAG_ref &cur_bag);

	int relocated( SBAG &show_bag, SBAG_ref &cur_bag ,std::string &match_file);
	int adjust_amcl_pos( const SBAG &ref_bag, SBAG &cur_bag);

private:

	F32 laser_dx_;
	F32 laser_dy_;


	SPos laser_diff_;

	SPos odom2laser(const SPos& w_pos);
	SPos odom_diff2laser_diff(SPos pos_amcl , SPos pos_ref);
	SPos laser_diff2odom_diff(const SPos& laser_diff);


	pos_laser_map pos_laser_map_;

	PSMNode psm_node_;
};


#endif//_POS_LASER_REF_WANGHONGTAO_2017_12_12_
