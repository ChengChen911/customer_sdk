#ifndef _POS_LASER_MAP_WANGHONGTAO_2017_12_12_
#define _POS_LASER_MAP_WANGHONGTAO_2017_12_12_

#include <map>

#include "Comm.h"
#include "RobotStruct.h"
#include "buffer_con.hpp"
#include "Geometry.h"
#include "bag/bag.h"

#include "quadtree/QuadTree.h"

#include "laser_filter/laser_filter.h"

// #include "ref_feature_loc/q4tree_object.h"
// #include "ref_feature_loc/q4tree_node.hp



class pos_laser_map{

public:

	pos_laser_map();
	~pos_laser_map();


	void load_all();
	void save(const std::string file_name,const SBAG &bag);
	void del( const std::string file_name);
	void del_all();

	void put(SBAG &bag);
	int get( std::vector<SBAG_ref> &res, const SPos &pos);

	void debug_print();
	SBAG_ref debug_all_print();
	void tree_test();

	

private:
	laser_filter l_filter_;
	bool b_first_frame_;
	void load_bag( std::string path );
	void load_new_bag( std::string path);
	void put( const std::string file_name, SBAG &bag);
	

	//std::multimap< Point2s , SBAG> mp ;
private://data tree
	void init_tree();
	void free_tree();
	bool get_laser_para(F32 &dx,F32 &dy,F32 &angle,const std::string file_name);
	Line make_line(const F32 &x,const F32 &y,const F32 &th_deg);

	
	//QuadTreeNode<q4object> *quadTree_;
	QuadTree<int>* qtree_;
	std::map<int,SBAG_ref> m_bag_list_;
	int m_bag_id_;
	boost::mutex map_data_lock_;

	F32 ref_scan_diff_x_;
	F32 ref_scan_diff_y_;
	F32 ref_scan_diff_th_;

	//diff reflaser_
private:
	void init_current_laser();
	void laserdiff2odomdiff(SPos &amcl_pos,const SPos &laser_ref_para);
	bool laserdiff(const SPos &laser_ref,const SPos &laser_cur);
	SPos laser_para_cur_;
	int b_use_diff_ref_;

public:
	SBAG ref_bag_;
	SBAG cur_bag_;
};



#endif//_POS_LASER_MAP_WANGHONGTAO_2017_12_12_
