#ifndef _BAG_WANGHONGTAO_20151230_
#define _BAG_WANGHONGTAO_20151230_

#include <string>
#include <vector>
#include <fstream>

#include <boost/thread.hpp>
#include "buffer_con.hpp"

#include "RobotStruct.h"
#include "robot/Geometry.h"

typedef struct _SBAG
{
	SOdomSpeed odom;
	SLaser laser;
	SPos amcl_pos;
}SBAG;

typedef struct _SBAG_ref
{
	SBAG bag_;
	SPos laser_para_;
	std::string file_name_;

	Line ln_x_axis_;
	Line ln_y_axis_;

}SBAG_ref;

class cbag{
	
public:
	cbag();
	~cbag();
	
	int get_size(std::string file_name);

	bool load_Bag(SBAG &itbag);
	bool load_Bag(std::string file_name,SBAG &itbag);

	void put_back_odom( const SOdomSpeed &odom);
	void put_odom( const SOdomSpeed &odom,const bool &force_input = false);
	void put_laser(const SLaser &laser);
	void put_amcl(const SPos &amcl_pos );

	void put_hyper_pos(const SPos &amcl_pos );

	void save(bool begin);
	bool save_one(const SBAG &bag);
	
	void get_one(SBAG &bag);

	void begin_save();
	void end_save(std::string file_name);
	void ref_save(std::string file_name);

	void close();
	void show_pro();
	void show_over();

	void get_original_diff(F32 &ox,F32 &oy,F32 &oa);

	void set_recoder_hyper_pos(bool rec);

private:

	SBAG last_bag_;
	std::vector<SBAG>::iterator it;
	int index;

	CSBuffer<SBAG,10> bag_buf_;

	std::fstream fs_;

	void init_thread();
	void end_thread();
	void th_run();
	
	SPos  last_hyper_pos_;

	boost::thread* bag_th_;
	bool b_run_;

	std::string log_file_name_;

	std::string json_path_;
	int itotal_;
	int icount_;

	bool recoder_hyper_pos_;

private:
	void set_max_min(const F32 &x,const F32 &y);
	F32 x_max_;
	F32 x_min_;
	F32 y_max_;
	F32 y_min_;
	F32 rotation_;
};


#endif//_BAG_WANGHONGTAO_20151230_
