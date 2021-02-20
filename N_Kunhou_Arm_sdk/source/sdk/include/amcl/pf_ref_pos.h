#ifndef _PF_REF_POS_WANGHONGTAO_2017_12_12_
#define _PF_REF_POS_WANGHONGTAO_2017_12_12_

#include "Comm.h"
#include "buffer_con.hpp"
#include "bag/bag.h"

class pf_ref_pos
{
public:
	pf_ref_pos();
	~pf_ref_pos();

	void init(std::string str_path);

	void set_odom( const SOdomSpeed &odom );
	SPos get_hyper_amcl( const SPos &amcl_pos );

protected:
private:

	void adjusting_tf( const SPos &amcl_pos , const SPos &odom_pos );
	SPos get_pos_after_tf( const SPos &odom_pos );

	double tf_x_;
	double tf_y_;
	double tf_a_;

	double wx_;
	double wy_;
	double wa_;

	SBuffer<SPos> hyper_pos;
	THSafe<SPos> amcl_pos_;
// ref laser pos
	cbag ref_list;
};





#endif//_PF_REF_POS_WANGHONGTAO_2017_12_12_
