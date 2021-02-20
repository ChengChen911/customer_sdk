#ifndef _PF_ODOM_POS_WANGHONGTAO_2017_12_12_
#define _PF_ODOM_POS_WANGHONGTAO_2017_12_12_

#include "Comm.h"
#include "buffer_con.hpp"

class pf_odom_pos
{
public:
	pf_odom_pos();
	~pf_odom_pos();

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
};





#endif//_PF_ODOM_POS_WANGHONGTAO_2017_12_12_
