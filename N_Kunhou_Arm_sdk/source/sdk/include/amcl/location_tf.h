#ifndef _LOCATION_TF_WANGHONGTAO_2017_12_12_
#define _LOCATION_TF_WANGHONGTAO_2017_12_12_

#include "Comm.h"

class location_tf
{
public:
	location_tf();
	~location_tf();

	void adjusting_tf( const SPos &amcl_pos , const SPos &odom_pos );
	SPos get_pos_after_tf( const SPos &odom_pos );
	SPos get_pos();
protected:
private:

	double tf_x_;
	double tf_y_;
	double tf_a_;

	double wx_;
	double wy_;
	double wa_;
	boost::mutex mu_p_;

	SPos w_pos_;
};





#endif//_location_tf_WANGHONGTAO_2017_12_12_
