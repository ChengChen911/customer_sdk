#ifndef _ODOM_INTERPOLATER_20191125_
#define _ODOM_INTERPOLATER_20191125_

#include "Comm.h"
#include "robot/Geometry.h"
#include "buffer_con.hpp"


// a predefine size(SIZE) ring buffer  : put over head , if empty get will waiting
template<typename T,int SIZE>
class ValBuffer
{
private:
	boost::mutex mu;
	boost::condition_variable_any cond_get;

	boost::circular_buffer<T> buffer_list;

public:
	explicit ValBuffer() : buffer_list(SIZE)
	{};
	size_t size(){
		boost::mutex::scoped_lock lock(mu);
		return buffer_list.size();
	};
	void put(T in)
	{

		{
			boost::mutex::scoped_lock lock(mu);
			if (buffer_list.full())
			{
				//buffer_list.pop_front();
				//std::cout << "full !! over head ..." << std::endl;
			}

			buffer_list.push_back(in);

		}
		//std::cout<<"buffer size:"<<buffer_list.size()<<std::endl;
		cond_get.notify_one();

	};
	void get(T &out)
	{
		{
			boost::mutex::scoped_lock lock(mu);

			while (buffer_list.empty())
			{
				//std::cout << "empty waiting..." << std::endl;
				cond_get.wait(lock);
			}
			out = buffer_list.front();
			buffer_list.pop_front();

		}

	};
	void clear()
	{
		boost::mutex::scoped_lock lock(mu);
		while(!buffer_list.empty()){
			buffer_list.pop_front();
		}
	};
	void get_min(T &out)
	{
		T  val_min ;
		{
			boost::mutex::scoped_lock lock(mu);
			if(buffer_list.size()<1)
				return ;

			val_min = buffer_list[0];
			for(int i = 1;i<buffer_list.size();i++){
				if(fabs(val_min)>fabs(buffer_list[i]))
					val_min = buffer_list[i];
			}

			out = val_min;
		}
	}
	void get_max(T &out)
	{
		T  val_max ;
		{
			boost::mutex::scoped_lock lock(mu);
			if(buffer_list.size()<1)
				return ;

			val_max = buffer_list[0];
			for(int i = 1;i<buffer_list.size();i++){
				if(fabs(val_max)<fabs(buffer_list[i]))
					val_max = buffer_list[i];
			}

			out = val_max;
		}
	}
	void get_avg(T &out)
	{

		T  val_sum ;
		{
			boost::mutex::scoped_lock lock(mu);
			if(buffer_list.size()<1)
				return ;

			for(int i = 1;i<buffer_list.size();i++){
				val_sum += buffer_list[i];

			}

			out = val_sum/buffer_list.size();
		}
	}
};

class odometry_interpolator
{
public:
	odometry_interpolator();
	~odometry_interpolator();

	void scanmatch_interpolator( const SPos &amcl_pos , const SOdomSpeed &odom_pos );
	SPos update_odom( const SOdomSpeed &odom_pos ,F32 &range_diff,F32 &angle_diff,F32& vx_max,F32& vw_max);
	SPos get_location();
	void clear_accumulate();
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
	SOdomSpeed last_odom_;
	VecPosition vp_diff_;
	F32 angleRad_diff_;


	bool  first_odom_;
	SOdomSpeed interpolator_odom_;

	ValBuffer<F32,5> buffer_vx_;
	ValBuffer<F32,5> buffer_vw_;
};

#endif
