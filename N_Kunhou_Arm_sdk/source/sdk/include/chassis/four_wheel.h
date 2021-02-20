#ifndef _CHASSIS_FOUR_WHEEL_WANGHONGTAO_2015_11_30_
#define _CHASSIS_FOUR_WHEEL_WANGHONGTAO_2015_11_30_

#include "TimerDiff.h"
#include "chassis/driver_base.h"

class cTransferDevice;

class four_wheel : Driver_base
{
public:
	four_wheel();
	~four_wheel();

protected:

private:
	std::string port_;
	std::string port_lift_;
	cTransferDevice* pTransDevice_;
	cTransferDevice* pTransDevice_lift_;
	F32 reduction_ratio_;
	U32 counts_;

	U32 one_wheel_counts_;

private://virtual customized :TODO
	void setPara(std::string name ,std::string value);
	bool open_transfer_device();
	bool open_transfer_device_lift();
	bool init_driver();
	void close_driver();
	void mechanism_back();
	void mechanism_up();
	void mechanism_down();
	void mechanism_stop();
	void setRPM(int id,F32 v);
	void voltage();
	F32 getSpeed(int id);

	F32 getDiffAngle(int id);

	void setAngle(int id,F32 rad_angle);
	F32 getAngle(int id);
	bool first_odo_;

private:
	//copley chassis example, customer should not do like this
	void init_customer_para();

	F32 GetDiffPos(int id );
	unsigned short CRC16(unsigned char *puchMsg, unsigned short usDataLen);
	void SendData( U8* s_data,U16 len );
	void SendData_lift_( U8* s_data,U16 len );
	void ReadData( U8* r_data,int &len,int need,int timeout );
	void ReadData_lift_( U8* r_data,int &len,int need,int timeout );

	cTimerDiff com_dt_;

	cTimerDiff left_dt_;
	S32 left_speed_;
	S32 right_speed_;


	S32 left_pos_;
	S32 last_left_pos_;
	S32 deta_left_pos_;

	F32 left_set_v_;
	F32 left_current_v_;

	cTimerDiff right_dt_;
	S32 right_pos_ ;		    //pos
	S32 last_right_pos_;
	S32 deta_right_pos_;

	F32 right_set_v_;
	F32 right_current_v_;

	F32 c_left_;
	F32 c_right_;

	boost::mutex mu_action;
	U8 fork_status_;
	U32 fnc_code_;
	S32 dio_fork_status_;
	U8 resback[128];
    S32 up_hight;
    S32 down_hight;
    U16 voltagetest;
    U8 resv[128];

    int charger_status_;
    int agv_id_;

	int count;
	bool first_time;
	U32 fnc_code_previous_;

	bool hasTask;
	bool hasBattery;
};



#endif//_CHASSIS_COPLEY_DRIVER_WANGHONGTAO_2015_11_30_
