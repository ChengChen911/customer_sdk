#ifndef ZWD_TETRADSTEER_CHENG_H_
#define ZWD_TETRADSTEER_CHENG_H_


#include "TimerDiff.h"
#include "chassis/driver_base.h"
#include "chassis/canalyst2.h"

class cTransferDevice;

class zwd_tetradsteer : Driver_base
{
public:
	zwd_tetradsteer();
	~zwd_tetradsteer();

protected:

private:

	std::string port_;
	//SerialPort* sport_;
	cTransferDevice* pTransDevice_;
	F32 reduction_ratio_;
	U32 counts_;
	U32 one_wheel_counts_;

	F32 reduction_ratio_steer_;
	U32 counts_steer_;
	U32 one_wheel_counts_steer_;

private://virtual customized :TODO
	void setPara(std::string name ,std::string value);
	bool open_transfer_device();
	bool init_driver();
	void close_driver();

	void set_speed(U32 id, S16 &speed);
	void set_steer(U32 id, S32 &angle);

	void setRPM(int id,F32 v);
	F32 getSpeed(int id);

	F32 getDiffAngle(int id);

	void setAngle(int id,F32 rad_angle);
	F32 getAngle(int id);

	void get_steer_angle_ctl(U8 *data, F32 &angle, bool &ctl, bool &get_ctl);
	void get_speed_pos(U8 *data, S32 &pos, F32 &speed);

private:

	void init_customer_para();

	F32 GetDiffPos(int id );

	void Init_Speed_Driver(U32 id);
	void Init_Steer_Driver(U32 id);

	void Canopen_NMT_setOP ();
    void CANopen_Send_SDO ( U8 OP, U8 Index_low, U8 Index_high , U8 SubIndex , S32 data);
    void CANopen_Send_PDO (U32 data1, S32 data2);
    void CANopen_Send_PDO_CUSTOM(U32 id, U8 *buf,const int num);

    void CANopen_readPos ();


	bool b_run_;

	bool first_odo_;
	cTimerDiff dt_;
	cTimerDiff left_dt_;


	bool cal1_;
	bool cal2_;
	bool cal3_;
	bool cal4_;

	bool get_cal1_;
	bool get_cal2_;
	bool get_cal3_;
	bool get_cal4_;

	S32 left_pos_1_;
	S32 last_left_pos_1_;
	S32 deta_left_pos_1_;

	S32 left_pos_2_;
	S32 last_left_pos_2_;
	S32 deta_left_pos_2_;

	S32 right_pos_3_;		    //pos
	S32 last_right_pos_3_;
	S32 deta_right_pos_3_;

	S32 right_pos_4_;		    //pos
	S32 last_right_pos_4_;
	S32 deta_right_pos_4_;

	F32 c_speed_left_1_;
	F32 c_speed_left_2_;
	F32 c_speed_right_3_;
	F32 c_speed_right_4_;

	F32 c_angle_left_1_;
	F32 c_angle_left_2_;
	F32 c_angle_right_3_;
	F32 c_angle_right_4_;

	U32 COBID;
	U8 CANmsg_send[16];
	U8 CANmsg_receive[1024];

private://canalyst2
	canalyst2 canalyst2_;

};




#endif /* MOONS_DRIVER_H_ */
