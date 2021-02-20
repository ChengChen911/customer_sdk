/*
 * KH_Kiva_driver.h
 *
 *  Created on: Aug 26, 2020
 *      Author: SudekiMing
 */

#ifndef  KH_KIVA_DRIVER_H_
#define KH_KIVA_DRIVER_H_

#include "TimerDiff.h"
#include "chassis/driver_base.h"
#include "chassis/canalyst2.h"

#define CLOSE_ABSOLUTE_POS	    0x2F
#define START_ABSOLUTE_POS    	0x3F

class cTransferDevice;

class KH_Kiva_driver: Driver_base
{
public:
	KH_Kiva_driver();
	~KH_Kiva_driver();

protected:

private:

	std::string port_;
	//SerialPort* sport_;
	cTransferDevice* pTransDevice_;
	F32 reduction_ratio_;
	U32 counts_;
	U32 one_wheel_counts_;
	U32 jack_speed_;

private://virtual customized :TODO
	void setPara(std::string name ,std::string value);
	void close_driver();
	bool open_transfer_device();
	bool init_driver();

	void setRPM(int id,F32 v);
	void setAngle(int id,F32 rad_angle);
	F32 getSpeed(int id);
	F32 getAngle(int id);
	F32 getDiffAngle(int id);

private:
	boost::mutex mu_action;
	S32 sub_index_;
	S32 fnc_code_;
	S32 para_;

	bool hasTask;
	bool isWorking;
	bool is_top_;
	bool is_bottom_;
	U8 din_id_top_;
	U8 din_id_bottom_;
	S32 fnc_time_;

	bool init_climb_;
	bool first_init_;

	S32 climb_reset_up_;
	S32 climb_reset_down_;
	S32 set_enc_top_;
	S32 set_enc_bottom_;
	S32 cur_enc_;
	U16 updown_time_;
	U8 set_enc_top_data_[4];
	U8 set_enc_bottom_data_[4];

	U8 dio_fork_status_;

	U8 Rotation_Scram_;
	S32 Rotation_InitPos_;

private:
	void setfunction(void);
	bool init_updown();
	S32 get_din_value( int id,SDI  din );
	bool Rotate_FindZero( void );

private:
	typedef enum
	{
		Position_Mode = 0x01,
		Immediate_Velocity_Mode = -0x03,
		Velocity_Mode = 0x03,
		Pulse_Mode = -0x04 ,
		Torque_Mode = 0x04  ,
		Origin_Mode = 0x06

	} KINCO_MODE;

	canalyst2 canalyst2_;
	cTimerDiff dt_;
	cTimerDiff  Time_dt_;

	U32 COBID;

	bool b_run_;

	bool first_odo_;

	S32 left_pos_;
	S32 last_left_pos_;
	S32 deta_left_pos_;

	S32 right_pos_ ;
	S32 last_right_pos_;
	S32 deta_right_pos_;

	F32 c_left_;
	F32 c_right_;

	S32 rotation_pos_;

	boost::mutex l1_lock_;
	boost::mutex r1_lock_;


private:
	void init_KH_Kiva_para();

	F32 GetDiffPos(int id);

	void CANopen_NMT_setOP ();
    void CANopen_Send_SDO ( U8 OP, U8 Index_low, U8 Index_high , U8 SubIndex , S32 data);
    void CANopen_Send_PDO ( S32 data);

    void Kinco_DRV_SetMode(U8 Driver_ID,S16 Work_Mode);
    void Kinco_DRV_SetCtrlWord(U8 Driver_ID,S16 Work_Status);
    void Kinco_DRV_SetVelocity(U8 Driver_ID,S32 Motor_Velocity);
    void Kinco_DRV_SetPos(U8 Driver_ID,U32 Motor_Velocity,S32 Motor_Pos);
    void Kinco_DRV_GetVelocity ();
    void Kinco_DRV_GetPos ();

};

#endif /* KH_KIVA_DRIVER_H_ */
