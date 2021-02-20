/*
 * huizhuan_driver.h
 *
 *  Created on: Sep 6, 2018
 *      Author: kunhou
 */

#ifndef HUIZHUAN_DRIVER_H_
#define HUIZHUAN_DRIVER_H_

#include "chassis/driver_base.h"
#include "TimerDiff.h"

class huizhuan_driver:Driver_base
{
public:

	huizhuan_driver();
	~huizhuan_driver();

private:
	void setPara(std::string name ,std::string value);

	bool init_driver();
	void close_driver();
	void setRPM(int id,F32 v);
	bool open_transfer_device();

	F32 getSpeed(int id);
	F32 getDiffAngle(int id);

private:
	bool first_odo_;
	std::string port_;
	cTransferDevice* pTransDevice_;
	std::string port_plc_;
	cTransferDevice* pTransDevice_plc_;
	cTimerDiff dt;

	S16 SetAngle_;
	S16 SetSpeed_;
	bool is_forward_;
	bool is_turn_;
	bool is_manual_right;

	S16 current_angle_;
	S16 current_speed_;

	bool current_forward_;

	F32 reduction_ratio_;
	U32 counts_;
	U32 one_wheel_counts_;

	S16 init_angle_pos_;
	S16 dete_pos_;
	S16 pos_;
	S16 last_pos_;

private:
	U8 DO0_;
	U8 DO1_;
	U8 DO2_;
	U8 DO3_;
	U8 DO01_idle_;
	U8 DO02_run_;
	U8 DO03_error_;
	U8 DO04_obs1_;
	U8 DO05_obs2_;
	U8 DO06_obs3_;
	U8 DO07_obs4_;
	U8 DO10_leftlight1_;
	U8 DO11_leftlight2_;
	U8 DO12_leftlight3_;
	U8 DO13_rightlight1_;
	U8 DO14_rightlight2_;
	U8 DO15_rightlight3_;
	U8 DO16_relay1_;
	U8 DO17_relay2_;
	U8 DO24_relay3_;
	U8 DO25_relay4_;
	U8 DO20_horn1_;
	U8 DO21_horn2_;
	U8 DO22_horn3_;
	U8 DO23_horn4_;
	U8 DO26_res1_;
	U8 DO27_res2_;
	U8 DO30_res3_;
	U8 DO31_res4_;
	U8 DO32_res5_;

	U8 DI0_;
	U8 DI1_;
	U8 DI2_;
	U8 DI3_;
	U8 DI00_mode_;
	U8 DI02_scram_;
	U8 DI03_coll_;
	U8 DI04_obs1_;
	U8 DI05_obs2_;
	U8 DI06_obs3_;
	U8 DI07_obs4_;
	U8 DI10_W_;
	U8 DI11_S_;
	U8 DI12_A_;
	U8 DI13_D_;
	U8 DI14_res1_;
	U8 DI15_res2_;
	U8 DI16_res3_;
	U8 DI17_res4_;
	U8 DI21_res5_;

	U8 MODE_;//0:auto;1:manual:
	U8 SCRAM_;//0:run;1:stop
	U8 OBS_;
	U8 Trun_light_;//0:no trun;1:left;2:right

	void send_plc();
	void read_plc();

private:
	bool SendData( U8* s_data,U16 len);
	bool ReadData(  U8* r_data,int &len,int except);
	bool SendData_plc( U8* s_data,U16 len);
	bool ReadData_plc( U8* r_data,int &len,int except);
	void initDataPacket(int id);
	void setAngle( int id,F32 rad_angle );
	F32 getAngle( int id );
};







#endif /* HUIZHUAN_DRIVER_H_ */
