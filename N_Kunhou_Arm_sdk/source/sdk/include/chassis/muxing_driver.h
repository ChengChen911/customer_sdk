#ifndef _MUXING_DRIVER_CHENG_H_
#define _MUXING_DRIVER_CHENG_H_

#include "TimerDiff.h"
#include "chassis/driver_base.h"

class cTransferDevice;

class muxing_driver : Driver_base
{

public:
	muxing_driver();
	~muxing_driver();

protected:

private://virtual
	void setPara(std::string name ,std::string value);

	bool init_driver();
	void close_driver();
	void setRPM(int id,F32 v);
	//void setAngle(int id,F32 rad_angle);
	bool open_transfer_device();

	F32 getSpeed(int id);
	F32 getDiffAngle(int id);

private:

	S16 Set_Left_Speed_;
	S16 Set_Right_Speed_;

	std::string port_;
	cTransferDevice* pTransDevice_;

	F32 reduction_ratio_;
	U32 counts_;
	U32 one_wheel_counts_;

	S16 C_Left_Speed_;
	S16 C_Right_Speed_;

	F32 Gyro_;

	S32 Left_dete_pos_;
	U32 Left_pos_;
	U32 Left_last_pos_;
	S32 Right_dete_pos_;
	U32 Right_pos_;
	U32 Right_last_pos_;

	U16 laser_;
	U8 laser_gears_;
	U8 laser_status_;

	cTimerDiff test_dt_;

public:
	typedef enum{IDLE=0,LEFT_MOTOR_LOST=1,RIGHT_MOTOR_LOST=2,ROTATE_MOTOR_LOST=3,LIFT_MOTOR_LOST=4,
				LEFT_MOTOR_OVERCUR=5,RIGHT_MOTOR_OVERCUR=6,ROTATE_MOTOR_OVERCUR=7,LIFT_MOTOR_OVERCUR=8,
				LEFT_ENC=9,RIGHT_ENC=10,ROTATE_ENC=11,LIFT_ENC=12,
				DRIVER_HOT=13,LOW_POWER=14,BATTERY_HOT=15,WIFI_LOST=16}_ERR_STATUS;
	_ERR_STATUS err_status_;

private:
	F32 GetDiffPos(int id );

	void Send_Read_data(U8* s_data,U16 s_len, U8* r_data,int &r_len,const int &need);
	void read_reg(U16 reg_s, U16 reg_num, U8* get_data);
	bool write_reg(U16 reg_s, U16 reg_num, U8* reg);

	void SendData( U8* s_data,U16 len);
	bool ReadData(  U8* r_data,int &len,const int &need);

	void Creat_Err_Code(U8 err_code, U8 err_para);

public:
	_ERR_STATUS getErrStatus();
	F32 GetGyroAngle();
	void GetLaserStutas();
	void Battery_para(U16 &vol, U16 &cur, U16 &per);
	void Set_RotateLift(bool lift, bool rotate);
	void Get_RotateLift(bool &lift, bool &rotate);
	void Set_laser_safe(U8 gears);
	U8 Get_laser_status();

};

#endif /* _MUXING_DRIVER_CHENG_H_ */
