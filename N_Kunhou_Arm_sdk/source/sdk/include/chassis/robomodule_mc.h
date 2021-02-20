#ifndef _ROBO_DRIVER_MC_CHENG_H_
#define _ROBO_DRIVER_MC_CHENG_H_

#include "TimerDiff.h"
#include "chassis/driver_base.h"
#include "linux_driver/gpio.h"

class cTransferDevice;

class robomodule_mc : Driver_base
{

public:
	robomodule_mc();
	~robomodule_mc();

protected:

private://virtual
	void setPara(std::string name ,std::string value);

	bool init_driver();
	void close_driver();
	void setRPM(int id,F32 v);
	//void setAngle(int id,F32 rad_angle);
	F32 GetDiffPos(int id );
	void setAngle(int id,F32 rad_angle);
	F32 getAngle( int id );
	bool open_transfer_device();

	F32 getSpeed(int id);
	F32 getDiffAngle(int id);
private:
	typedef struct
	{
		bool rec_speed;
		bool rec_ctl;
	}rec_;

	gpio gpio_dio_ ;
	int scram_;

	std::string port_;
	std::string port1_;
	std::string port2_;
	std::string port3_;
	cTransferDevice* pTransDevice_;
	cTransferDevice* pTransDevice1_;
	cTransferDevice* pTransDevice2_;
	cTransferDevice* pTransDevice3_;

	F32 reduction_ratio_;
	U32 counts_;
	U32 one_wheel_counts_;

	S16 temp_pwm_;

	typedef enum{HEAD_3,KEY1_3,KEY2_3,CUR0_3,CUR1_3,SPEED0_3,SPEED1_3,POS0_3,POS1_3,POS2_3,CRC3} State_speed_pos;
	typedef struct
	{
		State_speed_pos sta_speed_pos_;

		S16 Set_Speed_;
		S16 C_Speed_buf_;
		F32 C_Speed_;
		S32 pos_buf_;
		S32 pos_;
		S32 last_pos_;
		S32 deta_pos_;

		U8 speed_data_[2];
		U8 speed_pos_data_[4];

		boost::mutex mu_speed_;
		boost::mutex mu_pos_;
	}Wheel_;
	Wheel_ wheel_[4];

private:
	bool th_get_speed_pos_run_;

private:
	bool open_ttyS2_device();
	bool open_ttyS3_device();
	bool open_ttyUSB0_device();
	bool open_ttyUSB1_device();
	void Send_Driver(int id, U8* s_data,U16 len);
	void Read_Driver(int id, U8* r_data,int &len,const int &need);
	void SendData( U8* s_data,U16 len,cTransferDevice* pTransDevice);
	void ReadData(  U8* r_data,int &len,const int &need,cTransferDevice* pTransDevice);

	void th_Get_Speed_pos(void);
	void state_mathine_speed_pos(int id, Wheel_ &wheel, U8 data);

	bool Reset_driver(int id);
	bool Mode_driver(int id);
	bool Receive_Message(int id, rec_ rec_msg, U8 timeout);
	void Send_Speed(int id, S16 Speed, S16 PWM_max);

	void init_customer_para();

};

#endif /* _ROBO_DRIVER_CHENG_H_ */
