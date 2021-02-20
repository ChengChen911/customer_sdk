#ifndef _CHASSIS_RUIJIE_DRIVER_H_
#define _CHASSIS_RUIJIE_DRIVER_H_

#include "driver_base.h"

class cTransferDevice;

class Ruijie_driver : Driver_base
{
public:
	Ruijie_driver();
	~Ruijie_driver();

protected:

private://virtual
	void setPara(std::string name ,std::string value);
	bool open_transfer_device();
	bool init_driver();
	void close_driver();

	void setRPM(int id,F32 v);
	F32 getSpeed(int id);
	F32 getDiffAngle(int id);

 	void setAngle(int id,F32 rad_angle);
 	F32 getAngle(int id);

private:
	void SendData( U8* s_data,U16 len );
	void ReadData( U8* r_data,int &len,int need );

	void Sendspeed();
	void Readspeedodmo();

private:
	std::string port_;
	cTransferDevice* pTransDevice_;
	F32 reduction_ratio_;
	U32 counts_;
	U32 one_wheel_counts_;

	F32 Vx_;
	F32 Vy_;
	F32 Vw_;
	F32 C_vx_;
	F32 C_vy_;
	F32 C_vw_;

	F32 X_;
	F32 Y_;
	F32 W_;
	F32 X_last_;
	F32 Y_last_;
	F32 W_last_;
	F32 X_dete_;
	F32 Y_dete_;
	F32 W_dete_;
	
	bool first_read_;

};



#endif//_CHASSIS_RUIJIE_DRIVER_H_
