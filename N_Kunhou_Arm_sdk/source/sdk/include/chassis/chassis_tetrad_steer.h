#ifndef _CHASSIS_TETRAD_STEER_
#define _CHASSIS_TETRAD_STEER_
#include "Geometry.h"

#include "chassis/chassis_base.h"
#include "chassis/driver_base.h"


class Chassis_tetrad_steer : Chassis_base
{
public:

	Chassis_tetrad_steer();
	~Chassis_tetrad_steer();

private://virtual
	//
	void setPara(std::string name , std::string value);
	bool initDriver(std::string para);

	//get odom in Base link (origin point is robot rotation center)
	void getOdomInBaseLink(F32 &dx,F32 &dy, F32 &th);
	void getOdomSpeed(F32 &vx,F32 &vy, F32 &th);

	void setOdomSpeed(F32 vx, F32 vy, F32 vw);

private:
	F32 Or_min_;
	F32 Odmo_;

	F32 O_a_;
	F32 Lx_, Ly_;
	F32 D1_, D2_, D3_, D4_, L1_, L2_, L3_,L4_;
	F32 wheel_dia_;

	F32 Y1_,Y2_,Y3_,Y4_;      //angle of sensor

	F32 vx_;
	F32 vy_;
	F32 vth_;

	F32 set_vx_;
	F32 set_vy_;
	F32 set_vth_;
	cTimerDiff dt_;

	VecPosition steer1_;
	VecPosition steer2_;
	VecPosition steer3_;
	VecPosition steer4_;

	F32 roation_angle1_;
	F32 roation_angle2_;
	F32 roation_angle3_;
	F32 roation_angle4_;
	F32 steer_max_angle_;

	F32 Set_Speed1;
	F32 Set_Speed2;
	F32 Set_Speed3;
	F32 Set_Speed4;
	F32 Set_Angle1;
	F32 Set_Angle2;
	F32 Set_Angle3;
	F32 Set_Angle4;

	void Control_unit_init();
	void resolution(F32 &speed1,F32 &speed2,F32 &speed3,F32 &speed4,F32 &angle1,F32 &angle2,F32 &angle3,F32 &angle4,const F32& Vx,const F32& Vy,const F32& W);

	F32 Speed2RPM( F32 speed );
	F32 RPM2Speed( F32 RPM );
	void Angle_access(F32 &angle, F32 &speed);


protected:
private:

};


#endif//












