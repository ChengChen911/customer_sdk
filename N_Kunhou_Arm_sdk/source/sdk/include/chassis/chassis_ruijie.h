#ifndef _CHASSIS_RUIJIE_H_
#define _CHASSIS_RUIJIE_H_

#include "chassis_base.h"

class Chassis_ruijie : Chassis_base
{
public:

	Chassis_ruijie();
	~Chassis_ruijie();

private://virtual
	//
	void setPara(std::string name , std::string value);
	bool initDriver(std::string para);

	//get odom in Base link (origin point is robot rotation center)
	void getOdomInBaseLink(F32 &dx,F32 &dy, F32 &th);
	void getOdomSpeed(F32 &vx,F32 &vy, F32 &th);

	void setOdomSpeed(F32 vx, F32 vy, F32 vw);

	

private:

	F32 wheel_dia_;	
	F32 axle_;		

protected:
private:
};



#endif//_CHASSIS_RUIJIE_H_
