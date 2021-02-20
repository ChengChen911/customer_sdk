#ifndef MENGLI_DRIVER_H_
#define MENGLI_DRIVER_H_

#include "driver_base.h"
#include "TimerDiff.h"
class cTransferDevice;

class mengli_driver : Driver_base
{
public:
	mengli_driver();
	~mengli_driver();

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
 	void SendPos(SPos amcl_pos);
	void SendData( U8* s_data,U16 len );
	void ReadData( U8* r_data,int &len,int need );
	void Data(F32 &current_number,U8 *b);
private:
	cTimerDiff dt_;
	std::string port_;
	cTransferDevice* pTransDevice_;
	F32 reduction_ratio_;
	U32 counts_;
	U32 one_wheel_counts_;
	bool first_do_;

	F32 set_speed1_;
	F32 set_speed2_;
	F32 set_angle1_;
	F32 set_angle2_;

	F32 cur_speed1_;
	F32 cur_speed2_;
	F32 cur_angle1_;
	F32 cur_angle2_;

	S32 cur_pos1_;
	S32 cur_pos2_;
	S32 last_pos1_;
	S32 last_pos2_;

	F32 dia_;

};



#endif /* MENGLI_DRIVER_H_ */
