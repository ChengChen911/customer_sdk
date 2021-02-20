#ifndef KINCO_CAN_DRIVER_H_
#define KINCO_CAN_DRIVER_H_

#include "TimerDiff.h"
#include "chassis/driver_base.h"
#include "chassis/canalyst2.h"

class cTransferDevice;

class kinco_can_driver: Driver_base
{
public:
	kinco_can_driver();
	~kinco_can_driver();

protected:

private:

	std::string port_;
	//SerialPort* sport_;
	cTransferDevice* pTransDevice_;
	F32 reduction_ratio_;
	U32 counts_;

	U32 one_wheel_counts_;


private:

	S32 l1_pos_;
	S32 r1_pos_;
	S32 l2_pos_;
	S32 r2_pos_;

	U32 COBID;
	U8 CANmsg_send[16];
	U8 CANmsg_receive[1024];

private://virtual customized :TODO
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

	void init_customer_para();

	F32 GetDiffPos(int id);


	void GetSpeedLR();
	void SendData( U8* s_data,U16 len );
	void ReadData(   U8* r_data,int &len,const int &need );
	cTimerDiff dt_;

	void Canopen_NMT_setOP ();
    void CANopen_Send_SDO ( U8 OP, U8 Index_low, U8 Index_high , U8 SubIndex , U8 data1);
    void CANopen_Send_PDO ( S32 data);
    void CANopen_readPos ( );

	bool b_run_;

	bool first_odo_;

	S32 left_pos_;
	S32 last_left_pos_;
	S32 deta_left_pos_;

	S32 right_pos_ ;		    //pos
	S32 last_right_pos_;
	S32 deta_right_pos_;

	F32 c_left_;
	F32 c_right_;

	boost::mutex l1_lock_;
	boost::mutex r1_lock_;
	boost::mutex l2;
	boost::mutex r2;


private://canalyst2
	canalyst2 canalyst2_;

};




#endif /* KINCO_CAN_DRIVER_H_ */
