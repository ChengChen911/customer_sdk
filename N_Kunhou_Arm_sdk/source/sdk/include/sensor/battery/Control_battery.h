/*
 * Control_battery.h
 *
 *  Created on: Feb 24, 2018
 *      Author: dl
 */

#ifndef CONTROL_BATTERY_H_
#define CONTROL_BATTERY_H_

#include "MyDefine.h"
#include <string>
#include <boost/thread/mutex.hpp>

class cTransferDevice;

typedef struct SBattery_
{
	float precent;
	U8 state;
	S16 Voltage_max;
	S16 Voltage_min;
	float Voltage_total;
}SBattery;


class Control_battery {
public:
	Control_battery();
	~Control_battery();

public:
	void Connected(void);
	void SendData( U8* s_data,U16 len);
	void ReadData( U8* r_data,int &len,int need,int timeout);
	bool isConnected();
	void disconnect();
	void Charging_Start(void);
	void Charging_Stop(void);
	void AGV_Status(void);
	void Voltage_Total(void);
	void Run_Status(void);
	void BMS_Status(void);
	void Get_Status(SBattery &state);
	void Emergency_Connect(void);
	void Emergency_disConnect(void);
	void Clear(void);
private:
	bool connected;
	std::string port_;
	boost::mutex mu_battery_;
	SBattery m_battery;
	cTransferDevice* pTransDevice;

};

#endif /* CONTROL_BATTERY_H_ */
