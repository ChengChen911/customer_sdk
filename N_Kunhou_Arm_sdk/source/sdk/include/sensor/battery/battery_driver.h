/*
 * battery_driver.h
 *
 *  Created on: Nov 21, 2018
 *      Author: dl
 */

#ifndef BATTERY_DRIVER_H_
#define BATTERY_DRIVER_H_

#include "MyDefine.h"
#include <iostream>
#include <vector>

#include "Comm.h"

typedef struct Battery_SATE_
{
	U8 state;
	U16 err_code;
	float charge_electric;
	float discharge_electric;
	float precent;
	float temperature;
	float Voltage_total;
}Battery_SATE;


class battery_driver {
public:
	battery_driver();
	virtual ~battery_driver();

	virtual bool Get_Connect_State()=0;
	void Init_All_Para(std::string para);
	virtual void Set_Battery_Para(std::string name,std::string para)=0;

	virtual void SendData( U8* s_data,U16 len)=0;
	virtual void ReadData( U8* r_data,int &len,int need,int timeout)=0;

	virtual void disconnect()=0;
	virtual bool Charging_Start()=0;
	virtual bool Charging_Stop()=0;
	virtual void Clear_State()=0;
	virtual bool Get_BMS_Battery()=0;
	virtual U8 Get_Battery_ID() const=0;

	virtual bool Init_battery_devices()=0;
	virtual void Get_Battery_Status(Battery_SATE &state)=0;

	virtual void Set_Charging_Station(U8 id)=0;
	virtual U8 Get_Charging_Station()=0;

	virtual void Set_Charging_State(bool state)=0;

	virtual void Set_Battery_State(int state);

};

#endif /* BATTERY_DRIVER_H_ */
