/*
 * Hawk_battery.h
 *
 *  Created on: Nov 21, 2018
 *      Author: dl
 */

#ifndef HAWK_BATTERY_H_
#define HAWK_BATTERY_H_

#include "battery/battery_driver.h"

#include <iostream>
#include <boost/thread/mutex.hpp>

#include "MyDefine.h"
#include "Comm.h"
class cTransferDevice;

class Hawk_battery : battery_driver{
public:
	Hawk_battery();
	~Hawk_battery();

public:
	bool Get_Connect_State();
	void disconnect();

	bool Init_battery_devices();
	bool Charging_Start();
	bool Charging_Stop();
    void Set_Battery_Para(std::string name,std::string para);
    void ReadData( U8* r_data,int &len,int need,int timeout);
    void SendData( U8* s_data,U16 len);

    void Get_Battery_Status(Battery_SATE &state);
    void Clear_State();

    bool Get_BMS_Battery();
    void Set_Charging_Station(U8 id);
    U8 Get_Charging_Station();
    U8 Get_Battery_ID() const;
    void Set_Charging_State(bool state);

private:
    U8 Find_Battery_State(U8 &state_);
    float Get_Battery_Soc(void);
    float Get_Battery_Voltage(void);
    U16 Get_Battery_Status(void);
    void Get_Battery_Fault(void);
    bool Get_Battery_Current_Status(void);
    void Battery_Fault(U16 &state_);
    bool Hawk_Charge_Start();
    bool Hawk_Charge_Stop();
    bool Turn_Off_Charge(void);
    bool Turn_On_Charge(void);
    void Set_Battery_State(int state);
private:
	bool connected;
	bool rs_485;
	std::string port_;
	U32 baudrate_battery;
	U8 battery_id;

	Battery_SATE battery_state;
	cTransferDevice *pTransDevice;

	U8 charging_station_id;
	boost::mutex mtx_battery;

	bool battery_charge_state;
	bool battery_close_state;
	S16 delay_cnt_;
	bool close_state;
};


#endif /* HAWK_BATTERY_H_ */
