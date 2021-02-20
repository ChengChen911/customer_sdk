/*
 * Interlayer_battery.h
 *
 *  Created on: Jun 28, 2020
 *      Author: SudekiMing
 */

#ifndef  INTERLAYER_BATTERY_H_
#define INTERLAYER_BATTERY_H_

#include "battery/battery_driver.h"

#include <iostream>
#include <boost/thread/mutex.hpp>

#include "MyDefine.h"

class cTransferDevice;

class Interlayer_battery : battery_driver
{
public:
	Interlayer_battery();
	virtual ~Interlayer_battery();

public:
	bool Get_Connect_State(void);
	void disconnect(void);
	bool Init_battery_devices(void);
	void Set_Battery_Para(std::string name,std::string para);

	void SendData( U8* s_data,U16 len);
	void ReadData( U8* r_data,int &len,int need,int timeout);

	bool Charging_Start(void);
	bool Charging_Stop(void);
	bool Charging_Clear(void);

    void Get_Battery_Status(Battery_SATE &state);
    void Clear_State();
    U8 Get_Battery_ID() const;

    void Set_Charging_Station(U8 id);
    U8 Get_Charging_Station(void);

    bool Get_BMS_Battery(void);

private:
    void Battery_FaultLog(U16 &fault_code);
    void Set_Battery_WorkInfo(U8 battery_type,U8 battery_id,U8 charge_id);
    void Get_Battery_RunStatus(U8 &state);
    void Get_Battery_ParamInfo(F32 &voltage,F32 &precent,F32 &charge_electric,F32 &discharge_electric,F32 &temperature,U16 &err_code);

    void Emergency_Connect(void);
    void Emergency_disConnect(void);
    void Set_Charging_State(bool state);
    void Set_Battery_State(int state);

private:
	bool connected;
	bool rs_485;
	std::string port_;
	U32 baudrate_battery;
	U8 battery_type;
	U8 battery_id;
	U8 charging_station_id;

	Battery_SATE battery_state;
	cTransferDevice *pTransDevice;

	boost::mutex mtx_battery;

	bool battery_charge_state;

	bool V24_;

};

#endif /* INTERLAYER_BATTERY_H_ */
