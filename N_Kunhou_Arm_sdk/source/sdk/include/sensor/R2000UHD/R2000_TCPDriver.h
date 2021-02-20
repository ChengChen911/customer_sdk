/*
 * R2000_TCPDriver.cpp
 *
 *  Created on: Oct 30, 2020
 *      Author: neo
 */

#ifndef R2000_TCPDRIVER_H_
#define R2000_TCPDRIVER_H_

#include <string>

#include <boost/thread.hpp>


#include "MyDefine.h"
#include "R2000UHD/DataConvert.h"
#include "R2000UHD/SendCommand.h"
#include "buffer_con.hpp"
#include <mutex>
#include <condition_variable>
#include <deque>
#include <array>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/circular_buffer.hpp>
#include "R2000Driver/packet_structure.h"



typedef struct _scanData2 {

	F32 h_distance[25200];
	S32 h_echo[25200];
    F32 h_angle[25200];

} scanData2;


typedef struct _headData2{
    U8 magic[2];
    U16 packet_type;
    U32 packet_size;
    U16 header_size;
    U16 scan_number;
    U16 packet_number;
    U64 timestamp_raw;
    U64 timestamp_sync;
    U32 status_flags;
    U32 scan_frequency;
    U16 num_points_scan;
    U16 num_points_packet;
    U16 first_index;
    S32 first_angle;
    S32 angular_increment;
    double angular_increment_real;
    U32 output_status;
    U32 field_status;
}headData2;



class R2000_TCPDriver {
public:
	R2000_TCPDriver();
	virtual ~R2000_TCPDriver();

	void disconnect();


	bool isConnected();

	void Laser_Init(const std::string ip);

	void startMeas(const std::string ip);


	void stopMeas();

	bool getData(scanData2& data);

	void getLdata(float* ldata,scanData2& data,const int laser_lanmark_R2000_);
	void getBLdata(float* ldata,scanData2& data,const int laser_lanmark_R2000_);
	void getRAll_data(float* ldata,scanData2& data);
	void getRAll_echo(S32* ldata,scanData2& data);
	void getRAll_angle(float* ldata,scanData2& data);

	bool isReadStatus();
	void Set_Laser_Para(const F32 start_angle_,const int laser_lanmark_R2000_,const int laser_R2000_echo_);
	void getEchodata(S32* ldata,scanData2& data,const int laser_lanmark_R2000_);
	void getBEchodata(S32* ldata,scanData2& data,const int laser_lanmark_R2000_);

	void th_Stop(void);
	void ReConnect_Thread();
	void Feed_Watchdog();
	bool isBreakConnect();
//	void Head_Package_Size(void);
	void fetch_data(scanData2&);
	void recoder_laser_data(const scanData2&  scan,std::string fi_name);

	std::size_t getFullScansAvailable() const;
	pepperl_fuchs::ScanData getScan();
private:
	void connect(std::string host, int port = 2111);
	void Laser_Init_Para(void);
	void run_(void);
	void ReConnect_laser();

//	bool Scan_Num_Check(U16 current_num);
	bool Scan_Index_Check(U16 current_index);
	bool Package_Charge(headData2 &head_,U8 *buffer,scanData2& data);
	void Set_Connect(bool state_);


private:
    bool handleNextPacket();

    //! Search for magic header bytes in the internal ring buffer
    //! @returns Position of possible packet start, which normally should be zero
    int findPacketStart();

    //! Try to read a packet from the internal ring buffer
    //! @returns True on success, False otherwise
    bool retrievePacket( std::size_t start, pepperl_fuchs::PacketTypeB* p );
    void readBufferFront(char* dst, std::size_t numbytes );

    //! Write fast at the back of the internal ring buffer
    //! @param src Source buffer
    //! @numbytes Number of bytes to write
    void writeBufferBack(char* src, std::size_t numbytes );
    //! Buffer in case of UDP receiver
    std::array< char, 65536 > udp_buffer_;

    //! Internal ringbuffer for temporarily storing reveived data
    boost::circular_buffer<char> ring_buffer_;

    //! Protection against data races between ROS and IO threads
    std::mutex data_mutex_;

    //! Data notification condition variable
    std::condition_variable data_notifier_;

    //! Double ended queue with sucessfully received and parsed data, organized as single complete scans
    std::deque<pepperl_fuchs::ScanData> scan_data_dq;
    int scanvalid_range_cnt;
private:
	THSafe<bool> connected;
	//bool connected;
	bool debug;
	int sockDesc;

	headData2 m_headData;
	DataConvert m_DataConvert;
	SendCommand m_sendmand;

	S16 m_ReadCount;
	THSafe<bool> IsReadDate;
	bool IsScanRead;
	//bool IsReadDate;
	std::string  laser_ip_;
	int  laser_landmark_exist;
	int  laser_echo_exist;
	S16 receive_cout;

	std::string laser_para_;
	std::string lstart_angle_;
	std::string pack_type;
private:
	bool thread_run_;
	boost::thread *th_;
	int conect_ret;
	U16 feed_cout=0;

	U16 last_scan_index;
	U16 scan_first_index_count;
	bool last_index_check;

	U16 laser_back_cout;

	U32 head_size_cout;
	THSafe<bool> head_break_exit;
	bool laser_head_check;

	THSafe<bool> handle_ready_;
	scanData2 scan_data_;
	bool write_log_;
};

#endif    /* R2000_TCPDriver_H_ */

