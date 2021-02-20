#ifndef _ROBOT_STRUCT_WANGHONGTAO_20150831_
#define _ROBOT_STRUCT_WANGHONGTAO_20150831_

#include "KHTypes.h"
#include <vector>
#include <map>

#define LASER_COUNT 541
#define FPGA_LASER_BIN_CNT 512

#define FPGA_LASER_BIN_CNT_256 256
#define FPGA_LASER_BIN_CNT_512 512

#define LASER_DIFF ((LASER_COUNT - FPGA_LASER_BIN_CNT) / 2)


#define LASER_COUNT_URG 1081

typedef enum{
	IDLE = 0,
	PLAN = 1,
	MOVE = 2,
	RECOVERY = 3,
	ERR = 4
}Status;

typedef enum{
	FRONT = 1,
	BACK = 2,
	LEFT = 3,
	RIGHT = 4
}Direction;

typedef enum
{
	dynamic = 1,
	fix = 2
}MODEL_TYPE;

typedef enum{
	NONE_ERR = 0,
	GLOBAL_PLANNER_ERR = 1,
	LOCAL_PLANNER_ERR = 2,
	OBSTACLE_DETECT = 3,
	AMCL_ERR = 4,
	OUT_OF_LINE_ERR = 5,
	STOPPING_ERR = 6,
	OBSTACLE_STOP = 7,
	COMMAND_STOP = 8,
	COLLISION_STOP = 9
}ERR_CODE;

typedef enum{
	DIFF2WD = 0,
	FORK_STEER = 1,
	FORK_DIFF = 2,
	SINGLE = 4,
	MULIT = 8,
	OMNI = 16,
	TYPE1 = 32,
	TYPE2 = 64
}Motion_type;

typedef struct _SSerialNum
{

	U32 serial_num;
	U32 motion_type;
	U32 d1;
	U32 d2;
	U8 main_v;
	U8 sub_v;
	U8 c1;
	U8 c2;
}SSerialNum;

typedef struct _SGridXY
{
	U16 x_;
	U16 y_;
	S8 value_;

	S8 dir_;
	S8 speed_;
	S8 ptype_;

}SGridXY;

typedef struct _SLaser
{
	U32 stamp_;
	F32 start_angle_;
	F32 resolution_;	//
	F32 range_max_;
	F32 range_min_;

	F32 data_[LASER_COUNT];
	U8  seg_[LASER_COUNT];
	U32 used_;

} SLaser;

typedef struct _SLaser_para
{
	std::string laser_ip_;
	bool reverse_;
	F32 laser_start_angle_;
	F32 laser_range_max_;
	F32 laser_range_min_;
	F32 laser_dx_;
	F32 laser_dy_;
	F32 laser_resolution_;
} SLaser_para;

typedef struct _Shape_xy
{
	F32 x1_;
	F32 y1_;
	F32 x2_;
	F32 y2_;
	F32 x3_;
	F32 y3_;
	F32 x4_;
	F32 y4_;
}Shape_xy;

typedef struct _SLaserXY
{

	F32 x_[LASER_COUNT];
	F32 y_[LASER_COUNT];
	U8 used_[LASER_COUNT];

} SLaserXY;

typedef struct _SPos
{
	F32 x_;
	F32 y_;
	F32 th_;

}SPos;

typedef struct _SSpeed
{
	F32 vx_;	//speed vx_
	F32 vy_;	//speed vy_
	F32 vw_;	//speed vw_

}SSpeed;

typedef struct _SOdomSpeed
{
	F32 x_;
	F32 y_;
	F32 th_;

	F32 vx_;	//speed vx_
	F32 vy_;	//speed vy_
	F32 vw_;	//speed vw_

}SOdomSpeed;

typedef struct _Sxy
{
	F32 x_;
	F32 y_;

}Sxy;

typedef struct _S16xy
{
	S16 x_;
	S16 y_;

}S16xy;

typedef struct _SLaser_xy
{
	Sxy xy_[LASER_COUNT];
	U32 used_size_;

}SLaser_xy;


typedef struct _SPair_Odo_Laser
{

	SPos pos_;
	SLaser laser_;

}SPair_Odo_Laser;

typedef struct _SInit_Pos
{
	SPos pos_;

	U32 psize_;

	F32 para1_;
	F32 para2_;
	F32 para3_;
	F32 para4_;

}SInit_Pos;

typedef struct _STarget_Pos
{
	U8 model_;
	U8  stop_;
	U16 tmp2_;

	F32 x_;
	F32 y_;
	F32 th_;

}STarget_Pos;

typedef struct _SRunPath
{
	U8 replan_;
	U8 backward_;
	U8 tmp1_;
	U8 tmp2_;
	char path_name_[200];

}SRunPath;

typedef struct _SAMCL_RES
{

	F32 entropy_;
	F32 hz_;
	S32 stamp_;

}SAMCL_RES;

typedef struct _SparticleFAW
{

	F32 x;
	F32 y;
	F32 th;
	F32 aw;

}SparticleFAW;

typedef struct _SparticlePub
{
	U32 pwsize_;
	SparticleFAW data_w_[8192];

}SparticlePub;

typedef struct _Srecoder
{

	char cmd_file_[200];

}Srecoder;

typedef struct _SRunStatus
{

	Status status_;
	U8 err_code_;
	U8 arrived_;
	U8 back_;

	F32 sx_;
	F32 sy_;
	F32 sw_;

	F32 cx_;
	F32 cy_;
	F32 cw_;

}SRunStatus;

#define MAX_DI 24
#define MAX_DO 24
#define MAX_AI 8
#define MAX_AO 8

typedef struct _SDIAI
{
	U8 di_[MAX_DI];
	S32 ai_[MAX_AI];

}SDIAI;

typedef struct _SDOAO
{
	U8 do_[MAX_DO];
	S32 ao_[MAX_AO];

}SDOAO;

typedef struct _SDO
{
	U32 used_;
	U8 id_[MAX_DO];
	U8 do_[MAX_DO];

}SDO;

typedef struct _SAO
{
	U32 used_;
	U8 id_[MAX_AO];
	S32 ao_[MAX_AO];

}SAO;

typedef struct _SDI
{
	U32 used_;
	U8 id_[MAX_DI];
	S32 di_[MAX_DI];

}SDI;

typedef struct _SSub_task_upload
{
	U32 agv_id_;

	U32 sub_task_index_;
	U32 prority_;
	int current_step_;
	int max_step_;

	std::vector<SGridXY> run_path_;

}SSub_task_upload;

typedef struct _SSub_task_path
{
	U32 agv_id_;

	U32 sub_task_index_;
	U32 prority_;

	std::vector<SGridXY> run_path_;

}SSub_task_path;

typedef struct _SSub_task_path_aux
{
	U32 agv_id_;

	U32 sub_task_index_;
	U32 prority_;
	U32 max_step_;
}SSub_task_path_aux;


typedef struct _SSub_task_fnc
{
	U32 agv_id_;

	U32 sub_task_index_;
	U32 prority_;
	U32 function_code_; // 0 fork down 1 fork up  3 pos

	int current_step_;
	int max_step_;

}SSub_task_fnc;

typedef struct _SFork_state
{
	U32 fnc_code_;
	S32 para_;               //0 - 10000    mm
	S32 current_status_;     //0:action 1:finished

}SFork_state;


typedef struct _SControl
{
	U32 agv_id_;
	F32 max_speed_;
	int pause_run_slow;

}SControl;

#define THREAD_COUNTS   10

typedef struct _Health_Status
{
	F32 frequency[THREAD_COUNTS] ;     //unit :  Hz
	int error_code[THREAD_COUNTS];     //flag,  0:not work   1:low   2: normal
}Health_Status;


typedef struct _MapProcess
{
	U8 processbar_type_;            //  1: mapping  , 2: set_amclmap, 3: set_globalmap
	S32 counts_;
	S32 total_;
}MapProcess;

typedef struct _SLaserSafe_Info
{
	U8 laser_id_;
	U8 check_result_;
	U8 active_frame_id_;
	SLaser laser_scan_;

}SLaserSafe_Info;

typedef struct _SLaserSafe_Frames
{
	SLaser frame_[32];
	F32 laser_dx_;
	F32 laser_dy_;
	U8  laser_id_;
}SLaserSafe_Frames;

typedef struct _SBattery_level
{
	U32 agv_id_;
	U32 agv_type_;
	F32 percents_;
}SBattery_level;

typedef struct _SBattery_status
{
	U32 agv_id_;
	U32 agv_type_;
	U16 status_;//0:idle 1:charging 2:charge finish 3:fault 4:行程到位
	U16 err_code_;
	F32 charge_electric_;//A
	F32 discharge_electric_;//A
	F32 soc_percents_;//%
	F32 temperature_;//degree centigrade
	F32 voltage_;//V
}SBattery_status;

typedef struct _SGyro_Data
{
	F32 x_;
	F32 y_;
	F32 z_;
	F32 roll_;
	F32 yaw_;
	F32 pitch_;

}SGyro_Data;

typedef struct _SRION_Gyro_Data
{
	F32 angle_;
	F32 angle_v_;
	F32 advance_acc_;
}SRION_Gyro_Data;

typedef struct _SDynamic_Speed
{
	U8 id_;			   //id stand for one kind of speed limit type
	U16 control_time_;  //10 means speed limit continue in 10 seconds
	U8 back_;

	F32 vx_;
	F32 vy_;
	F32 vw_;
	

}SDynamic_Speed;

typedef struct _GyroAngle
{
	F32 angle_;


}GyroAngle;

typedef struct _QRCode
{
	S32 qrcode_;


}QRCode;

typedef struct _SLaser_Landmark
{
    U8 numbers;
    F32 Position_x[LASER_COUNT];
    F32 Position_y[LASER_COUNT];
    F32 orient[LASER_COUNT];
    S32 ID[LASER_COUNT];
} SLaser_L;

typedef struct _SExtra_Pos    //外加的定位值
{
	U8 is_active_;  //是否生效
	SPos pos_;

}SExtra_Pos;

typedef struct _Struck_Para
{
	SPos pos_;
	F32 truck_length_;
	F32 truck_width_;

}Struck_Para;

typedef enum{
	TASK_NEW = 1,
	TASK_DOING = 2,
	TASK_DONE = 3,
	TASK_CANCEL =4,
	TASK_ERROR = 5,
	TASK_REJECT = 6,
	TASK_WAITDONE = 7,
	TASK_ABORT = 8,
	TASK_FAILURE = 9
}Task_State;

typedef struct _SFnc_Task_Download
{
	int sub_index_;
	int fnc_code_;
	U8 parameter_num_;
	char paras_[500];
	char check_list_[500];

}SFnc_Task_Download;

typedef struct _SFnc_Task_Upload
{
	int sub_index_;
	int fnc_code_;
	Task_State task_state_;
	int error_code_;
	char feedback_[500];
}SFnc_Task_Upload;

typedef struct _SAmcl_Confidence_Level
{
	bool  match_;
	F32 confidence_level_;
	F32 match_threshold_;
	U8 location_mode_;
}SAmcl_Confidence_Level;

typedef struct _SFnc_Action_Download
{
	int sub_index_;
	int fnc_code_;
	char paras_[500];
	char check_list_[500];
}SFnc_Action_Download;

typedef struct _SFnc_Action_Upload
{
	int sub_index_;
	int fnc_code_;
	Task_State task_state_;
	int error_code_;
	char feedback_[500];
}SFnc_Action_Upload;

typedef struct _SAgv_State_Data
{
	char state_data_[500];

}SAgv_State_Data;

typedef struct _SAgv_Error_Data
{
	char error_data_[500];
}SAgv_Error_Data;

typedef struct _SFork_Status
{
	U8  id_;
	F32 fork_x_;
	F32 fork_y_;
	F32 fork_height_;
	U8  fork_state_;    //1.forward  2.back
}SFork_Status;

typedef struct _SMatched_Landmark
{
    U8 numbers;
    F32 Position_x[LASER_COUNT];
    F32 Position_y[LASER_COUNT];
    F32 orient[LASER_COUNT];
    S32 ID[LASER_COUNT];
    S32 State[LASER_COUNT];
} SMatched_Landmark;

typedef struct _ScanData {
	U32 stamp_;
	F32 start_angle_;
	F32 resolution_;
	U32 beam_num_;
	F32 h_distance[8400];
	S32 h_echo[8400];
    F32 h_angle[8400];

} ScanData;

#define CODE_MAX_LEN 100
typedef struct _SMaterialCode {

	U8 code_id_;
	U8 code_len_;
    char material_code_[CODE_MAX_LEN];
	U8 detect_success_;   // 0 : no scandata;1:success ; 2: code error
} SMaterialCode;

typedef struct _ForkLiftStatus {
	U8 ControlMode_;//0:auto 1:manual
	U8 Scram_;//0:no stop 1:stop
	U8 NoCanData_;//0:normal 1:nodata
	U8 ForkLiftErr_;//0:normal 1:reset
} ForkLiftStatus;


typedef struct _SKHTimeStamp{
	int yy_;
	int mm_;
	int dd_;
	int hour_;
	int min_;
	int sec_;
	int milsec; //ms
}KHTimeStamp;

typedef struct _SProcess_Report
{
	S8 process_name_[100];
	S8 description[200];
	KHTimeStamp time_;
} SProcess_Report;

typedef struct _SError_Report
{
	int error_code_;
	S8 description[200];
	KHTimeStamp time_;
} SError_Report;

/*dynamic_area*/
typedef struct _Sdynamic_area_transmit
{
	S32 area_id_;
	U8 enable_code_[32];
	Sxy input_[8];
	Sxy output_[8];
} Sdynamic_area_transmit;

typedef struct _Sdynamic_area_set//dataserver to dynamic_area
{
	Sdynamic_area_transmit  area_set_[128];
} Sdynamic_area_set;

typedef struct _Sdynamic_area//interprocess_dynamic_area
{
	S32 area_id_;
	U8 enable_code_[32];
	std::map<int, Sxy> input_;
	std::map<int, Sxy> output_;
} Sdynamic_area;

typedef struct _Sdynamic_area_taskcode//dataserver to dynamic_area
{
	S32 task_id_;
	U8 task_code_[32];
	U8 paras_[256];
} Sdynamic_area_taskcode;

typedef struct _Sdynamic_area_fnc//dynamic_area to others
{
	U8 function_code_;
	U8 paras_[256];
} dynamic_area_fnc;

typedef struct _Sdynamic_area_feedback//dynamic_area to others
{
	S32 active_area_id_;
	U8 enable_code_[32];
	U8 task_code_[32];
	U8 active_code_[32];
} dynamic_area_feedback;
/*dynamic_area*/

typedef struct _SActive_tasks
{
	int total_live_;
	SFnc_Task_Upload active_tasks_[16];
} SActive_tasks;


#endif//_ROBOT_STRUCT_WANGHONGTAO_20150831_

