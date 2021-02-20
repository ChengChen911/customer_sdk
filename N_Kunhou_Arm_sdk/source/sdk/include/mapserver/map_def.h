#ifndef _MAP_DEF_WANGHONGTAO_20151105_
#define _MAP_DEF_WANGHONGTAO_20151105_


#define AMCL_MAP "amcl_map"
#define GLOBAL_INF_MAP		 "global_inf_map"
//#define GLOBAL_COST_MAP	 "global_cost_map"
#define LASER_INF_MAP		 "laser_inf_map"
//#define LOCAL_COST_MAP		 "local_cost_map"
#define LOCAL_PATH_MAP		 "local_path_map"
#define GOAL_MAP			 "goal_map"
#define PROHIBIT_MAP		 "prohibit_map"
#define PLANNER_MAP			 "planner_map"

#define ODOM_LAYER			 "odom_layer"
#define WORLD_LAYER			 "world_layer"


#define GLOBAL_MAP_WIDTH 8192
#define GLOBAL_MAP_HEIGHT 8192

#define LOCAL_MAP_WIDTH 256
#define LOCAL_MAP_HEIGHT 256

#define GLOBAL_MAP_SIZE (GLOBAL_MAP_WIDTH*GLOBAL_MAP_HEIGHT)
#define LOCAL_MAP_SIZE  (LOCAL_MAP_WIDTH*LOCAL_MAP_HEIGHT)

//bind define should change both
#define MAP_RESOLUTION 0.05
#define F32_2_S16 20
//bind define should change both

#define GLOBAL_ORI_X (-MAP_RESOLUTION*GLOBAL_MAP_WIDTH/2)
#define GLOBAL_ORI_Y (-MAP_RESOLUTION*GLOBAL_MAP_HEIGHT/2)

#define LOCAL_ORI_X (-MAP_RESOLUTION*LOCAL_MAP_WIDTH/2)
#define LOCAL_ORI_Y (-MAP_RESOLUTION*LOCAL_MAP_HEIGHT/2)

#define MAP_OCCUPY_MAX 127
#define MAP_EMPTY		 0
#define MAP_UNKNOWN		-1
#define MAP_ERR			-128



#define ProhibitMap_BIT_REDUCE1 0
#define ProhibitMap_BIT_REDUCE2 1
#define ProhibitMap_BIT_STOP    2
#define ProhibitMap_BIT_PAUSE   6
#define ProhibitMap_BIT_SELF_PAUSE   3
#define ProhibitMap_BIT_COLLISION_STOP   4
#define ProhibitMap_BIT_MANUAL_PAUSE   5


#define CPL_BIT(value,bit) (value^=(1<<bit))   
#define SET_BIT(value,bit) (value|=(1<<bit))   
#define CLR_BIT(value,bit) (value&=~(1<<bit))  
#define GET_BIT(value,bit) (value&(1<<bit))>>bit    

#define SET_PAUSE(value,bit)    SET_BIT(value,bit)
#define SET_CONTINE(value,bit)  CLR_BIT(value,bit)

#define SET_STOP(value,bit)     SET_BIT(value,bit)
#define SET_REDU1(value,bit)    SET_BIT(value,bit)
#define SET_REDU2(value,bit)    SET_BIT(value,bit)


#endif//_MAP_DEF_WANGHONGTAO_20151105_
