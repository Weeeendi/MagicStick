/**
 * @file Obj_state.c
 * @author your name (you@domain.com)
 * @brief 物体状态机
 * @version 0.1
 * @date 2022-02-25
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "stdio.h"
#include "stdbool.h"
#include "Obj_state.h"
#include "Myqueue.h"


#define ACCEL_SONSOR 1
#define GYRO_SONSOR  2
#define YAW_SONSOR   3

#define ACTIVE_UP  120
#define ACTIVE_DOWN -120
#define ACTIVE_LEFT 1
#define ACTIVE_RIGHT 1


const uint8_t have_extremum = 1;
const uint8_t no_extremum = 0;

/**
 * @brief find function extrememum
 * 
 * @param x 
 * @return uint8_t have_extermun or no_extermun
 */
uint8_t find_extremum(int16_t x,Ex *extremum)
{
    //每次将x前插进入数组
    extremum->buffer[2] = extremum->buffer[1];
    extremum->buffer[1] = extremum->buffer[0];
    extremum->buffer[0] = x;

    if((extremum->buffer[1]>extremum->buffer[0])&&(extremum->buffer[1]>extremum->buffer[2]))
    {
        extremum->extern_ = local_maximum;
        return have_extremum;
    }   
    else if((extremum->buffer[1]<extremum->buffer[0])&&(extremum->buffer[1]<extremum->buffer[2]))
    {
        extremum->extern_ = local_minimum;
        return have_extremum;
    }
    extremum->extern_ = nothing;
    return no_extremum;
}

///**
// * @brief 激发函数
// * 
// * @param Tempnum 
// * @return int 
// */
//static int active_fun(int Tempnum)
//{
//    if(Tempnum>=ACTIVE_UP)
//        return 1;
//    else if(Tempnum<=ACTIVE_DOWN)
//        return -1;
//    else 
//        return 0;
//}

/**
 * @brief initialize Magicstick state
 * 
 * @param TempState 
 */
ObjS init_Magicstick_state(ObjS TempState){
    TempState.actived = 0;
    TempState.MoveDirection = noMove;
    return TempState;
}

ObjS Change_Magicstick_state(ObjS TempState,enum DefaultDirection Dir,bool CurAct)
{
    TempState.actived = CurAct;
    TempState.MoveDirection = Dir; 
    return TempState;
}

/**
 * @brief 求平均值
 * 
 * @param sensor_value 
 * @param average 
 * @return uint16_t 
 */
uint16_t average_value(INT16_XYZ sensor_value,INT16_XYZ *average)
{
	static int32_t tempgx=0,tempgy=0,tempgz=0;
	static LinkQueue linkqueue;
	static uint8_t _cnt = 0;
	//create a new node
  QueuePtr sensor_value_queue = (QueuePtr)malloc(sizeof(QNode));

  sensor_value_queue->data = 0;
  sensor_value_queue->next = NULL;
	
	//init pointer numbers
	if(_cnt==0){
	linkqueue.front = sensor_value_queue;
	linkqueue.rear = sensor_value_queue;
	linkqueue.len = 0;
	}
	
	Enqueue(&linkqueue,sensor_value);
	
	tempgx += sensor_value.x;
	tempgy += sensor_value.y;
	tempgz += sensor_value.z;
	_cnt++;
	
	if(_cnt == 20) {		
	//过去的_cnt个点求平均,删除首节点
	INT16_XYZ TempXYZ;
	DeQueue(&linkqueue,&TempXYZ);
	tempgx -=  TempXYZ.x;
	tempgy -=  TempXYZ.y;
  tempgx -=  TempXYZ.z;
  _cnt--;		
	}
	average->x = tempgx/_cnt;
	average->y = tempgy/_cnt;
	average->z = tempgz/_cnt;
	
	return 0;
}

/**
 * @brief 判断设备向左移动
 * 
 * @param sensor_value 
 * @return true 
 * @return false 
 */
bool move_left_judge(int16_t sensor_value)
{	
    //Todo
	uint8_t ret = 0;
	static Ex left_About = {nothing,0,0,0};
	ret = find_extremum(sensor_value,&left_About);
	
	//GYZ极大值且值大于100
	if((ret == have_extremum)&&(left_About.extern_ == local_maximum)&&(left_About.buffer[1]>100))
		return true;
	return false;
}

/**
 * @brief 判断设备向右移动
 * 
 * @param sensor_value 
 * @return true 
 * @return false 
 */
bool move_right_judge(int16_t sensor_value)
{
	    //Todo
	uint8_t ret = 0;
	static Ex right_About = {nothing,0,0,0};
	ret = find_extremum(sensor_value,&right_About);
	
	//GYZ极大值且值大于150
	if((ret == have_extremum)&&(right_About.extern_ == local_minimum)&&(right_About.buffer[1]<(-120)))
		return true;

	return false;
	
	//Todo
}

/**
 * @brief 判断设备向上移动
 * 
 * @param sensor_value 
 * @return true 
 * @return false 
 */
bool move_up_judge(int16_t sensor_value)
{
    //Todo
	uint8_t ret = 0;
	static Ex  up_About = {nothing,0,0,0};
	ret = find_extremum(sensor_value,&up_About);
	
	//极大值且值大于700
	if((ret == have_extremum)&&(up_About.extern_ == local_maximum)&&(up_About.buffer[1]>200))
		return true;
	
	return false;
}

/**
 * @brief 判断设备向上移动
 * 
 * @param sensor_value 
 * @return true 
 * @return false 
 */
bool move_down_judge(int16_t sensor_value)
{
	 //Todo
	uint8_t ret = 0;
	static Ex  down_About = {nothing,0,0,0};
	ret = find_extremum(sensor_value,&down_About);
	
	//极大值且值大于700
	if((ret == have_extremum)&&(down_About.extern_ == local_minimum)&&(down_About.buffer[1]<(-250)))
		return true;
	return false;
}



ObjS StateJudge_op(ObjS TempState,INT16_XYZ Temp_XYZ,uint8_t sensor)
{
	switch(sensor){
		case GYRO_SONSOR:
			//
			if(move_up_judge(Temp_XYZ.x)) TempState.MoveDirection =  MoveUp;
		  else if(move_down_judge(Temp_XYZ.x)) TempState.MoveDirection =  MoveDown;
		  else if(move_left_judge(Temp_XYZ.z)) TempState.MoveDirection =  MoveLeft;
		  else if(move_right_judge(Temp_XYZ.z)) TempState.MoveDirection =  MoveRight;
		  else TempState.MoveDirection =  noMove;
		default:  ;
			// Todo 
	}	
	return TempState;
} 

