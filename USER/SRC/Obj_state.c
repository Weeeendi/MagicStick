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
#include "MPU9250.h"
#include "Myqueue.h"


#define ACCEL_SONSOR 1
#define GYRO_SONSOR  2
#define YAW_SONSOR   3

#define ACTIVE_UP  120
#define ACTIVE_DOWN -120
#define ACTIVE_LEFT 1
#define ACTIVE_RIGHT 1

ObjS Magicstick_state;

const uint8_t have_extremum = 1;
const uint8_t no_extremum = 0;
/**
 * @brief find function extrememum
 * 
 * @param x 
 * @return uint8_t have_extermun or no_extermun
 */
uint8_t find_extremum(uint16_t x,Ex *extremum)
{
	static uint16_t buffer[3] = {0};
    //每次将x前插进入数组
    buffer[2] = buffer[1];
    buffer[1] = buffer[0];
    buffer[0] = x;

    if(buffer[1]>buffer[0]&&buffer[1]>buffer[2])
    {
        extremum->extern_ = local_maximum;
        extremum->value =  buffer[1];
        return have_extremum;
    }   
    else if(buffer[1]<buffer[0]&&buffer[1]<buffer[2])
    {
        extremum->extern_ = local_minimum;
        extremum->value =  buffer[1];
        return have_extremum;
    }
    
    return no_extremum;
}

/**
 * @brief 激发函数
 * 
 * @param Tempnum 
 * @return int 
 */
int active_fun(int Tempnum)
{
    if(Tempnum>=ACTIVE_UP)
        return 1;
    else if(Tempnum<=ACTIVE_DOWN)
        return -1;
    else 
        return 0;
}

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
  QueuePtr sensor_value_queue = (QueuePtr)malloc(sizeof(QNode));

  sensor_value_queue->data = 0;
  sensor_value_queue->next = NULL;
	
  static LinkQueue linkqueue;

	linkqueue.front = sensor_value_queue;
	linkqueue.rear = sensor_value_queue;
	linkqueue.len = 0;
	
	static uint8_t _cnt = 0;
	
	Enqueue(&linkqueue,&sensor_value);
	
	tempgx += sensor_value.x;
	tempgy += sensor_value.y;
	tempgz += sensor_value.z;
	_cnt++;
	
	if()
	//过去的20个点求平均
	if(_cnt==20) 
		_cnt--;
	average->x = tempgx/_cnt;
	average->y = tempgy/_cnt;
	average->z = tempgz/_cnt;
	
}

/**
 * @brief 判断设备向左移动
 * 
 * @param sensor_value 
 * @return true 
 * @return false 
 */
bool move_left_judge(uint16_t sensor_value)
{
    //Todo
}

/**
 * @brief 判断设备向右移动
 * 
 * @param sensor_value 
 * @return true 
 * @return false 
 */
bool move_right_judge(uint16_t sensor_value)
{
    //Todo
}

/**
 * @brief 判断设备向上移动
 * 
 * @param sensor_value 
 * @return true 
 * @return false 
 */
bool move_up_judge(uint16_t sensor_value)
{
    //Todo
}

/**
 * @brief 判断设备向上移动
 * 
 * @param sensor_value 
 * @return true 
 * @return false 
 */
bool move_down_judge(uint16_t sensor_value)
{
    //Todo
}



ObjS StateJudge_op(ObjS TempState,INT16_XYZ TempXYZ,uint8_t sensor)
{
	switch(sensor){
		case GYRO_SONSOR:
			//
		
		default:  ;
			// Todo 
	}	
	return TempState;
} 

