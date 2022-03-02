/**
 * @file Obj_state.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-02-25
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef OBJ_STATE_H
#define OBJ_STATE_H

#include "include.h"

enum DefaultDirection{
    MoveUp = 0,
    MoveDown,
    MoveLeft,
    MoveRight,
    noMove         
};


typedef enum Whichextremum{
        local_minimum,
        local_maximum
}WhichEx;
/**
 * @brief 极值性质加值
 * 
 */
typedef struct extremum
{
    WhichEx extern_;
    uint16_t value;
    /* data */
}Ex;


/**
 * @brief Object state
 * 
 * @arg actived 0：no active
 */
typedef struct Obj_State{
    unsigned char MoveDirection;
    unsigned char actived; 
}ObjS;

#endif

