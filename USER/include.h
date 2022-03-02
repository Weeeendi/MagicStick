#ifndef _INCLUDE_H_
#define _INCLUDE_H_

#include	<stdio.h>
#include	<math.h>
#include	<stdlib.h>
#include	<string.h>
#include  <stdarg.h>
//#include 	<intrins.h>
#include	<ctype.h>
#include "stm32f10x.h"
#include "stm32f10x_lib.h"

#define         ENABLE_BOOT                             //ʹ��BOOT(�̼���������)
//(7+1K)+28K+28K
#define         BASIC_FLASH_ADDR                        0x08000000

#ifdef ENABLE_BOOT
  #define         OFFSET_PARA                             0x1c00        
#else
  #define         OFFSET_PARA                             0xfc00         
#endif

#define         OFFSET_FIRMWARE_L                       0x2000                  //�̼��������е�ַ
#define         OFFSET_FIRMWARE_H                       0x9000                  //�����̼��洢��ַ

#define         PARA_ADDR                               (BASIC_FLASH_ADDR + OFFSET_PARA)
#define         FIREWARE_ADDR_L                         (BASIC_FLASH_ADDR + OFFSET_FIRMWARE_L)   //
#define         FIREWARE_ADDR_H                         (BASIC_FLASH_ADDR + OFFSET_FIRMWARE_H)   //

#define         FIREWARE_UPDATE_FLAG                           0x55555555

//#include "hal_key.h"
//#include "hal_rgb.h"

#include "Uart.h"
//#include "user_timer.h"
//#include "user_flash.h"

typedef struct {
  uint32_t magic_code;
  uint8_t led_switch;
  uint8_t work_mode;
  uint8_t bright_value;
  uint8_t colour[14];
  uint8_t scene[14];
} TYPE_BUFFER_S;

#endif
