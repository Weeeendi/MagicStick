/**
 * @file main.c
 * @author w117 (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-02-14
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "include.h"
#include "mcu_api.h"
#include "MPU9250.h"
#include "Obj_state.h"
#include "Hal_Key.h"
#include "Timer.h"


GPIO_InitTypeDef GPIO_InitStructure;
ErrorStatus HSEStartUpStatus;

#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)		) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )

extern unsigned char TX_DATA[4];  	 //
extern unsigned char BUF[10];       //
extern char  test; 				 //IIC
	 
extern FusionVector3 gyroscopeSensitivity;
extern FusionVector3 accelerometerSensitivity;
extern FusionVector3 hardIronBias;


extern const uint8_t have_extremum;
extern const uint8_t no_extremum;

/* functions -----------------------------------------------*/
void RCC_Configuration(void);
void GPIO_Configuration(void);
void NVIC_Configuration(void);
void USART1_Configuration(void);
void WWDG_Configuration(void);
void Delay(u32 nTime);
void Delayms(vu32 m);  
/* fusion val ----------------------------------------------*/
FusionBias fusionBias;
FusionAhrs fusionAhrs;

uint8_t	data_to_send[50];

float samplePeriod = 0.1f; // replace this value with actual sample period in seconds
/*
********************************************************************************
**function			 : RCC_Configuration(void)
* Description    : RCC_Configuration
* Input          : None
* Output         : None
* Return         : None
********************************************************************************
*/
void RCC_Configuration(void)
{   
  /* RCC system reset(for debug purpose) */
  RCC_DeInit();

  /* Enable HSE */
  RCC_HSEConfig(RCC_HSE_ON);

  /* Wait till HSE is ready */
  HSEStartUpStatus = RCC_WaitForHSEStartUp();

  if(HSEStartUpStatus == SUCCESS)
  {
    /* HCLK = SYSCLK */
    RCC_HCLKConfig(RCC_SYSCLK_Div1); 
  
    /* PCLK2 = HCLK */
    RCC_PCLK2Config(RCC_HCLK_Div1); 

    /* PCLK1 = HCLK/2 */
    RCC_PCLK1Config(RCC_HCLK_Div2);

    /* Flash 2 wait state */
    FLASH_SetLatency(FLASH_Latency_2);
    /* Enable Prefetch Buffer */
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

    /* PLLCLK = 8MHz * 9 = 72 MHz */
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);

    /* Enable PLL */ 
    RCC_PLLCmd(ENABLE);

    /* Wait till PLL is ready */
    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
    {
    }

    /* Select PLL as system clock source */
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

    /* Wait till PLL is used as system clock source */
    while(RCC_GetSYSCLKSource() != 0x08)
    {
    }
  } 
   /* Enable GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF, GPIOG and AFIO clocks */
//  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB , ENABLE);
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD , ENABLE);
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE | RCC_APB2Periph_GPIOF , ENABLE);
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG | RCC_APB2Periph_AFIO  , ENABLE);  
}

/*
********************************************************************************
* function			 : GPIO_Configuration(void)
* Description    : GPIO_Configuration
* Input          : None
* Output         : None
* Return         : None
********************************************************************************
*/
void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE);
   /* Configure USART1 Tx (PA.09) as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;				 //
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		 // 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 // 0MHz
  GPIO_Init(GPIOA, &GPIO_InitStructure);				 // 
    
  /* Configure USART1 Rx (PA.10) as input floating */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;			  //
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	  //
  GPIO_Init(GPIOA, &GPIO_InitStructure);				  //

}


/*
********************************************************************************
* function			 : NVIC_Configuration(void)
* Description    : NVIC_Configuration
* Input          : None
* Output         : None
* Return         : None
********************************************************************************
*/
void NVIC_Configuration(void)
{ 
  NVIC_InitTypeDef NVIC_InitStructure;  
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0); 
 
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_Init(&NVIC_InitStructure);

}

 /*
********************************************************************************
* Function Name  : WWDG_Configuration(void)
* Description    : WWDG    Configuration
* Input          : None
* Output         : None
* Return         : None
********************************************************************************
*/
void WWDG_Configuration(void)
{
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_WWDG, ENABLE);	
  WWDG_SetPrescaler(WWDG_Prescaler_8);	              //  WWDG clock counter = (PCLK1/4096)/8 = 244 Hz (~4 ms)  
  WWDG_SetWindowValue(0x41);		                 // Set Window value to 0x41
  WWDG_Enable(0x50);		       // Enable WWDG and set counter value to 0x7F, WWDG timeout = ~4 ms * 64 = 262 ms 
  WWDG_ClearFlag();			       // Clear EWI flag
  WWDG_EnableIT();			       // Enable EW interrupt
}



/*
********************************************************************************
* Function Name  : WWDG_IRQHandler(void)
* Description    : WWDG
* Input          : None
* Output         : None
* Return         : None
********************************************************************************
*/ 

void WWDG_IRQHandler(void)
{
  /* Update WWDG counter */
  WWDG_SetCounter(0x50);
	
  /* Clear EWI flag */
  WWDG_ClearFlag(); 
}
 //************************************************
void  USART1_SendData(uchar SendData)
{
	USART_SendData(USART1, SendData);
	while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
}
//********MPU9250 Initialization************************
void Init_MPU9250(void)
{
/*
   Single_Write(GYRO_ADDRESS,PWR_M, 0x80);   //
   Single_Write(GYRO_ADDRESS,SMPL, 0x07);    //
   Single_Write(GYRO_ADDRESS,DLPF, 0x1E);    //��2000��
   Single_Write(GYRO_ADDRESS,INT_C, 0x00 );  //
   Single_Write(GYRO_ADDRESS,PWR_M, 0x00);   //
*/
  Single_Write(GYRO_ADDRESS,PWR_MGMT_1, 0x00);	//�������״̬
	Single_Write(GYRO_ADDRESS,SMPLRT_DIV, 0x07);
	Single_Write(GYRO_ADDRESS,CONFIG, 0x06);
	Single_Write(GYRO_ADDRESS,GYRO_CONFIG, 0x18);
	Single_Write(ACCEL_ADDRESS,ACCEL_CONFIG, 0x08);
	Single_Write(ACCEL_ADDRESS,ACCEL_CONFIG2, 0x06);
  //----------------
//	Single_Write(GYRO_ADDRESS,0x6A,0x00);//close Master Mode	

}
	
/**
 * @brief  串口发送一段数据
 * @param[in] {in} 发送数据指针
 * @return Null
 */
void u1_printf(char *fmt,...)
{
	char buffer[100];
	uint16_t i=0;
	va_list arg_ptr;
	va_start(arg_ptr,fmt);
	vsnprintf(buffer,100,fmt,arg_ptr);
	while(i<100&&buffer[i])
	{		
		USART_SendData(USART1,buffer[i]);
		while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
		i++;
	}
	va_end(arg_ptr);
}


void ANO_DT_Send_Data(uint8_t *buf, uint8_t _cnt){  
	uint16_t i=0;
	while(i<_cnt){
		USART_SendData(USART1,buf[i]);
		while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
		i++;
	}
}

/**
 * @brief 向匿名上位机发送9轴数据
 * 
 * @param a_x 
 * @param a_y 
 * @param a_z 
 * @param g_x 
 * @param g_y 
 * @param g_z 
 * @param m_x 
 * @param m_y 
 * @param m_z 
 */
void ANO_DT_Send_Senser(s16 a_x,s16 a_y,s16 a_z,s16 g_x,s16 g_y,s16 g_z,s16 m_x,s16 m_y,s16 m_z)
{
	u8 _cnt=0;
	vs16 _temp;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x02;
	data_to_send[_cnt++]=0;
	
	_temp = (s16)a_x;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (s16)a_y;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (s16)a_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = (s16)g_x;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (s16)g_y;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (s16)g_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = (s16)m_x;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (s16)m_y;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (s16)m_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}


/**
 * @brief 向匿名上位机发送姿态
 * 
 * @param eulerAngles 
 */
void ANO_DT_Send_Gesture(FusionEulerAngles eulerAngles)
{
 	uint8_t _cnt=0;		  //data_len
	uint16_t _temp=0;	  //两字节参数
	uint8_t checknum = 0; //校验和
	uint32_t high = 0;
	unsigned char param[20];
 	param[_cnt++] = 0xAA;
	param[_cnt++] = 0xAA;
 	param[_cnt++] = 0x01;
	param[_cnt++] = 0x06;

	_temp = (int)(eulerAngles.angle.roll*100);
	param[_cnt++]=BYTE1(_temp);
	param[_cnt++]=BYTE0(_temp);
	_temp = (int)(eulerAngles.angle.pitch*100);
	param[_cnt++]=BYTE1(_temp);
	param[_cnt++]=BYTE0(_temp);	
	_temp = (int)(eulerAngles.angle.yaw*100);
	param[_cnt++]=BYTE1(_temp);
	param[_cnt++]=BYTE0(_temp);
	
	param[_cnt++] = BYTE0(high);
	param[_cnt++] = BYTE1(high);
	param[_cnt++] = BYTE2(high);
	param[_cnt++] = BYTE3(high);
	param[_cnt++] = 0x00;
	param[_cnt++] = 0x01;
 	//param[9] = get_check_sum(param,9);
    param[3] = _cnt-4;
    
    for(uint8_t i=0;i<_cnt;i++){
		checknum+=param[i];
	}
	
	param[_cnt++] = checknum;

  ANO_DT_Send_Data(param,_cnt);
 // u1_printf("%s \r\n",param);
}

ObjS MagicStick_state;
/*
********************************************************************************
* Function Name  : main(void)
* Description    : 
* Input          : None
* Output         : None
* Return         : None
********************************************************************************
*/
int main(void)
{ 
	RCC_Configuration();		 //RCC
	USART1_Init(115200);
	USART3_Init(9600);	 //UART3
	//WWDG_Configuration();  //Witch dog
	I2C_GPIO_Config();		 //IIC
	Init_MPU9250();		     //MPU9250
	wifi_protocol_init();  //UART PROCESS
	
	TIM3_Count_Init(99,719);//1ms
	KEY_Init();
	
	
  // Initialise gyroscope bias correction algorithm
	FusionBiasInitialise(&fusionBias, 0.5f, samplePeriod); // stationary threshold = 0.5 degrees per second

	// Initialise AHRS algorithm
	FusionAhrsInitialise(&fusionAhrs, 0.5f); // gain = 0.5

	// Set optional magnetic field limits
	FusionAhrsSetMagneticField(&fusionAhrs, 0.0f, 4912.0f); // valid magnetic field range = 20 uT to 70 uT
		
	INT16_XYZ ACCEL_XYZ = {0};
	INT16_XYZ GYRO_XYZ = {0};
	INT16_XYZ MAG_XYZ = {0};

	//魔棒状态初始化
		MagicStick_state = init_Magicstick_state(MagicStick_state);
    while(1)
    {	
			wifi_uart_service();
			Key_Scan();
			READ_MPU9250_ACCEL(&ACCEL_XYZ);  //obtian Accel	 
			READ_MPU9250_GYRO(&GYRO_XYZ);   //obtian gyro 
			READ_MPU9250_MAG(&MAG_XYZ);	   //obtian MAG		
	
			// Calibrate gyroscope
			FusionVector3 uncalibratedGyroscope = {
				 .axis.x = GYRO_XYZ.x, /* replace this value with actual gyroscope x axis measurement in lsb */
				 .axis.y = GYRO_XYZ.y, /* replace this value with actual gyroscope y axis measurement in lsb */
				 .axis.z = GYRO_XYZ.z, /* replace this value with actual gyroscope z axis measurement in lsb */
			};
			FusionVector3 uncalibratedAccelerometer = {
				 .axis.x = ACCEL_XYZ.x, /* replace this value with actual accelerometer x axis measurement in lsb */
				 .axis.y = ACCEL_XYZ.y, /* replace this value with actual accelerometer y axis measurement in lsb */
				 .axis.z = ACCEL_XYZ.z, /* replace this value with actual accelerometer z axis measurement in lsb */
			};
			FusionVector3 uncalibratedMagnetometer = {
				.axis.x = MAG_XYZ.x, /* replace this value with actual magnetometer x axis measurement in uT */
				.axis.y = MAG_XYZ.y, /* replace this value with actual magnetometer y axis measurement in uT */
				.axis.z = MAG_XYZ.z, /* replace this value with actual magnetometer z axis measurement in uT */
			};
			
			//u1_printf("GyroscopeX = %0.1f, \tGyroscopeY = %0.1f,\t GyroscopeZ = %0.1f  \r\n", uncalibratedGyroscope.axis.x, uncalibratedGyroscope.axis.y, uncalibratedGyroscope.axis.z);
			//u1_printf("AccelerometerX = %0.1f, \tAccelerometerY = %0.1f,\t AccelerometerZ = %0.1f \r\n", uncalibratedAccelerometer.axis.x, uncalibratedAccelerometer.axis.y, uncalibratedAccelerometer.axis.z);
			//u1_printf("%0.1f\t %0.1f\t %0.1f \r\n", uncalibratedMagnetometer.axis.x, uncalibratedMagnetometer.axis.y,uncalibratedMagnetometer.axis.z);	 
	 
			FusionVector3 calibratedGyroscope = FusionCalibrationInertial(uncalibratedGyroscope, FUSION_ROTATION_MATRIX_IDENTITY, gyroscopeSensitivity, FUSION_VECTOR3_ZERO);
			FusionVector3 calibratedMagnetometer = FusionCalibrationMagnetic(uncalibratedMagnetometer, FUSION_ROTATION_MATRIX_IDENTITY, hardIronBias);
			FusionVector3 calibratedAccelerometer = FusionCalibrationInertial(uncalibratedAccelerometer, FUSION_ROTATION_MATRIX_IDENTITY, accelerometerSensitivity, FUSION_VECTOR3_ZERO);
			
		  //ANO_DT_Send_Senser(ACCEL_XYZ.x,ACCEL_XYZ.y,ACCEL_XYZ.z,GYRO_XYZ.x,GYRO_XYZ.y,GYRO_XYZ.z,MAG_XYZ.x,MAG_XYZ.y,MAG_XYZ.z);
			// Update gyroscope bias correction algorithm
			calibratedGyroscope = FusionBiasUpdate(&fusionBias, calibratedGyroscope);

			// Update AHRS algorithm
			FusionAhrsUpdate(&fusionAhrs, calibratedGyroscope, calibratedAccelerometer, calibratedMagnetometer, samplePeriod);

			// Print Euler angles
			FusionEulerAngles eulerAngles = FusionQuaternionToEulerAngles(FusionAhrsGetQuaternion(&fusionAhrs));

     	//u1_printf("Roll = %0.1f,\tPitch = %0.1f,\t Yaw = %0.1f\r\n", eulerAngles.angle.roll, eulerAngles.angle.pitch, eulerAngles.angle.yaw);	
			//u1_printf("%0.1f \t%0.1f\t %0.1f\r\n", eulerAngles.angle.roll, eulerAngles.angle.pitch, eulerAngles.angle.yaw);	

			switch(StateJudge_op(MagicStick_state,GYRO_XYZ,2).MoveDirection){
				case MoveUp:
					u1_printf("Move direction is up\n");
				  break;
				case MoveDown:
					u1_printf("Move direction is down\n");
				  break;
				case MoveLeft:
					u1_printf("Move direction is left\n");
				  break;
				case MoveRight:
					u1_printf("Move direction is right\n");
				  break;
				default:
					u1_printf("No Move!!!\n");
					break;
			}
			
			//ANO_DT_Send_Gesture(eulerAngles);
     }
}

/*************END***************/
