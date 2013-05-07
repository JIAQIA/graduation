/**
  ******************************************************************************
  * @file    Audio_playback_and_record/src/stm32f4xx_it.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    28-October-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
#include "stm32f4_discovery.h"
#include "keyboard.h"
#include "key_code_def.h"
#include "auto_ctrl_def.h"
#include "inter_polate_def.h"
//#include "data_deal_def.h"

/** @addtogroup STM32F4-Discovery_Audio_Player_Recorder
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
__IO uint8_t PauseResumeStatus = 2, Count = 0, LED_Toggle = 0;
uint16_t capture = 0;
uint16_t capture2 = 0;
//uint16_t Keyboard_Change_Flag = 1;
extern __IO uint16_t CCR_Val;
//extern __IO uint8_t RepeatState, AudioPlayStart;
//extern uint8_t Buffer[];

#if defined MEDIA_USB_KEY
__IO uint16_t Time_Rec_Base = 0;
 extern USB_OTG_CORE_HANDLE          USB_OTG_Core;
 extern USBH_HOST                    USB_Host;
 extern FIL file;
 extern __IO uint8_t Data_Status;
 extern __IO uint32_t XferCplt ;
 extern __IO uint8_t Command_index;
 
#endif /* MEDIA_USB_KEY */
//存放当前水标需要显示的位置与当前字符需要显示的位置，数组的两个坐标为PhotoNum.x & PhotoNum.y
uint16_t Cursor_X_Location[7][5] = {{0,0},{0x9B,0xA0,0xA0,0xA0},{0xD4,0xD4,0xD4},{0xC0,0xCC,0xD8,0xC5},{0x94,0xBC},{0xCF,0xCF,0xCF},{0,0}};
uint16_t String_X_Location[7][5] = {{0,0},{158,158,158,158},{0xD2,0xD2,0xD2},{0xC0,0xC0,0xC0,0xC3},{0x90,0x90},{0x7F,0x7F,0x7F},{0,0}};
uint16_t Cursor_Y_Location[7][5] = {{0,0},{0x41,0x81,0xAD,0xD6},{0x5C,0x86,0xB1},{0x88,0x88,0x88,0xC9},{0x92,0x92},{0x46,0x6F,0x9A},{0,0}};
uint16_t String_Y_Location[7][5] = {{0,0},{65,107,149,0xBE},{0x5C,0x86,0xB1},{0x70,0x70,0x70,0xB1},{0x7A,0x7A,},{0x46,0x6F,0x9A},{0,0}};
PhotoNum_Init PhotoNum;
//__IO uint16_t CCR1_Val,CCR2_Val,CCR3_Val,CCR4_Val;
unsigned char i = 0;
//unsigned char  Cnt[4]; //一个数组，这个数组的每个元素对应一个通道，用来判断装PWM得高电平还是低电平数
//unsigned int  T[4];//周期数组
//unsigned int  R[4];//模拟的比较寄存器数组，一样的每个通道对应一个数组元素
//unsigned int  Rh[4];//模拟的PWM高电平比较寄存器
//unsigned int  Rl[4]; //模拟的PWM低电平比较寄存器
//unsigned char F[4];//占空比数组 
/*********************************************************************
函数名称：PWM_Output
功    能：按照上一个插补周期所算出的输出频率与是否发射激光的标志位输出
参    数：无
返回值  ：无
**********************************************************************/
void PWM_Output()
{
  int Period[2]; 
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  TIM_TimeBaseInitTypeDef TIM_BaseInitStructure;
  TIM_OCInitTypeDef TIM_OCInitStructure;
  //TIM_InternalClockConfig(TIM5);
  /* TIM5 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
//  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
//  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);
  /* GPIOA clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB, ENABLE);
  
  /* GPIOA Configuration: TIM3 CH1 (PC6) and TIM3 CH2 (PC7) */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOA, &GPIO_InitStructure); 
  /* Connect TIM5 pins to AF2 */  
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM5);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOB, &GPIO_InitStructure); 
  /* Connect TIM5 pins to AF2 */  
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_TIM4);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOE, &GPIO_InitStructure); 
  /* Connect TIM5 pins to AF2 */  
  //GPIO_PinAFConfig(GPIOE, GPIO_PinSource3, GPIO_AF_TIM5);

//       
//       for(i = 1; i < 4; i++)
//       {
//              Cnt[i]= 0;
//              T[i]  = 0;
//              R[i]  = 0;
//              Rh[i] = 0;
//              Rl[i] = 0;
//              F[i]  = 0;
//       }    
//       //t的范围为（0~65536）    
//       //T[0] = 450;        //F=40K
//       T[1] = pulse_out[0];        //F=30K
//       T[2] = pulse_out[1];        //F=20K
//       T[3] = 1800;   //F=10K
//       //F(占空比)的范围为（0~100）
//       //F[0] = 40;
//       F[1] = 50;	//控制步进电机的占空比任意值，此处取50%
//       F[2] = 50;
//       F[3] = 50;    
//       for(i = 1; i < 4; i++)
//       {
//              Rh[i] = (T[i] * F[i]) / 100;
//              Rl[i] = T[i] - Rh[i];
//       }    
//       //R[0] = Rl[0];
//       R[1] = Rl[1];
//       R[2] = Rl[2];
//       R[3] = Rl[3];
//      
//       //CCR1_Val = R[0];
//       CCR2_Val = R[1];
//       CCR3_Val = R[2];
//       CCR4_Val = R[3];
  if(axis_dir[0] == SET_POS)
    GPIO_SetBits(GPIOE,GPIO_Pin_7); //GPIOE_PIN_3接X向步进电机控制器方向控制引脚
  else
    GPIO_ResetBits(GPIOE,GPIO_Pin_7);
  if(axis_dir[1]  == SET_POS)
    GPIO_SetBits(GPIOE,GPIO_Pin_8); //GPIOE_PIN_4接Y向步进电机控制器方向控制引脚
  else
    GPIO_ResetBits(GPIOE,GPIO_Pin_8);
  if(OpenLaser == 1)
    GPIO_SetBits(GPIOE,GPIO_Pin_9); //GPIOE_PIN_5接激光电源控制器，置位则开激光
  else
    GPIO_ResetBits(GPIOE,GPIO_Pin_9);
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
 
  NVIC_InitStructure.NVIC_IRQChannel=TIM5_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority=1;
  NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
 
  NVIC_Init(&NVIC_InitStructure);     
  
  Period[0] = (int)(33600/pulse_out[0]) - 1;
  Period[1] = (int)(33600/pulse_out[1]) - 1;
  TIM_BaseInitStructure.TIM_Prescaler=9;//10分频
  TIM_BaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up;
  TIM_BaseInitStructure.TIM_Period = Period[0];
  TIM_BaseInitStructure.TIM_ClockDivision=0;
  TIM_BaseInitStructure.TIM_RepetitionCounter=0;
  TIM_TimeBaseInit(TIM5,&TIM_BaseInitStructure);

  TIM_BaseInitStructure.TIM_Period = Period[1];
  TIM_TimeBaseInit(TIM4,&TIM_BaseInitStructure);
  
  TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_Pulse=(int)(Period[0] / 2);
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	   //选择输出比较状态
  TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable; //选择互补输出比较状态
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //设置互补输出极性
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set; //选择空闲状态下得非工作状态
  TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset; //选择互补空闲状态下的非工作状态
  TIM_OC2Init(TIM5,&TIM_OCInitStructure);
  TIM_OC2PreloadConfig(TIM5,TIM_OCPreload_Disable); //关闭影子寄存器，使得在改变CCR寄存器后可以及时的响应。
  TIM_Cmd(TIM5,ENABLE);
  
//  TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_Toggle;
//  TIM_OCInitStructure.TIM_Pulse=CCR3_Val;
//  TIM_OC3Init(TIM5,&TIM_OCInitStructure);
//  TIM_OC3PreloadConfig(TIM5,TIM_OCPreload_Disable);
  TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_Pulse=(int)(Period[1] / 2);
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	   //选择输出比较状态
  TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable; //选择互补输出比较状态
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //设置互补输出极性
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set; //选择空闲状态下得非工作状态
  TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset; //选择互补空闲状态下的非工作状态
  TIM_OC2Init(TIM4,&TIM_OCInitStructure);
  TIM_OC2PreloadConfig(TIM4,TIM_OCPreload_Disable); //关闭影子寄存器，使得在改变CCR寄存器后可以及时的响应。
  TIM_Cmd(TIM4,ENABLE);
//  TIM_ITConfig(TIM5,TIM_IT_Update,ENABLE);
//  TIM_ITConfig(TIM5,TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4,ENABLE);
    
}
/****************************************************************
函数名称: Dis_Cursor
功    能: 显示光标,Horizontal横向显示,vertical纵向显示
参    数: 无
返回值  : 无
*****************************************************************/
void Dis_Cursor_Horizontal()
{
  Tx_Start();
  Tx_Send(0x44);
  Tx_Send(0x01);
  Tx_Send(0x00);
  Tx_Send(Cursor_X_Location[PhotoNum.x][PhotoNum.y]);
  Tx_Send(0x00);
  Tx_Send(Cursor_Y_Location[PhotoNum.x][PhotoNum.y]);
  Tx_Send(0x1F);
  Tx_Send(0x03);
  Tx_End();
}
void Dis_Cursor_Vertical()
{
  Tx_Start();
  Tx_Send(0x44);
  Tx_Send(0x01);
  Tx_Send(0x00);
  Tx_Send(Cursor_X_Location[PhotoNum.x][PhotoNum.y]);
  Tx_Send(0x00);
  Tx_Send(Cursor_Y_Location[PhotoNum.x][PhotoNum.y]);
  Tx_Send(0x03);
  Tx_Send(0x18);
  Tx_End();
}
/****************************************************************
函数名称: Tx_Start
功    能: 串口1发送起始数据
参    数: 无
返回值  : 无
*****************************************************************/
void Tx_Start()
{
  USART_SendData(USART1,0xAA);
  while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET); 
}
/****************************************************************
函数名称: Tx_Send
功    能: 串口一发送数据
参    数: 待发送的数据
返回值  : 无
*****************************************************************/
void Tx_Send(uint16_t data)
{

	    USART_SendData(USART1,data);
		while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET); 
	
}

/****************************************************************
函数名称: Tx_End
功    能: 串口1发送CC 33 C3 3C数据,使显示器终止接收
参    数: 无
返回值  : 无
*****************************************************************/
void Tx_End()
{
  uint16_t End_Buff[] = {0xCC,0x33,0xC3,0x3C};
  uint8_t i = 0;   // value of loop
  for(;i<4;i++)
  {
    USART_SendData(USART1,End_Buff[i]);
	while (USART_GetFlagStatus(USART1,USART_FLAG_TC) == RESET);
  }
}
/*发送一个字符串*/

void SendStr_Uart(u8* data)
{
	uint8_t *str = data;
	Tx_Start();
	Tx_Send(0x6F);
	Tx_Send(0x00);
	Tx_Send(String_X_Location[PhotoNum.x][PhotoNum.y]);
	Tx_Send(0x00);
	Tx_Send(String_Y_Location[PhotoNum.x][PhotoNum.y]);

	for(;*str != '\0' ;)
	{
		if(USART_GetITStatus(USART1, USART_IT_TXE) == RESET)
		{
	    USART_SendData(USART1,*str++);
		while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);				  
		} 		
  
	}
	Tx_End();
}	
//发送一个数据
void SendNum_Uart(uint16_t Number)			
{
  uint16_t a,b,c;
  a = Number/100;
  b = (Number - a*100)/10;
  c = Number - a*100 - b*10;
  Tx_Start();
  Tx_Send(0x6F);
  Tx_Send(0x00);
  Tx_Send(String_X_Location[PhotoNum.x][PhotoNum.y]);
  Tx_Send(0x00);
  Tx_Send(String_Y_Location[PhotoNum.x][PhotoNum.y]);
  
  Tx_Send(a + 0x30);
  
  Tx_Send(b + 0x30);
  
  Tx_Send(c + 0x30);
  Tx_End();													  
}
/* 函数名称：SendFloat_Uart()  ****************
*  发送一个浮点数，三位整数部分，两位小数部分*/
void SendFloat_Uart(float Number)
{
  uint16_t a,b,c,d,e;
  a = (uint16_t)(Number/100);
  b = (uint16_t)((Number - a*100)/10);
  c = (uint16_t)(Number - a*100 - b*10);
  d = (uint16_t)((Number - a*100 - b*10)*10);
  e = (uint16_t)((Number - a*100 - b*10)*100 - d*10);
  Tx_Start();
  Tx_Send(0x6F);
  Tx_Send(0x00);
  Tx_Send(String_X_Location[PhotoNum.x][PhotoNum.y]);
  Tx_Send(0x00);
  Tx_Send(String_Y_Location[PhotoNum.x][PhotoNum.y]);
  Tx_Send(a + 0x30);
  Tx_Send(b + 0x30);
  Tx_Send(c + 0x30);
  Tx_Send(0x2E);
  Tx_Send(d + 0x30);
  Tx_Send(e + 0x30);
  Tx_End();		
}
///****************************************************************
//函数名称: Delay			   在Rwfile.c中有定义
//功    能: 在扫描按键时，用于延时消抖
//参    数: nCount -- 延时长度
//返回值  : 无
//*****************************************************************/
//
//void Delay(vu32 nCount)
//{
//  for(; nCount!= 0;nCount--);
//}
/****************************************************************
函数名称：Display_Config
功    能：配置显示器显示第PhotoNum.x页
参    数：PhotoNum.x
返回值  ：无
****************************************************************/
void Display_Config(uint8_t Num)
{
  char String_1[] = "user.txt";
  //uint16_t Speed = user_para.speed_scal;		5-14
  char *SendBuff;
  char String_2[] = "设置";
  switch (Num)
  {
    case 0:
	  break;
	case 1:
	  //Tx_End();
	  Tx_Start();
	  Tx_Send(0x70);
	  Tx_Send(0x01);
	  Tx_End();
	  Delay(1);
	  ManuNum = 1;
      PhotoNum.x = 1;
	  PhotoNum.y = 0;
	  SendBuff = String_1;
      Dis_Cursor_Vertical();
      SendStr_Uart(SendBuff);
      PhotoNum.y++;
      SendNum_Uart(user_para.speed_scal);
      PhotoNum.y++;
      SendNum_Uart(user_para.spindle_scal);
      PhotoNum.y++;
      SendNum_Uart(ManuNum);
	  //PhotoNum.x = 1;
	  PhotoNum.y = 0;
	  break;
	case 2:
	  Tx_Start();
	  Tx_Send(0x70);
	  Tx_Send(0x02);
	  Tx_End();
	  PhotoNum.x = 2;
	  PhotoNum.y = 0;
	  SendBuff = String_2;
	  Dis_Cursor_Horizontal();
	  SendStr_Uart(SendBuff);
      PhotoNum.y++;
	  SendStr_Uart(SendBuff);
      PhotoNum.y++;
	  SendStr_Uart(SendBuff);
      //PhotoNum.x = 2;
	  PhotoNum.y = 0;
	  break;
	case 3:
	  Tx_Start();
	  Tx_Send(0x70);
	  Tx_Send(0x03);
	  Tx_End();
	  PhotoNum.x = 3;
	  PhotoNum.y = 0;
	  SpotsprayMs = 0;
	  SpotsprayPower = 50;
	  Dis_Cursor_Vertical();
	  SendNum_Uart(SpotsprayMs);
	  PhotoNum.y++;
	  SendNum_Uart(SpotsprayPower);
	  //PhotoNum.x = 3;
	  PhotoNum.y = 0;
	  break;
	case 4:
	  Tx_Start();
	  Tx_Send(0x70);
	  Tx_Send(0x04);
	  Tx_End();
	  PhotoNum.x = 4;
	  PhotoNum.y = 0;
	  SpotsprayMm = 0.0;
	  SendFloat_Uart(SpotsprayMm);
	  Dis_Cursor_Vertical();
	  break;
	case 5:
	  Tx_Start();
	  Tx_Send(0x70);
	  Tx_Send(0x05);
	  Tx_End();
	  PhotoNum.x = 5;
	  PhotoNum.y = 0;
	  SendFloat_Uart(user_para.feed_vel);
	  PhotoNum.y++;
	  SendFloat_Uart(user_para.spindle_rev);
	  PhotoNum.y++;
	  SendFloat_Uart(user_para.swift_vel);
	  PhotoNum.y = 0;
	  break;
  }	  
}
/****************************************************************
函数名称: Palette_Cfg
功    能: 调色板的配置
参    数: 无
返回值  : 无
*****************************************************************/
void Palette_Cfg()
{
  Tx_Start();
  Tx_Send(0x40);
  Tx_Send(0x00);
  Tx_Send(0x00);
  Tx_Send(0xFF);
  Tx_Send(0xFF);
  Tx_End();
}
/**
  * @brief 实现键盘的读入
  * @param None
  */

uint16_t Key_Read()
{
   uint16_t Keyboard_Val = 0;
   //unsigned long tmpFlag=0 ;  //保存需要的中断标志位
   unsigned char i = 0 ;  //循环变量
   //tmpFlag = EXTI->PR & Keyboard_EXTI_Line ;  //只取设定过的标志位
   STM_EVAL_LEDOn(LED3);
   //EXTI->PR = tmpFlag ;
//   if(Keyboar_Change_Flag == 0)
//   {
//     if(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_6) == Bit_SET ||\
//	           GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_7) == Bit_SET || \
//	           GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_8) == Bit_SET )
//     {
//	   Keyboard_Change_Flag = 1;
//	   return Keyboard_Val;
//	 } 
//   }
   if(GPIO_ReadInputDataBit(Keyboard_Control_Port_2,Keyboard_Row_1) == Bit_RESET)
   {
     GPIO_ResetBits(Keyboard_Control_Port ,Keyboard_Line) ;
     for(i=0 ;i<4 ;i++)
     {
        GPIO_SetBits(Keyboard_Control_Port ,(Keyboard_LineBase<<i)) ;
        if(GPIO_ReadInputDataBit(Keyboard_Control_Port_2 ,Keyboard_Row_1))
        {
          Delay(DELAY_COUNT) ;  //延时消抖
          if(GPIO_ReadInputDataBit(Keyboard_Control_Port_2 ,Keyboard_Row_1))
          {
              Keyboard_Val = 1+i ;
              //Keyboard_Change_Flag = 1 ;
              break ;
          }
        }
     }
	 GPIO_ResetBits(Keyboard_Control_Port ,Keyboard_Line) ;
//	 if(GPIO_ReadInputDataBit(Keyboard_Control_Port_2,Keyboard_Row_1) == Bit_RESET)
//	   Keyboard_Change_Flag = 0;
//	 else
//	   Keyboard_Change_Flag = 1;
     
   }else if(GPIO_ReadInputDataBit(Keyboard_Control_Port_2,Keyboard_Row_2) == Bit_RESET)
   {
     GPIO_ResetBits(Keyboard_Control_Port ,Keyboard_Line) ;
     for(i=0 ;i<4 ;i++)
     {
        GPIO_SetBits(Keyboard_Control_Port ,(Keyboard_LineBase<<i)) ;
        if(GPIO_ReadInputDataBit(Keyboard_Control_Port_2 ,Keyboard_Row_2))
        {
          Delay(DELAY_COUNT) ;  //延时消抖
          if(GPIO_ReadInputDataBit(Keyboard_Control_Port_2 ,Keyboard_Row_2))
          {
              Keyboard_Val = 5+i ;
              //Keyboard_Change_Flag = 1 ;
              break ;
          }
        }
     }
	 GPIO_ResetBits(Keyboard_Control_Port ,Keyboard_Line) ;
//	 if(GPIO_ReadInputDataBit(Keyboard_Control_Port_2,Keyboard_Row_2) == Bit_RESET)
//	   Keyboard_Change_Flag = 0;
     
   }else if(GPIO_ReadInputDataBit(Keyboard_Control_Port_2,Keyboard_Row_3) == Bit_RESET)
   {
     GPIO_ResetBits(Keyboard_Control_Port ,Keyboard_Line) ;
     for(i=0 ;i<4 ;i++)
     {
        GPIO_SetBits(Keyboard_Control_Port ,(Keyboard_LineBase<<i)) ;
        if(GPIO_ReadInputDataBit(Keyboard_Control_Port_2 ,Keyboard_Row_3))
        {
          Delay(DELAY_COUNT) ;  //延时消抖
          if(GPIO_ReadInputDataBit(Keyboard_Control_Port_2 ,Keyboard_Row_3))
          {
              Keyboard_Val = 9+i ;
              //Keyboard_Change_Flag = 1 ;
              break ;
          }
        }
	  }
	  GPIO_ResetBits(Keyboard_Control_Port ,Keyboard_Line) ;
//	  if(GPIO_ReadInputDataBit(Keyboard_Control_Port_2,Keyboard_Row_3) == Bit_RESET)
//	    Keyboard_Change_Flag = 0;
	  	 
   }
   return Keyboard_Val;
}

/**
  * @brief 针对不同的键盘输入进行不同的响应，PhotoNum.x是第X张图，PhotoNum.y是第X图中第Y个变量
  * @param KeyNum
  */
void Key_Process(uint16_t KeyNum)
{
  float OrignCoordinates[] = {0,0,0};
  switch (KeyNum)
  {
    case ResetKey:	//运动到坐标原点,再停车
	  
	      appoint_position(OrignCoordinates,0x01);
		  code_seg_pre_treat();
		  while(1)
		  {
	   	    if((interpl_segment_prev->basecmd == PROGRAM_IDLE) && (interpl_segment_curr->basecmd == PROGRAM_IDLE)) 
             //检测任务完成             
             {
               appoint_position(OrignCoordinates,0x02);
			   Delay(1); 
			   while(1)
		      {
	   	        if((interpl_segment_prev->basecmd == PROGRAM_IDLE) && (interpl_segment_curr->basecmd == PROGRAM_IDLE)) 
                //检测任务完成
				{
				  if(mech_status.stat == STA_WORKING)
                  {
	                ctrl_flag &= (~BLEND_PERMIT); //blend_permit = ST_FAULSE;
		            mech_status.stat = STA_STOPPING;
		          }
                  if(mech_status.stat == STA_PAUSED)  mech_status.stat = STA_STOPPED;
	              if(mech_status.stat == STA_FINISHED)  mech_status.stat = STA_STOPPED;
				  break; 
				}          
              }
			 }
		  }
		  
	case SpotsprayKey:
	  switch (PhotoNum.x)
	  {
	    case 0:
		  break;
		case 1:
		  switch (PhotoNum.y)
		  {
		    case 0:
			  break;
			case 1:
			  break;
			case 2:
			  break;
			case 3:
			  break;
			case 4:
			  break;
			default:
			  break;
		  }
		  break;
		case 2:
		  switch (PhotoNum.y)
		  {
		    case 0:
			  break;
			case 1:
			  break;
			case 2:
			  break;
			case 3:
			  break;
			case 4:
			  break;
			default:
			  break;
		  }
		  break;
		case 3:
		  switch (PhotoNum.y)
		  {
		    case 0:
			  break;
			case 1:
			  break;
			case 2:
			  break;
			case 3:
			  break;
			case 4:
			  break;
			default:
			  break;
		  }
		  break;
		case 4:
		  switch (PhotoNum.y)
		  {
		    case 0:
			  break;
			case 1:
			  break;
			case 2:
			  break;
			case 3:
			  break;
			case 4:
			  break;
			default:
			  break;
		  }
		  break;
        case 5:
		  switch (PhotoNum.y)
		  {
		    case 0:
			  break;
			case 1:
			  break;
			case 2:
			  break;
			case 3:
			  break;
			case 4:
			  break;
			default:
			  break;
		  }
		  break;
		case 6:
		  switch (PhotoNum.y)
		  {
		    case 0:
			  break;
			case 1:
			  break;
			case 2:
			  break;
			case 3:
			  break;
			case 4:
			  break;
			default:
			  break;
		  }
		  break;
		default:
		  break;
	  }


	  break;
	case StopKey:
	  switch (PhotoNum.x)
	  {
	    case 0:
		  break;
		case 1:
		  
		case 2:
		  		  
		case 3:
		  
		case 4:
		  
        case 5:
		  
		case 6:
		  if(mech_status.stat == STA_WORKING)
           {
	        ctrl_flag &= (~BLEND_PERMIT); //blend_permit = ST_FAULSE;
		    mech_status.stat = STA_STOPPING;
		   }
          if(mech_status.stat == STA_PAUSED)  mech_status.stat = STA_STOPPED;
	      if(mech_status.stat == STA_FINISHED)  mech_status.stat = STA_STOPPED;
		  break; 
		default:
		  break;
	  }

	  break;
	case MenuKey:
	  switch (PhotoNum.x)
	  {
	    case 0:
		  break;
		case 1:
		case 2:
		case 3:
		case 4:
        case 5:
		case 6:
		  if(PhotoNum.x != 2)
		    PhotoNum.x = 2;
		  Display_Config(PhotoNum.x);
		  break;
		default:
		  break;
	  }

	  break;
	case TestKey:
	  switch (PhotoNum.x)
	  {
	    case 0:
		  break;
		case 1:
		  switch (PhotoNum.y)
		  {
		    case 0:
			  break;
			case 1:
			  break;
			case 2:
			  break;
			case 3:
			  break;
			case 4:
			  break;
			default:
			  break;
		  }
		  break;
		case 2:
		  switch (PhotoNum.y)
		  {
		    case 0:
			  break;
			case 1:
			  break;
			case 2:
			  break;
			case 3:
			  break;
			case 4:
			  break;
			default:
			  break;
		  }
		  break;
		case 3:
		  switch (PhotoNum.y)
		  {
		    case 0:
			  break;
			case 1:
			  break;
			case 2:
			  break;
			case 3:
			  break;
			case 4:
			  break;
			default:
			  break;
		  }
		  break;
		case 4:
		  switch (PhotoNum.y)
		  {
		    case 0:
			  break;
			case 1:
			  break;
			case 2:
			  break;
			case 3:
			  break;
			case 4:
			  break;
			default:
			  break;
		  }
		  break;
        case 5:
		  switch (PhotoNum.y)
		  {
		    case 0:
			  break;
			case 1:
			  break;
			case 2:
			  break;
			case 3:
			  break;
			case 4:
			  break;
			default:
			  break;
		  }
		  break;
		case 6:
		  switch (PhotoNum.y)
		  {
		    case 0:
			  break;
			case 1:
			  break;
			case 2:
			  break;
			case 3:
			  break;
			case 4:
			  break;
			default:
			  break;
		  }
		  break;
		default:
		  break;
	  }

	  break;
	case CarriageKey:
	  switch (PhotoNum.x)
	  {
	    case 0:
		  break;
		case 1:
		  break;
		case 2:
		  switch (PhotoNum.y)
		  {
		    case 0:
			  PhotoNum.x = 5;
			  Display_Config(PhotoNum.x);
			  break;
			case 1:
			  PhotoNum.x = 3;
			  Display_Config(PhotoNum.x);
			  break;
			case 2:
			  PhotoNum.x = 4;
			  Display_Config(PhotoNum.x);
			  break;
			default:
			  break;
		  }
		  break;

		default:
		  break;
	  }

	  break;
    case StartPauseKye:
	  switch (PhotoNum.x)
	  {
	    case 0:
		  break;
		case 1:
	 	case 2:
		case 3:
		case 4:
        case 5:
		  if(mech_status.stat == STA_WORKING)
           {
		    ctrl_flag &= (~BLEND_PERMIT); //blend_permit = ST_FAULSE;  // 禁止程序段转接，本段完成后自然暂停
    		mech_status.stat = STA_PAUSING;
           }else if(mech_status.stat == STA_PAUSED)
		   {
		     ctrl_flag |= BLEND_PERMIT;         //blend_permit = ST_TRUE;
			 mech_status.stat = STA_WORKING;
		   }
		  break;  
		
		default:
		  break;
	  }

	  break;
	case ExitKey:
	  switch (PhotoNum.x)
	  {
	    case 0:
		  break;
		case 1:
		  break;
		case 2:
		  PhotoNum.x = 1;
		  Display_Config(PhotoNum.x);
		  break;
		case 3:
		case 4:
        case 5:
		  PhotoNum.x = 2;
		  Display_Config(PhotoNum.x);
		  break;
		default:
		  break;
	  }

	  break;
	case UpKey:
	  switch (PhotoNum.x)
	  {
	    case 0:
		  break;
		case 1:
		  switch (PhotoNum.y)
		  {
		    case 0:
			  break;
			case 1:
			  if(user_para.speed_scal < 100)
			    user_para.speed_scal++;
			  SendNum_Uart(user_para.speed_scal);
			  break;
			case 2:
			  if(user_para.spindle_scal < 100)
			    user_para.spindle_scal++;
			  SendNum_Uart(user_para.spindle_scal);
			  break;
			case 3:
			  ManuNum++;
			  SendNum_Uart(ManuNum);
			  break;
			default:
			  break;
		  }
		  break;
		case 2:
		  break;
		case 3:
		  switch (PhotoNum.y)
		  {
		    case 0:
			  if(SpotsprayMs < 900)
			    SpotsprayMs += 100;
			  SendNum_Uart(SpotsprayMs);
			  break;
			case 1:
			  if(SpotsprayMs < 990)
			    SpotsprayMs += 10;
			  SendNum_Uart(SpotsprayPower);
			  break;
			case 2:
			  if(SpotsprayMs < 999)
			    SpotsprayMs++;
			  SendNum_Uart(SpotsprayPower);
			  break;
			case 3:
			  if(SpotsprayPower < 100)
			    SpotsprayPower++;
			  SendNum_Uart(SpotsprayPower);
			  break;
			default:
			  break;
		  }
		  break;
		case 4:
		  switch (PhotoNum.y)
		  {
		    case 0:
			  if(SpotsprayMm < 999)
			    SpotsprayMm++;
			  SendFloat_Uart(SpotsprayMm);
			  break;
			case 1:
			  SpotsprayMm += 0.01;
			  SendFloat_Uart(SpotsprayMm);
			  break;
			default:
			  break;
		  }
		  break;
        case 5:
		  break;
		case 6:
		  break;
		default:
		  break;
	  }

	  break;
	case DownKey:
	  switch (PhotoNum.x)
	  {
	    case 0:
		  break;
		case 1:
		  switch (PhotoNum.y)
		  {
		    case 0:
			  break;
			case 1:
			  if(user_para.speed_scal > 0)
			    user_para.speed_scal--;
			  SendNum_Uart(user_para.speed_scal);
			  break;
			case 2:
			  if(user_para.spindle_scal > 0)
			    user_para.spindle_scal--;
			  SendNum_Uart(user_para.spindle_scal);
			  break;
			case 3:
			  if(ManuNum > 0)
			    ManuNum--;
			  SendNum_Uart(ManuNum);
			  break;
			default:
			  break;
		  }
		  break;
		case 2:
		  break;
		case 3:
		  switch (PhotoNum.y)
		  {
		    case 0:
			  if(SpotsprayMs > 100)
			    SpotsprayMs -= 100;
			  SendNum_Uart(SpotsprayMs);
			  break;
			case 1:
			  if(SpotsprayMs > 10)
			    SpotsprayMs -= 10;
			  SendNum_Uart(SpotsprayPower);
			  break;
			case 2:
			  if(SpotsprayMs > 1)
			    SpotsprayMs--;
			  SendNum_Uart(SpotsprayPower);
			  break;
			case 3:
			  if(SpotsprayPower > 0)
			    SpotsprayPower--;
			  SendNum_Uart(SpotsprayPower);
			  break;
			default:
			  break;
		  }
		  break;
		case 4:
		  switch (PhotoNum.y)
		  {
		    case 0:
			  if(SpotsprayMm > 0)
			    SpotsprayMm--;
			  SendFloat_Uart(SpotsprayMm);
			  break;
			case 1:
			  SpotsprayMm -= 0.01;
			  SendFloat_Uart(SpotsprayMm);
			  break;
			default:
			  break;
		  }
		  break;
        case 5:
		  break;
		case 6:
		  break;
		default:
		  break;
	  }

	  break;
	case LeftKey:
	  switch (PhotoNum.x)
	  {
	    case 0:
		  break;
		case 1:
		  if(PhotoNum.y > 0 && PhotoNum.y <=3)
		    PhotoNum.y--;
		  else
		    PhotoNum.y = 3;
		  if(PhotoNum.y == 0)
		    Dis_Cursor_Vertical();
		  else
		    Dis_Cursor_Horizontal(); 
		  break;
		case 2:
		  if(PhotoNum.y > 0 && PhotoNum.y <=2)
		    PhotoNum.y--;
		  else
		    PhotoNum.y = 2; 
		  Dis_Cursor_Horizontal(); 
		  break;
		case 3:
		  if(PhotoNum.y > 0 && PhotoNum.y <=3)
		    PhotoNum.y--;
		  else
		    PhotoNum.y = 3;
		  Dis_Cursor_Horizontal(); 
		  break;
		case 4:
		  if(PhotoNum.y > 0 && PhotoNum.y <=1)
		    PhotoNum.y--;
		  else
		    PhotoNum.y = 1;
		  Dis_Cursor_Horizontal();
		  break;
        case 5:
		  if(PhotoNum.y > 0 && PhotoNum.y <=2)
		    PhotoNum.y--;
		  else
		    PhotoNum.y = 2; 
		  Dis_Cursor_Vertical(); 
		  break;
		default:
		  break;
	  }

	  break;
	case RightKey:
	  switch (PhotoNum.x)
	  {
	    case 0:
		  break;
		case 1:
		  if(PhotoNum.y >= 0 && PhotoNum.y < 3)
		    PhotoNum.y++;
		  else
		    PhotoNum.y = 0;
		  if(PhotoNum.y == 0)
		    Dis_Cursor_Vertical();
		  else
		    Dis_Cursor_Horizontal(); 
		  break;
		case 2:
		  if(PhotoNum.y >= 0 && PhotoNum.y < 2)
		    PhotoNum.y++;
		  else
		    PhotoNum.y = 0; 
		  Dis_Cursor_Horizontal(); 
		  break;
		case 3:
		  if(PhotoNum.y >= 0 && PhotoNum.y < 3)
		    PhotoNum.y++;
		  else
		    PhotoNum.y = 0;
		  Dis_Cursor_Horizontal(); 
		  break;
		case 4:
		  if(PhotoNum.y >= 0 && PhotoNum.y < 1)
		    PhotoNum.y++;
		  else
		    PhotoNum.y = 0;
		  Dis_Cursor_Horizontal();
		  break;
        case 5:
		  if(PhotoNum.y >= 0 && PhotoNum.y < 2)
		    PhotoNum.y++;
		  else
		    PhotoNum.y = 0; 
		  Dis_Cursor_Vertical(); 
		  break;
		default:
		  break;
	  }


	  break;
	default:
	  break;
  }  
}

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
  TimingDelay_Decrement();
#if defined MEDIA_USB_KEY
  if ( Command_index == 1)
  {
    Time_Rec_Base ++;
  }
#endif
}


/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/
/**
  * @brief  This function handles External line 1 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI1_IRQHandler(void)
{
  /* Check the clic on the accelerometer to Pause/Resume Playing */
  if(EXTI_GetITStatus(EXTI_Line1) != RESET)
  {
    if( Count==1)
    {
      PauseResumeStatus = 1;
      Count = 0;
    }
    else
    {
      PauseResumeStatus = 0;
      Count = 1;
    }
    /* Clear the EXTI line 1 pending bit */
    EXTI_ClearITPendingBit(EXTI_Line1);
  }
}

/**
  * @brief  This function handles TIM4 global interrupt request.
  * @param  None
  * @retval None
  */
void TIM4_IRQHandler(void)
{
//   uint8_t clickreg = 0;
//
//  
//  /* Checks whether the TIM interrupt has occurred */
//  if (TIM_GetITStatus(TIM4, TIM_IT_CC1) != RESET)
//  {
//    TIM_ClearITPendingBit(TIM4, TIM_IT_CC1);
//
//    capture2 = TIM_GetCapture1(TIM4);
//    TIM_SetCompare1(TIM4, capture2 + CCR_Val);
//  }
}

#if defined MEDIA_USB_KEY
/**
  * @brief  EXTI0_IRQHandler
  *         This function handles External line 0 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI0_IRQHandler(void)       //五向按键响应
{
  /* Checks whether the User Button EXTI line is asserted*/
  if (EXTI_GetITStatus(EXTI_Line0) != RESET) 
  { 

  } 
  /* Clears the EXTI's line pending bit.*/ 
  EXTI_ClearITPendingBit(EXTI_Line0);
}


/**
  * @brief  This function handles TIM2 global interrupt request.
  * @param  None
  * @retval None
  */
void TIM2_IRQHandler(void)              
{
// if(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_12))
//    {
//    STM_EVAL_LEDOff(LED6);
//    }
//    else
//      STM_EVAL_LEDOn(LED6);

  USB_OTG_BSP_TimerIRQ();
}
void TIM3_IRQHandler(void)
{   
       
//	STM_EVAL_LEDToggle(LED5);
    TIM_ClearITPendingBit(TIM3, TIM_FLAG_Update); //清中断
	PWM_Output();
    margin_check();  //限位检测，计算dist_sum_act，curr_posi。并将 mech_status.pulse_out清零。
   // sport_output();  //脉冲输出

    auto_process();
    freq_data_cal(AXIS_X); //计算下一周期定时器的分频值
    freq_data_cal(AXIS_Y);
    freq_data_cal(AXIS_Z);
        
//    capture2 = TIM_GetCapture1(TIM3);
//    TIM_SetCompare1(TIM3, capture2 + CCR_Val);
//  }
  
//  auto_process();
}
//定时器5的中断函数
void TIM5_IRQHandler()
{
       
//       if(TIM_GetITStatus(TIM5,TIM_IT_CC2)!=RESET)
//       {
//              TIM_ClearITPendingBit(TIM5,TIM_IT_CC2);
//              Cnt[1]=(~Cnt[1])&0x01;
//              if(Cnt[1]==0x01)  
//                     R[1]+=Rl[1];
//              else
//                     R[1] += Rh[1];
//              if(R[1]>65535)
//                     R[1]=R[1]-65535;
//              CCR2_Val=R[1];
//              TIM_SetCompare2(TIM5,CCR2_Val);
//       }    
//       if(TIM_GetITStatus(TIM5,TIM_IT_CC3)!=RESET)
//       {
//              TIM_ClearITPendingBit(TIM5,TIM_IT_CC3);
//              Cnt[2]=(~Cnt[2])&0x01;
//              if(Cnt[2]==0x01)  
//                     R[2]+=Rl[2];
//              else
//                     R[2] += Rh[2];
//              if(R[2]>65535)
//                     R[2]=R[2]-65535;
//              CCR3_Val=R[2];
//              TIM_SetCompare3(TIM5,CCR3_Val);
//       }    
//       if(TIM_GetITStatus(TIM5,TIM_IT_CC4)!=RESET)
//       {
//              TIM_ClearITPendingBit(TIM5,TIM_IT_CC4);
// 
//              Cnt[3] = (~Cnt[3])&0x01;
//             
//              if(Cnt[3]==0x01)  
//                     R[3]+=Rl[3];
//              else
//                     R[3] += Rh[3];
//              if(R[3]>65535)
//                     R[3]=R[3]-65535;
//              CCR4_Val=R[3];
//              TIM_SetCompare4(TIM5,CCR4_Val);                     
//       }
}
//串口中断程序
void USART1_IRQHandler(void)
{
   uint16_t temp;
   STM_EVAL_LEDOn(LED4);
   if( USART_GetITStatus(USART1, USART_IT_RXNE) == SET  )
    {
	    USART_ClearITPendingBit(USART1,USART_IT_RXNE);
		temp = USART_ReceiveData(USART1);
        USART_SendData(USART1,temp);
	}
}

/**
  * @brief  This function handles USB-On-The-Go FS global interrupt request.
  * @param  None
  * @retval None
  */
void OTG_FS_IRQHandler(void)
{
  USBH_OTG_ISR_Handler(&USB_OTG_Core);	   //不断的处理usb状态
}
#endif /* MEDIA_USB_KEY */

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/


/**
  * @}
  */ 
  
/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
