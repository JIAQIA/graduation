/*----------------------------------------------

          本程序实现4*5键盘的扫描
          从左到右，从上到下，键值
          依次为1-20

------------------------------------------------*/

#ifndef __KEYBOARD_H
#define __KEYBOARD_H

//#include "stm32f10x_lib.h"

//选择扫描模式
#define Interrupt_Scan   //中断扫描模式 ,要在NVIC在中打开对应中断
//#define Timer_Scan     //定时器触发扫描

#define DELAY_COUNT    0x01

/* 键盘控制引脚定义 */
#define Keyboard_Control_Port   GPIOE
#define Keyboard_Line_1    GPIO_Pin_3
#define Keyboard_Line_2    GPIO_Pin_4
#define Keyboard_Line_3    GPIO_Pin_5
#define Keyboard_Line_4    GPIO_Pin_6
#define Keyboard_Control_Port_2   GPIOD
#define Keyboard_Row_1    GPIO_Pin_6
#define Keyboard_Row_2    GPIO_Pin_7
#define Keyboard_Row_3    GPIO_Pin_8
//#define Keyboard_Row_4    GPIO_Pin_8

#define Keyboard_LineBase Keyboard_Line_1
#define Keyboard_RowBase  Keyboard_Row_1
#define Keyboard_Line   (Keyboard_Line_1 | Keyboard_Line_2 | Keyboard_Line_3 | Keyboard_Line_4)
#define Keyboard_Row   (Keyboard_Row_1 | Keyboard_Row_2 | Keyboard_Row_3)

#ifdef Interrupt_Scan   /* 中断扫描模式宏定义 */

#define Keyboard_EXTI_Row1   EXTI_Line5
#define Keyboard_EXTI_Row2   EXTI_Line6
#define Keyboard_EXTI_Row3   EXTI_Line7
#define ResetKey             0x01
#define SpotsprayKey         0x02
#define StopKey              0x03
#define MenuKey				 0x04
#define TestKey				 0x05
#define CarriageKey			 0x06
#define StartPauseKye		 0x07
#define ExitKey				 0x08
#define UpKey				 0x09
#define DownKey				 0x0A
#define LeftKey				 0x0B
#define RightKey			 0x0C

//#define Keyboard_EXTI_Row4   EXTI_Line8

//#define Keyboard_EXTI_PortSource  GPIO_PortSourceGPIOD
#define Keyboard_EXTI_PinSource1 GPIO_PinSource5
#define Keyboard_EXTI_PinSource2 GPIO_PinSource6
#define Keyboard_EXTI_PinSource3 GPIO_PinSource7
//#define Keyboard_EXTI_PinSource4 GPIO_PinSource8

#define Keyboard_IRQ_Channel  EXTI9_5_IRQChannel
#define Keyboard_EXTI_Line    (Keyboard_EXTI_Row1 | Keyboard_EXTI_Row2 | Keyboard_EXTI_Row3)

#endif  /* 中断扫描模式宏定义 */
/* 键盘全局变量声明 */
//unsigned int Keyboard_Val ;    //当前键值
//unsigned char Keyboard_Change_Flag ;  //键值改变标志，读取新的键值后由主程序清零

/* 键盘接口函数声明 */
//#ifdef Interrupt_Scan
//extern void Init_Keyboard_Interrupt(void) ;//键盘初始化为键盘扫描模式
//#endif

void Delay(vu32 nCount) ;  //用于延时消抖
uint16_t Key_Read(void);
void Tx_Start(void);
void Tx_Send(uint16_t data);
void Tx_End(void);
void SendStr_Uart(uint8_t* str);
void Key_Process(uint16_t);
void Dis_Cursor_Horizontal(void);
void Dis_Cursor_Vertical(void);
void SendNum_Uart(uint16_t);
void SendFloat_Uart(float);
void Display_Config(uint8_t);
void Palette_Cfg(void);
typedef struct
{
  uint8_t x; //第X张图
  uint8_t y; //第Y项变量
} PhotoNum_Init;

#endif /* KEYBOARD_H */
