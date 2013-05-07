/*----------------------------------------------

          ������ʵ��4*5���̵�ɨ��
          �����ң����ϵ��£���ֵ
          ����Ϊ1-20

------------------------------------------------*/

#ifndef __KEYBOARD_H
#define __KEYBOARD_H

//#include "stm32f10x_lib.h"

//ѡ��ɨ��ģʽ
#define Interrupt_Scan   //�ж�ɨ��ģʽ ,Ҫ��NVIC���д򿪶�Ӧ�ж�
//#define Timer_Scan     //��ʱ������ɨ��

#define DELAY_COUNT    0x01

/* ���̿������Ŷ��� */
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

#ifdef Interrupt_Scan   /* �ж�ɨ��ģʽ�궨�� */

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

#endif  /* �ж�ɨ��ģʽ�궨�� */
/* ����ȫ�ֱ������� */
//unsigned int Keyboard_Val ;    //��ǰ��ֵ
//unsigned char Keyboard_Change_Flag ;  //��ֵ�ı��־����ȡ�µļ�ֵ��������������

/* ���̽ӿں������� */
//#ifdef Interrupt_Scan
//extern void Init_Keyboard_Interrupt(void) ;//���̳�ʼ��Ϊ����ɨ��ģʽ
//#endif

void Delay(vu32 nCount) ;  //������ʱ����
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
  uint8_t x; //��X��ͼ
  uint8_t y; //��Y�����
} PhotoNum_Init;

#endif /* KEYBOARD_H */
