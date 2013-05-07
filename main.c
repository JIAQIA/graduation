#include "main.h"
#include "key_code_def.h"
#include "keyboard.h"
#include "data_deal_Def.h" //5-10 加

USB_OTG_CORE_HANDLE          USB_OTG_Core;
USBH_HOST                    USB_Host;

RCC_ClocksTypeDef RCC_Clocks;
__IO uint8_t RepeatState = 0;
__IO uint16_t CCR_Val = 16826;
u8 FSReady=0x00;
uint8_t TxChangePic[] = {0xAA,0x70,0x00,0xCC,0x33,0xC3,0x3c};
int test;
uint16_t Key_Num;
uint16_t Keyboard_Change_Flag = 1;
extern PhotoNum_Init PhotoNum;
void Keyboard_Init(void);
void USART1_AF_Config(void);
void Display_Init(void);

extern void ReadPara(u8 readMech);
//extern __IO uint8_t LED_Toggle;

/* Private function prototypes -----------------------------------------------*/
//static void TIM_LED_Config(void);
/* Private functions ---------------------------------------------------------*/
void SendData_Uart(u8 data[],int len)
  {
	int i=0;
	
	for(i=0;i<len;i++)
	{
	    USART_SendData(USART1,data[i]);
		//Delay(0x0FFF);
		while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);				  
	}
  }			

/**
  * @brief  Main program.
  * @param  None
  * @retval None
*/
int main(void)
{    
   
  /* Initialize LEDS */
  STM_EVAL_LEDInit(LED3);
  STM_EVAL_LEDInit(LED4);
  STM_EVAL_LEDInit(LED5);
  STM_EVAL_LEDInit(LED6);  
  /* Green Led On: start of application */
      
  /* SysTick end of count event each 10ms */
  RCC_GetClocksFreq(&RCC_Clocks); 
  SysTick_Config(RCC_Clocks.HCLK_Frequency / 100);  
  Keyboard_Init();
  USART1_AF_Config();
  Display_Init();

  /* Configure TIM4 Peripheral to manage LEDs lighting */
//  TIM_LED_Config();
//--------------------------------------------------------------------- 
//硬件初始化
  /* Initialize User Button */
  STM_EVAL_PBInit(BUTTON_USER, BUTTON_MODE_EXTI); 
  TIM3_Config();  
  /* Init Host Library */
  USBH_Init(&USB_OTG_Core, USB_OTG_FS_CORE_ID, &USB_Host, &USBH_MSC_cb, &USR_Callbacks);
//---------------------------------------------------------------------------  
  while (!FSReady) //配置好文件系统后退出
  {
    /* Host Task handler */
    USBH_Process(&USB_OTG_Core, &USB_Host);  //识别出U盘 
  }
  if(FSReady)  //进入运行状态
    {    
	
    //参数初始化----------------------------------------------------
       mech_status_init();
       public_para_init(0);
       system_para_load(MECH_PARA_SET);	//读取U盘中的系统参数信息加载 到code_buffer1中
       system_para_dispatch(MECH_PARA_SET); //把 code_buffer1中的东西复制到mech_para中（繁琐）
       system_para_load(USER_PARA_SET);
       system_para_dispatch(USER_PARA_SET);       
       code_buf_init(); 
	   //显示第一页
	   PhotoNum.x++;
	   Display_Config(PhotoNum.x); 
 /*      轴的初始化，在手动文件里面。         
        axis_init(AXIS_X);
        axis_init(AXIS_Y);
        axis_init(AXIS_Z); 
  */      
  //自动加工--------------------------------------------------------------------
        interp_segment_init();       //插补段处理  interpolation segment initialization         
        ctrl_manner = CMD_AUTO;      //CMD_MANU 5-10改           
        mech_flag = 0x0000;
        ctrl_flag = 0x0000; 
        auto_ctrl();
        /* TIM3 enable counter */
        TIM_Cmd(TIM3, ENABLE);       //test屏蔽
    //背景程序----------------------------------------------------------
        while(1)
        {          
            
            code_seg_pre_treat();        //段预处理
            mech_code_load_FL();          //代码读取      读进缓冲区的代码都用完了才下一步规划
            if(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_6) == Bit_RESET ||\
	           GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_7) == Bit_RESET || \
	           GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_8) == Bit_RESET ) 
	            {
	      			Key_Num = Key_Read();
					if(Keyboard_Change_Flag == 1)
					{
	   				  Key_Process(Key_Num);
					  Keyboard_Change_Flag = 0;
					}
	 			}
          	if(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_6) == Bit_SET &&\
	           GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_7) == Bit_SET &&\
	           GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_8) == Bit_SET )
			   {
			       Keyboard_Change_Flag = 1;
			   }    
           
        }
    }
 
  
//#endif
  
}
/*初始化Keyboard*/
void Keyboard_Init()
{
    GPIO_InitTypeDef GPIO_InitStructure;
    //NVIC_InitTypeDef NVIC_InitStructure;
	//EXTI_InitTypeDef EXTI_InitStructure;

    /*Enable GPIO clock*/
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);
    /*Enable SYSCFG clock*/
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);

    /*配置PA1~4 输入低电平*/
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin = Keyboard_Line;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(Keyboard_Control_Port,&GPIO_InitStructure);
    GPIO_ResetBits(Keyboard_Control_Port,Keyboard_Line);

    /*配置ROW1~ROW3为上拉*/
    GPIO_InitStructure.GPIO_Pin = Keyboard_Row;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_Init(Keyboard_Control_Port_2,&GPIO_InitStructure);
}

/*初始化口串口*/
void USART1_AF_Config()
{
    GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	/*通过GPIO_PinAFConfig()函数将PB6与PB7连接到USART1_Tx，USART2_Rx*/
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource7,GPIO_AF_USART1);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;//Tx线
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;//Rx线
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	//GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	/*进行USART的初始化，波特率BaudRate = 115200， Word Length = 8， One Stop Bit， No parity无奇偶校验。*/
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);
	USART_ClockInitStructure.USART_Clock = USART_Clock_Disable;	 //此处为何要将时钟信号屏蔽？
	USART_ClockInitStructure.USART_CPOL = USART_CPOL_Low;		 //意义参见Note记录的STM32F4相关资料
	USART_ClockInitStructure.USART_CPHA = USART_CPHA_2Edge;		 //意义参见Note记录的STM32F4相关资料
	USART_ClockInitStructure.USART_LastBit = USART_LastBit_Disable;
	//USART_Init(USART1, &USART_InitStructure);
	USART_ClockInit(USART1, &USART_ClockInitStructure);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	//USART_ITConfig(USART1,USART_IT_TC,ENABLE);
	USART_Cmd(USART1, ENABLE);
	USART_GetFlagStatus(USART1, USART_FLAG_TC);
	PhotoNum.x = 0;
	PhotoNum.y = 0;
}
/*显示器的初始化  还未完善，要结合从U盘中读取的数据进行初始化*/
void Display_Init()
{
	PhotoNum.x = 0;
	PhotoNum.y = 0;
	Palette_Cfg();
	SendData_Uart(TxChangePic,7);
	Delay(10);
}


/**
  * @brief  Configures the TIM Peripheral for Led toggling.
  * @param  None
  * @retval None
  */
//static void TIM_LED_Config(void)
//{
//  TIM_OCInitTypeDef  TIM_OCInitStructure;
//  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
//  NVIC_InitTypeDef NVIC_InitStructure;
//  uint16_t prescalervalue = 0;
//  
//  /* TIM4 clock enable */
//  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
//  //RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
//  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
//  
//  /* Enable the TIM4 gloabal Interrupt */
//  NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//  NVIC_Init(&NVIC_InitStructure);
//
//  /* Initialize Leds mounted on STM324F4-EVAL board */
//  STM_EVAL_LEDInit(LED3);
//  STM_EVAL_LEDInit(LED4);
//  STM_EVAL_LEDInit(LED6);
//  
//  /* Compute the prescaler value */
//  prescalervalue = (uint16_t) ((SystemCoreClock ) / 550000) - 1;
//  
//  /* Time base configuration 时钟初始化*/
//  TIM_TimeBaseStructure.TIM_Period = 65535;			//自动重装寄存器，累计0xFFFF个频率后产生更新或中断
//  TIM_TimeBaseStructure.TIM_Prescaler = prescalervalue;	  //预分频，此处为SystemCoreClock/550000-1
//  TIM_TimeBaseStructure.TIM_ClockDivision = 0;			  //TIM_ClockDivision作用是在未分频之前，根据要求建立新的分频器，确定一定的延时时间，在此时间内完成一定的预期功能
//  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //定时器计数模式，向上计数
//  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
//  
//  /* Enable TIM4 Preload register on ARR */
//  TIM_ARRPreloadConfig(TIM4, ENABLE);
//  
//  /* TIM PWM1 Mode configuration: Channel */
//  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;	 //选择定时器模式
//  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	 //选择输出比较状态
//  TIM_OCInitStructure.TIM_Pulse = CCR_Val;		  //设置了待装入捕获比较器的脉冲值
//  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;	  //设置了输出极性High就是高电平有效时段，Low就是低电平有效段
//  /****************************************************
//  TIM_OCMode			 描述
//  TIM_OCMode_Timing      TIM输出比较时间模式，中断时管脚无变化
//  TIM_OCMode_Active      TIM输出比较时间模式，中断时管脚强制为有效电平
//  TIM_OCMode_Inactive    TIM输出比较时间模式，中断时管脚强制为无效电平
//  TIM_OCMode_Toggle      TIM输出比较时间模式，中断时管脚状态翻转
//  TIM_OCMode_PWM1        TIM脉冲宽度调制模式1
//  TIM_OCMode_PWM2        TIM脉冲宽度调制模式2
//  PS:有效电平是高是低，取决于CCER寄存器里CCxP位的设置，两种PWM模式，区别在于通道的电平极性是相反的。
//  *****************************************************/
//  /* Output Compare PWM1 Mode configuration: Channel2 */
//  TIM_OC1Init(TIM4, &TIM_OCInitStructure);
//  TIM_OC1PreloadConfig(TIM4,TIM_OCPreload_Disable);
//  /************************************************
//  *TIMx_CCRx寄存器能够在任何时候通过软件进行更新以*
//  *控制输出波形，条件是未使用预装载寄存器（OCxPE＝*
//  *‘0’，否则TIMx_CCR影子寄存器只能在发生下一次更新*
//  *事件时被更新）。这里设置Disable就是为了后面在中*
//  *断服务子程序可以修改TIMx_CCR实时起作用		  *
//  ************************************************/  
//  /* TIM Interrupts enable */
//  TIM_ITConfig(TIM4, TIM_IT_CC1 , ENABLE);
//  
//  /* TIM4 enable counter */
//  TIM_Cmd(TIM4, ENABLE);
//}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

static void TIM3_Config(void)
{
//  TIM_OCInitTypeDef  TIM_OCInitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
//  uint16_t prescalervalue = 0;
  
  /* TIM4 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
  
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  
  /* Enable the TIM3 gloabal Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);


  /* Compute the prescaler value */
 // prescalervalue = (uint16_t) ((SystemCoreClock ) / 550000) - 1;
    //prescalervalue =65535;//(uint16_t) ((SystemCoreClock ) /10000) - 1;//(uint16_t) ((SystemCoreClock ) /10000) - 1;  //(uint16_t) ((SystemCoreClock ) /10000) - 1 //    11999;// 6000;//65535;//
  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period =2000-1;//40000-1;//65535;//40000-1; //0x9c40  5-15这样定时岂不是2ms?
  TIM_TimeBaseStructure.TIM_Prescaler = 168-1;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Down;
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
  
  /* Enable TIM3 Preload register on ARR 即TIM_Period*/
  TIM_ARRPreloadConfig(TIM3, ENABLE);           
 
  TIM_ClearITPendingBit(TIM3, TIM_IT_Update);  
  
  /* TIM IT enable */
  TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE); 
  //TIM_Cmd(TIM3,ENABLE);
  /* TIM PWM1 Mode configuration: Channel */
//  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
//  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
//  TIM_OCInitStructure.TIM_Pulse = CCR_Val;
//  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
//  
//  /* Output Compare PWM1 Mode configuration: Channel2 */
//  TIM_OC1Init(TIM3, &TIM_OCInitStructure);
//  TIM_OC1PreloadConfig(TIM3,TIM_OCPreload_Disable);
//    
//  /* TIM Interrupts enable */
//  TIM_ITConfig(TIM3, TIM_IT_CC1 , ENABLE);
  
 
}