#include "main.h"
#include "key_code_def.h"
#include "keyboard.h"
#include "data_deal_Def.h" //5-10 ��

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
//Ӳ����ʼ��
  /* Initialize User Button */
  STM_EVAL_PBInit(BUTTON_USER, BUTTON_MODE_EXTI); 
  TIM3_Config();  
  /* Init Host Library */
  USBH_Init(&USB_OTG_Core, USB_OTG_FS_CORE_ID, &USB_Host, &USBH_MSC_cb, &USR_Callbacks);
//---------------------------------------------------------------------------  
  while (!FSReady) //���ú��ļ�ϵͳ���˳�
  {
    /* Host Task handler */
    USBH_Process(&USB_OTG_Core, &USB_Host);  //ʶ���U�� 
  }
  if(FSReady)  //��������״̬
    {    
	
    //������ʼ��----------------------------------------------------
       mech_status_init();
       public_para_init(0);
       system_para_load(MECH_PARA_SET);	//��ȡU���е�ϵͳ������Ϣ���� ��code_buffer1��
       system_para_dispatch(MECH_PARA_SET); //�� code_buffer1�еĶ������Ƶ�mech_para�У�������
       system_para_load(USER_PARA_SET);
       system_para_dispatch(USER_PARA_SET);       
       code_buf_init(); 
	   //��ʾ��һҳ
	   PhotoNum.x++;
	   Display_Config(PhotoNum.x); 
 /*      ��ĳ�ʼ�������ֶ��ļ����档         
        axis_init(AXIS_X);
        axis_init(AXIS_Y);
        axis_init(AXIS_Z); 
  */      
  //�Զ��ӹ�--------------------------------------------------------------------
        interp_segment_init();       //�岹�δ���  interpolation segment initialization         
        ctrl_manner = CMD_AUTO;      //CMD_MANU 5-10��           
        mech_flag = 0x0000;
        ctrl_flag = 0x0000; 
        auto_ctrl();
        /* TIM3 enable counter */
        TIM_Cmd(TIM3, ENABLE);       //test����
    //��������----------------------------------------------------------
        while(1)
        {          
            
            code_seg_pre_treat();        //��Ԥ����
            mech_code_load_FL();          //�����ȡ      �����������Ĵ��붼�����˲���һ���滮
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
/*��ʼ��Keyboard*/
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

    /*����PA1~4 ����͵�ƽ*/
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin = Keyboard_Line;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(Keyboard_Control_Port,&GPIO_InitStructure);
    GPIO_ResetBits(Keyboard_Control_Port,Keyboard_Line);

    /*����ROW1~ROW3Ϊ����*/
    GPIO_InitStructure.GPIO_Pin = Keyboard_Row;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_Init(Keyboard_Control_Port_2,&GPIO_InitStructure);
}

/*��ʼ���ڴ���*/
void USART1_AF_Config()
{
    GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	/*ͨ��GPIO_PinAFConfig()������PB6��PB7���ӵ�USART1_Tx��USART2_Rx*/
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource7,GPIO_AF_USART1);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;//Tx��
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;//Rx��
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	//GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	/*����USART�ĳ�ʼ����������BaudRate = 115200�� Word Length = 8�� One Stop Bit�� No parity����żУ�顣*/
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);
	USART_ClockInitStructure.USART_Clock = USART_Clock_Disable;	 //�˴�Ϊ��Ҫ��ʱ���ź����Σ�
	USART_ClockInitStructure.USART_CPOL = USART_CPOL_Low;		 //����μ�Note��¼��STM32F4�������
	USART_ClockInitStructure.USART_CPHA = USART_CPHA_2Edge;		 //����μ�Note��¼��STM32F4�������
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
/*��ʾ���ĳ�ʼ��  ��δ���ƣ�Ҫ��ϴ�U���ж�ȡ�����ݽ��г�ʼ��*/
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
//  /* Time base configuration ʱ�ӳ�ʼ��*/
//  TIM_TimeBaseStructure.TIM_Period = 65535;			//�Զ���װ�Ĵ������ۼ�0xFFFF��Ƶ�ʺ�������»��ж�
//  TIM_TimeBaseStructure.TIM_Prescaler = prescalervalue;	  //Ԥ��Ƶ���˴�ΪSystemCoreClock/550000-1
//  TIM_TimeBaseStructure.TIM_ClockDivision = 0;			  //TIM_ClockDivision��������δ��Ƶ֮ǰ������Ҫ�����µķ�Ƶ����ȷ��һ������ʱʱ�䣬�ڴ�ʱ�������һ����Ԥ�ڹ���
//  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //��ʱ������ģʽ�����ϼ���
//  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
//  
//  /* Enable TIM4 Preload register on ARR */
//  TIM_ARRPreloadConfig(TIM4, ENABLE);
//  
//  /* TIM PWM1 Mode configuration: Channel */
//  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;	 //ѡ��ʱ��ģʽ
//  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	 //ѡ������Ƚ�״̬
//  TIM_OCInitStructure.TIM_Pulse = CCR_Val;		  //�����˴�װ�벶��Ƚ���������ֵ
//  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;	  //�������������High���Ǹߵ�ƽ��Чʱ�Σ�Low���ǵ͵�ƽ��Ч��
//  /****************************************************
//  TIM_OCMode			 ����
//  TIM_OCMode_Timing      TIM����Ƚ�ʱ��ģʽ���ж�ʱ�ܽ��ޱ仯
//  TIM_OCMode_Active      TIM����Ƚ�ʱ��ģʽ���ж�ʱ�ܽ�ǿ��Ϊ��Ч��ƽ
//  TIM_OCMode_Inactive    TIM����Ƚ�ʱ��ģʽ���ж�ʱ�ܽ�ǿ��Ϊ��Ч��ƽ
//  TIM_OCMode_Toggle      TIM����Ƚ�ʱ��ģʽ���ж�ʱ�ܽ�״̬��ת
//  TIM_OCMode_PWM1        TIM�����ȵ���ģʽ1
//  TIM_OCMode_PWM2        TIM�����ȵ���ģʽ2
//  PS:��Ч��ƽ�Ǹ��ǵͣ�ȡ����CCER�Ĵ�����CCxPλ�����ã�����PWMģʽ����������ͨ���ĵ�ƽ�������෴�ġ�
//  *****************************************************/
//  /* Output Compare PWM1 Mode configuration: Channel2 */
//  TIM_OC1Init(TIM4, &TIM_OCInitStructure);
//  TIM_OC1PreloadConfig(TIM4,TIM_OCPreload_Disable);
//  /************************************************
//  *TIMx_CCRx�Ĵ����ܹ����κ�ʱ��ͨ��������и�����*
//  *����������Σ�������δʹ��Ԥװ�ؼĴ�����OCxPE��*
//  *��0��������TIMx_CCRӰ�ӼĴ���ֻ���ڷ�����һ�θ���*
//  *�¼�ʱ�����£�����������Disable����Ϊ�˺�������*
//  *�Ϸ����ӳ�������޸�TIMx_CCRʵʱ������		  *
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
  TIM_TimeBaseStructure.TIM_Period =2000-1;//40000-1;//65535;//40000-1; //0x9c40  5-15������ʱ����2ms?
  TIM_TimeBaseStructure.TIM_Prescaler = 168-1;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Down;
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
  
  /* Enable TIM3 Preload register on ARR ��TIM_Period*/
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