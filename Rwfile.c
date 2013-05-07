#include"main.h"

extern FATFS fatfs;
extern FIL file;
extern FIL fileR;
extern DIR dir;
extern FILINFO fno;
extern uint16_t *CurrentPos;

uint16_t buffer1[_MAX_SS] ={0x00};
UINT BytesRead;
static char* FileName ;
static __IO uint32_t TimingDelay;
//__IO uint32_t WaveCounter=0x0003;

bool ReadPara(u8 para_type,unsigned int *Para_buffer)
{
 char path[] = "0:/";
      /* Get the read out protection status */
  if (f_opendir(&dir, path)!= FR_OK)
  {
    while(1)
    {
      STM_EVAL_LEDToggle(LED5);  //亮LED5报错
      Delay(10);
    }    
  }
  else
  { 
    if(para_type==0x01)    // 读机床参数
    {  
        FileName = MECH_NAME;
         /* Open the wave file to be played */
        if (f_open(&fileR, FileName , FA_READ) != FR_OK)
        {
          STM_EVAL_LEDOn(LED5);          
        }
        else   
        {            
          // f_lseek(&fileR, WaveCounter);   这个函数可以定位到特定位置
          f_read(&fileR,Para_buffer,112, &BytesRead);// 从mech.txt里面读出来两个mech_para到code_buffer1         
        }
    }
    else     //读用户参数
    {
     FileName = USER_NAME;
         /* Open the wave file to be played */
        if (f_open(&fileR, FileName , FA_READ) != FR_OK)
        {
          STM_EVAL_LEDOn(LED5);          
        }
        else    
        {            
          f_read (&fileR,Para_buffer,120, &BytesRead);//从user.txt里面读出来两个user_para到code_buffer1              
        }
    }
  }
  return TRUE;
}

/**
  * @brief  Inserts a delay time.
  * @param  nTime: specifies the delay time length, in 10 ms.
  * @retval None
  */
void Delay(__IO uint32_t nTime)
{
  TimingDelay = nTime;
  
  while(TimingDelay != 0);
}

/**
  * @brief  Decrements the TimingDelay variable.
  * @param  None
  * @retval None
  */
void TimingDelay_Decrement(void)
{
  if (TimingDelay != 0x00)
  { 
    TimingDelay--;
  }
}

/**
  * @brief  MEMS accelerometre management of the timeout situation.
  * @param  None.
  * @retval None.
  */
uint32_t LIS302DL_TIMEOUT_UserCallback(void)
{
  /* MEMS Accelerometer Timeout error occured */
  while (1)
  {   
  }
}

void Code_read_data(uint32_t Flash_rw_pointer,int CODE_Length,unsigned int * Code_buffer)
{
  char path[] = "0:/";
      /* Get the read out protection status */
  if (f_opendir(&dir, path)!= FR_OK)
  {
    while(1)
    {
      STM_EVAL_LEDToggle(LED5);  //亮LED5报错
      Delay(10);
    }    
  }
  else
  { 
      
        FileName = CODE_NAME;
         /* Open the wave file to be played */
        if (f_open(&fileR, FileName , FA_READ) != FR_OK)
        {
          STM_EVAL_LEDOn(LED5);          
        }
        else   
        {            
          f_lseek(&fileR, Flash_rw_pointer);   //这个函数可以定位到特定位置
          f_read(&fileR,Code_buffer,CODE_Length, &BytesRead);// 从mech.txt里面读出来两个mech_para到code_buffer1         
        }
    }
}



