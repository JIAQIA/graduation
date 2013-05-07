/*****************************************************************/
//系统配置模块
/*****************************************************************/
 #ifndef _system_def_h_
 #define _system_def_h_ 


//******************************************
 #define SYSCLK                  (unsigned long)168000000 //20000000   //系统时钟（Hz)	5-15改

 #define PULSE_TIMER_CLK         (float)21000000    //脉冲发生定时器时钟(Hz)5-15 这个变量应该没用，原DSP中定时器需要
 #define CLK_PER_USECOND         (float)5 
// #define CLK_PER_TERM            (float)50000   //PULSE_TIMER_CLK*INTERP_CYC 
// #define INTERP_CYC_MS           10                //ms为单位的查补周期
// #define INTERP_CYC              (float)0.01       //插补周期(s) 
// #define INTERP_CYC_2            (float)0.0001     //插补周期的平方 
// #define INTERP_FREQ_2           10000              //插补周期的倒数平方

 #define CLK_PER_TERM            (float)42000   //PULSE_TIMER_CLK*INTERP_CYC/2 
 #define CLK_PER_TERM_D          (long)42000

 #define INTERP_CYC_MS           4                //ms为单位的查补周期
 #define INTERP_CYC              (float)0.004       //插补周期(s) 
 #define INTERP_CYC_2            (float)0.000016     //插补周期的平方 
 #define INTERP_FREQ_2           (long)62500  

/********************************************/

 #define AXIS_SUM         3     //系统轴数

 #define AXIS_X           0     //用于数组处理
 #define AXIS_Y           1
 #define AXIS_Z           2

//-----------------------------------------------------------------
 #define BAUDRATE        9600    //115200            // Baud rate of UART in bps

 #endif



