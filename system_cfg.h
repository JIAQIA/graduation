/*****************************************************************/
//ϵͳ����ģ��
/*****************************************************************/
 #ifndef _system_def_h_
 #define _system_def_h_ 


//******************************************
 #define SYSCLK                  (unsigned long)168000000 //20000000   //ϵͳʱ�ӣ�Hz)	5-15��

 #define PULSE_TIMER_CLK         (float)21000000    //���巢����ʱ��ʱ��(Hz)5-15 �������Ӧ��û�ã�ԭDSP�ж�ʱ����Ҫ
 #define CLK_PER_USECOND         (float)5 
// #define CLK_PER_TERM            (float)50000   //PULSE_TIMER_CLK*INTERP_CYC 
// #define INTERP_CYC_MS           10                //msΪ��λ�Ĳ鲹����
// #define INTERP_CYC              (float)0.01       //�岹����(s) 
// #define INTERP_CYC_2            (float)0.0001     //�岹���ڵ�ƽ�� 
// #define INTERP_FREQ_2           10000              //�岹���ڵĵ���ƽ��

 #define CLK_PER_TERM            (float)42000   //PULSE_TIMER_CLK*INTERP_CYC/2 
 #define CLK_PER_TERM_D          (long)42000

 #define INTERP_CYC_MS           4                //msΪ��λ�Ĳ鲹����
 #define INTERP_CYC              (float)0.004       //�岹����(s) 
 #define INTERP_CYC_2            (float)0.000016     //�岹���ڵ�ƽ�� 
 #define INTERP_FREQ_2           (long)62500  

/********************************************/

 #define AXIS_SUM         3     //ϵͳ����

 #define AXIS_X           0     //�������鴦��
 #define AXIS_Y           1
 #define AXIS_Z           2

//-----------------------------------------------------------------
 #define BAUDRATE        9600    //115200            // Baud rate of UART in bps

 #endif



