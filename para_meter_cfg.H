/*****************************************************************************/
//插补程序底层函数有关定义
/*****************************************************************************/
#ifndef _PARA_METER_CFG_H_
#define _PARA_METER_CFG_H_

#include "system_cfg.h"
#include "cf_card_cfg.h"


//-------布尔常量定义---------------------------------------
#define ST_TRUE         1
#define ST_FAULSE       0

//-------截断误差，作为浮点数判别的标准--------------------
#define TRUNCATE_ERR     0.00001 

//------代码段加载相关的常量定义--------------------------------
#define CODE_SEGMENT_SIZE    22    //每个代码段的长度
#define END_OF_CODE          0xFFFFFFFF 

#define DEFA_PARA_SECTOR     2    //默认参数保存扇区
#define USER_PARA_SECTOR     3    //用户设置参数保存扇区
#define FIELD_PARA_SECTOR    4    //工作现场状态保存扇区

#define CODE_BEGIN_SECTOR    5000    //加工代码起始扇区

//-------串行文件传输的文件头结构-------------------------------------
typedef struct{
    unsigned int name[8];
    unsigned long length;
    unsigned long counter;  
   }typ_file_code;
   
//-------mech code plan result define---------------------- 
typedef struct{          //插补时间结构定义
     unsigned int motion_stat;         //运动状态
     unsigned int motion_stat_next;    //下一周期的运动状态     
     
	 //运动时间
     long term_cmd;          //本段计划运行周期数
     long term_run;          //已经运行的周期数
     long term_rem;          //剩余插补周期计数器，初值等于term_cmd + term_add
  
     unsigned int term_add;  //加速周期数
      
     float time_delta;       //加减速中每个周期的变化量－加速度（用时间表示）
     float time_length;      //插补得到的实际速度值（用时间表示）
    }typ_interp_time; 
  
//-------mech code segment define------------------------
typedef struct{                    //程序段结构定义
     unsigned long label;          //段标号
     unsigned int  basecmd;        //程序功能字（命令字）
     float axis_dist[AXIS_SUM];    //当前段各轴（尚未完成的）运动分量，有符号数
     float dist;                   //当前段空间总位移（mm）或延时时间（秒），无符号数
    
     float vel;                    //合成（命令）速度，由程序给定，无符号数    

     unsigned int nice_stop;      //TRUE-准停，FALSE-允许转接

     float axis_vel[AXIS_SUM];     //实际速度在各轴的分量,有符号数,用单个插补周期的位移量表示(vel*override_scal*INTERP_CYC)
     float dist_out[AXIS_SUM];     //当前段当前周期各轴的计划输出量,有符号数
     
     typ_interp_time interp_time[3];  //插补时间变量组 
     
    }typ_interp_segment;         

//-------electronic parameters define---------------- 
typedef struct{
     int    pulse_timer_type;             //脉冲发生定时器的类型，1－加1定时器， 0－减1定时器
     int    pulse_act_type[AXIS_SUM];     //脉冲在伺服驱动器上的作用方式，0-开始有效(正逻辑)，1-中间有效(反逻辑)
     float  servo_echo_delay[AXIS_SUM];   //伺服装置的响应延时时间，单位为 秒
     float  clk_per_term;                 //一个周期内主时钟脉冲的个数，即MAIN_CLK*interp_cyc
    }typ_elec_para;               

//--------mechanical parameters define---------------
typedef struct{ 

     float max_acc[AXIS_SUM];       //各轴允许的最大加速度
     float max_vel[AXIS_SUM];       //各轴允许的最大速度
     float pulse_cor[AXIS_SUM];     //各轴的脉冲当量

     float spindle_rev_acc;         //主轴变速所采用的加速度    
     float spindle_rev_max;         //主轴转速的最大值

     float utmost_dimen[AXIS_SUM];  //各轴向的极限尺寸（相对于机床圆点）
    
    }typ_mech_para;   

//---------user parameters define----------------------
typedef struct{
     float feed_vel;                 //工进速度(默认值)
     unsigned int  speed_scal;       //运动速度修调系数
          
     float spindle_rev;              //主轴速度
     unsigned int  spindle_scal;     //主轴转速修调系数     

     float swift_vel;                //快移速度
     float slow_vel;                 //各轴的点动速度,用于定位操作
     float spindle_on_time;          //主轴启动时间(s)

     float tool_block_dim[AXIS_SUM]; //对刀块尺寸(z)
     float tool_set_pos[AXIS_SUM];   //对刀完成后刀具停放位置(z),相对于对刀块

     long  override_echo_term;       //响应速度修调的时间（周期数），剩余周期数低于该值不修调

     unsigned int  speed_scal_old;   
     unsigned int  spindle_scal_old;
    }typ_user_para;

//--------mechine status parameter define---------------
typedef struct{
     unsigned int  stat;             //设备运行状态 
     unsigned int  error;            //错误类型字

     unsigned long label;            //程序程序段标号

     float sp_rev;                   //主轴转速
     float dist_sum[AXIS_SUM];       //周期内各轴输出的总位移量，带符号
     float curr_posi[AXIS_SUM];      //当前绝对坐标位置
     float prog_orig[AXIS_SUM];      //程序原点的绝对坐标

     float last_posi[AXIS_SUM];      //上一点绝对坐标 
     float dist_tail[AXIS_SUM];      //输出位移量的尾数寄存器，带符号数      
 }typ_mech_status; 

//---------单轴定长度运行时指定距离-----------------------
typedef struct{              
    unsigned int  axis_num;                  //轴号    
    float         distance;                  //距离
  }type_fixed;
//****************************************************************
//*******USB通讯相关变量*********************************
extern unsigned int usb_rece_stat;
 #define USB_COMM_STOP        0x00
 #define USB_COMM_READY       0x31

 #define USB_MECH_CODE_LOAD   0x32
 #define USB_MECH_CODE_TRANS  0x33
 #define USB_WORK_PARA_RECE   0x34
 
extern unsigned int   comm_echo_back;
extern typ_file_code  file_code;


//********通用变量声明***********************************
extern type_fixed fixed_dist;  //定长度运行信息存储区

//-------工作参数变量-----------------------------------
extern typ_elec_para     elec_para;
extern typ_mech_para     mech_para;
extern typ_user_para     user_para;
extern typ_mech_status   mech_status;

//------运行状态寄存器-----------------------------------
extern unsigned int mech_flag;

 #define TOOL_ADJUST          0x0001
 #define STOP_MARGIN_SCAN     0x0002  //限位检测


 #define OFF_LINE_DO          0x0020
 #define HAND_CTRL_OK         0x0040
 #define CF_CARD_OK           0x0080 
 
 #define SPINDLE_ON           0x0100



//------运行控制标志寄存器及其相关定义---------------------
extern unsigned int ctrl_flag;       
 #define NEXT_SEG_LOAD        0x0001
 #define NEXT_SEG_RENEW       0x0002
 #define NEXT_SEG_READY       0x0004

 #define INSERT_SEG_FULL      0x0010
 #define INSERT_SEG_READY     0x0020 
 
 #define CODE_QUEUE_FULL      0x0100
 #define BLEND_PERMIT         0x0200
 #define TASK_FINISH          0x0400
 #define BEING_AUTO_RUN       0x0800

 #define CODE_BUF_FULL        0x1000 
 


//------各轴的运动方向标志寄存器及其相关定义-------------
extern char axis_dir[AXIS_SUM];
 #define POS             1                  
 #define NEG             -1
 #define STOP            0

//------运行方式标志------------------------------------
extern unsigned char  ctrl_manner;
//------周期内各轴输出的总位移量，带符号-------------------
//extern float xdata axis_out_sum[AXIS_SUM]; 

//------周期内脉冲定时器的设定置 -------------------------
extern unsigned int  freq_const[AXIS_SUM];

//------周期内输出的脉冲数-------------------------------
extern unsigned int  pulse_out[AXIS_SUM];

//------程序段转接时间，用插补周期数表示-------------------
extern unsigned int  blend_term; 


//------运动暂停时间，根据代码要求在logic_cmd_treat()中进行设置
//extern float xdata dwell_time;              //暂停时间


//**********************************************************************


//-------用于加工处理------------------------------------------
extern unsigned int code_head_index;      //代码队列的首指针
extern unsigned int code_tail_index;      //代码队列的尾指针
//extern unsigned char mech_code_queue[260]; //加工代码队列
//extern unsigned char * mech_code_queue;
//------------------------------------------------------------
//**********************************************************
//读写CF卡所需的扇区缓冲区定义（各单元模块通用）
extern unsigned long sec_read_index;    //扇区读操作指针
extern unsigned long sec_wrt_index;  //扇区写操作指针
extern unsigned int  code_rw_index;     //扇区内代码指针
//extern unsigned char sector_buf[260]; //扇区操作缓冲区 
//extern unsigned char * sector_buf;

//------------function announce------------------------------------------
void field_para_save(void);
void system_para_load(unsigned char parameter_num);
//parameter_num:0-不读CF卡扇区，
//              1-读CF卡默认参数扇区, 
//              2-读CF卡用户设定参数扇区,
//              3-读CF卡工作现场参数扇区 
//成功返回1，失败返回0
void fill_code_buf(void);
void code_queue_init(void);
void mech_code_locate(long seg_label);
char mech_code_load(typ_interp_segment *interpl_segment);  

void Main_rece_deal(void);


void public_para_init(void);
void mech_status_init(void);



void elec_para_init(void);
void mech_para_init(void);
void user_para_init(void);


#endif
//**********************************************************




 