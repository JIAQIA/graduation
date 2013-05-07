//************************************************************ 
//系统基本变量以及函数定义
//************************************************************ 
#ifndef _PARA_METER_CFG_H_
#define _PARA_METER_CFG_H_

#include "system_cfg.h"
//#include "cf_card_cfg.h"

//-------布尔常量定义----------------------------------------
#define ST_TRUE         1
#define ST_FAULSE       0

//-------截断误差，作为浮点数判别的标准----------------------
#define TRUNCATE_ERR    0.00001 

//-------脉冲发生器所用定时器的参数定义-------------------
#define FREQCONST_MAX_F   (float)65530    //定时器分频常数的最大值
#define FREQCONST_MIN_F   (float)24

//-------键盘缓冲区大小定义----------------------------------
#define KEY_BUFFER      0x05 


//------代码段加载相关的常量定义-----------------------------
#define CODE_SEGMENT_SIZE    24    //每个代码段的长度(双字节）  0424修改  现在是但字节计算
#define CODE_BUF_LENGTH     120 //代码缓冲区的长度（双字节）0405修改，由110改成120不知道为什么写成了110
#define END_OF_CODE          0xFFFFFFFF 


//--------加工数量--------------------------------------------
extern int ManuNum;
//------是否开激光--------------------------------------
extern char OpenLaser;

//--------点射配置--------------------------------------------
extern int SpotsprayPower;   //点射功率
extern int SpotsprayMs;	  //点射时间(ms)
extern float SpotsprayMm;    //点动距离(mm)
//-------mech code plan result define---------------------- 
typedef struct{          //插补时间结构定义
     int motion_stat;         //运动状态
     int motion_stat_next;    //下一周期的运动状态     
     
	 //运动时间
     long term_cmd;          //本段计划运行周期数
     long term_run;          //已经运行的周期数
     long term_rem;          //剩余插补周期计数器，初值等于term_cmd + term_add
  
     int term_add;           //加速周期数
      
     float time_delta;       //加减速中每个周期的变化量－加速度（用时间表示）
     float time_length;      //插补得到的实际速度值（用时间表示）
    }typ_interp_time; 
  
//-------mech code segment define------------------------
typedef struct{                    //程序段结构定义
     unsigned long label;          //段标号
     int  basecmd;        //程序功能字（命令字） 0xff
     float axis_dist[3];    //当前段各轴（尚未完成的）运动分量，有符号数
     float dist;                   //当前段空间总位移（mm）或延时时间（秒），无符号数

     float vel;                    //合成（命令）速度，由程序给定，无符号数    

     float axis_vel[3];     //实际速度在各轴的分量,有符号数,用单个插补周期的位移量表示(vel*speed_scal*INTERP_CYC)
     float dist_out[3];     //当前段当前周期各轴的计划输出量,有符号数

     typ_interp_time interp_time;  //插补时间变量组 
     
     float axis_seg[3];
    }typ_interp_segment;         


//--------mechanical parameters define---------------
typedef struct{
     float max_acc[AXIS_SUM];            //各轴允许的最大加速度
     float max_vel[AXIS_SUM];            //各轴允许的最大速度
     float pulse_cor[AXIS_SUM];          //各轴的脉冲当量

     float spindle_rev_acc;         //主轴变速所采用的加速度    
     float spindle_rev_max;         //主轴转速的最大值

     float utmost_dimen[AXIS_SUM];  //各轴向的极限尺寸（相对于机床圆点）
    }typ_mech_para;   

//---------user parameters define----------------------
typedef struct{
     float feed_vel;                 //工进速度(默认值)
     int  speed_scal;       //运动速度修调系数
          
     float spindle_rev;              //主轴速度
     int  spindle_scal;     //主轴转速修调系数     

     float swift_vel;                //快移速度
     float slow_vel;                 //各轴的点动速度,用于定位操作
     int spindle_on_time;            //主轴启动时间(ms)

     float tool_block_dim[AXIS_SUM]; //对刀块尺寸(z)
     float tool_set_pos[AXIS_SUM];   //对刀完成后刀具停放位置(z),相对于对刀块

     unsigned int override_echo_term; //响应速度修调的时间（周期数），剩余周期数低于该值不修调
     unsigned int blend_time;         //程序段转接时间(周期数)

    }typ_user_para;

//--------mechine status parameter define---------------
typedef struct{
     int  stat;             //设备运行状态 
     int  error;            //错误类型字
     int  comm_flag;
     int  code_buf_stat;    //code_permit/code_reject

     unsigned long label;            //程序程序段标号
     int spindle_rev;                //主轴转速，0－停止,正数－正转,负数－反转
     long dist_sum_act[AXIS_SUM];   //周期内各轴输出的实际总位移量，带符号
     float curr_posi[AXIS_SUM];      //当前绝对坐标位置
     float prog_orig[AXIS_SUM];      //程序原点的绝对坐标

     float dist_sum[AXIS_SUM];       //周期内各轴输出的计算总位移量，带符号
     float last_posi[AXIS_SUM];      //上一点绝对坐标 

     float dist_tail[AXIS_SUM];      //输出位移量的尾数寄存器，带符号数      
 
     int pulse_out[AXIS_SUM];
     float dist_tail_a[3];
     //long act_pulse[AXIS_SUM];
     //long aim_pulse[AXIS_SUM];      //由加工线段决定的坐标 
     int  speed_scal_old;   
     int  spindle_scal_old;
 }typ_mech_status;
 
       
//-------串行文件传输的文件头结构-------------------------------
typedef struct{
    unsigned int name[8];
    unsigned long length;
    unsigned long counter;  
   }typ_file_code;

//---------单轴定长度运行时指定距离----------------------------
typedef struct{              
    unsigned int  axis_num;                  //轴号    
    float         distance;                  //距离
  }typ_fixed;

//********************************************************

//********通用变量声明************************************
//-------------------------------------------------------
extern typ_file_code      file_code;    //串行传输文件头格式
extern typ_fixed          fixed_dist;   //定长度运行信息存储格式

//-------工作参数变量-----------------------------------
extern typ_mech_para     mech_para;
extern typ_user_para     user_para;
extern typ_mech_status   mech_status;

//------运行状态寄存器-----------------------------------
extern unsigned int mech_flag;

 #define CF_CARD_OK           0x0001 
 #define HAND_CTRL_OK         0x0002
 #define SPINDLE_ON           0x0004 
 
 #define TOOL_ADJUST          0x0010
 #define SIMULATION           0x0020
 #define OFF_LINE_WORK        0x0040
 

 #define MARGIN_SCAN_OFF      0x0100  //限位检测
 #define MARGIN_EXCEED        0x0200
 #define ROM_READ_ERR         0x0400
 #define ROM_WRITE_ERR        0x0800


 #define USB_COMM_READY       0x1000 
 #define STA_PARA_SET_OK      0x2000 
 #define CODE_DOWN_ENABLE     0x4000
 #define CODE_DOWN_FINISH     0x8000

 
//------运行控制标志寄存器及其相关定义---------------------
extern unsigned int ctrl_flag;       
 #define NEXT_SEG_EMPTY       0x0001
 #define NEXT_SEG_RENEW       0x0002
 #define NEXT_SEG_READY       0x0004

 #define INSERT_SEG_FULL      0x0010
 #define INSERT_SEG_READY     0x0020 
 
 #define CODE_BUF_READY       0x0100
 #define BLEND_PERMIT         0x0200
 #define TASK_FINISH          0x0400
 #define BEING_AUTO_RUN       0x0800

 #define CODE_BUF1_FULL       0x1000 
 #define CODE_BUF2_FULL       0x2000
 #define CODE_BUF1_EMPTY      0x4000
 #define CODE_BUF2_EMPTY      0x8000

//------外部开关逻辑输入寄存器---------------------------
extern unsigned int stat_logic;
/* #define TOOL_A            0x0001    
 #define LIMIT_XP          0x0002
 #define LIMIT_YP          0x0004
 #define LIMIT_ZP          0x0008
 #define LIMIT_XN          0x0010
 #define LIMIT_YN          0x0020
 #define LIMIT_ZN          0x0040
 #define TOOL_B            0x0080
*/
 #define LIMIT_XP          0x0002
 #define LIMIT_XN          0x0008
 #define LIMIT_YP          0x0010
 #define LIMIT_YN          0x0040
 #define LIMIT_ZP          0x0020  //no use
 #define LIMIT_ZN          0x0080  //no use

 //#define IO_APPLY_DEF      0x005A
 #define LIMIT_IO_DEF      0x005A
//------设备标识字--------------------------------------
extern unsigned int  plant_ID[2];

//------运行方式标志------------------------------------
extern unsigned int  ctrl_manner;

//------各轴的运动方向标志寄存器及其相关定义-------------
extern int axis_dir[AXIS_SUM];
 
 #define SET_POS           1
 #define SET_NEG           -1
 #define SET_STOP          0 

//------周期内脉冲定时器的设定置 -------------------------
extern unsigned int  freq_const[AXIS_SUM];
extern unsigned char residue[AXIS_SUM];
//------周期内输出的脉冲数-------------------------------
extern unsigned int  pulse_out[AXIS_SUM];

//------速度修调系数临时备份-----------------------------
extern unsigned int speed_scal;

//------运动暂停时间，根据代码要求在logic_cmd_treat()中进行设置
//extern float xdata dwell_time;              //暂停时间

//**********************************************************************

//----------键盘处理函数-----------------------------------
extern void key_buffer_init(void);
extern int key_write(unsigned int key_value);
extern unsigned int key_read(void);
extern int key2_write(unsigned int key_value);
extern unsigned int key2_read(void);

void read_plant_ID(unsigned int *ID_code);
void write_plant_ID(unsigned int *ID_code);
void system_para_load(int para_type);
void system_para_save(int para_type);
void system_para_dispatch(int para_type);

void mech_para_init(void);
void user_para_init(void); 

void public_para_init(int type_ll);
void mech_status_init(void);

void data_rece_init(unsigned int *buffer);
int data_rece_right(void);
void code_rece_finish(void);

//3.12屏蔽 void code_buf_init(void);
int code_buf_fill(unsigned int input_buf[], int byte_num);
int mech_code_load(typ_interp_segment *interpl_segment); 

void freq_data_cal(int axis_num);
void margin_check(void);
void save_position_to(float position[]);
int position_at(float aim_position[]);
int being_motion(void);

void delay_ms(int time_ms);

#endif
//********************************************************** 