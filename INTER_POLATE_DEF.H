/*****************************************************************************/
//插补程序底层函数有关定义
/*****************************************************************************/
#ifndef _interpola_def_h_
#define _interpola_def_h_

#include "system_cfg.h"
#include "data_deal_def.h"

//-------可以插补的最小距离--------------------------------------
#define LINE_MIN_DIST    0.0001  
#define ARC_MIN_ANGLE    0.001 

#define PI                3.1415926
#define PI_2              6.2831853 

//****************常量定义中的数值分配*******************
//  0x00-0x3f用于全局模块的常量定义(general_def.h)
//  0x40-0x4f用于单轴控制模块的常量(axis_ctrl_sub.c)
//  0x50-0x5f用于多轴控制模块的常量(interpola_sub.c)
//  0x60-0x6f用于复位控制模块的常量(mech_init_ctrl.c)
//  0x70-0x7f用于串行通讯模块以及键盘

//------插补中间过程状态的常量定义(use for motion_stat)------------
#define INP_MOTION_ACC   0x53         //正常加速
#define INP_MOTION_UNI   0x52         //匀速运行
#define INP_MOTION_DEC   0x51         //正常减速 
#define INP_MOTION_END   0x50         //运动停止
//------程序段状态标识---------------------------------------------
//interpl_segment.basecmd == PROGRAM_IDLE 表示本段代码完成
 #define PROGRAM_IDLE    0xFF         
 
//********variable announcement************************************

//------------function announce------------------------------------------
// 线性速度控制
extern void linear_vel_ctrl(typ_interp_time * interp_time);

// 直线插补运算                                             
extern void line_interp_deal(typ_interp_segment * interpl_segment);
extern void arc_interp_deal(typ_interp_segment * interpl_segment);
 
// 直线插补程序规划 
extern void line_interp_plan(typ_interp_segment * interpl_segment);
extern void arc_interp_plan(typ_interp_segment * interpl_segment);

// 程序段规划处理                                          
extern int segment_plan(typ_interp_segment *interpl_segment);

// 程序段加工处理
extern void segment_process(typ_interp_segment *interpl_segment);

//**********************************************************

#endif
//********************************************************** 