#ifndef _AUTO_CTRL_DEF_H_
#define _AUTO_CTRL_DEF_H_

//************************************************************/
#include "data_deal_def.h"

//--------定义轴号(unsigned int axis_num)---------------------------
//用于函数appoint_position(float position[], unsigned int axis_num)
#define X_AXIS       0x0001
#define Y_AXIS       0x0002
#define Z_AXIS       0x0004
#define XYZ_AXIS     0x0007

//-------加工代码段变量定义-----------------------------------------
extern typ_interp_segment *interpl_segment_prev;
extern typ_interp_segment *interpl_segment_curr;
extern typ_interp_segment *interpl_segment_next;
extern typ_interp_segment *interpl_segment_insert;

extern typ_interp_segment *temp_segment_pointer;

//**********************************************************

//------function announcement---------------------------

extern void interp_segment_init(void);
extern int linear_vel_override(typ_interp_segment * modify_pointer);
// 线性速度修调 

extern void appoint_position(float position[], unsigned int axis_num);
 //从当前位置到达指定的位置
 //position[]指定要到达的位置，给出空间坐标
 //axis_num: 指定需要运动的轴号，0001-X,0002-Y,0004-Z,
 //出口：将上述运动转换为待插补直线段放入interpl_segment_insert段中                            

extern void insert_dwell_time(float time); //time's unit is second
extern void motion_abolish(void);

extern void code_seg_pre_treat(void);
extern void auto_key_echo(unsigned int key_value);
extern void auto_process(void);

void insert_segment(void);
void auto_ctrl(void) ;

#endif
