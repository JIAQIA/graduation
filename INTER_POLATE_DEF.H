/*****************************************************************************/
//�岹����ײ㺯���йض���
/*****************************************************************************/
#ifndef _interpola_def_h_
#define _interpola_def_h_

#include "system_cfg.h"
#include "data_deal_def.h"

//-------���Բ岹����С����--------------------------------------
#define LINE_MIN_DIST    0.0001  
#define ARC_MIN_ANGLE    0.001 

#define PI                3.1415926
#define PI_2              6.2831853 

//****************���������е���ֵ����*******************
//  0x00-0x3f����ȫ��ģ��ĳ�������(general_def.h)
//  0x40-0x4f���ڵ������ģ��ĳ���(axis_ctrl_sub.c)
//  0x50-0x5f���ڶ������ģ��ĳ���(interpola_sub.c)
//  0x60-0x6f���ڸ�λ����ģ��ĳ���(mech_init_ctrl.c)
//  0x70-0x7f���ڴ���ͨѶģ���Լ�����

//------�岹�м����״̬�ĳ�������(use for motion_stat)------------
#define INP_MOTION_ACC   0x53         //��������
#define INP_MOTION_UNI   0x52         //��������
#define INP_MOTION_DEC   0x51         //�������� 
#define INP_MOTION_END   0x50         //�˶�ֹͣ
//------�����״̬��ʶ---------------------------------------------
//interpl_segment.basecmd == PROGRAM_IDLE ��ʾ���δ������
 #define PROGRAM_IDLE    0xFF         
 
//********variable announcement************************************

//------------function announce------------------------------------------
// �����ٶȿ���
extern void linear_vel_ctrl(typ_interp_time * interp_time);

// ֱ�߲岹����                                             
extern void line_interp_deal(typ_interp_segment * interpl_segment);
extern void arc_interp_deal(typ_interp_segment * interpl_segment);
 
// ֱ�߲岹����滮 
extern void line_interp_plan(typ_interp_segment * interpl_segment);
extern void arc_interp_plan(typ_interp_segment * interpl_segment);

// ����ι滮����                                          
extern int segment_plan(typ_interp_segment *interpl_segment);

// ����μӹ�����
extern void segment_process(typ_interp_segment *interpl_segment);

//**********************************************************

#endif
//********************************************************** 