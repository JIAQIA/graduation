#ifndef _AUTO_CTRL_DEF_H_
#define _AUTO_CTRL_DEF_H_

//************************************************************/
#include "data_deal_def.h"

//--------�������(unsigned int axis_num)---------------------------
//���ں���appoint_position(float position[], unsigned int axis_num)
#define X_AXIS       0x0001
#define Y_AXIS       0x0002
#define Z_AXIS       0x0004
#define XYZ_AXIS     0x0007

//-------�ӹ�����α�������-----------------------------------------
extern typ_interp_segment *interpl_segment_prev;
extern typ_interp_segment *interpl_segment_curr;
extern typ_interp_segment *interpl_segment_next;
extern typ_interp_segment *interpl_segment_insert;

extern typ_interp_segment *temp_segment_pointer;

//**********************************************************

//------function announcement---------------------------

extern void interp_segment_init(void);
extern int linear_vel_override(typ_interp_segment * modify_pointer);
// �����ٶ��޵� 

extern void appoint_position(float position[], unsigned int axis_num);
 //�ӵ�ǰλ�õ���ָ����λ��
 //position[]ָ��Ҫ�����λ�ã������ռ�����
 //axis_num: ָ����Ҫ�˶�����ţ�0001-X,0002-Y,0004-Z,
 //���ڣ��������˶�ת��Ϊ���岹ֱ�߶η���interpl_segment_insert����                            

extern void insert_dwell_time(float time); //time's unit is second
extern void motion_abolish(void);

extern void code_seg_pre_treat(void);
extern void auto_key_echo(unsigned int key_value);
extern void auto_process(void);

void insert_segment(void);
void auto_ctrl(void) ;

#endif