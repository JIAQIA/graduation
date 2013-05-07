//************************************************************ 
//ϵͳ���������Լ���������
//************************************************************ 
#ifndef _PARA_METER_CFG_H_
#define _PARA_METER_CFG_H_

#include "system_cfg.h"
//#include "cf_card_cfg.h"

//-------������������----------------------------------------
#define ST_TRUE         1
#define ST_FAULSE       0

//-------�ض�����Ϊ�������б�ı�׼----------------------
#define TRUNCATE_ERR    0.00001 

//-------���巢�������ö�ʱ���Ĳ�������-------------------
#define FREQCONST_MAX_F   (float)65530    //��ʱ����Ƶ���������ֵ
#define FREQCONST_MIN_F   (float)24

//-------���̻�������С����----------------------------------
#define KEY_BUFFER      0x05 


//------����μ�����صĳ�������-----------------------------
#define CODE_SEGMENT_SIZE    24    //ÿ������εĳ���(˫�ֽڣ�  0424�޸�  �����ǵ��ֽڼ���
#define CODE_BUF_LENGTH     120 //���뻺�����ĳ��ȣ�˫�ֽڣ�0405�޸ģ���110�ĳ�120��֪��Ϊʲôд����110
#define END_OF_CODE          0xFFFFFFFF 


//--------�ӹ�����--------------------------------------------
extern int ManuNum;
//------�Ƿ񿪼���--------------------------------------
extern char OpenLaser;

//--------��������--------------------------------------------
extern int SpotsprayPower;   //���书��
extern int SpotsprayMs;	  //����ʱ��(ms)
extern float SpotsprayMm;    //�㶯����(mm)
//-------mech code plan result define---------------------- 
typedef struct{          //�岹ʱ��ṹ����
     int motion_stat;         //�˶�״̬
     int motion_stat_next;    //��һ���ڵ��˶�״̬     
     
	 //�˶�ʱ��
     long term_cmd;          //���μƻ�����������
     long term_run;          //�Ѿ����е�������
     long term_rem;          //ʣ��岹���ڼ���������ֵ����term_cmd + term_add
  
     int term_add;           //����������
      
     float time_delta;       //�Ӽ�����ÿ�����ڵı仯�������ٶȣ���ʱ���ʾ��
     float time_length;      //�岹�õ���ʵ���ٶ�ֵ����ʱ���ʾ��
    }typ_interp_time; 
  
//-------mech code segment define------------------------
typedef struct{                    //����νṹ����
     unsigned long label;          //�α��
     int  basecmd;        //�������֣������֣� 0xff
     float axis_dist[3];    //��ǰ�θ��ᣨ��δ��ɵģ��˶��������з�����
     float dist;                   //��ǰ�οռ���λ�ƣ�mm������ʱʱ�䣨�룩���޷�����

     float vel;                    //�ϳɣ�����ٶȣ��ɳ���������޷�����    

     float axis_vel[3];     //ʵ���ٶ��ڸ���ķ���,�з�����,�õ����岹���ڵ�λ������ʾ(vel*speed_scal*INTERP_CYC)
     float dist_out[3];     //��ǰ�ε�ǰ���ڸ���ļƻ������,�з�����

     typ_interp_time interp_time;  //�岹ʱ������� 
     
     float axis_seg[3];
    }typ_interp_segment;         


//--------mechanical parameters define---------------
typedef struct{
     float max_acc[AXIS_SUM];            //��������������ٶ�
     float max_vel[AXIS_SUM];            //�������������ٶ�
     float pulse_cor[AXIS_SUM];          //��������嵱��

     float spindle_rev_acc;         //������������õļ��ٶ�    
     float spindle_rev_max;         //����ת�ٵ����ֵ

     float utmost_dimen[AXIS_SUM];  //������ļ��޳ߴ磨����ڻ���Բ�㣩
    }typ_mech_para;   

//---------user parameters define----------------------
typedef struct{
     float feed_vel;                 //�����ٶ�(Ĭ��ֵ)
     int  speed_scal;       //�˶��ٶ��޵�ϵ��
          
     float spindle_rev;              //�����ٶ�
     int  spindle_scal;     //����ת���޵�ϵ��     

     float swift_vel;                //�����ٶ�
     float slow_vel;                 //����ĵ㶯�ٶ�,���ڶ�λ����
     int spindle_on_time;            //��������ʱ��(ms)

     float tool_block_dim[AXIS_SUM]; //�Ե���ߴ�(z)
     float tool_set_pos[AXIS_SUM];   //�Ե���ɺ󵶾�ͣ��λ��(z),����ڶԵ���

     unsigned int override_echo_term; //��Ӧ�ٶ��޵���ʱ�䣨����������ʣ�����������ڸ�ֵ���޵�
     unsigned int blend_time;         //�����ת��ʱ��(������)

    }typ_user_para;

//--------mechine status parameter define---------------
typedef struct{
     int  stat;             //�豸����״̬ 
     int  error;            //����������
     int  comm_flag;
     int  code_buf_stat;    //code_permit/code_reject

     unsigned long label;            //�������α��
     int spindle_rev;                //����ת�٣�0��ֹͣ,��������ת,��������ת
     long dist_sum_act[AXIS_SUM];   //�����ڸ��������ʵ����λ������������
     float curr_posi[AXIS_SUM];      //��ǰ��������λ��
     float prog_orig[AXIS_SUM];      //����ԭ��ľ�������

     float dist_sum[AXIS_SUM];       //�����ڸ�������ļ�����λ������������
     float last_posi[AXIS_SUM];      //��һ��������� 

     float dist_tail[AXIS_SUM];      //���λ������β���Ĵ�������������      
 
     int pulse_out[AXIS_SUM];
     float dist_tail_a[3];
     //long act_pulse[AXIS_SUM];
     //long aim_pulse[AXIS_SUM];      //�ɼӹ��߶ξ��������� 
     int  speed_scal_old;   
     int  spindle_scal_old;
 }typ_mech_status;
 
       
//-------�����ļ�������ļ�ͷ�ṹ-------------------------------
typedef struct{
    unsigned int name[8];
    unsigned long length;
    unsigned long counter;  
   }typ_file_code;

//---------���ᶨ��������ʱָ������----------------------------
typedef struct{              
    unsigned int  axis_num;                  //���    
    float         distance;                  //����
  }typ_fixed;

//********************************************************

//********ͨ�ñ�������************************************
//-------------------------------------------------------
extern typ_file_code      file_code;    //���д����ļ�ͷ��ʽ
extern typ_fixed          fixed_dist;   //������������Ϣ�洢��ʽ

//-------������������-----------------------------------
extern typ_mech_para     mech_para;
extern typ_user_para     user_para;
extern typ_mech_status   mech_status;

//------����״̬�Ĵ���-----------------------------------
extern unsigned int mech_flag;

 #define CF_CARD_OK           0x0001 
 #define HAND_CTRL_OK         0x0002
 #define SPINDLE_ON           0x0004 
 
 #define TOOL_ADJUST          0x0010
 #define SIMULATION           0x0020
 #define OFF_LINE_WORK        0x0040
 

 #define MARGIN_SCAN_OFF      0x0100  //��λ���
 #define MARGIN_EXCEED        0x0200
 #define ROM_READ_ERR         0x0400
 #define ROM_WRITE_ERR        0x0800


 #define USB_COMM_READY       0x1000 
 #define STA_PARA_SET_OK      0x2000 
 #define CODE_DOWN_ENABLE     0x4000
 #define CODE_DOWN_FINISH     0x8000

 
//------���п��Ʊ�־�Ĵ���������ض���---------------------
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

//------�ⲿ�����߼�����Ĵ���---------------------------
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
//------�豸��ʶ��--------------------------------------
extern unsigned int  plant_ID[2];

//------���з�ʽ��־------------------------------------
extern unsigned int  ctrl_manner;

//------������˶������־�Ĵ���������ض���-------------
extern int axis_dir[AXIS_SUM];
 
 #define SET_POS           1
 #define SET_NEG           -1
 #define SET_STOP          0 

//------���������嶨ʱ�����趨�� -------------------------
extern unsigned int  freq_const[AXIS_SUM];
extern unsigned char residue[AXIS_SUM];
//------�����������������-------------------------------
extern unsigned int  pulse_out[AXIS_SUM];

//------�ٶ��޵�ϵ����ʱ����-----------------------------
extern unsigned int speed_scal;

//------�˶���ͣʱ�䣬���ݴ���Ҫ����logic_cmd_treat()�н�������
//extern float xdata dwell_time;              //��ͣʱ��

//**********************************************************************

//----------���̴�����-----------------------------------
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

//3.12���� void code_buf_init(void);
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