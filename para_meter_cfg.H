/*****************************************************************************/
//�岹����ײ㺯���йض���
/*****************************************************************************/
#ifndef _PARA_METER_CFG_H_
#define _PARA_METER_CFG_H_

#include "system_cfg.h"
#include "cf_card_cfg.h"


//-------������������---------------------------------------
#define ST_TRUE         1
#define ST_FAULSE       0

//-------�ض�����Ϊ�������б�ı�׼--------------------
#define TRUNCATE_ERR     0.00001 

//------����μ�����صĳ�������--------------------------------
#define CODE_SEGMENT_SIZE    22    //ÿ������εĳ���
#define END_OF_CODE          0xFFFFFFFF 

#define DEFA_PARA_SECTOR     2    //Ĭ�ϲ�����������
#define USER_PARA_SECTOR     3    //�û����ò�����������
#define FIELD_PARA_SECTOR    4    //�����ֳ�״̬��������

#define CODE_BEGIN_SECTOR    5000    //�ӹ�������ʼ����

//-------�����ļ�������ļ�ͷ�ṹ-------------------------------------
typedef struct{
    unsigned int name[8];
    unsigned long length;
    unsigned long counter;  
   }typ_file_code;
   
//-------mech code plan result define---------------------- 
typedef struct{          //�岹ʱ��ṹ����
     unsigned int motion_stat;         //�˶�״̬
     unsigned int motion_stat_next;    //��һ���ڵ��˶�״̬     
     
	 //�˶�ʱ��
     long term_cmd;          //���μƻ�����������
     long term_run;          //�Ѿ����е�������
     long term_rem;          //ʣ��岹���ڼ���������ֵ����term_cmd + term_add
  
     unsigned int term_add;  //����������
      
     float time_delta;       //�Ӽ�����ÿ�����ڵı仯�������ٶȣ���ʱ���ʾ��
     float time_length;      //�岹�õ���ʵ���ٶ�ֵ����ʱ���ʾ��
    }typ_interp_time; 
  
//-------mech code segment define------------------------
typedef struct{                    //����νṹ����
     unsigned long label;          //�α��
     unsigned int  basecmd;        //�������֣������֣�
     float axis_dist[AXIS_SUM];    //��ǰ�θ��ᣨ��δ��ɵģ��˶��������з�����
     float dist;                   //��ǰ�οռ���λ�ƣ�mm������ʱʱ�䣨�룩���޷�����
    
     float vel;                    //�ϳɣ�����ٶȣ��ɳ���������޷�����    

     unsigned int nice_stop;      //TRUE-׼ͣ��FALSE-����ת��

     float axis_vel[AXIS_SUM];     //ʵ���ٶ��ڸ���ķ���,�з�����,�õ����岹���ڵ�λ������ʾ(vel*override_scal*INTERP_CYC)
     float dist_out[AXIS_SUM];     //��ǰ�ε�ǰ���ڸ���ļƻ������,�з�����
     
     typ_interp_time interp_time[3];  //�岹ʱ������� 
     
    }typ_interp_segment;         

//-------electronic parameters define---------------- 
typedef struct{
     int    pulse_timer_type;             //���巢����ʱ�������ͣ�1����1��ʱ���� 0����1��ʱ��
     int    pulse_act_type[AXIS_SUM];     //�������ŷ��������ϵ����÷�ʽ��0-��ʼ��Ч(���߼�)��1-�м���Ч(���߼�)
     float  servo_echo_delay[AXIS_SUM];   //�ŷ�װ�õ���Ӧ��ʱʱ�䣬��λΪ ��
     float  clk_per_term;                 //һ����������ʱ������ĸ�������MAIN_CLK*interp_cyc
    }typ_elec_para;               

//--------mechanical parameters define---------------
typedef struct{ 

     float max_acc[AXIS_SUM];       //���������������ٶ�
     float max_vel[AXIS_SUM];       //��������������ٶ�
     float pulse_cor[AXIS_SUM];     //��������嵱��

     float spindle_rev_acc;         //������������õļ��ٶ�    
     float spindle_rev_max;         //����ת�ٵ����ֵ

     float utmost_dimen[AXIS_SUM];  //������ļ��޳ߴ磨����ڻ���Բ�㣩
    
    }typ_mech_para;   

//---------user parameters define----------------------
typedef struct{
     float feed_vel;                 //�����ٶ�(Ĭ��ֵ)
     unsigned int  speed_scal;       //�˶��ٶ��޵�ϵ��
          
     float spindle_rev;              //�����ٶ�
     unsigned int  spindle_scal;     //����ת���޵�ϵ��     

     float swift_vel;                //�����ٶ�
     float slow_vel;                 //����ĵ㶯�ٶ�,���ڶ�λ����
     float spindle_on_time;          //��������ʱ��(s)

     float tool_block_dim[AXIS_SUM]; //�Ե���ߴ�(z)
     float tool_set_pos[AXIS_SUM];   //�Ե���ɺ󵶾�ͣ��λ��(z),����ڶԵ���

     long  override_echo_term;       //��Ӧ�ٶ��޵���ʱ�䣨����������ʣ�����������ڸ�ֵ���޵�

     unsigned int  speed_scal_old;   
     unsigned int  spindle_scal_old;
    }typ_user_para;

//--------mechine status parameter define---------------
typedef struct{
     unsigned int  stat;             //�豸����״̬ 
     unsigned int  error;            //����������

     unsigned long label;            //�������α��

     float sp_rev;                   //����ת��
     float dist_sum[AXIS_SUM];       //�����ڸ����������λ������������
     float curr_posi[AXIS_SUM];      //��ǰ��������λ��
     float prog_orig[AXIS_SUM];      //����ԭ��ľ�������

     float last_posi[AXIS_SUM];      //��һ��������� 
     float dist_tail[AXIS_SUM];      //���λ������β���Ĵ�������������      
 }typ_mech_status; 

//---------���ᶨ��������ʱָ������-----------------------
typedef struct{              
    unsigned int  axis_num;                  //���    
    float         distance;                  //����
  }type_fixed;
//****************************************************************
//*******USBͨѶ��ر���*********************************
extern unsigned int usb_rece_stat;
 #define USB_COMM_STOP        0x00
 #define USB_COMM_READY       0x31

 #define USB_MECH_CODE_LOAD   0x32
 #define USB_MECH_CODE_TRANS  0x33
 #define USB_WORK_PARA_RECE   0x34
 
extern unsigned int   comm_echo_back;
extern typ_file_code  file_code;


//********ͨ�ñ�������***********************************
extern type_fixed fixed_dist;  //������������Ϣ�洢��

//-------������������-----------------------------------
extern typ_elec_para     elec_para;
extern typ_mech_para     mech_para;
extern typ_user_para     user_para;
extern typ_mech_status   mech_status;

//------����״̬�Ĵ���-----------------------------------
extern unsigned int mech_flag;

 #define TOOL_ADJUST          0x0001
 #define STOP_MARGIN_SCAN     0x0002  //��λ���


 #define OFF_LINE_DO          0x0020
 #define HAND_CTRL_OK         0x0040
 #define CF_CARD_OK           0x0080 
 
 #define SPINDLE_ON           0x0100



//------���п��Ʊ�־�Ĵ���������ض���---------------------
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
 


//------������˶������־�Ĵ���������ض���-------------
extern char axis_dir[AXIS_SUM];
 #define POS             1                  
 #define NEG             -1
 #define STOP            0

//------���з�ʽ��־------------------------------------
extern unsigned char  ctrl_manner;
//------�����ڸ����������λ������������-------------------
//extern float xdata axis_out_sum[AXIS_SUM]; 

//------���������嶨ʱ�����趨�� -------------------------
extern unsigned int  freq_const[AXIS_SUM];

//------�����������������-------------------------------
extern unsigned int  pulse_out[AXIS_SUM];

//------�����ת��ʱ�䣬�ò岹��������ʾ-------------------
extern unsigned int  blend_term; 


//------�˶���ͣʱ�䣬���ݴ���Ҫ����logic_cmd_treat()�н�������
//extern float xdata dwell_time;              //��ͣʱ��


//**********************************************************************


//-------���ڼӹ�����------------------------------------------
extern unsigned int code_head_index;      //������е���ָ��
extern unsigned int code_tail_index;      //������е�βָ��
//extern unsigned char mech_code_queue[260]; //�ӹ��������
//extern unsigned char * mech_code_queue;
//------------------------------------------------------------
//**********************************************************
//��дCF��������������������壨����Ԫģ��ͨ�ã�
extern unsigned long sec_read_index;    //����������ָ��
extern unsigned long sec_wrt_index;  //����д����ָ��
extern unsigned int  code_rw_index;     //�����ڴ���ָ��
//extern unsigned char sector_buf[260]; //�������������� 
//extern unsigned char * sector_buf;

//------------function announce------------------------------------------
void field_para_save(void);
void system_para_load(unsigned char parameter_num);
//parameter_num:0-����CF��������
//              1-��CF��Ĭ�ϲ�������, 
//              2-��CF���û��趨��������,
//              3-��CF�������ֳ��������� 
//�ɹ�����1��ʧ�ܷ���0
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




 