//**********************************************
#ifndef _KEY_CODE_DEF_H_
#define _KEY_CODE_DEF_H_

//**********************************************
//------�����������(�޲�������)------------------------
//һ�������,����ϵͳ������ʽ���趨----------------------
#define FIRST_CLASS          0xee    

 #define CMD_INIT            0xeb
 #define CMD_AUTO            0xea      
 #define CMD_MANU            0xe9
 
 #define STAT_FREE           0xe7

 #define RUN_MANNER_KEY      0xe7
//-------------------------------------------------------
 
 #define CMD_OFF_LINE_WORK   0xe5   //���߹��� 
 #define CMD_ON_LINE_WORK    0xe4   //���߹���
 
 #define CMD_STOP_INSTANCE   0xe3   //����ͣ��

 #define CMD_STATION_SCAN_EN 0xe2   //��λʹ��
 #define CMD_STATION_SCAN_QU 0xe1   //��λ����
 
 #define ASK_RECE_READY      0xe0   //ѯ�ʽ���״̬

 #define STAT_DATA_REQ       0xdf   //״̬��������(COM����)
 #define STAT_FLAG_REQ       0xde   //״̬��־���(COM����)

 #define STAT_DATA_REQUEST   0xdd   //״̬��������(USB����)
 #define STAT_FLAG_REQUIRE   0xdc   //״̬��־���(USB����)

 #define CMD_SET_HOME        0xda   //��Ϊ��еԭ��
 #define CMD_SET_PHOME       0xd9   //��Ϊ����ԭ��
 #define CMD_SAVE_POSITION   0xd8   //���浱ǰλ�� 

 #define CMD_TOOL_SPEED_EN   0xd7   //�Ե��ٶ�ʹ��
 #define CMD_TOOL_SPEED_QU   0xd6   //�Ե��ٶȳ��� 

 #define CMD_FIEL_PARA_LOAD  0xd5   //�����ֳ���������
 #define CMD_SET_PARA_LOAD   0xd4   //�û��趨��������
 #define CMD_DEF_PARA_LOAD   0xd3   //Ĭ�ϲ�������

 #define CMD_CURR_PARA_SAVE  0xd1   //��ǰ��������

 #define CMD_MECH_OFF        0xd0   //�����ر� 
//----------------------------------------------------------------------- 

//----------------------------------------------------------------------- 
//---------------------------------------------------
//��������������ڿ��ƻ������˶�-----------------------  
#define SECOND_CLASS      0xcf      
 //---���ڻ�����ʼ��--------------------
 #define CMD_BACK_HOME     0xcc      
 #define CMD_TOOL_X        0xcb
 #define CMD_TOOL_Y        0xca
 #define CMD_TOOL_Z        0xc9

#define MIN_INIT_KEY      0xc9

 //---�����Զ�����----------------------
 #define CMD_SIMU          0xc8 
 #define CMD_RUN           0xc7       
 #define CMD_PAUSE         0xc6
 #define CMD_STOP          0xc5
 #define CMD_ESTOP         0xc4

#define MIN_AUTO_KEY      0xc4

 //---�����ֶ�����------------------------
 #define CMD_GO_MECH_HOME  0xc3       //���ػ���ԭ��
 #define CMD_GO_PROG_HOME  0xc2       //���س���ԭ��
 #define CMD_GO_LAST_POS   0xc1       //���س���������һ��
 #define CMD_GO_SET_DIST   0xc0       //����ָ��λ��(��ź�λ����fixed_dist����ָ��)

 #define AXIS_XP           0xbf       //����X�������˶�
 #define AXIS_NXP          0xbe       //ֹͣX�������˶�
 #define AXIS_XN           0xbd       //����X�Ḻ���˶�
 #define AXIS_NXN          0xbc       //ֹͣX�������˶�

 #define AXIS_YP           0xbb       //����Y�������˶�
 #define AXIS_NYP          0xba       //ֹͣY�������˶�
 #define AXIS_YN           0xb9       //����Y�Ḻ���˶�
 #define AXIS_NYN          0xb8       //ֹͣY�������˶�

 #define AXIS_ZP           0xb7       //����Z�������˶�
 #define AXIS_NZP          0xb6       //ֹͣZ�������˶�
 #define AXIS_ZN           0xb5       //����Z�Ḻ���˶�
 #define AXIS_NZN          0xb4       //ֹͣZ�������˶�

 #define SPINDLE_CW        0xb3
 #define SPINDLE_CC        0xb2
 #define SPINDLE_OFF       0xb1

 #define AXIS_STOP         0xb0
  
#define MIN_MANU_KEY       0xb0

//--------------------------------------------------------
//���������
#define THIRD_CLASS          0xaf       //����������������,�������������С�ڸ�ֵ

 #define K_SPEED_INC         0xae       //�����޵�ϵ������
 #define K_SPEED_DEC         0xad       //�����޵�ϵ����С
 #define K_SPINDLE_INC       0xac       //�����޵�ϵ������
 #define K_SPINDLE_DEC       0xab       //�����޵�ϵ����С

 #define K_PAGE_UP           0xa7
 #define K_PAGE_DOWN         0xa6
 #define K_SELE_LEFT         0xa5
 #define K_SELE_RIGHT        0xa4

 #define PLANT_ID_SET        0xa1       //�����豸����
 
#define MIN_KEY_CODE         0xa0       //��С�ļ��룬С�ڸ�ֵ��Ϊ�Ǽ���
//------------------------------------------------
#define KEY_NULL             0x00       //�޼���*

//------------------------------------------------
//-------ͨѶ��Ϣ��ʶ��(������������)----------------
 #define EOT                   0xec   //ͨѶ������

 #define WORK_PARA_TRANS       0x90   //���в�������
   #define MECH_PARA_SET       0x91   //ϵͳĬ��������Ϣ
   #define USER_PARA_SET       0x92   //�û�������Ϣ
   #define FIELD_PARA_SET      0x93   //�����ֳ���Ϣ 
 
 #define MECH_CODE_TRANS       0x95   //�ӹ����봫��
 #define  CODE_DOWN_LOAD       0x94   //�ӹ��������� 


 #define SPEED_SET             0x96   //�ٶ��Լ��޵�ϵ������
 #define SPINDLE_SET           0x97   //����ת���Լ��޵�ϵ������  
 #define POSITION_SET          0x98   //ָ�����������е���;���(���fixed_dist�ṹ����)

//-------ͨѶ������-------------------------------- 
 #define CODE_REJECT           0x9a   //ֹͣ���뷢��  mech_status.code_buf_stat
 #define CODE_PERMIT           0x9b   //������뷢��
 
 //-------ͨѶ״̬���------------------------------------
 #define STA_COMM_WAITING      0x9c      //����ͨѶ mech_status.comu_flag
 #define STA_USB_COMM_OK       0x9d
 #define STA_USB_COMM_ERR      0x9e

 #define STA_TEST_REQUIRE      0x9f
 #define STA_PAUS_REQUIRE      0x8f
 #define STA_WAKE_REQUIRE      0x7f  
//********************************************************

//----------״̬��ʶ�붨��(����mech_status.stat)----------
//-------������ʼ���������״̬���-----------------------
 #define STA_HOME_START        0x70    //�ػ�еԭ�㿪ʼ
 #define STA_HOME_OK           0x71    //�ػ�еԭ�����
 #define STA_HOME_STOP         0x72    //�ػ�еԭ����ֹ

 #define STA_TOOLX_SET         0x73    //�Ե���ʼ
 #define STA_TOOLY_SET         0x74    //�Ե���ʼ
 #define STA_TOOLZ_SET         0x75    //�Ե���ʼ
 #define STA_TOOL_OK           0x76    //�Ե����
 #define STA_TOOL_STOP         0x77    //�Ե���ֹ

 #define STA_MECH_HOME         0x78    //�ڻ���ԭ��
// #define STA_PROG_HOME         0x79    //�ڳ���ԭ��

//-------�����Զ��������״̬����---------------------------
 #define STA_UNKNOWN           0x00
 #define STA_READY             0x80      //����׼����
 #define STA_WORKING           0x81      //���ڹ���״̬
 #define STA_PAUSING           0x82      //������ͣ
 #define STA_PAUSED            0x83      //������ͣ״̬ 
 #define STA_STOPPING          0x84      //����ֹͣ 
 #define STA_ESTOPPING         0x85      //���ڽ���ֹͣ״̬ 

//#define STA_SIMULATION        0x86
 #define STA_MANUAL            0x87
 
 #define STA_STOPPED           0x88      //����ͣ��״̬

 #define STA_BROKEN            0x8a      //�����ж� 
 #define STA_FINISHED          0x8b      //����ֹͣ

 #define STA_STOP_INSTANCE     0x8c
 
//-------������������ض���-----------------------------------    
 #define NO_ERROR         0x00
 #define SPEED_OVERFLOW   0x01
 
 #define STATION_EXCEED   0x21

 #define FILE_LOAD_ERROR  0x31

 #define PARA_METER_ERR   0x41
 
 #define MECH_CODE_ERROR  0x51
 
 #define ROM_READ_ERROR   0x61
 #define ROM_WRT_ERROR    0x62
 
 #define CF_CARD_ERROR    0x80
 #define CF_CARD_READ_ERR 0x81
 #define CF_CARD_WRT_ERR  0x82

//=====================================================================
//===============================================
//-------CNC�����ڲ����ƴ��� -----------------------------
 #define PROGRAM_END     99          //�������
 #define G_DWELL         0x08          //�˶���ͣ

 
//-------��׼���붨��-------------------------------------
 #define G_00            0x00
 #define G_01            0x01
 #define G_02            0x02
 #define G_03            0x03
 #define G_04            0x04

#endif

