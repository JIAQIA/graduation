#include "string.h"

#include "system_cfg.h"
#include "data_deal_def.h"
#include "At2404.h"
//#include "M25p16.h"		  //��usb�й�

#include "key_code_def.h"

#include "auto_ctrl_def.h"
//#include "manu_ctrl_def.h"

//-------------------------------------------------------
typ_file_code      file_code;    //���д����ļ�ͷ��ʽ
typ_fixed          fixed_dist;   //������������Ϣ�洢��ʽ

//-------������������-----------------------------------
typ_mech_para     mech_para;
typ_user_para     user_para;
typ_mech_status   mech_status;

//------�豸��ʶ��--------------------------------------
unsigned int plant_ID[2];

//------���з�ʽ��־------------------------------------
unsigned int ctrl_manner;

//------ͨѶ������--------------------------------------
unsigned int  comm_ctrl_flag;

//------�ӹ�����----------------------------------------
int ManuNum;
//------�Ƿ񿪼���--------------------------------------
char OpenLaser;
//--------��������--------------------------------------------
int SpotsprayPower;   //���书��
int SpotsprayMs;	  //����ʱ��(ms)
float SpotsprayMm;    //�㶯����(mm)

//------�豸״̬�Ĵ���-----------------------------------
unsigned int mech_flag;
 
//------���п��Ʊ�־�Ĵ���������ض���-------------------
unsigned int ctrl_flag; 

//------�ⲿ�����߼�����Ĵ���---------------------------
unsigned int stat_logic;

//------������˶������־�Ĵ���������ض���-------------
int  axis_dir[AXIS_SUM];

//------���������嶨ʱ�����趨�� ------------------------
unsigned int freq_const[AXIS_SUM];
unsigned char residue[AXIS_SUM];

//------�����������������-------------------------------
unsigned int pulse_out[AXIS_SUM];

//------�ٶ��޵�ϵ����ʱ����-----------------------------
unsigned int speed_scal;

//------�˶���ͣʱ�䣬���ݴ���Ҫ����logic_cmd_treat()�н�������
//float xdata dwell_time;              //��ͣʱ��

//************************************************************
//-------���ڶ�дFLASH--------------------------------
/*12��04.19����
long M25_rw_pointer=0;
*/
//04.19���
 int  Flash_rw_pointer=0;
  
//-------���ڼӹ�����-----------------------------------------
unsigned int *mech_code_index;      //�ӹ��������ָ��
unsigned int mech_code_count;       //�ӹ����������

//-------�������ݴ��뻺��-------------------------------------
unsigned int buf_write_index;

//3.14����#pragma DATA_SECTION(code_buffer1, ".ext_bss")
unsigned int code_buffer1[CODE_BUF_LENGTH];
 
//3.14����#pragma DATA_SECTION(code_buffer2, ".ext_bss")
unsigned int code_buffer2[CODE_BUF_LENGTH]; 

//-------���̻�������ر�������------------------------
//һ��(������ʽ)��������������βָ��
unsigned int key_head;
unsigned int key_tail;
unsigned int key_buffer[KEY_BUFFER+1]; 

//����(�˶�����)��������������βָ��
unsigned int key2_head;
unsigned int key2_tail;
unsigned int key2_buffer[KEY_BUFFER+1];

//------------------------------------------------------------

//*******��������*********************************************
/*3.12����
//���뱣��----------------------------------------------
void mech_code_save_FL(void) 
{ 
 if(ctrl_flag & CODE_BUF1_FULL)
   {
    M25_wrt_data(M25_rw_pointer,CODE_BUF_LENGTH*2,(unsigned int *)code_buffer1);
    M25_rw_pointer += 256;
	ctrl_flag &= ~CODE_BUF1_FULL;                          
	ctrl_flag |= CODE_BUF1_EMPTY; 
    mech_status.code_buf_stat = CODE_PERMIT;           
   }
 else if(ctrl_flag & CODE_BUF2_FULL) 
   {
	M25_wrt_data(M25_rw_pointer,CODE_BUF_LENGTH*2,(unsigned int *)code_buffer2);
    M25_rw_pointer += 256;
    ctrl_flag &= ~CODE_BUF2_FULL;
	ctrl_flag |= CODE_BUF2_EMPTY; 
	mech_status.code_buf_stat = CODE_PERMIT;
   } 
 else {;}
 //-------------------------------------------------------------        
 if(mech_flag & CODE_DOWN_FINISH)
   {
    mech_flag &= ~CODE_DOWN_ENABLE;
    mech_flag &= ~CODE_DOWN_FINISH;
    mech_status.code_buf_stat = CODE_REJECT;
   } 
}
//-----------------------------------------------------------
//----�����ȡ-----------------------------------------------
void mech_code_load_FL(void)
{
 if(ctrl_flag & CODE_BUF1_EMPTY)
   {
    M25_read_data(M25_rw_pointer,CODE_BUF_LENGTH*2,(unsigned int *)code_buffer1);
    M25_rw_pointer += 256;     
    ctrl_flag &= ~CODE_BUF1_EMPTY;                   
    ctrl_flag |= CODE_BUF1_FULL;
    ctrl_flag |= CODE_BUF_READY; 
   }    
 else if(ctrl_flag & CODE_BUF2_EMPTY)
   {
    M25_read_data(M25_rw_pointer,CODE_BUF_LENGTH*2,(unsigned int *)code_buffer2);
    M25_rw_pointer += 256;   
    ctrl_flag &= ~CODE_BUF2_EMPTY;                   
    ctrl_flag |= CODE_BUF2_FULL;  
    ctrl_flag |= CODE_BUF_READY; 
   } 
 else{;}

}
//-----------------------------------------------------------
//�������ؿ�ʼ����
//-----------------------------------------------------------
void data_rece_init(unsigned int *buffer)
{
 memcpy((unsigned int *)&file_code, buffer,10);
 file_code.counter = 0;
 //-------------------------------------------
 if(file_code.name[0] == CODE_DOWN_LOAD)
   {
    M25_bulk_erase();  
    M25_rw_pointer = 0; 
    mech_flag |= CODE_DOWN_ENABLE;
    mech_flag &= ~CODE_DOWN_FINISH; 
   } 
}         
//-----------------------------------------------------------               
//�������ؽ�������
//-----------------------------------------------------------
int data_rece_right(void)
{
 if(file_code.counter == file_code.length) return(1);
 else return(0);
}
//------------------------------------------------------------
void code_rece_finish(void)
{
 if(ctrl_flag & CODE_BUF1_EMPTY)
   { 
    ctrl_flag &= ~CODE_BUF1_EMPTY;
    ctrl_flag |= CODE_BUF1_FULL;
   }                   
 else if(ctrl_flag & CODE_BUF2_EMPTY)
   { 
    ctrl_flag &= ~CODE_BUF2_EMPTY;
    ctrl_flag |= CODE_BUF2_FULL;
   }
 else return;
  
 buf_write_index = 0;
 ctrl_flag |= CODE_BUF_READY;   
 mech_status.code_buf_stat = CODE_REJECT;  
 //-------------------------------------------------
 if(mech_flag & CODE_DOWN_ENABLE) mech_flag |= CODE_DOWN_FINISH;  //---
}
*/
//***********************************************************************
//�ӹ�����ļ��ش�����غ���
//***********************************************************************
//������г�ʼ��
//-----------------------------------------------------------------------
void code_buf_init(void)
{
 mech_code_count = 0;      //������е���ָ��

 ctrl_flag &= ~CODE_BUF1_FULL;	  //��ctrl_flag��ΪCODE_BUF1_EMPTY
 ctrl_flag &= ~CODE_BUF2_FULL;
 ctrl_flag |= CODE_BUF1_EMPTY;
 ctrl_flag &= ~CODE_BUF2_EMPTY;
 
 ctrl_flag &= ~CODE_BUF_READY; 	  //ctrl_flagδ׼����,��������û�д���
 mech_code_count = CODE_BUF_LENGTH+1;
 buf_write_index = 0;
 mech_status.code_buf_stat = CODE_PERMIT;
 //-------------------------------------------------
 Flash_rw_pointer = 0;
}


//***********************************************************************
// ����: ��������뻺����
// ���:                                                               
// ����ֵ: 1-���뻺��������0-���뻺����δ��                                                            
// ע��:
//***********************************************************************
int code_buf_fill(unsigned int input_buf[], int byte_num)
{
 file_code.counter += byte_num;
 
 byte_num = byte_num/2;
 
 if(ctrl_flag & CODE_BUF1_EMPTY)
   {
    memcpy((unsigned int *)&code_buffer1[buf_write_index], (unsigned int *)input_buf, byte_num);
    buf_write_index += byte_num;
    if(buf_write_index >= CODE_BUF_LENGTH)
      {
       buf_write_index = 0;
       ctrl_flag &= ~CODE_BUF1_EMPTY;                   
       ctrl_flag |= CODE_BUF1_FULL;
       ctrl_flag |= CODE_BUF_READY;   
       mech_status.code_buf_stat = CODE_REJECT;                          
       return(1);
      }
    else return(0);
   }   
 else if(ctrl_flag & CODE_BUF2_EMPTY)
   {
    memcpy((unsigned int *)&code_buffer2[buf_write_index], (unsigned int *)input_buf, byte_num);
    buf_write_index += byte_num;
    if(buf_write_index >= CODE_BUF_LENGTH)
      {
       buf_write_index = 0;
       ctrl_flag &= ~CODE_BUF2_EMPTY;                   
       ctrl_flag |= CODE_BUF2_FULL;  
       ctrl_flag |= CODE_BUF_READY;   
       mech_status.code_buf_stat = CODE_REJECT;                              
       return(1);
      }
    else return(0);
   }   
 else
   {
    mech_status.code_buf_stat = CODE_REJECT;     
    return(1);
   } 
}

//*********************************************************************
// ����: ����μ���                                                  
// ���:                                                             
// ���:0��δ���س���Σ�1�����µĳ���μ��ص�interpl_segment��                                                           
// ע��: ��������seg_labelָ���ĳ���ν��н��ͣ��˶����������ص�interpl_segment����
//       �����ؿ��������ص�plc_buffer�У������˶���ʱʱ����ص�dwell_termҲ���԰����ڴˣ�
//*********************************************************************
int mech_code_load(typ_interp_segment *interpl_segment) 
{ 
  //----�жϴ�������Ƿ�Ϊ��---------------------------------------
	if(mech_code_count >= CODE_BUF_LENGTH)  //��������Ѿ�����CODE_BUFFER��ѱ�־λ���
	  {
		if(ctrl_flag & CODE_BUF1_FULL)      
		 {
		  mech_code_index = (unsigned int *)code_buffer1;
		  mech_code_count = 0;
		  ctrl_flag &= ~CODE_BUF1_FULL;     //                    
		  ctrl_flag |= CODE_BUF2_EMPTY; 
		  mech_status.code_buf_stat = CODE_PERMIT;           
		 }
		else if(ctrl_flag & CODE_BUF2_FULL) 
		 {
		  mech_code_index = (unsigned int *)code_buffer2;
		  mech_code_count = 0;
		  ctrl_flag &= ~CODE_BUF2_FULL;
		  ctrl_flag |= CODE_BUF1_EMPTY; 
		  mech_status.code_buf_stat = CODE_PERMIT;
		 } 
		else
		 {
		  ctrl_flag &= ~CODE_BUF_READY;
		  mech_status.code_buf_stat = CODE_PERMIT;
		  return(0);
		 }   
     }
   //----�δ�����أ���code_buffer�����24�ֽڵ����ݴ洢��interpl_segment�---------------------------------------------------- 
 	memcpy((unsigned int *)interpl_segment, (unsigned int *)mech_code_index,CODE_SEGMENT_SIZE);
	mech_code_index += CODE_SEGMENT_SIZE/4;  //�ض�λ��ʼ��ַ ÿ4���ֽڹ���һ����Ч���� 5-13
	mech_code_count += CODE_SEGMENT_SIZE;
	//---------------------------------------------------------------       
  return(1);  
}
//**********************************************************************
//������ϵͳ��������
//��ڣ��������ͺţ�1��Ĭ�ϲ�����2���û����ò�����3��
//����ֵ��
//ע�ͣ����øú���ǰ���뽫��ز������뻺����code_buffer1[]
//**********************************************************************
void system_para_dispatch(int para_type) 
{
 if(para_type == MECH_PARA_SET)
   {
    memcpy((unsigned int *)&mech_para, (unsigned int *)code_buffer1,sizeof(mech_para));
   }
 else if(para_type == USER_PARA_SET)
   {
    memcpy((unsigned int *)&user_para, (unsigned int *)code_buffer1,sizeof(user_para));

    //user_para.override_echo_term = 500; ///= INTERP_CYC_MS; //��Ӧ�ٶ��޵���ʱ�䣨����������ʣ�����������ڸ�ֵ���޵�
    //user_para.blend_time =1; ///= INTERP_CYC_MS;
    speed_scal = user_para.speed_scal;
   }
 else{;}
}
/*3.12����  
//------------------------------------------------------------
void system_para_save(int para_type)
{
 if(para_type == MECH_PARA_SET)
  {
   if(ROM_write(MECH_PARA_ADDR,(unsigned int *)&mech_para,sizeof(mech_para)*2))
     {
      mech_flag &= ~ROM_WRITE_ERR;
      mech_status.error = ROM_WRT_ERROR;
     }
   else mech_flag |= ROM_WRITE_ERR;  
  }
 else if(para_type == USER_PARA_SET)
  {
   if(ROM_write(USER_PARA_ADDR,(unsigned int *)&user_para,sizeof(user_para)*2))
     {
      mech_flag &= ~ROM_WRITE_ERR;
      mech_status.error = ROM_WRT_ERROR;
     }
   else mech_flag |= ROM_WRITE_ERR;  
  }
 else {;}
}

//------------------------------------------------------------
void system_para_load(int para_type)
{
 if(para_type == MECH_PARA_SET)
  {
   if(ROM_read(MECH_PARA_ADDR,(unsigned int *)code_buffer1, sizeof(mech_para)*2))
     {
      mech_flag &= ~ROM_READ_ERR;
      mech_status.error = ROM_READ_ERROR;
     }
   else mech_flag |= ROM_READ_ERR;
  }
 else if(para_type == USER_PARA_SET)
  {
   if(ROM_read(USER_PARA_ADDR,(unsigned int *)code_buffer1, sizeof(user_para)*2))
     {
      mech_flag &= ~ROM_READ_ERR;
      mech_status.error = ROM_READ_ERROR;
     }
   else mech_flag |= ROM_READ_ERR;  
  }
 else {;}
}

//------------------------------------------------------------
void read_plant_ID(unsigned int *ID_code)
{
 if(ROM_read(PLANT_ID_ADDR,ID_code,4)) mech_flag &= ~ROM_READ_ERR;
 else mech_flag |= ROM_READ_ERR;
 return;
}

void write_plant_ID(unsigned int *ID_code)
{
 if(ROM_write(PLANT_ID_ADDR,ID_code,4)) mech_flag &= ~ROM_WRITE_ERR;
 else mech_flag |= ROM_WRITE_ERR;
 return;
}
*/
//-------------------------------------------------------------
//��д��������
void system_para_load(int para_type)
{
 if(para_type == MECH_PARA_SET)      //��е�������أ�112���ֽڼ��أ����飩
  {
   //if(ROM_read(MECH_PARA_ADDR,(unsigned int *)code_buffer1, sizeof(mech_para)*2))  //��ROM���������Ҫ��USB�������
     if(ReadPara(0x01,(unsigned int *)code_buffer1))
     {
      mech_flag &= ~ROM_READ_ERR;
      mech_status.error = ROM_READ_ERROR;
     }
   else mech_flag |= ROM_READ_ERR;
  }
 else if(para_type == USER_PARA_SET)   //�û���������
  {
   //if(ROM_read(USER_PARA_ADDR,(unsigned int *)code_buffer1, sizeof(user_para)*2))
     if(ReadPara(0x00,(unsigned int *)code_buffer1))
     {
      mech_flag &= ~ROM_READ_ERR;
      mech_status.error = ROM_READ_ERROR;
     }
   else mech_flag |= ROM_READ_ERR;  
  }
 else {;}
}
//----------------------------------------------------------------------
//��е������ʼ��
void mech_para_init(void)   
 {
  mech_para.spindle_rev_acc = 100;          //������������õļ��ٶ�    
  mech_para.spindle_rev_max = 30000;        //����ת�ٵ����/��Сֵ

  mech_para.max_acc[AXIS_X] = 100.0;  //��������������ٶ�
  mech_para.max_acc[AXIS_Y] = 100.0;  //��������������ٶ�
  mech_para.max_acc[AXIS_Z] = 100.0;  //��������������ٶ�

  mech_para.max_vel[AXIS_X] = 100.0;  //�������������ٶ�
  mech_para.max_vel[AXIS_Y] = 100.0;  //�������������ٶ�
  mech_para.max_vel[AXIS_Z] = 100.0;  //�������������ٶ�

  mech_para.pulse_cor[AXIS_X] = 0.003125;  //��������嵱��
  mech_para.pulse_cor[AXIS_Y] = 0.003125; 
  mech_para.pulse_cor[AXIS_Z] = 0.003125; 
  
  mech_para.utmost_dimen[AXIS_X] = 300.0;  //������ļ��޳ߴ磨mm,����ڻ���Բ�㣩
  mech_para.utmost_dimen[AXIS_Y] = 300.0;  //������ļ��޳ߴ磨mm,����ڻ���Բ�㣩
  mech_para.utmost_dimen[AXIS_Z] = 100.0;  //������ļ��޳ߴ磨mm,����ڻ���Բ�㣩
 }                  
//---------------------------------------------------------------------
// �û�������ʼ��
void user_para_init(void)   
{
 user_para.speed_scal = 100;       //�˶��ٶ��޵�ϵ��
 user_para.spindle_scal = 100;     //����ת���޵�ϵ��
 
 speed_scal = user_para.speed_scal;
 
 user_para.swift_vel = 20.0;   //35     //���ٶ�λ�ٶ�(Ĭ��ֵ)
 user_para.feed_vel = 15.0;    //30     //�����ٶ�(Ĭ��ֵ)
 user_para.slow_vel = 5.0;         //����ĵ㶯�ٶ�,���ڶ�λ����
   
 user_para.spindle_rev = 30000;             //�����ٶ�
 user_para.spindle_on_time = 1000;          //��������ʱ��(ms)

 user_para.override_echo_term = 3000;    //��Ӧ�Զ��ٶ��޵���ʣ��������
 user_para.blend_time = 1;

 user_para.tool_block_dim[AXIS_X] = 10.0;        //�Ե���ߴ�(x)
 user_para.tool_block_dim[AXIS_Y] = 10.0;        //�Ե���ߴ�(y)
 user_para.tool_block_dim[AXIS_Z] = 10.0;        //�Ե���ߴ�(z)

 user_para.tool_set_pos[AXIS_X] = 2.0;      //�Ե���ɺ󵶾�ͣ��λ��(x),����ڶԵ���
 user_para.tool_set_pos[AXIS_Y] = 2.0;      //�Ե���ɺ󵶾�ͣ��λ��(y),����ڶԵ���
 user_para.tool_set_pos[AXIS_Z] = 2.0;      //�Ե���ɺ󵶾�ͣ��λ��(z),����ڶԵ���
}

//****************************************************************************

//״̬������ʼ��
void mech_status_init(void)
{
 unsigned int index;

 mech_status.stat = STA_MANUAL; //�豸����״̬ 
 mech_status.error = NO_ERROR;     //����������
 mech_status.comm_flag = STA_COMM_WAITING;
 mech_status.code_buf_stat = CODE_REJECT; 
 mech_status.label = 0;          //�������α��
 mech_status.spindle_rev = 0;
 mech_status.speed_scal_old = 100;   
 mech_status.spindle_scal_old = 100;
 
 for(index = 0; index < AXIS_SUM; index ++)
    {
     mech_status.curr_posi[index] = 0.0;          //����λ��
     mech_status.prog_orig[index] = 0.0;    //����ԭ������
     mech_status.last_posi[index] = 0.0;   //��һ���������    
     mech_status.dist_tail[index] = 0.0;      //���λ������β���Ĵ�������������     
     mech_status.dist_sum[index] = 0.0;
     mech_status.dist_sum_act[index] = 0;
     mech_status.pulse_out[index] = 0;
     mech_status.dist_tail_a[index] = 0.0;
    }
}
//******************************************************************
//����������ʼ��
void public_para_init(int type_ll)   
{
 int index;
 
 for(index = 0; index < AXIS_SUM; index ++)
    {    
     axis_dir[index] = SET_STOP;
     pulse_out[index] = 0;  
     freq_const[index] = 4000;
     mech_status.dist_sum[index] = 0.0;
     mech_status.dist_sum_act[index] = 0;
     mech_status.pulse_out[index] = 0;
     mech_status.dist_tail[index] = 0.0;   //ע��˴�����ȥ��         
     mech_status.dist_tail_a[index] = 0.0;
    }
 mech_status.spindle_rev = 0; 

 ctrl_flag = 0x0000;
 mech_flag &= ~USB_COMM_READY;
 
 if(type_ll)
   {
    for(index = 0; index < AXIS_SUM; index ++) mech_status.curr_posi[index] = 0.0;		 //��ǰ��������λ��
   }
}
//-------------------------------------------------------------------
void margin_check(void)
{
 int axis_idx;
 int ct_flag = 0;
 
 for(axis_idx=0;axis_idx<AXIS_SUM;axis_idx++)
  { 
   mech_status.dist_sum_act[axis_idx] += mech_status.pulse_out[axis_idx];
   mech_status.pulse_out[axis_idx] = 0;
   mech_status.curr_posi[axis_idx] = mech_status.dist_sum_act[axis_idx] * mech_para.pulse_cor[axis_idx];

   if(mech_status.curr_posi[axis_idx] > mech_para.utmost_dimen[axis_idx]) ct_flag++;
   if(mech_status.curr_posi[axis_idx] < -mech_para.utmost_dimen[axis_idx]) ct_flag++;
  }
   
 if(ct_flag == 0)
   {
    mech_flag &= ~MARGIN_EXCEED;  //δ�����ӹ���Ե
   }    
 else
   {    
    //if(mech_flag & MARGIN_SCAN_OFF) mech_flag &= ~MARGIN_EXCEED; 
    //else  mech_flag |= MARGIN_EXCEED;
    mech_flag |= MARGIN_EXCEED;
   } 
}
//*****************************************************************
//���������浱ǰλ������
//��ڣ�
//����ֵ��
//˵����
//*****************************************************************
void save_position_to(float position_buf[])
{
 unsigned int index;
 for(index = 0; index < AXIS_SUM; index ++)  
    {
     position_buf[index] = mech_status.curr_posi[index];
    }
}
//*****************************************************************
//�������жϵ�ǰλ��
//��ڣ�Ŀ��λ��
//����ֵ�������ǰλ��λ��Ŀ��λ���򷵻�1�����򷵻�0
//ע�ͣ�
//*****************************************************************
int position_at(float aim_position[])
{
 unsigned int index;
 float length;

 length = 0.0;

 for(index = 0; index<AXIS_SUM; index++)
    {
     length += fabs(aim_position[index] - mech_status.curr_posi[index]);
    }
 if(length <= TRUNCATE_ERR) return(1);
 else return(0);
}
//******************************************************************
//�������ж��Ƿ����˶�״̬
//��ڣ�
//����ֵ��1�����˶���0��ֹͣ
//ע�ͣ�
//******************************************************************
int being_motion(void)
{
 int index;
 
 for(index = 0; index<AXIS_SUM; index++)
    {
     if(pulse_out[index]) return(1);
    }
 return(0);   
}
//----------------------------------------------------
void delay_ms(int time_ms)
{
	unsigned int j;
	
	if(time_ms <=0) return;
	while(time_ms--)
	   {
	    for(j=0;j<10000;j++);
	   } 
}
//***************************************************************
// ����: ���̽ӿں���                                          
// ���:                                                       
// ����ֵ:                                                    
// ע��: key_buffer[]��ż�ֵ,key_head,key_tail�Ǽ�ָ��    
//***************************************************************
void key_buffer_init(void)
{
 key_head = 0;
 key_tail = 0;
 key2_head = 0;
 key2_tail = 0;
}
//--------------------------------------------------------------
unsigned int key_read(void)
{
  unsigned int key_value;

  if(key_head == key_tail) key_value = 0x00;  //key_buffer empty
  else
    {
     key_value = key_buffer[key_head];
     key_head ++;
     if(key_head >= KEY_BUFFER) key_head = 0;
    }
  return(key_value);
}
//-----------------------------------------------------------------
int key_write(unsigned int key_value)
{
  unsigned int pointer;

  pointer = key_tail + 1;
  if(pointer >= KEY_BUFFER) pointer = 0;
  if(pointer == key_head) return(0x00);  // key_buffer full
  else
    {
     key_buffer[key_tail] = key_value;
     key_tail = pointer;
     return(0x01);
    }
 }

//***************************************************************
unsigned int key2_read(void)
{
  unsigned int key_value;

  if(key2_head == key2_tail) key_value = 0x00;  //key_buffer empty
  else
    {
     key_value = key2_buffer[key2_head];
     key2_head ++;
     if(key2_head >= KEY_BUFFER) key2_head = 0;
    }
  return(key_value);
}
//-----------------------------------------------------------------
int key2_write(unsigned int key_value)
{
  unsigned int pointer;

  pointer = key2_tail + 1;
  if(pointer >= KEY_BUFFER) pointer = 0;
  if(pointer == key2_head) return(0x00);  // key_buffer full
  else
    {
     key2_buffer[key2_tail] = key_value;
     key2_tail = pointer;
     return(0x01);
    }
 }
 
/*****************************************************************/
/* ����: ��ʱ����Ƶֵ�Լ�ʵ������λ��������                      */
/* ���:                                                         */
/* ����ֵ:                                                       */
/* ע��:                                                         */
/*****************************************************************/
void freq_data_cal(int axis_num)
{
 float temp;

 //����Ƶ�ʼ���
 mech_status.dist_tail_a[axis_num] += mech_status.dist_sum[axis_num];
  //temp = mech_status.dist_sum[axis_num];
 
 temp = mech_status.dist_tail_a[axis_num];


 
 if(temp >= mech_para.pulse_cor[axis_num])
   {
    axis_dir[axis_num] = SET_POS;
   }
 else if(temp <= -mech_para.pulse_cor[axis_num])
   {
    axis_dir[axis_num] = SET_NEG;
    temp = -temp;
   }
 else     //���벻�㣬ֱ�ӷ���
   {
    pulse_out[axis_num] = 0;
    axis_dir[axis_num] = SET_STOP;
    mech_status.pulse_out[axis_num] = 0;
    return;   
   }
 //���ھ����㹻�ĶΣ����㶨ʱ���ķ�Ƶ��ֵ��ע�����ֵΪnʱ��ʵ�ʷ�Ƶ��Ϊn+1
 temp = CLK_PER_TERM * mech_para.pulse_cor[axis_num] / temp;
 
 if(temp > FREQCONST_MAX_F)
   {
    temp = FREQCONST_MAX_F;
   }
 else if(temp <= FREQCONST_MIN_F)
   {
    temp = FREQCONST_MIN_F;
   }
 else{;}
 
  //����ʵ��λ����(������)
  //The old algorithm
  //pulse_out[axis_num] = (unsigned int)(CLK_PER_TERM / temp + 0.5);  
  pulse_out[axis_num] = (unsigned int)(CLK_PER_TERM / temp + 0.5); 
  
  temp = CLK_PER_TERM / pulse_out[axis_num] - 1;
  freq_const[axis_num] = (unsigned int)temp;
  residue[axis_num] = (unsigned char)((temp - freq_const[axis_num])*256);  
  
  //freq_const[axis_num] = (unsigned long)(CLK_PER_TERM / pulse_out[axis_num]); // -1;
  //residue[axis_num] = CLK_PER_TERM_D % pulse_out[axis_num];
  //freq_const[axis_num] = (unsigned int)(CLK_PER_TERM / pulse_out[axis_num]) +1;
 
  //����ʵ�����λ�����Լ�����
  mech_status.pulse_out[axis_num] = pulse_out[axis_num] * axis_dir[axis_num];

  temp = mech_status.pulse_out[axis_num] * mech_para.pulse_cor[axis_num];
  mech_status.dist_tail_a[axis_num] -= temp;
     
  return;
} 
//********************************************************************
/*------------------------4.19��д----------------------------*/
//��U�����ȡ�ӹ������code_buffer1��code_buffer2
void mech_code_load_FL(void)
{
 if(ctrl_flag & CODE_BUF1_EMPTY)
   {
    Code_read_data(Flash_rw_pointer,CODE_BUF_LENGTH,(unsigned int *)code_buffer1);   //һ�ζ��������//ʮ��
    Flash_rw_pointer +=120;      //��λ��ȡ�����λ�� 4.26�޸�
    ctrl_flag &= ~CODE_BUF1_EMPTY;                   
    ctrl_flag |= CODE_BUF1_FULL;
    ctrl_flag |= CODE_BUF_READY; 
   }    
 else if(ctrl_flag & CODE_BUF2_EMPTY)
   {
    Code_read_data(Flash_rw_pointer,CODE_BUF_LENGTH,(unsigned int *)code_buffer2);
    Flash_rw_pointer +=120;          // ��λ��ȡ�����λ��
    ctrl_flag &= ~CODE_BUF2_EMPTY;                   
    ctrl_flag |= CODE_BUF2_FULL;  
    ctrl_flag |= CODE_BUF_READY; 
   } 
 else{;}
}
