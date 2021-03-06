#include "string.h"

#include "system_cfg.h"
#include "data_deal_def.h"
#include "At2404.h"
//#include "M25p16.h"		  //与usb有关

#include "key_code_def.h"

#include "auto_ctrl_def.h"
//#include "manu_ctrl_def.h"

//-------------------------------------------------------
typ_file_code      file_code;    //串行传输文件头格式
typ_fixed          fixed_dist;   //定长度运行信息存储格式

//-------工作参数变量-----------------------------------
typ_mech_para     mech_para;
typ_user_para     user_para;
typ_mech_status   mech_status;

//------设备标识字--------------------------------------
unsigned int plant_ID[2];

//------运行方式标志------------------------------------
unsigned int ctrl_manner;

//------通讯控制字--------------------------------------
unsigned int  comm_ctrl_flag;

//------加工数量----------------------------------------
int ManuNum;
//------是否开激光--------------------------------------
char OpenLaser;
//--------点射配置--------------------------------------------
int SpotsprayPower;   //点射功率
int SpotsprayMs;	  //点射时间(ms)
float SpotsprayMm;    //点动距离(mm)

//------设备状态寄存器-----------------------------------
unsigned int mech_flag;
 
//------运行控制标志寄存器及其相关定义-------------------
unsigned int ctrl_flag; 

//------外部开关逻辑输入寄存器---------------------------
unsigned int stat_logic;

//------各轴的运动方向标志寄存器及其相关定义-------------
int  axis_dir[AXIS_SUM];

//------周期内脉冲定时器的设定置 ------------------------
unsigned int freq_const[AXIS_SUM];
unsigned char residue[AXIS_SUM];

//------周期内输出的脉冲数-------------------------------
unsigned int pulse_out[AXIS_SUM];

//------速度修调系数临时备份-----------------------------
unsigned int speed_scal;

//------运动暂停时间，根据代码要求在logic_cmd_treat()中进行设置
//float xdata dwell_time;              //暂停时间

//************************************************************
//-------用于读写FLASH--------------------------------
/*12年04.19屏蔽
long M25_rw_pointer=0;
*/
//04.19添加
 int  Flash_rw_pointer=0;
  
//-------用于加工处理-----------------------------------------
unsigned int *mech_code_index;      //加工代码队列指针
unsigned int mech_code_count;       //加工代码计数器

//-------用于数据代码缓存-------------------------------------
unsigned int buf_write_index;

//3.14屏蔽#pragma DATA_SECTION(code_buffer1, ".ext_bss")
unsigned int code_buffer1[CODE_BUF_LENGTH];
 
//3.14屏蔽#pragma DATA_SECTION(code_buffer2, ".ext_bss")
unsigned int code_buffer2[CODE_BUF_LENGTH]; 

//-------键盘缓冲区相关变量定义------------------------
//一级(工作方式)按键缓冲区及首尾指针
unsigned int key_head;
unsigned int key_tail;
unsigned int key_buffer[KEY_BUFFER+1]; 

//二级(运动控制)按键缓冲区及首尾指针
unsigned int key2_head;
unsigned int key2_tail;
unsigned int key2_buffer[KEY_BUFFER+1];

//------------------------------------------------------------

//*******函数定义*********************************************
/*3.12屏蔽
//代码保存----------------------------------------------
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
//----代码读取-----------------------------------------------
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
//数据下载开始处理
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
//数据下载结束处理
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
//加工代码的加载处理相关函数
//***********************************************************************
//代码队列初始化
//-----------------------------------------------------------------------
void code_buf_init(void)
{
 mech_code_count = 0;      //代码队列的首指针

 ctrl_flag &= ~CODE_BUF1_FULL;	  //将ctrl_flag置为CODE_BUF1_EMPTY
 ctrl_flag &= ~CODE_BUF2_FULL;
 ctrl_flag |= CODE_BUF1_EMPTY;
 ctrl_flag &= ~CODE_BUF2_EMPTY;
 
 ctrl_flag &= ~CODE_BUF_READY; 	  //ctrl_flag未准备好,即缓冲区没有代码
 mech_code_count = CODE_BUF_LENGTH+1;
 buf_write_index = 0;
 mech_status.code_buf_stat = CODE_PERMIT;
 //-------------------------------------------------
 Flash_rw_pointer = 0;
}


//***********************************************************************
// 函数: 填充程序代码缓冲区
// 入口:                                                               
// 返回值: 1-代码缓冲区满，0-代码缓冲区未满                                                            
// 注释:
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
// 函数: 程序段加载                                                  
// 入口:                                                             
// 返�:0－未加载程序段，1－将新的程序段加载到interpl_segment中                                                           
// 注释: 本函数将seg_label指定的程序段进行解释，运动控制量加载到interpl_segment段中
//       （开关控制量加载到plc_buffer中，并将运动延时时间加载到dwell_term也可以安排在此）
//*********************************************************************
int mech_code_load(typ_interp_segment *interpl_segment) 
{ 
  //----判断代码队列是否为空---------------------------------------
	if(mech_code_count >= CODE_BUF_LENGTH)  //如果代码已经读进CODE_BUFFER则把标志位清空
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
   //----段代码加载（把code_buffer里面的24字节的数据存储到interpl_segment里）---------------------------------------------------- 
 	memcpy((unsigned int *)interpl_segment, (unsigned int *)mech_code_index,CODE_SEGMENT_SIZE);
	mech_code_index += CODE_SEGMENT_SIZE/4;  //重定位起始地址 每4个字节构成一个有效数据 5-13
	mech_code_count += CODE_SEGMENT_SIZE;
	//---------------------------------------------------------------       
  return(1);  
}
//**********************************************************************
//函数：系统参数加载
//入口：参数类型号：1－默认参数，2－用户配置参数，3－
//返回值：
//注释：调用该函数前必须将相关参数置入缓冲区code_buffer1[]
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

    //user_para.override_echo_term = 500; ///= INTERP_CYC_MS; //响应速度修调的时间（周期数），剩余周期数低于该值不修调
    //user_para.blend_time =1; ///= INTERP_CYC_MS;
    speed_scal = user_para.speed_scal;
   }
 else{;}
}
/*3.12屏蔽  
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
//重写参数加载
void system_para_load(int para_type)
{
 if(para_type == MECH_PARA_SET)      //机械参数加载，112个字节加载（两遍）
  {
   //if(ROM_read(MECH_PARA_ADDR,(unsigned int *)code_buffer1, sizeof(mech_para)*2))  //从ROM里面读，我要从USB里面读。
     if(ReadPara(0x01,(unsigned int *)code_buffer1))
     {
      mech_flag &= ~ROM_READ_ERR;
      mech_status.error = ROM_READ_ERROR;
     }
   else mech_flag |= ROM_READ_ERR;
  }
 else if(para_type == USER_PARA_SET)   //用户参数加载
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
//机械参数初始化
void mech_para_init(void)   
 {
  mech_para.spindle_rev_acc = 100;          //主轴变速所采用的加速度    
  mech_para.spindle_rev_max = 30000;        //主轴转速的最大/最小值

  mech_para.max_acc[AXIS_X] = 100.0;  //各轴允许的最大加速度
  mech_para.max_acc[AXIS_Y] = 100.0;  //各轴允许的最大加速度
  mech_para.max_acc[AXIS_Z] = 100.0;  //各轴允许的最大加速度

  mech_para.max_vel[AXIS_X] = 100.0;  //各轴允许的最大速度
  mech_para.max_vel[AXIS_Y] = 100.0;  //各轴允许的最大速度
  mech_para.max_vel[AXIS_Z] = 100.0;  //各轴允许的最大速度

  mech_para.pulse_cor[AXIS_X] = 0.003125;  //各轴的脉冲当量
  mech_para.pulse_cor[AXIS_Y] = 0.003125; 
  mech_para.pulse_cor[AXIS_Z] = 0.003125; 
  
  mech_para.utmost_dimen[AXIS_X] = 300.0;  //各轴向的极限尺寸（mm,相对于机床圆点）
  mech_para.utmost_dimen[AXIS_Y] = 300.0;  //各轴向的极限尺寸（mm,相对于机床圆点）
  mech_para.utmost_dimen[AXIS_Z] = 100.0;  //各轴向的极限尺寸（mm,相对于机床圆点）
 }                  
//---------------------------------------------------------------------
// 用户参数初始化
void user_para_init(void)   
{
 user_para.speed_scal = 100;       //运动速度修调系数
 user_para.spindle_scal = 100;     //主轴转速修调系数
 
 speed_scal = user_para.speed_scal;
 
 user_para.swift_vel = 20.0;   //35     //快速定位速度(默认值)
 user_para.feed_vel = 15.0;    //30     //工进速度(默认值)
 user_para.slow_vel = 5.0;         //各轴的点动速度,用于定位操作
   
 user_para.spindle_rev = 30000;             //主轴速度
 user_para.spindle_on_time = 1000;          //主轴启动时间(ms)

 user_para.override_echo_term = 3000;    //响应自动速度修调的剩余周期数
 user_para.blend_time = 1;

 user_para.tool_block_dim[AXIS_X] = 10.0;        //对刀块尺寸(x)
 user_para.tool_block_dim[AXIS_Y] = 10.0;        //对刀块尺寸(y)
 user_para.tool_block_dim[AXIS_Z] = 10.0;        //对刀块尺寸(z)

 user_para.tool_set_pos[AXIS_X] = 2.0;      //对刀完成后刀具停放位置(x),相对于对刀块
 user_para.tool_set_pos[AXIS_Y] = 2.0;      //对刀完成后刀具停放位置(y),相对于对刀块
 user_para.tool_set_pos[AXIS_Z] = 2.0;      //对刀完成后刀具停放位置(z),相对于对刀块
}

//****************************************************************************

//状态变量初始化
void mech_status_init(void)
{
 unsigned int index;

 mech_status.stat = STA_MANUAL; //设备运行状态 
 mech_status.error = NO_ERROR;     //错误类型字
 mech_status.comm_flag = STA_COMM_WAITING;
 mech_status.code_buf_stat = CODE_REJECT; 
 mech_status.label = 0;          //程序程序段标号
 mech_status.spindle_rev = 0;
 mech_status.speed_scal_old = 100;   
 mech_status.spindle_scal_old = 100;
 
 for(index = 0; index < AXIS_SUM; index ++)
    {
     mech_status.curr_posi[index] = 0.0;          //坐标位置
     mech_status.prog_orig[index] = 0.0;    //程序原点坐标
     mech_status.last_posi[index] = 0.0;   //上一点绝对坐标    
     mech_status.dist_tail[index] = 0.0;      //输出位移量的尾数寄存器，带符号数     
     mech_status.dist_sum[index] = 0.0;
     mech_status.dist_sum_act[index] = 0;
     mech_status.pulse_out[index] = 0;
     mech_status.dist_tail_a[index] = 0.0;
    }
}
//******************************************************************
//公共变量初始化
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
     mech_status.dist_tail[index] = 0.0;   //注意此处不可去掉         
     mech_status.dist_tail_a[index] = 0.0;
    }
 mech_status.spindle_rev = 0; 

 ctrl_flag = 0x0000;
 mech_flag &= ~USB_COMM_READY;
 
 if(type_ll)
   {
    for(index = 0; index < AXIS_SUM; index ++) mech_status.curr_posi[index] = 0.0;		 //当前绝对坐标位置
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
    mech_flag &= ~MARGIN_EXCEED;  //未超过加工边缘
   }    
 else
   {    
    //if(mech_flag & MARGIN_SCAN_OFF) mech_flag &= ~MARGIN_EXCEED; 
    //else  mech_flag |= MARGIN_EXCEED;
    mech_flag |= MARGIN_EXCEED;
   } 
}
//*****************************************************************
//函数：保存当前位置坐标
//入口：
//返回值：
//说明：
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
//函数：判断当前位置
//入口：目标位置
//返回值：如果当前位置位于目标位置则返回1，否则返回0
//注释：
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
//函数：判断是否处于运动状态
//入口：
//返回值：1－在运动，0－停止
//注释：
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
// 函数: 键盘接口函数                                          
// 入口:                                                       
// 返回值:                                                    
// 注释: key_buffer[]存放键值,key_head,key_tail是键指针    
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
/* 函数: 定时器分频值以及实际周期位移量计算                      */
/* 入口:                                                         */
/* 返回值:                                                       */
/* 注释:                                                         */
/*****************************************************************/
void freq_data_cal(int axis_num)
{
 float temp;

 //进行频率计算
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
 else     //距离不足，直接返回
   {
    pulse_out[axis_num] = 0;
    axis_dir[axis_num] = SET_STOP;
    mech_status.pulse_out[axis_num] = 0;
    return;   
   }
 //对于距离足够的段，计算定时器的分频数值，注意计算值为n时，实际分频数为n+1
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
 
  //计算实际位移量(脉冲数)
  //The old algorithm
  //pulse_out[axis_num] = (unsigned int)(CLK_PER_TERM / temp + 0.5);  
  pulse_out[axis_num] = (unsigned int)(CLK_PER_TERM / temp + 0.5); 
  
  temp = CLK_PER_TERM / pulse_out[axis_num] - 1;
  freq_const[axis_num] = (unsigned int)temp;
  residue[axis_num] = (unsigned char)((temp - freq_const[axis_num])*256);  
  
  //freq_const[axis_num] = (unsigned long)(CLK_PER_TERM / pulse_out[axis_num]); // -1;
  //residue[axis_num] = CLK_PER_TERM_D % pulse_out[axis_num];
  //freq_const[axis_num] = (unsigned int)(CLK_PER_TERM / pulse_out[axis_num]) +1;
 
  //计算实际输出位移量以及余数
  mech_status.pulse_out[axis_num] = pulse_out[axis_num] * axis_dir[axis_num];

  temp = mech_status.pulse_out[axis_num] * mech_para.pulse_cor[axis_num];
  mech_status.dist_tail_a[axis_num] -= temp;
     
  return;
} 
//********************************************************************
/*------------------------4.19重写----------------------------*/
//从U盘里读取加工代码给code_buffer1和code_buffer2
void mech_code_load_FL(void)
{
 if(ctrl_flag & CODE_BUF1_EMPTY)
   {
    Code_read_data(Flash_rw_pointer,CODE_BUF_LENGTH,(unsigned int *)code_buffer1);   //一次读进来五段//十段
    Flash_rw_pointer +=120;      //定位读取代码的位置 4.26修改
    ctrl_flag &= ~CODE_BUF1_EMPTY;                   
    ctrl_flag |= CODE_BUF1_FULL;
    ctrl_flag |= CODE_BUF_READY; 
   }    
 else if(ctrl_flag & CODE_BUF2_EMPTY)
   {
    Code_read_data(Flash_rw_pointer,CODE_BUF_LENGTH,(unsigned int *)code_buffer2);
    Flash_rw_pointer +=120;          // 定位读取代码的位置
    ctrl_flag &= ~CODE_BUF2_EMPTY;                   
    ctrl_flag |= CODE_BUF2_FULL;  
    ctrl_flag |= CODE_BUF_READY; 
   } 
 else{;}
}
