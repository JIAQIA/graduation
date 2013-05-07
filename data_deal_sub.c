#include "string.h"

#include "system_cfg.h"
#include "data_deal_def.h"
#include "At2404.h"
//#include "M25p16.h"		  //ÓëusbÓĞ¹Ø

#include "key_code_def.h"

#include "auto_ctrl_def.h"
//#include "manu_ctrl_def.h"

//-------------------------------------------------------
typ_file_code      file_code;    //´®ĞĞ´«ÊäÎÄ¼şÍ·¸ñÊ½
typ_fixed          fixed_dist;   //¶¨³¤¶ÈÔËĞĞĞÅÏ¢´æ´¢¸ñÊ½

//-------¹¤×÷²ÎÊı±äÁ¿-----------------------------------
typ_mech_para     mech_para;
typ_user_para     user_para;
typ_mech_status   mech_status;

//------Éè±¸±êÊ¶×Ö--------------------------------------
unsigned int plant_ID[2];

//------ÔËĞĞ·½Ê½±êÖ¾------------------------------------
unsigned int ctrl_manner;

//------Í¨Ñ¶¿ØÖÆ×Ö--------------------------------------
unsigned int  comm_ctrl_flag;

//------¼Ó¹¤ÊıÁ¿----------------------------------------
int ManuNum;
//------ÊÇ·ñ¿ª¼¤¹â--------------------------------------
char OpenLaser;
//--------µãÉäÅäÖÃ--------------------------------------------
int SpotsprayPower;   //µãÉä¹¦ÂÊ
int SpotsprayMs;	  //µãÉäÊ±¼ä(ms)
float SpotsprayMm;    //µã¶¯¾àÀë(mm)

//------Éè±¸×´Ì¬¼Ä´æÆ÷-----------------------------------
unsigned int mech_flag;
 
//------ÔËĞĞ¿ØÖÆ±êÖ¾¼Ä´æÆ÷¼°ÆäÏà¹Ø¶¨Òå-------------------
unsigned int ctrl_flag; 

//------Íâ²¿¿ª¹ØÂß¼­ÊäÈë¼Ä´æÆ÷---------------------------
unsigned int stat_logic;

//------¸÷ÖáµÄÔË¶¯·½Ïò±êÖ¾¼Ä´æÆ÷¼°ÆäÏà¹Ø¶¨Òå-------------
int  axis_dir[AXIS_SUM];

//------ÖÜÆÚÄÚÂö³å¶¨Ê±Æ÷µÄÉè¶¨ÖÃ ------------------------
unsigned int freq_const[AXIS_SUM];
unsigned char residue[AXIS_SUM];

//------ÖÜÆÚÄÚÊä³öµÄÂö³åÊı-------------------------------
unsigned int pulse_out[AXIS_SUM];

//------ËÙ¶ÈĞŞµ÷ÏµÊıÁÙÊ±±¸·İ-----------------------------
unsigned int speed_scal;

//------ÔË¶¯ÔİÍ£Ê±¼ä£¬¸ù¾İ´úÂëÒªÇóÔÚlogic_cmd_treat()ÖĞ½øĞĞÉèÖÃ
//float xdata dwell_time;              //ÔİÍ£Ê±¼ä

//************************************************************
//-------ÓÃÓÚ¶ÁĞ´FLASH--------------------------------
/*12Äê04.19ÆÁ±Î
long M25_rw_pointer=0;
*/
//04.19Ìí¼Ó
 int  Flash_rw_pointer=0;
  
//-------ÓÃÓÚ¼Ó¹¤´¦Àí-----------------------------------------
unsigned int *mech_code_index;      //¼Ó¹¤´úÂë¶ÓÁĞÖ¸Õë
unsigned int mech_code_count;       //¼Ó¹¤´úÂë¼ÆÊıÆ÷

//-------ÓÃÓÚÊı¾İ´úÂë»º´æ-------------------------------------
unsigned int buf_write_index;

//3.14ÆÁ±Î#pragma DATA_SECTION(code_buffer1, ".ext_bss")
unsigned int code_buffer1[CODE_BUF_LENGTH];
 
//3.14ÆÁ±Î#pragma DATA_SECTION(code_buffer2, ".ext_bss")
unsigned int code_buffer2[CODE_BUF_LENGTH]; 

//-------¼üÅÌ»º³åÇøÏà¹Ø±äÁ¿¶¨Òå------------------------
//Ò»¼¶(¹¤×÷·½Ê½)°´¼ü»º³åÇø¼°Ê×Î²Ö¸Õë
unsigned int key_head;
unsigned int key_tail;
unsigned int key_buffer[KEY_BUFFER+1]; 

//¶ş¼¶(ÔË¶¯¿ØÖÆ)°´¼ü»º³åÇø¼°Ê×Î²Ö¸Õë
unsigned int key2_head;
unsigned int key2_tail;
unsigned int key2_buffer[KEY_BUFFER+1];

//------------------------------------------------------------

//*******º¯Êı¶¨Òå*********************************************
/*3.12ÆÁ±Î
//´úÂë±£´æ----------------------------------------------
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
//----´úÂë¶ÁÈ¡-----------------------------------------------
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
//Êı¾İÏÂÔØ¿ªÊ¼´¦Àí
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
//Êı¾İÏÂÔØ½áÊø´¦Àí
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
//¼Ó¹¤´úÂëµÄ¼ÓÔØ´¦ÀíÏà¹Øº¯Êı
//***********************************************************************
//´úÂë¶ÓÁĞ³õÊ¼»¯
//-----------------------------------------------------------------------
void code_buf_init(void)
{
 mech_code_count = 0;      //´úÂë¶ÓÁĞµÄÊ×Ö¸Õë

 ctrl_flag &= ~CODE_BUF1_FULL;	  //½«ctrl_flagÖÃÎªCODE_BUF1_EMPTY
 ctrl_flag &= ~CODE_BUF2_FULL;
 ctrl_flag |= CODE_BUF1_EMPTY;
 ctrl_flag &= ~CODE_BUF2_EMPTY;
 
 ctrl_flag &= ~CODE_BUF_READY; 	  //ctrl_flagÎ´×¼±¸ºÃ,¼´»º³åÇøÃ»ÓĞ´úÂë
 mech_code_count = CODE_BUF_LENGTH+1;
 buf_write_index = 0;
 mech_status.code_buf_stat = CODE_PERMIT;
 //-------------------------------------------------
 Flash_rw_pointer = 0;
}


//***********************************************************************
// º¯Êı: Ìî³ä³ÌĞò´úÂë»º³åÇø
// Èë¿Ú:                                                               
// ·µ»ØÖµ: 1-´úÂë»º³åÇøÂú£¬0-´úÂë»º³åÇøÎ´Âú                                                            
// ×¢ÊÍ:
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
// º¯Êı: ³ÌĞò¶Î¼ÓÔØ                                                  
// Èë¿Ú:                                                             
// ·µµ:0£­Î´¼ÓÔØ³ÌĞò¶Î£¬1£­½«ĞÂµÄ³ÌĞò¶Î¼ÓÔØµ½interpl_segmentÖĞ                                                           
// ×¢ÊÍ: ±¾º¯Êı½«seg_labelÖ¸¶¨µÄ³ÌĞò¶Î½øĞĞ½âÊÍ£¬ÔË¶¯¿ØÖÆÁ¿¼ÓÔØµ½interpl_segment¶ÎÖĞ
//       £¨¿ª¹Ø¿ØÖÆÁ¿¼ÓÔØµ½plc_bufferÖĞ£¬²¢½«ÔË¶¯ÑÓÊ±Ê±¼ä¼ÓÔØµ½dwell_termÒ²¿ÉÒÔ°²ÅÅÔÚ´Ë£©
//*********************************************************************
int mech_code_load(typ_interp_segment *interpl_segment) 
{ 
  //----ÅĞ¶Ï´úÂë¶ÓÁĞÊÇ·ñÎª¿Õ---------------------------------------
	if(mech_code_count >= CODE_BUF_LENGTH)  //Èç¹û´úÂëÒÑ¾­¶Á½øCODE_BUFFERÔò°Ñ±êÖ¾Î»Çå¿Õ
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
   //----¶Î´úÂë¼ÓÔØ£¨°Ñcode_bufferÀïÃæµÄ24×Ö½ÚµÄÊı¾İ´æ´¢µ½interpl_segmentÀï£©---------------------------------------------------- 
 	memcpy((unsigned int *)interpl_segment, (unsigned int *)mech_code_index,CODE_SEGMENT_SIZE);
	mech_code_index += CODE_SEGMENT_SIZE/4;  //ÖØ¶¨Î»ÆğÊ¼µØÖ· Ã¿4¸ö×Ö½Ú¹¹³ÉÒ»¸öÓĞĞ§Êı¾İ 5-13
	mech_code_count += CODE_SEGMENT_SIZE;
	//---------------------------------------------------------------       
  return(1);  
}
//**********************************************************************
//º¯Êı£ºÏµÍ³²ÎÊı¼ÓÔØ
//Èë¿Ú£º²ÎÊıÀàĞÍºÅ£º1£­Ä¬ÈÏ²ÎÊı£¬2£­ÓÃ»§ÅäÖÃ²ÎÊı£¬3£­
//·µ»ØÖµ£º
//×¢ÊÍ£ºµ÷ÓÃ¸Ãº¯ÊıÇ°±ØĞë½«Ïà¹Ø²ÎÊıÖÃÈë»º³åÇøcode_buffer1[]
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

    //user_para.override_echo_term = 500; ///= INTERP_CYC_MS; //ÏìÓ¦ËÙ¶ÈĞŞµ÷µÄÊ±¼ä£¨ÖÜÆÚÊı£©£¬Ê£ÓàÖÜÆÚÊıµÍÓÚ¸ÃÖµ²»ĞŞµ÷
    //user_para.blend_time =1; ///= INTERP_CYC_MS;
    speed_scal = user_para.speed_scal;
   }
 else{;}
}
/*3.12ÆÁ±Î  
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
//ÖØĞ´²ÎÊı¼ÓÔØ
void system_para_load(int para_type)
{
 if(para_type == MECH_PARA_SET)      //»úĞµ²ÎÊı¼ÓÔØ£¬112¸ö×Ö½Ú¼ÓÔØ£¨Á½±é£©
  {
   //if(ROM_read(MECH_PARA_ADDR,(unsigned int *)code_buffer1, sizeof(mech_para)*2))  //´ÓROMÀïÃæ¶Á£¬ÎÒÒª´ÓUSBÀïÃæ¶Á¡£
     if(ReadPara(0x01,(unsigned int *)code_buffer1))
     {
      mech_flag &= ~ROM_READ_ERR;
      mech_status.error = ROM_READ_ERROR;
     }
   else mech_flag |= ROM_READ_ERR;
  }
 else if(para_type == USER_PARA_SET)   //ÓÃ»§²ÎÊı¼ÓÔØ
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
//»úĞµ²ÎÊı³õÊ¼»¯
void mech_para_init(void)   
 {
  mech_para.spindle_rev_acc = 100;          //Ö÷Öá±äËÙËù²ÉÓÃµÄ¼ÓËÙ¶È    
  mech_para.spindle_rev_max = 30000;        //Ö÷Öá×ªËÙµÄ×î´ó/×îĞ¡Öµ

  mech_para.max_acc[AXIS_X] = 100.0;  //¸÷ÖáÔÊĞíµÄ×î´ó¼ÓËÙ¶È
  mech_para.max_acc[AXIS_Y] = 100.0;  //¸÷ÖáÔÊĞíµÄ×î´ó¼ÓËÙ¶È
  mech_para.max_acc[AXIS_Z] = 100.0;  //¸÷ÖáÔÊĞíµÄ×î´ó¼ÓËÙ¶È

  mech_para.max_vel[AXIS_X] = 100.0;  //¸÷ÖáÔÊĞíµÄ×î´óËÙ¶È
  mech_para.max_vel[AXIS_Y] = 100.0;  //¸÷ÖáÔÊĞíµÄ×î´óËÙ¶È
  mech_para.max_vel[AXIS_Z] = 100.0;  //¸÷ÖáÔÊĞíµÄ×î´óËÙ¶È

  mech_para.pulse_cor[AXIS_X] = 0.003125;  //¸÷ÖáµÄÂö³åµ±Á¿
  mech_para.pulse_cor[AXIS_Y] = 0.003125; 
  mech_para.pulse_cor[AXIS_Z] = 0.003125; 
  
  mech_para.utmost_dimen[AXIS_X] = 300.0;  //¸÷ÖáÏòµÄ¼«ÏŞ³ß´ç£¨mm,Ïà¶ÔÓÚ»ú´²Ô²µã£©
  mech_para.utmost_dimen[AXIS_Y] = 300.0;  //¸÷ÖáÏòµÄ¼«ÏŞ³ß´ç£¨mm,Ïà¶ÔÓÚ»ú´²Ô²µã£©
  mech_para.utmost_dimen[AXIS_Z] = 100.0;  //¸÷ÖáÏòµÄ¼«ÏŞ³ß´ç£¨mm,Ïà¶ÔÓÚ»ú´²Ô²µã£©
 }                  
//---------------------------------------------------------------------
// ÓÃ»§²ÎÊı³õÊ¼»¯
void user_para_init(void)   
{
 user_para.speed_scal = 100;       //ÔË¶¯ËÙ¶ÈĞŞµ÷ÏµÊı
 user_para.spindle_scal = 100;     //Ö÷Öá×ªËÙĞŞµ÷ÏµÊı
 
 speed_scal = user_para.speed_scal;
 
 user_para.swift_vel = 20.0;   //35     //¿ìËÙ¶¨Î»ËÙ¶È(Ä¬ÈÏÖµ)
 user_para.feed_vel = 15.0;    //30     //¹¤½øËÙ¶È(Ä¬ÈÏÖµ)
 user_para.slow_vel = 5.0;         //¸÷ÖáµÄµã¶¯ËÙ¶È,ÓÃÓÚ¶¨Î»²Ù×÷
   
 user_para.spindle_rev = 30000;             //Ö÷ÖáËÙ¶È
 user_para.spindle_on_time = 1000;          //Ö÷ÖáÆô¶¯Ê±¼ä(ms)

 user_para.override_echo_term = 3000;    //ÏìÓ¦×Ô¶¯ËÙ¶ÈĞŞµ÷µÄÊ£ÓàÖÜÆÚÊı
 user_para.blend_time = 1;

 user_para.tool_block_dim[AXIS_X] = 10.0;        //¶Ôµ¶¿é³ß´ç(x)
 user_para.tool_block_dim[AXIS_Y] = 10.0;        //¶Ôµ¶¿é³ß´ç(y)
 user_para.tool_block_dim[AXIS_Z] = 10.0;        //¶Ôµ¶¿é³ß´ç(z)

 user_para.tool_set_pos[AXIS_X] = 2.0;      //¶Ôµ¶Íê³Éºóµ¶¾ßÍ£·ÅÎ»ÖÃ(x),Ïà¶ÔÓÚ¶Ôµ¶¿é
 user_para.tool_set_pos[AXIS_Y] = 2.0;      //¶Ôµ¶Íê³Éºóµ¶¾ßÍ£·ÅÎ»ÖÃ(y),Ïà¶ÔÓÚ¶Ôµ¶¿é
 user_para.tool_set_pos[AXIS_Z] = 2.0;      //¶Ôµ¶Íê³Éºóµ¶¾ßÍ£·ÅÎ»ÖÃ(z),Ïà¶ÔÓÚ¶Ôµ¶¿é
}

//****************************************************************************

//×´Ì¬±äÁ¿³õÊ¼»¯
void mech_status_init(void)
{
 unsigned int index;

 mech_status.stat = STA_MANUAL; //Éè±¸ÔËĞĞ×´Ì¬ 
 mech_status.error = NO_ERROR;     //´íÎóÀàĞÍ×Ö
 mech_status.comm_flag = STA_COMM_WAITING;
 mech_status.code_buf_stat = CODE_REJECT; 
 mech_status.label = 0;          //³ÌĞò³ÌĞò¶Î±êºÅ
 mech_status.spindle_rev = 0;
 mech_status.speed_scal_old = 100;   
 mech_status.spindle_scal_old = 100;
 
 for(index = 0; index < AXIS_SUM; index ++)
    {
     mech_status.curr_posi[index] = 0.0;          //×ø±êÎ»ÖÃ
     mech_status.prog_orig[index] = 0.0;    //³ÌĞòÔ­µã×ø±ê
     mech_status.last_posi[index] = 0.0;   //ÉÏÒ»µã¾ø¶Ô×ø±ê    
     mech_status.dist_tail[index] = 0.0;      //Êä³öÎ»ÒÆÁ¿µÄÎ²Êı¼Ä´æÆ÷£¬´ø·ûºÅÊı     
     mech_status.dist_sum[index] = 0.0;
     mech_status.dist_sum_act[index] = 0;
     mech_status.pulse_out[index] = 0;
     mech_status.dist_tail_a[index] = 0.0;
    }
}
//******************************************************************
//¹«¹²±äÁ¿³õÊ¼»¯
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
     mech_status.dist_tail[index] = 0.0;   //×¢Òâ´Ë´¦²»¿ÉÈ¥µô         
     mech_status.dist_tail_a[index] = 0.0;
    }
 mech_status.spindle_rev = 0; 

 ctrl_flag = 0x0000;
 mech_flag &= ~USB_COMM_READY;
 
 if(type_ll)
   {
    for(index = 0; index < AXIS_SUM; index ++) mech_status.curr_posi[index] = 0.0;		 //µ±Ç°¾ø¶Ô×ø±êÎ»ÖÃ
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
    mech_flag &= ~MARGIN_EXCEED;  //Î´³¬¹ı¼Ó¹¤±ßÔµ
   }    
 else
   {    
    //if(mech_flag & MARGIN_SCAN_OFF) mech_flag &= ~MARGIN_EXCEED; 
    //else  mech_flag |= MARGIN_EXCEED;
    mech_flag |= MARGIN_EXCEED;
   } 
}
//*****************************************************************
//º¯Êı£º±£´æµ±Ç°Î»ÖÃ×ø±ê
//Èë¿Ú£º
//·µ»ØÖµ£º
//ËµÃ÷£º
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
//º¯Êı£ºÅĞ¶Ïµ±Ç°Î»ÖÃ
//Èë¿Ú£ºÄ¿±êÎ»ÖÃ
//·µ»ØÖµ£ºÈç¹ûµ±Ç°Î»ÖÃÎ»ÓÚÄ¿±êÎ»ÖÃÔò·µ»Ø1£¬·ñÔò·µ»Ø0
//×¢ÊÍ£º
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
//º¯Êı£ºÅĞ¶ÏÊÇ·ñ´¦ÓÚÔË¶¯×´Ì¬
//Èë¿Ú£º
//·µ»ØÖµ£º1£­ÔÚÔË¶¯£¬0£­Í£Ö¹
//×¢ÊÍ£º
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
// º¯Êı: ¼üÅÌ½Ó¿Úº¯Êı                                          
// Èë¿Ú:                                                       
// ·µ»ØÖµ:                                                    
// ×¢ÊÍ: key_buffer[]´æ·Å¼üÖµ,key_head,key_tailÊÇ¼üÖ¸Õë    
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
/* º¯Êı: ¶¨Ê±Æ÷·ÖÆµÖµÒÔ¼°Êµ¼ÊÖÜÆÚÎ»ÒÆÁ¿¼ÆËã                      */
/* Èë¿Ú:                                                         */
/* ·µ»ØÖµ:                                                       */
/* ×¢ÊÍ:                                                         */
/*****************************************************************/
void freq_data_cal(int axis_num)
{
 float temp;

 //½øĞĞÆµÂÊ¼ÆËã
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
 else     //¾àÀë²»×ã£¬Ö±½Ó·µ»Ø
   {
    pulse_out[axis_num] = 0;
    axis_dir[axis_num] = SET_STOP;
    mech_status.pulse_out[axis_num] = 0;
    return;   
   }
 //¶ÔÓÚ¾àÀë×ã¹»µÄ¶Î£¬¼ÆËã¶¨Ê±Æ÷µÄ·ÖÆµÊıÖµ£¬×¢Òâ¼ÆËãÖµÎªnÊ±£¬Êµ¼Ê·ÖÆµÊıÎªn+1
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
 
  //¼ÆËãÊµ¼ÊÎ»ÒÆÁ¿(Âö³åÊı)
  //The old algorithm
  //pulse_out[axis_num] = (unsigned int)(CLK_PER_TERM / temp + 0.5);  
  pulse_out[axis_num] = (unsigned int)(CLK_PER_TERM / temp + 0.5); 
  
  temp = CLK_PER_TERM / pulse_out[axis_num] - 1;
  freq_const[axis_num] = (unsigned int)temp;
  residue[axis_num] = (unsigned char)((temp - freq_const[axis_num])*256);  
  
  //freq_const[axis_num] = (unsigned long)(CLK_PER_TERM / pulse_out[axis_num]); // -1;
  //residue[axis_num] = CLK_PER_TERM_D % pulse_out[axis_num];
  //freq_const[axis_num] = (unsigned int)(CLK_PER_TERM / pulse_out[axis_num]) +1;
 
  //¼ÆËãÊµ¼ÊÊä³öÎ»ÒÆÁ¿ÒÔ¼°ÓàÊı
  mech_status.pulse_out[axis_num] = pulse_out[axis_num] * axis_dir[axis_num];

  temp = mech_status.pulse_out[axis_num] * mech_para.pulse_cor[axis_num];
  mech_status.dist_tail_a[axis_num] -= temp;
     
  return;
} 
//********************************************************************
/*------------------------4.19ÖØĞ´----------------------------*/
//´ÓUÅÌÀï¶ÁÈ¡¼Ó¹¤´úÂë¸øcode_buffer1ºÍcode_buffer2
void mech_code_load_FL(void)
{
 if(ctrl_flag & CODE_BUF1_EMPTY)
   {
    Code_read_data(Flash_rw_pointer,CODE_BUF_LENGTH,(unsigned int *)code_buffer1);   //Ò»´Î¶Á½øÀ´Îå¶Î//Ê®¶Î
    Flash_rw_pointer +=120;      //¶¨Î»¶ÁÈ¡´úÂëµÄÎ»ÖÃ 4.26ĞŞ¸Ä
    ctrl_flag &= ~CODE_BUF1_EMPTY;                   
    ctrl_flag |= CODE_BUF1_FULL;
    ctrl_flag |= CODE_BUF_READY; 
   }    
 else if(ctrl_flag & CODE_BUF2_EMPTY)
   {
    Code_read_data(Flash_rw_pointer,CODE_BUF_LENGTH,(unsigned int *)code_buffer2);
    Flash_rw_pointer +=120;          // ¶¨Î»¶ÁÈ¡´úÂëµÄÎ»ÖÃ
    ctrl_flag &= ~CODE_BUF2_EMPTY;                   
    ctrl_flag |= CODE_BUF2_FULL;  
    ctrl_flag |= CODE_BUF_READY; 
   } 
 else{;}
}
