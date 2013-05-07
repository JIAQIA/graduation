/********************************************************************************/
//×Ô¶¯ÔËÐÐÄ£¿é
/********************************************************************************/ 
#include "math.h"
#include "string.h"
#include "stdlib.h"
#include "data_deal_def.h"
#include "auto_ctrl_def.h"

#include "key_code_def.h"
#include "inter_polate_def.h"

//#include "Basic_IO_cfg.h"

//-------¼Ó¹¤´úÂë¶Î±äÁ¿¶¨Òå---------------------------

#pragma DATA_SECTION(segment1, ".seg_bss")
typ_interp_segment segment1;
#pragma DATA_SECTION(segment2, ".seg_bss")
typ_interp_segment segment2;
#pragma DATA_SECTION(segment3, ".seg_bss")
typ_interp_segment segment3;
#pragma DATA_SECTION(segment4, ".seg_bss")
typ_interp_segment segment4;

typ_interp_segment  * interpl_segment_prev;
typ_interp_segment  * interpl_segment_curr;
typ_interp_segment  * interpl_segment_next;

typ_interp_segment  * interpl_segment_insert;

typ_interp_segment  * temp_segment_pointer;

//-------------------------------------------------------------------
/*****************************************************************/
//º¯Êý:  ¶Î´æ´¢½á¹¹³õÊ¼»¯µ÷                                            
//Èë¿Ú:                                                        
//·µ»ØÖµ
//×¢ÊÍ: 
//*****************************************************************
void interp_segment_init(void)
{
 interpl_segment_prev = &segment1;
 interpl_segment_curr = &segment2;
 interpl_segment_next = &segment3;
 interpl_segment_insert = &segment4;
 
 interpl_segment_prev->basecmd = PROGRAM_IDLE;

 interpl_segment_curr->basecmd = PROGRAM_IDLE;

 interpl_segment_next->basecmd = PROGRAM_IDLE;

 interpl_segment_insert->basecmd = PROGRAM_IDLE;

 interpl_segment_prev->interp_time.motion_stat = INP_MOTION_END;		   
 interpl_segment_prev->interp_time.motion_stat_next = INP_MOTION_END;

 interpl_segment_curr->interp_time.motion_stat = INP_MOTION_END;
 interpl_segment_curr->interp_time.motion_stat_next = INP_MOTION_END;
 interpl_segment_curr->interp_time.term_rem = 0;

 interpl_segment_next->interp_time.motion_stat = INP_MOTION_END;
 interpl_segment_next->interp_time.motion_stat_next = INP_MOTION_END;
 
 interpl_segment_insert->interp_time.motion_stat = INP_MOTION_END;
 interpl_segment_insert->interp_time.motion_stat_next = INP_MOTION_END;

}
/*****************************************************************/
//º¯Êý:  ÏßÐÔËÙ¶ÈÐÞµ÷                                            
//Èë¿Ú:  ±»ÐÞµ÷²¿·ÖµÄ´æ·ÅÎ»ÖÃ                                                              
//·µ»ØÖµ:1-½øÐÐÁËÐÞµ÷£¬0-Î´½øÐÐÐÞµ÷                                                       
//×¢ÊÍ:  ½«ÐèÒªÐÞµ÷µÄ¶Î·ÖÎªÁ½²¿·Ý£¬µ±Ç°¶Î±£ÁôÒ»²¿·Ö£¬²¢¾¡¿ì×ªÎª¼õËÙ
//       Ê£Óà²¿·ÖÌîÈëÏÂÒ»¶Î£¬×¼±¸°´ÕÕÐÂµÄËÙ¶ÈÐÞµ÷Á¿ÖØÐÂ¹æ»®
/*****************************************************************/   
int linear_vel_override(typ_interp_segment * modify_pointer)
{
 int axis_idx;

 //²»ÊÇÖ±Ïß²å²¹£¬²»½øÐÐÐÞµ÷
 if(interpl_segment_curr->basecmd != G_00)
   {
    mech_status.speed_scal_old = user_para.speed_scal;
    return(0);
   }

 //Ê£Óà²¿·Ö³¤¶È²»¹»£¬²»ÐÞµ÷( override_echo_term ÖÁÉÙ±ØÐë´óÓÚ¼õËÙµÄÖÜÆÚÊý£«10)
 if(interpl_segment_curr->interp_time.term_rem <= user_para.override_echo_term)
   {
    mech_status.speed_scal_old = user_para.speed_scal;
    return(0);
   }

 //½øÈë¼õËÙ½×¶Î²»ÐÞµ÷
 //if(interpl_segment_curr->interp_time[0].motion_stat_next == INP_MOTION_DEC)
 //  {
 //   speed_scal_old = speed_scal;
 //   return(0);
 //  }

 //¼´½«½øÈë¼õËÙ½×¶Î²»ÐÞµ÷£¨²î10¸öÖÜÆÚ£©
 if(interpl_segment_curr->interp_time.term_rem <= (interpl_segment_curr->interp_time.term_add + 10))
   {
    mech_status.speed_scal_old = user_para.speed_scal;
    return(0);
   }

 //Ö»ÓÐÔÚÔÈËÙ½×¶Î²ÅÔÊÐíÐÞµ÷ 
 if(interpl_segment_curr->interp_time.motion_stat_next == INP_MOTION_UNI)
   { 	 
    mech_status.speed_scal_old = user_para.speed_scal;
    
    //µ±Ç°¶Î¸´ÖÆÒ»·Ýµ½ÐÞÕý¶Î
    memcpy(modify_pointer, interpl_segment_curr, sizeof(typ_interp_segment));

    //5¸öÖÜÆÚºóµ±Ç°¶Î±äÎª¼õËÙ(£«5ÎªÁËÈ·±£next¶ÎÖØÐÂ¹æ»®µÄÊ±¼ä)
    interpl_segment_curr->interp_time.term_cmd = interpl_segment_curr->interp_time.term_run + 5;
    interpl_segment_curr->interp_time.term_rem = interpl_segment_curr->interp_time.term_add + 5;

    //Ê£Óà²¿·ÖÓÉÏÂÒ»¶Î°´ÕÕÐÂµÄËÙ¶ÈÖØÐÂ¹æ»®
    modify_pointer->interp_time.term_cmd -= interpl_segment_curr->interp_time.term_cmd;
    
    //¼ÆËãÏÂÒ»¶ÎÖÐ¸÷Öá£¨ÉÐÎ´Íê³ÉµÄ£©ÔË¶¯·ÖÁ¿£¬ÓÐ·ûºÅÊý
    modify_pointer->dist = 0.0;
    for(axis_idx = 0; axis_idx < AXIS_SUM; axis_idx ++)
       {
        modify_pointer->axis_dist[axis_idx] =
	           modify_pointer->axis_vel[axis_idx] * modify_pointer->interp_time.term_cmd;
        modify_pointer->dist += modify_pointer->axis_dist[axis_idx]
                                * modify_pointer->axis_dist[axis_idx];
       } 
    modify_pointer->dist = sqrt(modify_pointer->dist);
    return(1);
   }
 return(0);
}
//***************************************************************************


//*********************************************************************************
//º¯Êý£º´Óµ±Ç°Î»ÖÃµ½´ïÖ¸¶¨µÄÎ»ÖÃ
//Èë¿Ú£ºposition[]Ö¸¶¨Òªµ½´ïµÄÎ»ÖÃ£¬¸ø³ö¿Õ¼ä×ø±ê
//      axis_num: Ö¸¶¨ÐèÒªÔË¶¯µÄÖáºÅ£¬0001-X,0002-Y,0004-Z
//³ö¿Ú£º½«ÉÏÊöÔË¶¯×ª»»Îª´ý²å²¹Ö±Ïß¶Î·ÅÈëinterpl_segment_insert¶ÎÖÐ
//ËµÃ÷£º½«·µ»ØÔÝÍ£µãµÄÔË¶¯ÐÎ³É¿ÉÖ´ÐÐ³ÌÐò¶Î¼ÓÔØµ½next¶Î£¬²¢ÔÚcurr¶Î²åÈë±ØÒªµÄÑÓÊ±
//*********************************************************************************
void appoint_position(float position[], unsigned int axis_num)
{
 int axis_idx;
 
 interpl_segment_insert->basecmd = G_00;
 interpl_segment_insert->vel = user_para.feed_vel;

 interpl_segment_insert->axis_dist[0] = 0.0; 
 interpl_segment_insert->axis_dist[1] = 0.0; 
 interpl_segment_insert->axis_dist[2] = 0.0; 

 if(axis_num & X_AXIS)
   {
	interpl_segment_insert->axis_dist[0] = position[0] - mech_status.curr_posi[0];
   }
 if(axis_num & Y_AXIS)
   {  
    interpl_segment_insert->axis_dist[1] = position[1] - mech_status.curr_posi[1];
   }
 if(axis_num & Z_AXIS)
   {  
    interpl_segment_insert->axis_dist[2] = position[2] - mech_status.curr_posi[2];
   }
   
 interpl_segment_insert->dist = 0.0;
 for(axis_idx = 0; axis_idx < AXIS_SUM; axis_idx ++)
    {
     interpl_segment_insert->dist += interpl_segment_insert->axis_dist[axis_idx]
                                * interpl_segment_insert->axis_dist[axis_idx];
    } 
 interpl_segment_insert->dist = sqrt(interpl_segment_insert->dist);
 ctrl_flag |= INSERT_SEG_FULL;
 ctrl_flag &= ~INSERT_SEG_READY; 
}
//*********************************************************************************
//º¯Êý£º²åÈëÔË¶¯ÑÓÊ±µ½µ±Ç°²å²¹¶Î
//Èë¿Ú£ºÊ±¼äµ¥Î»Îª S
//³ö¿Ú£º
//ËµÃ÷£ºÔÚcurr¶Î²åÈë±ØÒªµÄÑÓÊ±
//*********************************************************************************
void insert_dwell_time(float dwell_time)
{
 interpl_segment_insert->basecmd = G_DWELL;
 interpl_segment_insert->dist = dwell_time;
 ctrl_flag |= INSERT_SEG_FULL;
 ctrl_flag &= ~INSERT_SEG_READY;
}
//----------------------------------------------------------------------------
/*void insert_segment(void)
{
 interpl_segment_next->label = 1;
 interpl_segment_next->basecmd = G_02; 
 interpl_segment_next->axis_dist[0] = 3.14/2;
 interpl_segment_next->axis_dist[1] = 0.1;
 interpl_segment_next->axis_dist[2] = 50.0;
 interpl_segment_next->dist = 3.14;
 interpl_segment_next->vel = 10;
}
*/
/*****************************************************************/
//º¯Êý£º¼õËÙÍ£³µ
//Èë¿Ú:
//·µ»ØÖµ:
//×¢ÊÍ:                                                        
/*****************************************************************/
void motion_abolish(void) 
{ 
 interpl_segment_curr->interp_time.term_cmd = interpl_segment_curr->interp_time.term_run;
 interpl_segment_curr->interp_time.term_rem = interpl_segment_curr->interp_time.term_add;
 interpl_segment_curr->interp_time.motion_stat_next = INP_MOTION_DEC;

 ctrl_flag &= (~BLEND_PERMIT); //½ûÖ¹³ÌÐò¶Î×ª½Ó
}

/***********************************************************************/
// º¯Êý: ³ÌÐò¶ÎÔ¤´¦Àí (ÚÖ÷º¯ÊýÑ­»·ÖÐµ÷ÓÃ)                                                   
// Èë¿Ú:                                                               
// ·µ»ØÖµ:                                                             
// ×¢ÊÍ: ±¾º¯Êý½«seg_labelÖ¸¶¨µÄ³ÌÐò¶Î½øÐÐ½âÊÍ£¬ÔË¶¯¿ØÖÆÁ¿¼ÓÔØµ½interpl_segment_next¶ÎÖÐ
//       ¿ª¹Ø¿ØÖÆÁ¿¼ÓÔØµ½plc_bufferÖÐ£¬²¢½«ÔË¶¯ÑÓÊ±Ê±¼ä¼ÓÔØµ½dwell_term
/***********************************************************************/
void code_seg_pre_treat(void) 
{
 if(ctrl_flag & INSERT_SEG_FULL)     //²¹³ä²åÈë¶Î´æÔÚ£¬ÔòÓÅÏÈ¶ÔÆä½øÐÐ¹æ»®
   {
    if(!(ctrl_flag & INSERT_SEG_READY))		 //ÈôÎ´¹æ»®Íê³É
      {
       segment_plan(interpl_segment_insert);
       ctrl_flag |= INSERT_SEG_READY; 
      }
    return;  
   }
   
 if(ctrl_flag & NEXT_SEG_RENEW)    //Õý³£³ÌÐò¶Î¹æ»® 
   { 
    if(segment_plan(interpl_segment_next)) //³É¹¦¹æ»®
      {
       ctrl_flag &= (~NEXT_SEG_RENEW);
       switch(interpl_segment_next->basecmd)
         {
          case G_00:
          case G_01:
          case G_02:
          case G_03:
          case G_DWELL:
               ctrl_flag |= NEXT_SEG_READY;
               break;
          case G_04://pause
               ctrl_flag &= (~BLEND_PERMIT); 
               mech_status.stat = STA_PAUSING;
               ctrl_flag |= NEXT_SEG_EMPTY;       
               break;
    
          case PROGRAM_END: //finish
               ctrl_flag |= TASK_FINISH;     
               ctrl_flag &= ~BLEND_PERMIT;   
               break;
          default:;
         }//end of switch
       }  
    else mech_status.error = MECH_CODE_ERROR;
    
    return;
   }

 if(ctrl_flag & NEXT_SEG_EMPTY)              //³ÌÐò¶Î¼ÓÔØµ½next¶Î
   {
    if(mech_code_load(interpl_segment_next))    //³ÌÐò´úÂë¼ÓÔØ³É¹¦ 
      {
       ctrl_flag &= (~NEXT_SEG_EMPTY);  
       ctrl_flag |= NEXT_SEG_RENEW; 
      }
    return;  
   }  
}

/*****************************************************************/
/* º¯Êý:×Ô¶¯ÔËÐÐÖÐµÄ°´¼ü´¦Àí                                     */
/* Èë¿Ú:                                                         */
/* ·µ»ØÖµ:                                                       */
/* ×¢ÊÍ:                                                         */
/*****************************************************************/ 
/*3.12ÆÁ±Î
void auto_key_echo(unsigned int key_value) 
{
 switch(key_value)
   {
    case CMD_RUN:    //Æô¶¯ÔËÐÐ
         //¿ªÊ¼ÐÂµÄ¼Ó¹¤
         if(mech_status.stat == STA_FINISHED)
           {
            public_para_init(1);				//¹«¹²±äÁ¿³õÊ¼»¯
            //ctrl_flag = 0x0000;
            mech_status.stat = STA_READY;
           } 
         if(mech_status.stat == STA_READY)    
           {
            mech_flag &= ~SIMULATION;
            user_para.speed_scal = speed_scal;
            
            code_buf_init(); 					   //´úÂë¶ÓÁÐ³õÊ¼»¯
 	        interpl_segment_prev->basecmd = PROGRAM_IDLE;
		    interpl_segment_curr->label = 0;

            //²åÈë¿ªÖ÷ÖáÑÓÊ±
            insert_dwell_time(user_para.spindle_on_time);
            mech_status.spindle_rev = (int)(user_para.spindle_rev * user_para.spindle_scal/100);
            mech_flag |= SPINDLE_ON;
            //-----------------------------------------------------------
            ctrl_flag |= BEING_AUTO_RUN;
           	ctrl_flag &= ~TASK_FINISH;  
            ctrl_flag |= BLEND_PERMIT;   
		    ctrl_flag |= NEXT_SEG_EMPTY;   	   
            ctrl_flag &= ~CODE_BUF_READY;   
		    mech_status.stat = STA_WORKING;  //½øÈë¹¤×÷×´Ì¬
     
            break;
           }         
         //--------------------------------------------------------------
         //´ÓÔÝÍ£ÖÐ»Ö¸´
         if(mech_status.stat == STA_PAUSED)
		   {
		    //Èç¹ûÖ÷ÖáÎ´¿ª£¬²åÈë¿ªÖ÷ÖáÑÓÊ±
            if(!(mech_flag & SPINDLE_ON) && !(mech_flag & SIMULATION))
              {
		       insert_dwell_time(user_para.spindle_on_time);
		       mech_status.spindle_rev = (int)(user_para.spindle_rev * user_para.spindle_scal/100);
               mech_flag |= SPINDLE_ON;
			  }
            //--------------------------------------------------- 
   		    ctrl_flag |= BLEND_PERMIT;         //blend_permit = ST_TRUE;
            mech_status.stat = STA_WORKING;    //½øÈë¹¤×÷×´Ì¬
		   } 
		 //------------------------------------------------------  
         break;  
    case CMD_SIMU:
         public_para_init(1);
         mech_flag |= SIMULATION; 
         speed_scal = user_para.speed_scal;
         user_para.speed_scal = 500;
         
         code_buf_init();
 	     interpl_segment_prev->basecmd = PROGRAM_IDLE;
		 interpl_segment_curr->label = 0;
         //-----------------------------------------------------------
         ctrl_flag |= BEING_AUTO_RUN;
         ctrl_flag &= ~TASK_FINISH;  
         ctrl_flag |= BLEND_PERMIT;   
		 ctrl_flag |= NEXT_SEG_EMPTY;   	   
         ctrl_flag &= ~CODE_BUF_READY;   
		 mech_status.stat = STA_WORKING;  //½øÈë¹¤×÷×´Ì¬
         break;           

    case CMD_PAUSE:        //ÔË¶¯ÔÝÍ£
         if(mech_status.stat == STA_WORKING)
           {
		    ctrl_flag &= (~BLEND_PERMIT); //blend_permit = ST_FAULSE;  // ½ûÖ¹³ÌÐò¶Î×ª½Ó£¬±¾¶ÎÍê³Éºó×ÔÈ»ÔÝÍ£
    		mech_status.stat = STA_PAUSING;
           }    
         break;

    case CMD_STOP:         //Í£Ö¹×Ô¶¯ÔËÐÐ
	     if(mech_status.stat == STA_WORKING)
           {
	        ctrl_flag &= (~BLEND_PERMIT); //blend_permit = ST_FAULSE;
		    mech_status.stat = STA_STOPPING;
		   }
         if(mech_status.stat == STA_PAUSED)  mech_status.stat = STA_STOPPED;
	     if(mech_status.stat == STA_FINISHED)  mech_status.stat = STA_STOPPED;
     	 break;

    case CMD_ESTOP:       //Á¢¼´Í£³µ
         if(interpl_segment_curr->interp_time.motion_stat > INP_MOTION_DEC)
           {
	        motion_abolish();
           }
         
         ctrl_flag &= (~BLEND_PERMIT); //½ûÖ¹³ÌÐò¶Î×ª½Ó 
         mech_status.stat = STA_ESTOPPING;
         break;

    default: break;  
   }   //end of switch
} 
*/
//3.15ÖØÐ´
void auto_ctrl()      //¼ì²âµ½¡°×Ô¶¯¡±¼ü°´ÏÂ£¬½øÐÐÒ»ÏÂ×Ô¶¯ÔËÐÐÇ°µÄ´¦Àí
{
		public_para_init(1);				//¹«¹²±äÁ¿³õÊ¼»¯
		user_para.speed_scal = speed_scal;	   //ÉèÖÃËÙ¶È
	    code_buf_init(); 					   //´úÂë¶ÓÁÐ³õÊ¼»¯
 	    interpl_segment_prev->basecmd = PROGRAM_IDLE;	 //¼ÓÔØ³ÌÐò¶Î¹¦ÄÜ×Ö
		interpl_segment_curr->label = 0;				 //¶Î±êºÅÖÃÁã
		//²åÈë¿ªÖ÷ÖáÑÓÊ±
		insert_dwell_time(user_para.spindle_on_time);
		mech_status.spindle_rev = (int)(user_para.spindle_rev * user_para.spindle_scal/100); //»ñµÃÖ÷Öá×ªËÙ
		mech_flag |= SPINDLE_ON;
		//-----------------------------------------------------------
		ctrl_flag |= BEING_AUTO_RUN;
		ctrl_flag &= ~TASK_FINISH;  
		ctrl_flag |= BLEND_PERMIT;   
		ctrl_flag |= NEXT_SEG_EMPTY;   	   
		ctrl_flag &= ~CODE_BUF_READY;   
		mech_status.stat = STA_WORKING;  //½øÈë¹¤×÷×´Ì¬
		/*
		//-------limit check-------------------------------
		if(stat_logic & LIMIT_IO_DEF) 
		{
			if(mech_status.stat == STA_WORKING)
			{
			key2_write(CMD_ESTOP); 
			mech_status.error = STATION_EXCEED;           
			} 
		} 
		//-------------------------------------------------           
          */  
        
} 

/*****************************************************************/
/* º¯Êý:×Ô¶¯ÔËÐÐ³ÌÐò                                             */
/* Èë¿Ú:                                                         */
/* ·µ»ØÖµ:                                                       */
/* ×¢ÊÍ:                                                         */
/*****************************************************************/ 
void auto_process(void)
{
 unsigned int axis_idx;

 //-------------³ÌÐò¶Î×Ô¶¯²å²¹´¦Àí-----------------------------------
 segment_process(interpl_segment_curr);
 segment_process(interpl_segment_prev);
 
 //¼ÆËãÒ»¸ö²å²¹ÖÜÆÚÄÚ¸÷ÖáÊä³öµÄºÏ³ÉÎ»ÒÆÁ¿
 for(axis_idx = 0; axis_idx < AXIS_SUM; axis_idx ++)
    {
     mech_status.dist_sum[axis_idx] = interpl_segment_curr->dist_out[axis_idx] 
                                    + interpl_segment_prev->dist_out[axis_idx];
    }   
 //-------------³ÌÐò¶Î×Ô¶¯×ª½Ó´¦Àí----------------------------------- 
 
 if(ctrl_flag & BLEND_PERMIT)
   {
    //Ö»ÓÐµ±Ç°¶ÎÊ£ÓàÖÜÆÚÊýÐ¡ÓÚ×ª½ÓÖÜÆÚÊýÇÒ×ª½ÓÔÊÐí²Å¿¼ÂÇ×ª½Ó
    if((interpl_segment_curr->interp_time.term_rem <= user_para.blend_time)
        &&(interpl_segment_prev->basecmd==PROGRAM_IDLE)) 
      {
	   //------Ö»Òª²åÈë¶Î´æÔÚ£¬¾Í²»½øÐÐÕý³£×ª½Ó£¬ÒÔ·À³ö´í----------------
       if(ctrl_flag & INSERT_SEG_FULL)  
         {        
          if(ctrl_flag & INSERT_SEG_READY) //Ö»ÓÐ¶îÍâ¶Ï×¼±¸ºÃ²Å²åÈë¶îÍâµÄ¶Î
            {
             temp_segment_pointer = interpl_segment_prev;	  //³ÌÐò¶Î×ª½Ó		
             interpl_segment_prev = interpl_segment_curr;    
	         interpl_segment_curr = interpl_segment_insert;
             interpl_segment_insert = temp_segment_pointer;

             interpl_segment_insert->basecmd = PROGRAM_IDLE;
             ctrl_flag &= (~INSERT_SEG_READY);  
             ctrl_flag &= (~INSERT_SEG_FULL);   
		    } 
	     } //end of if(insert_seg_full) 

	     //-------ÏÂÒ»¶Î×¼±¸ºÃÊ±½øÐÐÕý³£×ª½Ó-------------------------------	     
       else if(ctrl_flag & NEXT_SEG_READY) 
	     {
          temp_segment_pointer = interpl_segment_prev;	  //³ÌÐò¶Î×ª½Ó		
          interpl_segment_prev = interpl_segment_curr;    
          interpl_segment_curr = interpl_segment_next;
          interpl_segment_next = temp_segment_pointer;

          ctrl_flag &= ~NEXT_SEG_READY; 
          ctrl_flag |= NEXT_SEG_EMPTY;           
		  mech_status.label = interpl_segment_curr->label; //¸üÐÂ¶Î±êºÅ
 	     } //end of else if(next_seg_ready) 
       //-----------------------------------------------------------------
       else{;}
      } // end of if(interpl_segment_prev->basecmd == PROGRAM_IDLE)
   } //end of  if((interpl_segment_curr->interp_time[0].term_rem <= blend_term) && (blend_permit))    
  

 //-------------×Ô¶¯ËÙ¶ÈÐÞµ÷´¦Àí----------------------------------
 //ÐÞµ÷¶Î·ÅÈë²åÈë¶Î£¬ËùÒÔÒªÇóinterpl_segment_insertÎª¿Õ
 if(user_para.speed_scal != mech_status.speed_scal_old)
   {
    if(!(ctrl_flag & INSERT_SEG_FULL))
      { 
	     //if(linear_vel_override(interpl_segment_insert)) //Èç¹û½øÐÐËÙ¶ÈÐÞµ÷´¦Àí,ÐÞµ÷¶Î·ÅÈëinterpl_segment_insert 		   
	       {
	        //ctrl_flag |= INSERT_SEG_FULL;
	       }
      } 
   }   
 
 //-------------×´Ì¬¼ì²â´¦Àí-------------------------------------
 switch(mech_status.stat)
   {
    case STA_WORKING:
         if(!(ctrl_flag & TASK_FINISH)) break;
		 if((interpl_segment_prev->basecmd == PROGRAM_IDLE)  //¼ì²âÈÎÎñÍê³É
               && (interpl_segment_curr->basecmd == PROGRAM_IDLE))          
           {
            mech_status.spindle_rev = 0;
            mech_flag &= ~SPINDLE_ON;
            
            mech_status.stat = STA_FINISHED; 
            ctrl_flag &= (~BEING_AUTO_RUN);                 
			ctrl_flag &= (~BLEND_PERMIT);          
           }
		 break;     
    case STA_PAUSING: //³Ð½ÓÔË¶¯ÔÝÍ£ÃüÁî,¼ì²âÔÝÍ£ÊÇ·ñ½áÊø
         if((interpl_segment_curr->basecmd == PROGRAM_IDLE)   
		       && (interpl_segment_prev->basecmd == PROGRAM_IDLE)) //ÔÝÍ£Íê³É
           {
            mech_status.spindle_rev = 0; //Ö÷ÖáÍ£Ö¹
            mech_flag &= ~SPINDLE_ON;
            
            mech_status.stat = STA_PAUSED;
           }
         break;

    case STA_STOPPING: //³Ð½ÓÔË¶¯Í£Ö¹²Ù×÷,¼ì²âÍ£Ö¹¹ý³ÌÊÇ·ñ½áÊø
         if((interpl_segment_curr->basecmd == PROGRAM_IDLE)
		          && (interpl_segment_prev->basecmd == PROGRAM_IDLE))
           {
            mech_status.spindle_rev = 0;
            mech_flag &= ~SPINDLE_ON;
           
            mech_status.stat = STA_STOPPED;
            ctrl_flag &= (~BEING_AUTO_RUN);                   
           }
         break;

    case STA_ESTOPPING: //³Ð½Ó¼±Í£²Ù×÷£¬ÅÐ¶Ï¼±Í£¹ý³ÌÊÇ·ñ½áÊø
	     if((interpl_segment_curr->basecmd == PROGRAM_IDLE)
                 && (interpl_segment_prev->basecmd == PROGRAM_IDLE))
           {
		    mech_status.spindle_rev = 0;
            mech_flag &= ~SPINDLE_ON;
            
            mech_status.stat = STA_BROKEN;
            ctrl_flag &= (~BEING_AUTO_RUN);         
           }
		 break;
		 
    case STA_STOPPED:
    case STA_FINISHED:
    case STA_BROKEN:
         mech_flag &= ~USB_COMM_READY;
         if(mech_flag & SIMULATION)
		   { 
		    mech_flag &= ~SIMULATION;
            user_para.speed_scal = speed_scal;  
           } 
         break;
    default:break;
   }  //end of switch 
}  
/********************************************************************************/ 

