/********************************************************************************/
//�Զ�����ģ��
/********************************************************************************/ 
#include "math.h"
#include "string.h"
#include "stdlib.h"
#include "data_deal_def.h"
#include "auto_ctrl_def.h"

#include "key_code_def.h"
#include "inter_polate_def.h"

//#include "Basic_IO_cfg.h"

//-------�ӹ�����α�������---------------------------

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
//����:  �δ洢�ṹ��ʼ����                                            
//���:                                                        
//����ֵ
//ע��: 
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
//����:  �����ٶ��޵�                                            
//���:  ���޵����ֵĴ��λ��                                                              
//����ֵ:1-�������޵���0-δ�����޵�                                                       
//ע��:  ����Ҫ�޵��Ķη�Ϊ�����ݣ���ǰ�α���һ���֣�������תΪ����
//       ʣ�ಿ��������һ�Σ�׼�������µ��ٶ��޵������¹滮
/*****************************************************************/   
int linear_vel_override(typ_interp_segment * modify_pointer)
{
 int axis_idx;

 //����ֱ�߲岹���������޵�
 if(interpl_segment_curr->basecmd != G_00)
   {
    mech_status.speed_scal_old = user_para.speed_scal;
    return(0);
   }

 //ʣ�ಿ�ֳ��Ȳ��������޵�( override_echo_term ���ٱ�����ڼ��ٵ���������10)
 if(interpl_segment_curr->interp_time.term_rem <= user_para.override_echo_term)
   {
    mech_status.speed_scal_old = user_para.speed_scal;
    return(0);
   }

 //������ٽ׶β��޵�
 //if(interpl_segment_curr->interp_time[0].motion_stat_next == INP_MOTION_DEC)
 //  {
 //   speed_scal_old = speed_scal;
 //   return(0);
 //  }

 //����������ٽ׶β��޵�����10�����ڣ�
 if(interpl_segment_curr->interp_time.term_rem <= (interpl_segment_curr->interp_time.term_add + 10))
   {
    mech_status.speed_scal_old = user_para.speed_scal;
    return(0);
   }

 //ֻ�������ٽ׶β������޵� 
 if(interpl_segment_curr->interp_time.motion_stat_next == INP_MOTION_UNI)
   { 	 
    mech_status.speed_scal_old = user_para.speed_scal;
    
    //��ǰ�θ���һ�ݵ�������
    memcpy(modify_pointer, interpl_segment_curr, sizeof(typ_interp_segment));

    //5�����ں�ǰ�α�Ϊ����(��5Ϊ��ȷ��next�����¹滮��ʱ��)
    interpl_segment_curr->interp_time.term_cmd = interpl_segment_curr->interp_time.term_run + 5;
    interpl_segment_curr->interp_time.term_rem = interpl_segment_curr->interp_time.term_add + 5;

    //ʣ�ಿ������һ�ΰ����µ��ٶ����¹滮
    modify_pointer->interp_time.term_cmd -= interpl_segment_curr->interp_time.term_cmd;
    
    //������һ���и��ᣨ��δ��ɵģ��˶��������з�����
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
//�������ӵ�ǰλ�õ���ָ����λ��
//��ڣ�position[]ָ��Ҫ�����λ�ã������ռ�����
//      axis_num: ָ����Ҫ�˶�����ţ�0001-X,0002-Y,0004-Z
//���ڣ��������˶�ת��Ϊ���岹ֱ�߶η���interpl_segment_insert����
//˵������������ͣ����˶��γɿ�ִ�г���μ��ص�next�Σ�����curr�β����Ҫ����ʱ
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
//�����������˶���ʱ����ǰ�岹��
//��ڣ�ʱ�䵥λΪ S
//���ڣ�
//˵������curr�β����Ҫ����ʱ
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
//����������ͣ��
//���:
//����ֵ:
//ע��:                                                        
/*****************************************************************/
void motion_abolish(void) 
{ 
 interpl_segment_curr->interp_time.term_cmd = interpl_segment_curr->interp_time.term_run;
 interpl_segment_curr->interp_time.term_rem = interpl_segment_curr->interp_time.term_add;
 interpl_segment_curr->interp_time.motion_stat_next = INP_MOTION_DEC;

 ctrl_flag &= (~BLEND_PERMIT); //��ֹ�����ת��
}

/***********************************************************************/
// ����: �����Ԥ���� (�������ѭ���е���)                                                   
// ���:                                                               
// ����ֵ:                                                             
// ע��: ��������seg_labelָ���ĳ���ν��н��ͣ��˶����������ص�interpl_segment_next����
//       ���ؿ��������ص�plc_buffer�У������˶���ʱʱ����ص�dwell_term
/***********************************************************************/
void code_seg_pre_treat(void) 
{
 if(ctrl_flag & INSERT_SEG_FULL)     //�������δ��ڣ������ȶ�����й滮
   {
    if(!(ctrl_flag & INSERT_SEG_READY))		 //��δ�滮���
      {
       segment_plan(interpl_segment_insert);
       ctrl_flag |= INSERT_SEG_READY; 
      }
    return;  
   }
   
 if(ctrl_flag & NEXT_SEG_RENEW)    //��������ι滮 
   { 
    if(segment_plan(interpl_segment_next)) //�ɹ��滮
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

 if(ctrl_flag & NEXT_SEG_EMPTY)              //����μ��ص�next��
   {
    if(mech_code_load(interpl_segment_next))    //���������سɹ� 
      {
       ctrl_flag &= (~NEXT_SEG_EMPTY);  
       ctrl_flag |= NEXT_SEG_RENEW; 
      }
    return;  
   }  
}

/*****************************************************************/
/* ����:�Զ������еİ�������                                     */
/* ���:                                                         */
/* ����ֵ:                                                       */
/* ע��:                                                         */
/*****************************************************************/ 
/*3.12����
void auto_key_echo(unsigned int key_value) 
{
 switch(key_value)
   {
    case CMD_RUN:    //��������
         //��ʼ�µļӹ�
         if(mech_status.stat == STA_FINISHED)
           {
            public_para_init(1);				//����������ʼ��
            //ctrl_flag = 0x0000;
            mech_status.stat = STA_READY;
           } 
         if(mech_status.stat == STA_READY)    
           {
            mech_flag &= ~SIMULATION;
            user_para.speed_scal = speed_scal;
            
            code_buf_init(); 					   //������г�ʼ��
 	        interpl_segment_prev->basecmd = PROGRAM_IDLE;
		    interpl_segment_curr->label = 0;

            //���뿪������ʱ
            insert_dwell_time(user_para.spindle_on_time);
            mech_status.spindle_rev = (int)(user_para.spindle_rev * user_para.spindle_scal/100);
            mech_flag |= SPINDLE_ON;
            //-----------------------------------------------------------
            ctrl_flag |= BEING_AUTO_RUN;
           	ctrl_flag &= ~TASK_FINISH;  
            ctrl_flag |= BLEND_PERMIT;   
		    ctrl_flag |= NEXT_SEG_EMPTY;   	   
            ctrl_flag &= ~CODE_BUF_READY;   
		    mech_status.stat = STA_WORKING;  //���빤��״̬
     
            break;
           }         
         //--------------------------------------------------------------
         //����ͣ�лָ�
         if(mech_status.stat == STA_PAUSED)
		   {
		    //�������δ�������뿪������ʱ
            if(!(mech_flag & SPINDLE_ON) && !(mech_flag & SIMULATION))
              {
		       insert_dwell_time(user_para.spindle_on_time);
		       mech_status.spindle_rev = (int)(user_para.spindle_rev * user_para.spindle_scal/100);
               mech_flag |= SPINDLE_ON;
			  }
            //--------------------------------------------------- 
   		    ctrl_flag |= BLEND_PERMIT;         //blend_permit = ST_TRUE;
            mech_status.stat = STA_WORKING;    //���빤��״̬
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
		 mech_status.stat = STA_WORKING;  //���빤��״̬
         break;           

    case CMD_PAUSE:        //�˶���ͣ
         if(mech_status.stat == STA_WORKING)
           {
		    ctrl_flag &= (~BLEND_PERMIT); //blend_permit = ST_FAULSE;  // ��ֹ�����ת�ӣ�������ɺ���Ȼ��ͣ
    		mech_status.stat = STA_PAUSING;
           }    
         break;

    case CMD_STOP:         //ֹͣ�Զ�����
	     if(mech_status.stat == STA_WORKING)
           {
	        ctrl_flag &= (~BLEND_PERMIT); //blend_permit = ST_FAULSE;
		    mech_status.stat = STA_STOPPING;
		   }
         if(mech_status.stat == STA_PAUSED)  mech_status.stat = STA_STOPPED;
	     if(mech_status.stat == STA_FINISHED)  mech_status.stat = STA_STOPPED;
     	 break;

    case CMD_ESTOP:       //����ͣ��
         if(interpl_segment_curr->interp_time.motion_stat > INP_MOTION_DEC)
           {
	        motion_abolish();
           }
         
         ctrl_flag &= (~BLEND_PERMIT); //��ֹ�����ת�� 
         mech_status.stat = STA_ESTOPPING;
         break;

    default: break;  
   }   //end of switch
} 
*/
//3.15��д
void auto_ctrl()      //��⵽���Զ��������£�����һ���Զ�����ǰ�Ĵ���
{
		public_para_init(1);				//����������ʼ��
		user_para.speed_scal = speed_scal;	   //�����ٶ�
	    code_buf_init(); 					   //������г�ʼ��
 	    interpl_segment_prev->basecmd = PROGRAM_IDLE;	 //���س���ι�����
		interpl_segment_curr->label = 0;				 //�α������
		//���뿪������ʱ
		insert_dwell_time(user_para.spindle_on_time);
		mech_status.spindle_rev = (int)(user_para.spindle_rev * user_para.spindle_scal/100); //�������ת��
		mech_flag |= SPINDLE_ON;
		//-----------------------------------------------------------
		ctrl_flag |= BEING_AUTO_RUN;
		ctrl_flag &= ~TASK_FINISH;  
		ctrl_flag |= BLEND_PERMIT;   
		ctrl_flag |= NEXT_SEG_EMPTY;   	   
		ctrl_flag &= ~CODE_BUF_READY;   
		mech_status.stat = STA_WORKING;  //���빤��״̬
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
/* ����:�Զ����г���                                             */
/* ���:                                                         */
/* ����ֵ:                                                       */
/* ע��:                                                         */
/*****************************************************************/ 
void auto_process(void)
{
 unsigned int axis_idx;

 //-------------������Զ��岹����-----------------------------------
 segment_process(interpl_segment_curr);
 segment_process(interpl_segment_prev);
 
 //����һ���岹�����ڸ�������ĺϳ�λ����
 for(axis_idx = 0; axis_idx < AXIS_SUM; axis_idx ++)
    {
     mech_status.dist_sum[axis_idx] = interpl_segment_curr->dist_out[axis_idx] 
                                    + interpl_segment_prev->dist_out[axis_idx];
    }   
 //-------------������Զ�ת�Ӵ���----------------------------------- 
 
 if(ctrl_flag & BLEND_PERMIT)
   {
    //ֻ�е�ǰ��ʣ��������С��ת����������ת������ſ���ת��
    if((interpl_segment_curr->interp_time.term_rem <= user_para.blend_time)
        &&(interpl_segment_prev->basecmd==PROGRAM_IDLE)) 
      {
	   //------ֻҪ����δ��ڣ��Ͳ���������ת�ӣ��Է�����----------------
       if(ctrl_flag & INSERT_SEG_FULL)  
         {        
          if(ctrl_flag & INSERT_SEG_READY) //ֻ�ж����׼���òŲ������Ķ�
            {
             temp_segment_pointer = interpl_segment_prev;	  //�����ת��		
             interpl_segment_prev = interpl_segment_curr;    
	         interpl_segment_curr = interpl_segment_insert;
             interpl_segment_insert = temp_segment_pointer;

             interpl_segment_insert->basecmd = PROGRAM_IDLE;
             ctrl_flag &= (~INSERT_SEG_READY);  
             ctrl_flag &= (~INSERT_SEG_FULL);   
		    } 
	     } //end of if(insert_seg_full) 

	     //-------��һ��׼����ʱ��������ת��-------------------------------	     
       else if(ctrl_flag & NEXT_SEG_READY) 
	     {
          temp_segment_pointer = interpl_segment_prev;	  //�����ת��		
          interpl_segment_prev = interpl_segment_curr;    
          interpl_segment_curr = interpl_segment_next;
          interpl_segment_next = temp_segment_pointer;

          ctrl_flag &= ~NEXT_SEG_READY; 
          ctrl_flag |= NEXT_SEG_EMPTY;           
		  mech_status.label = interpl_segment_curr->label; //���¶α��
 	     } //end of else if(next_seg_ready) 
       //-----------------------------------------------------------------
       else{;}
      } // end of if(interpl_segment_prev->basecmd == PROGRAM_IDLE)
   } //end of  if((interpl_segment_curr->interp_time[0].term_rem <= blend_term) && (blend_permit))    
  

 //-------------�Զ��ٶ��޵�����----------------------------------
 //�޵��η������Σ�����Ҫ��interpl_segment_insertΪ��
 if(user_para.speed_scal != mech_status.speed_scal_old)
   {
    if(!(ctrl_flag & INSERT_SEG_FULL))
      { 
	     //if(linear_vel_override(interpl_segment_insert)) //��������ٶ��޵�����,�޵��η���interpl_segment_insert 		   
	       {
	        //ctrl_flag |= INSERT_SEG_FULL;
	       }
      } 
   }   
 
 //-------------״̬��⴦��-------------------------------------
 switch(mech_status.stat)
   {
    case STA_WORKING:
         if(!(ctrl_flag & TASK_FINISH)) break;
		 if((interpl_segment_prev->basecmd == PROGRAM_IDLE)  //����������
               && (interpl_segment_curr->basecmd == PROGRAM_IDLE))          
           {
            mech_status.spindle_rev = 0;
            mech_flag &= ~SPINDLE_ON;
            
            mech_status.stat = STA_FINISHED; 
            ctrl_flag &= (~BEING_AUTO_RUN);                 
			ctrl_flag &= (~BLEND_PERMIT);          
           }
		 break;     
    case STA_PAUSING: //�н��˶���ͣ����,�����ͣ�Ƿ����
         if((interpl_segment_curr->basecmd == PROGRAM_IDLE)   
		       && (interpl_segment_prev->basecmd == PROGRAM_IDLE)) //��ͣ���
           {
            mech_status.spindle_rev = 0; //����ֹͣ
            mech_flag &= ~SPINDLE_ON;
            
            mech_status.stat = STA_PAUSED;
           }
         break;

    case STA_STOPPING: //�н��˶�ֹͣ����,���ֹͣ�����Ƿ����
         if((interpl_segment_curr->basecmd == PROGRAM_IDLE)
		          && (interpl_segment_prev->basecmd == PROGRAM_IDLE))
           {
            mech_status.spindle_rev = 0;
            mech_flag &= ~SPINDLE_ON;
           
            mech_status.stat = STA_STOPPED;
            ctrl_flag &= (~BEING_AUTO_RUN);                   
           }
         break;

    case STA_ESTOPPING: //�нӼ�ͣ�������жϼ�ͣ�����Ƿ����
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

