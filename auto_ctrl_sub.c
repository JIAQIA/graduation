/********************************************************************************/
//自动运行模块
/********************************************************************************/ 
#include "math.h"
#include "string.h"
#include "stdlib.h"
#include "data_deal_def.h"
#include "auto_ctrl_def.h"

#include "key_code_def.h"
#include "inter_polate_def.h"

//#include "Basic_IO_cfg.h"

//-------加工代码段变量定义---------------------------

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
//函数:  段存储结构初始化调                                            
//入口:                                                        
//返回值
//注释: 
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
//函数:  线性速度修调                                            
//入口:  被修调部分的存放位置                                                              
//返回值:1-进行了修调，0-未进行修调                                                       
//注释:  将需要修调的段分为两部份，当前段保留一部分，并尽快转为减速
//       剩余部分填入下一段，准备按照新的速度修调量重新规划
/*****************************************************************/   
int linear_vel_override(typ_interp_segment * modify_pointer)
{
 int axis_idx;

 //不是直线插补，不进行修调
 if(interpl_segment_curr->basecmd != G_00)
   {
    mech_status.speed_scal_old = user_para.speed_scal;
    return(0);
   }

 //剩余部分长度不够，不修调( override_echo_term 至少必须大于减速的周期数＋10)
 if(interpl_segment_curr->interp_time.term_rem <= user_para.override_echo_term)
   {
    mech_status.speed_scal_old = user_para.speed_scal;
    return(0);
   }

 //进入减速阶段不修调
 //if(interpl_segment_curr->interp_time[0].motion_stat_next == INP_MOTION_DEC)
 //  {
 //   speed_scal_old = speed_scal;
 //   return(0);
 //  }

 //即将进入减速阶段不修调（差10个周期）
 if(interpl_segment_curr->interp_time.term_rem <= (interpl_segment_curr->interp_time.term_add + 10))
   {
    mech_status.speed_scal_old = user_para.speed_scal;
    return(0);
   }

 //只有在匀速阶段才允许修调 
 if(interpl_segment_curr->interp_time.motion_stat_next == INP_MOTION_UNI)
   { 	 
    mech_status.speed_scal_old = user_para.speed_scal;
    
    //当前段复制一份到修正段
    memcpy(modify_pointer, interpl_segment_curr, sizeof(typ_interp_segment));

    //5个周期后当前段变为减速(＋5为了确保next段重新规划的时间)
    interpl_segment_curr->interp_time.term_cmd = interpl_segment_curr->interp_time.term_run + 5;
    interpl_segment_curr->interp_time.term_rem = interpl_segment_curr->interp_time.term_add + 5;

    //剩余部分由下一段按照新的速度重新规划
    modify_pointer->interp_time.term_cmd -= interpl_segment_curr->interp_time.term_cmd;
    
    //计算下一段中各轴（尚未完成的）运动分量，有符号数
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
//函数：从当前位置到达指定的位置
//入口：position[]指定要到达的位置，给出空间坐标
//      axis_num: 指定需要运动的轴号，0001-X,0002-Y,0004-Z
//出口：将上述运动转换为待插补直线段放入interpl_segment_insert段中
//说明：将返回暂停点的运动形成可执行程序段加载到next段，并在curr段插入必要的延时
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
//函数：插入运动延时到当前插补段
//入口：时间单位为 S
//出口：
//说明：在curr段插入必要的延时
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
//函数：减速停车
//入口:
//返回值:
//注释:                                                        
/*****************************************************************/
void motion_abolish(void) 
{ 
 interpl_segment_curr->interp_time.term_cmd = interpl_segment_curr->interp_time.term_run;
 interpl_segment_curr->interp_time.term_rem = interpl_segment_curr->interp_time.term_add;
 interpl_segment_curr->interp_time.motion_stat_next = INP_MOTION_DEC;

 ctrl_flag &= (~BLEND_PERMIT); //禁止程序段转接
}

/***********************************************************************/
// 函数: 程序段预处理 (谥骱分械饔�)                                                   
// 入口:                                                               
// 返回值:                                                             
// 注释: 本函数将seg_label指定的程序段进行解释，运动控制量加载到interpl_segment_next段中
//       开关控制量加载到plc_buffer中，并将运动延时时间加载到dwell_term
/***********************************************************************/
void code_seg_pre_treat(void) 
{
 if(ctrl_flag & INSERT_SEG_FULL)     //补充插入段存在，则优先对其进行规划
   {
    if(!(ctrl_flag & INSERT_SEG_READY))		 //若未规划完成
      {
       segment_plan(interpl_segment_insert);
       ctrl_flag |= INSERT_SEG_READY; 
      }
    return;  
   }
   
 if(ctrl_flag & NEXT_SEG_RENEW)    //正常程序段规划 
   { 
    if(segment_plan(interpl_segment_next)) //成功规划
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

 if(ctrl_flag & NEXT_SEG_EMPTY)              //程序段加载到next段
   {
    if(mech_code_load(interpl_segment_next))    //程序代码加载成功 
      {
       ctrl_flag &= (~NEXT_SEG_EMPTY);  
       ctrl_flag |= NEXT_SEG_RENEW; 
      }
    return;  
   }  
}

/*****************************************************************/
/* 函数:自动运行中的按键处理                                     */
/* 入口:                                                         */
/* 返回值:                                                       */
/* 注释:                                                         */
/*****************************************************************/ 
/*3.12屏蔽
void auto_key_echo(unsigned int key_value) 
{
 switch(key_value)
   {
    case CMD_RUN:    //启动运行
         //开始新的加工
         if(mech_status.stat == STA_FINISHED)
           {
            public_para_init(1);				//公共变量初始化
            //ctrl_flag = 0x0000;
            mech_status.stat = STA_READY;
           } 
         if(mech_status.stat == STA_READY)    
           {
            mech_flag &= ~SIMULATION;
            user_para.speed_scal = speed_scal;
            
            code_buf_init(); 					   //代码队列初始化
 	        interpl_segment_prev->basecmd = PROGRAM_IDLE;
		    interpl_segment_curr->label = 0;

            //插入开主轴延时
            insert_dwell_time(user_para.spindle_on_time);
            mech_status.spindle_rev = (int)(user_para.spindle_rev * user_para.spindle_scal/100);
            mech_flag |= SPINDLE_ON;
            //-----------------------------------------------------------
            ctrl_flag |= BEING_AUTO_RUN;
           	ctrl_flag &= ~TASK_FINISH;  
            ctrl_flag |= BLEND_PERMIT;   
		    ctrl_flag |= NEXT_SEG_EMPTY;   	   
            ctrl_flag &= ~CODE_BUF_READY;   
		    mech_status.stat = STA_WORKING;  //进入工作状态
     
            break;
           }         
         //--------------------------------------------------------------
         //从暂停中恢复
         if(mech_status.stat == STA_PAUSED)
		   {
		    //如果主轴未开，插入开主轴延时
            if(!(mech_flag & SPINDLE_ON) && !(mech_flag & SIMULATION))
              {
		       insert_dwell_time(user_para.spindle_on_time);
		       mech_status.spindle_rev = (int)(user_para.spindle_rev * user_para.spindle_scal/100);
               mech_flag |= SPINDLE_ON;
			  }
            //--------------------------------------------------- 
   		    ctrl_flag |= BLEND_PERMIT;         //blend_permit = ST_TRUE;
            mech_status.stat = STA_WORKING;    //进入工作状态
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
		 mech_status.stat = STA_WORKING;  //进入工作状态
         break;           

    case CMD_PAUSE:        //运动暂停
         if(mech_status.stat == STA_WORKING)
           {
		    ctrl_flag &= (~BLEND_PERMIT); //blend_permit = ST_FAULSE;  // 禁止程序段转接，本段完成后自然暂停
    		mech_status.stat = STA_PAUSING;
           }    
         break;

    case CMD_STOP:         //停止自动运行
	     if(mech_status.stat == STA_WORKING)
           {
	        ctrl_flag &= (~BLEND_PERMIT); //blend_permit = ST_FAULSE;
		    mech_status.stat = STA_STOPPING;
		   }
         if(mech_status.stat == STA_PAUSED)  mech_status.stat = STA_STOPPED;
	     if(mech_status.stat == STA_FINISHED)  mech_status.stat = STA_STOPPED;
     	 break;

    case CMD_ESTOP:       //立即停车
         if(interpl_segment_curr->interp_time.motion_stat > INP_MOTION_DEC)
           {
	        motion_abolish();
           }
         
         ctrl_flag &= (~BLEND_PERMIT); //禁止程序段转接 
         mech_status.stat = STA_ESTOPPING;
         break;

    default: break;  
   }   //end of switch
} 
*/
//3.15重写
void auto_ctrl()      //检测到“自动”键按下，进行一下自动运行前的处理
{
		public_para_init(1);				//公共变量初始化
		user_para.speed_scal = speed_scal;	   //设置速度
	    code_buf_init(); 					   //代码队列初始化
 	    interpl_segment_prev->basecmd = PROGRAM_IDLE;	 //加载程序段功能字
		interpl_segment_curr->label = 0;				 //段标号置零
		//插入开主轴延时
		insert_dwell_time(user_para.spindle_on_time);
		mech_status.spindle_rev = (int)(user_para.spindle_rev * user_para.spindle_scal/100); //获得主轴转速
		mech_flag |= SPINDLE_ON;
		//-----------------------------------------------------------
		ctrl_flag |= BEING_AUTO_RUN;
		ctrl_flag &= ~TASK_FINISH;  
		ctrl_flag |= BLEND_PERMIT;   
		ctrl_flag |= NEXT_SEG_EMPTY;   	   
		ctrl_flag &= ~CODE_BUF_READY;   
		mech_status.stat = STA_WORKING;  //进入工作状态
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
/* 函数:自动运行程序                                             */
/* 入口:                                                         */
/* 返回值:                                                       */
/* 注释:                                                         */
/*****************************************************************/ 
void auto_process(void)
{
 unsigned int axis_idx;

 //-------------程序段自动插补处理-----------------------------------
 segment_process(interpl_segment_curr);
 segment_process(interpl_segment_prev);
 
 //计算一个插补周期内各轴输出的合成位移量
 for(axis_idx = 0; axis_idx < AXIS_SUM; axis_idx ++)
    {
     mech_status.dist_sum[axis_idx] = interpl_segment_curr->dist_out[axis_idx] 
                                    + interpl_segment_prev->dist_out[axis_idx];
    }   
 //-------------程序段自动转接处理----------------------------------- 
 
 if(ctrl_flag & BLEND_PERMIT)
   {
    //只有当前段剩余周期数小于转接周期数且转接允许才考虑转接
    if((interpl_segment_curr->interp_time.term_rem <= user_para.blend_time)
        &&(interpl_segment_prev->basecmd==PROGRAM_IDLE)) 
      {
	   //------只要插入段存在，就不进行正常转接，以防出错----------------
       if(ctrl_flag & INSERT_SEG_FULL)  
         {        
          if(ctrl_flag & INSERT_SEG_READY) //只有额外断准备好才插入额外的段
            {
             temp_segment_pointer = interpl_segment_prev;	  //程序段转接		
             interpl_segment_prev = interpl_segment_curr;    
	         interpl_segment_curr = interpl_segment_insert;
             interpl_segment_insert = temp_segment_pointer;

             interpl_segment_insert->basecmd = PROGRAM_IDLE;
             ctrl_flag &= (~INSERT_SEG_READY);  
             ctrl_flag &= (~INSERT_SEG_FULL);   
		    } 
	     } //end of if(insert_seg_full) 

	     //-------下一段准备好时进行正常转接-------------------------------	     
       else if(ctrl_flag & NEXT_SEG_READY) 
	     {
          temp_segment_pointer = interpl_segment_prev;	  //程序段转接		
          interpl_segment_prev = interpl_segment_curr;    
          interpl_segment_curr = interpl_segment_next;
          interpl_segment_next = temp_segment_pointer;

          ctrl_flag &= ~NEXT_SEG_READY; 
          ctrl_flag |= NEXT_SEG_EMPTY;           
		  mech_status.label = interpl_segment_curr->label; //更新段标号
 	     } //end of else if(next_seg_ready) 
       //-----------------------------------------------------------------
       else{;}
      } // end of if(interpl_segment_prev->basecmd == PROGRAM_IDLE)
   } //end of  if((interpl_segment_curr->interp_time[0].term_rem <= blend_term) && (blend_permit))    
  

 //-------------自动速度修调处理----------------------------------
 //修调段放入插入段，所以要求interpl_segment_insert为空
 if(user_para.speed_scal != mech_status.speed_scal_old)
   {
    if(!(ctrl_flag & INSERT_SEG_FULL))
      { 
	     //if(linear_vel_override(interpl_segment_insert)) //如果进行速度修调处理,修调段放入interpl_segment_insert 		   
	       {
	        //ctrl_flag |= INSERT_SEG_FULL;
	       }
      } 
   }   
 
 //-------------状态检测处理-------------------------------------
 switch(mech_status.stat)
   {
    case STA_WORKING:
         if(!(ctrl_flag & TASK_FINISH)) break;
		 if((interpl_segment_prev->basecmd == PROGRAM_IDLE)  //检测任务完成
               && (interpl_segment_curr->basecmd == PROGRAM_IDLE))          
           {
            mech_status.spindle_rev = 0;
            mech_flag &= ~SPINDLE_ON;
            
            mech_status.stat = STA_FINISHED; 
            ctrl_flag &= (~BEING_AUTO_RUN);                 
			ctrl_flag &= (~BLEND_PERMIT);          
           }
		 break;     
    case STA_PAUSING: //承接运动暂停命令,检测暂停是否结束
         if((interpl_segment_curr->basecmd == PROGRAM_IDLE)   
		       && (interpl_segment_prev->basecmd == PROGRAM_IDLE)) //暂停完成
           {
            mech_status.spindle_rev = 0; //主轴停止
            mech_flag &= ~SPINDLE_ON;
            
            mech_status.stat = STA_PAUSED;
           }
         break;

    case STA_STOPPING: //承接运动停止操作,检测停止过程是否结束
         if((interpl_segment_curr->basecmd == PROGRAM_IDLE)
		          && (interpl_segment_prev->basecmd == PROGRAM_IDLE))
           {
            mech_status.spindle_rev = 0;
            mech_flag &= ~SPINDLE_ON;
           
            mech_status.stat = STA_STOPPED;
            ctrl_flag &= (~BEING_AUTO_RUN);                   
           }
         break;

    case STA_ESTOPPING: //承接急停操作，判断急停过程是否结束
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

