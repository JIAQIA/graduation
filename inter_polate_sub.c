/*****************************************************************************/
//插补程序底层函数模块
/*****************************************************************************/
#include "math.h"
#include "string.h"

#include "key_code_def.h"
#include "inter_polate_def.h"

/*****************************************************************/
/* 函数: 多轴线性速度控制                                        */
/* 入口:                                                         */
/* 返回值:                                                       */
/* 注释:                                                         */
/*****************************************************************/
void linear_vel_ctrl(typ_interp_time * interp_time) 
{
 interp_time->motion_stat = interp_time->motion_stat_next;

 switch(interp_time->motion_stat)
   {
    case INP_MOTION_ACC:      //加速阶段

         //计算已经运行的周期数以及剩余周期数
  		 interp_time->term_run += 1; // interp_time->interp_increment;  
         interp_time->term_rem -= 1;

   		   //计算一个插补周期内新的时间长度(速度)
         interp_time->time_length += interp_time->time_delta;
         if(interp_time->time_length >1.0) interp_time->time_length =1.0;

         //判断加速过程是否结束
		   if(interp_time->term_run >= interp_time->term_add)
           {
            interp_time->motion_stat_next = INP_MOTION_UNI;
           }            
  		   if(interp_time->term_run >= interp_time->term_cmd)
           {
            interp_time->motion_stat_next = INP_MOTION_DEC;
           }
         break;
    case INP_MOTION_UNI:    //匀速阶段

         interp_time->term_run += 1;
         interp_time->term_rem -= 1;
  		   if(interp_time->term_run >= interp_time->term_cmd)
           {
            interp_time->motion_stat_next = INP_MOTION_DEC;
           }
         break;
    case INP_MOTION_DEC:    //减速阶段 
         interp_time->term_rem -= 1;
         if(interp_time->term_rem <= 0)   //<=0
           {
            interp_time->term_rem = 0;
            //interp_time->motion_stat_next = INP_MOTION_END;
           }
         interp_time->time_length -= interp_time->time_delta;
         if(interp_time->time_length < interp_time->time_delta)
           { 
            interp_time->time_length = interp_time->time_delta;
           }
         
         break;
    case INP_MOTION_END:   //运动停止
         interp_time->time_length = 0.0;
         interp_time->term_rem = 0;
 		 break;
    default:
         break;
   }//end of switch
}  //end of sub velocity_ctrl()

/*****************************************************************/
/*                     直线加工插补处理                          */
/*                                                               */
/*                                                               */
/*                                                               */
/*****************************************************************/
void line_interp_deal(typ_interp_segment * interpl_segment) 
{
 int axis_idx;
 double step_l;
   
 linear_vel_ctrl((typ_interp_time *)&interpl_segment->interp_time);
 
 if(interpl_segment->interp_time.motion_stat == INP_MOTION_END)
   {
    for(axis_idx = 0;axis_idx<AXIS_SUM; axis_idx++)
      {
       interpl_segment->dist_out[axis_idx] = 0.0; 
      }
    interpl_segment->basecmd = PROGRAM_IDLE;   
   }
 else 
   {
    //step_l = interpl_segment->vel * interpl_segment->interp_time.time_length;
    
    /*if(interpl_segment->dist <= step_l)
      {
       for(axis_idx = 0;axis_idx<AXIS_SUM; axis_idx++)
        {
         interpl_segment->dist_out[axis_idx] = interpl_segment->axis_dist[axis_idx] \
                                           - interpl_segment->axis_seg[axis_idx];
        }
       interpl_segment->interp_time.motion_stat_next = INP_MOTION_END;
      }
    else
      {       
       interpl_segment->dist -= step_l;
    */   
       if(interpl_segment->interp_time.term_rem<=1)
          interpl_segment->interp_time.motion_stat_next = INP_MOTION_END;
                               
       for(axis_idx = 0; axis_idx < AXIS_SUM; axis_idx++)
        {
         interpl_segment->dist_out[axis_idx] = interpl_segment->axis_vel[axis_idx] \
                                        * interpl_segment->interp_time.time_length;
         interpl_segment->axis_seg[axis_idx] += interpl_segment->dist_out[axis_idx];
        }
     // }
   }
}
/*****************************************************************/
/* 圆弧插补                                                      */
/* 通过插补将dist_out[]输出?也就是插补其实是为了得到正确的       */
/* dist_out[]                                                    */
/* ×￠êí:                                                        */
/*****************************************************************/
void arc_interp_deal(typ_interp_segment * interpl_segment) 
{
 double step_a, step_sqr, step_qua;
   
 linear_vel_ctrl((typ_interp_time *)&interpl_segment->interp_time); 
 
 if(interpl_segment->interp_time.motion_stat == INP_MOTION_END)
   {
    interpl_segment->dist_out[0]= 0.0;
    interpl_segment->dist_out[1]= 0.0;
    interpl_segment->dist_out[2]= 0.0;
    interpl_segment->basecmd = PROGRAM_IDLE;   
   } 
 else
   {
    step_a = interpl_segment->vel * interpl_segment->interp_time.time_length;
    
    if(interpl_segment->dist <= step_a)
     {    
      interpl_segment->dist_out[0] = interpl_segment->axis_dist[0]\
                                   - interpl_segment->axis_vel[0];
      interpl_segment->dist_out[1] = interpl_segment->axis_dist[1]\
                                   - interpl_segment->axis_vel[1];
      interpl_segment->dist_out[2] = 0.0; 
    
      interpl_segment->interp_time.motion_stat_next = INP_MOTION_END;    
     }      
    else
     {      
      interpl_segment->dist -= step_a;
  
      //step_a = 2.0 * sin(step_a/2.0); //////
   
	  step_sqr = step_a*step_a;
	  step_qua = sqrt(1-step_sqr/4)*step_a;
	  step_sqr /= 2.0;
	 
	  if(interpl_segment->basecmd == G_03)
	   {
	    interpl_segment->dist_out[0] = -step_sqr*interpl_segment->axis_vel[0]\
	                                  -step_qua*interpl_segment->axis_vel[1]; 
	    interpl_segment->dist_out[1] = step_qua*interpl_segment->axis_vel[0]\
	                                  -step_sqr*interpl_segment->axis_vel[1]; 
	    interpl_segment->dist_out[2] = 0.0;
	   } 
	  if(interpl_segment->basecmd == G_02)
	   {
	    interpl_segment->dist_out[0] = -step_sqr*interpl_segment->axis_vel[0]\
	                                  +step_qua*interpl_segment->axis_vel[1]; 
	    interpl_segment->dist_out[1] = -step_qua*interpl_segment->axis_vel[0]\
	                                  -step_sqr*interpl_segment->axis_vel[1]; 
	    interpl_segment->dist_out[2] = 0.0;    
	   }
	  interpl_segment->axis_vel[0] += interpl_segment->dist_out[0];
	  interpl_segment->axis_vel[1] += interpl_segment->dist_out[1];
	  //interpl_segment->axis_vel[2] += interpl_segment->dist_out[2]; 
     } 
   }          
}  

/*****************************************************************/
/* oˉêy: ?±??2?213ìDò1???                     直线插补规划       */
/* è??ú:                                                         */
/* ·μ???μ:                                                       */
/* ×￠êí:                                                         */
/*****************************************************************/
void line_interp_plan(typ_interp_segment * interpl_segment)   
{
 unsigned int axis_idx;
 unsigned int add_num;
 double temp_vel;
 
 if(interpl_segment->dist > LINE_MIN_DIST)	 //当前距离可插补
   { 
    if(interpl_segment->basecmd == G_00) temp_vel = user_para.swift_vel;
    else temp_vel = user_para.feed_vel;
                             
    //算出当前的运行速度
    temp_vel = temp_vel * INTERP_CYC * user_para.speed_scal / 100.0;			 //速度*主轴转速修调系数 *插补周期/100 	   貌似一个周期的运动距离
    //本段计划运行周期数
    interpl_segment->interp_time.term_cmd = (long)(1 + interpl_segment->dist / temp_vel);	 //本段计划运行周期数
	//再算回来本周期的运动距离
    temp_vel = interpl_segment->dist / (double)interpl_segment->interp_time.term_cmd;	  //再算回来本周期运动距离

    interpl_segment->interp_time.term_add = 3; //加速周期数 3

    for(axis_idx = 0; axis_idx < AXIS_SUM; axis_idx ++)
       {
	     //实际速度在各轴的分量,有符号数,=本周期运动速度*当前段各轴(尚未完成的)运动分量/当前段空间总位移5-11
        interpl_segment->axis_vel[axis_idx] = temp_vel * interpl_segment->axis_dist[axis_idx] / interpl_segment->dist;
		 //	实际速度在各轴的分量,有符号数,用单个插补周期的位移量表示。=本周期运动距离*当前段各轴（尚未完成的）运动分量/当前段空间总位移	  （？）
        interpl_segment->axis_seg[axis_idx] = 0.0;  //axis_seg clear 0
        // axis_vel*(freq^2)/允许的最大加速度 axis_vel是用位移表示的速度,速度除以两个时间单位是m/s2的量纲
		// 得到的add_num是加速度的周期数
        add_num = (unsigned int)(1 + fabs(interpl_segment->axis_vel[axis_idx]) * INTERP_FREQ_2 / mech_para.max_acc[axis_idx]);
        
        if(interpl_segment->interp_time.term_add < add_num)
          {
           interpl_segment->interp_time.term_add = add_num;//调整加速周期数
          }
       } 
           
    //判断如果加速度周期比运行周期要长   
    if(interpl_segment->interp_time.term_add > interpl_segment->interp_time.term_cmd)
      {
       //将目标速度降下来,也就是在一定加速度的情况下,需要的加速度周期就少了,或者是要完成一定的位移,需要的运行周期就长了
       temp_vel *= sqrt((double)interpl_segment->interp_time.term_cmd / (double)interpl_segment->interp_time.term_add);
       interpl_segment->interp_time.term_cmd = (long)(1 + interpl_segment->dist / temp_vel);

       //对加速度周期进行调整
       interpl_segment->interp_time.term_add = interpl_segment->interp_time.term_cmd;

       //重新算回速度值
       temp_vel = interpl_segment->dist / (double)interpl_segment->interp_time.term_cmd;

        //算出当前速度在各轴的分量   
       for(axis_idx = 0; axis_idx < AXIS_SUM; axis_idx ++)
          {     
           interpl_segment->axis_vel[axis_idx] = temp_vel * interpl_segment->axis_dist[axis_idx] / interpl_segment->dist;
          }
      }
      
    interpl_segment->vel = temp_vel;      
    
    //????????2?21?ü?úμ??ó?ù?è·?á?￡¨ó?ê±??±íê?￡?
    interpl_segment->interp_time.time_delta = 1.0 / interpl_segment->interp_time.term_add;
    interpl_segment->interp_time.time_length = 0.0;         
    interpl_segment->interp_time.term_run = 0;
    interpl_segment->interp_time.term_rem = interpl_segment->interp_time.term_cmd + interpl_segment->interp_time.term_add;
	interpl_segment->interp_time.motion_stat = INP_MOTION_ACC;
	interpl_segment->interp_time.motion_stat_next = INP_MOTION_ACC;
   } //end of if(interpl_segment->dist > MIN_DIST)
   
 else  //if(interpl_segment->dist < MIN_DIST)
   {
    //for(axis_idx = 0; axis_idx < AXIS_SUM; axis_idx ++)
    //   {
    //    mech_status.dist_tail[axis_idx] += interpl_segment->axis_dist[axis_idx];
    //   }      
    interpl_segment->interp_time.term_cmd = 0;
	interpl_segment->interp_time.term_add = 0;
    interpl_segment->interp_time.term_rem = 0;
    interpl_segment->interp_time.motion_stat = INP_MOTION_END;
    interpl_segment->interp_time.motion_stat_next = INP_MOTION_END;
   }
}

/*****************************************************************/
/*****************************************************************/
/* 函数: 程序段规划处理                                          */
/* 入口:                                                         */
/* 返回值:                                                       */
/* 注释:                                                         */
/*****************************************************************/
int segment_plan(typ_interp_segment *interpl_segment)   
{
 switch(interpl_segment->basecmd)
   {
    case G_00:
	     OpenLaser = 0;										  //快移速度
         interpl_segment->vel = user_para.swift_vel;   
         line_interp_plan(interpl_segment); 
         return(1);   
    case G_01:
	     OpenLaser = 1;
         interpl_segment->vel = user_para.feed_vel; 	 //工进速度
       	 line_interp_plan(interpl_segment);        
		 return(1);
	case G_02:	
    case G_03:
	     OpenLaser = 1;
    	 interpl_segment->vel = user_para.feed_vel;
       	 arc_interp_plan(interpl_segment);				   //圆弧插补
       	 return(1);
           
    case G_DWELL:
         interpl_segment->interp_time.term_rem 
                    = (long)(interpl_segment->dist / INTERP_CYC_MS);	
						//剩余插补周期计数器 （当前段空间总位移（mm）或延时时间（秒）/ ms为单位的查补周期 4ms
         return(1); 
         
    case G_04:
         return(1); 
         
    case PROGRAM_IDLE:
         interpl_segment->interp_time.motion_stat = INP_MOTION_END;	  //运动停止
         interpl_segment->interp_time.term_rem = 0;					  //剩余插补周期计数器
         return(1);
         
    case PROGRAM_END:
         interpl_segment->interp_time.motion_stat = INP_MOTION_END;
         interpl_segment->interp_time.term_rem = 0;
         return(1);
          //logic_cmd_treat();    //开关命令处理
    default: return(0);
   } // end of switch
  return(0); 
}


/**************************************************************************/
/* 函数:程序段加工处理                                                    */
/* 入口:                                                                  */
/* 返回值:                                                                */
/* 注释: 计算一个插补周期内程序段的输出位移量 interpl_segment->dist_out[] */
/**************************************************************************/ 
void segment_process(typ_interp_segment *interpl_segment) 
{
 unsigned int axis_idx;

 switch(interpl_segment->basecmd)
   {
    case G_00: 
    case G_01: 
         line_interp_deal(interpl_segment);         
  		 break;
  		 
    case G_02:
    case G_03:
         arc_interp_deal(interpl_segment);
         break;
         
    case G_DWELL:
         for(axis_idx = 0; axis_idx < AXIS_SUM; axis_idx ++) 
		    {
		     interpl_segment->dist_out[axis_idx] = 0.0;
            }
		 interpl_segment->interp_time.term_rem--;
         if(interpl_segment->interp_time.term_rem <= 0)
           {
            interpl_segment->basecmd = PROGRAM_IDLE;
           }
	     break;

    case PROGRAM_IDLE:
    case PROGRAM_END:
    default: 
  	     for(axis_idx = 0; axis_idx < AXIS_SUM; axis_idx ++)
            {
             interpl_segment->dist_out[axis_idx]=0.0;
            }         
         break;
   }//end of switch
}
	
//********************************************************************
/*****************************************************************/
/* 函数名:圆弧插补规划                                           */
/* è??ú:                                                         */
/* ·μ???μ:                                                       */
/* ×￠êí:                                                        */
/*****************************************************************/
void arc_interp_plan(typ_interp_segment * interpl_segment)   
{
 unsigned int add_num;
 double temp_vel, temp_acc;
 
 if(interpl_segment->dist > ARC_MIN_ANGLE)
   {                          
    //μ±?°μ???±ê?ù?è(ó?μ￥???ü?ú?úμ???ò?á?±í′?)
    temp_vel = user_para.feed_vel;
        
    temp_vel = temp_vel * INTERP_CYC * user_para.speed_scal / 100.0;
    
    if(temp_vel > interpl_segment->axis_dist[1]) temp_vel = interpl_segment->axis_dist[1]; 
 
    temp_vel /= interpl_segment->axis_dist[2];        

    //3????????ü?úêy
    interpl_segment->interp_time.term_cmd = (long)(1 + interpl_segment->dist / temp_vel);

    //·′??êμ?ê?ù?è(ó?μ￥???ü?ú?úμ???ò?á?±í′?)
    temp_vel = interpl_segment->dist / (double)interpl_segment->interp_time.term_cmd;

    //?????÷?á?ù?è·?á?(ó?μ￥???ü?ú?úμ???ò?á?±í′?,óD·?o?êy)ò??°?ó?ù1y3ì?ùDèμ??ü?úêy
    interpl_segment->interp_time.term_add = 3; //?ó?ù?ü?úêy×?D??μéè?a3(?ò?ü′ó),????ò???ìá1?×?1?μ?1???ê±??
 
    //if((interpl_segment->basecmd & X_Y_PLAIN) == X_Y_PLAIN){;} 
    //if((interpl_segment->basecmd & Y_Z_PLAIN) == Y_Z_PLAIN){;}   
    //if((interpl_segment->basecmd & Z_X_PLAIN) == Z_X_PLAIN){;}     
    temp_acc = mech_para.max_acc[0];
    temp_acc /= interpl_segment->axis_dist[2];
    add_num = (unsigned int)(1 + temp_vel * INTERP_FREQ_2 / temp_acc); 
       
    if(interpl_segment->interp_time.term_add < add_num)
      {
       interpl_segment->interp_time.term_add = add_num;
      }  
         
    //è?1??ó?ùê±??′óóú??DDê±??￡??òò??°?ó?ù?ü?úoí??DD?ü?úêy￡¨?t???àμè￡?    
    if(interpl_segment->interp_time.term_add > interpl_segment->interp_time.term_cmd)
      {
       //?ó?a?ù?ü′?μ?μ?×?′ó?ù?è(ó?μ￥???ü?ú?úμ???ò?á?±í′?)
       temp_vel *= sqrt((double)interpl_segment->interp_time.term_cmd / (double)interpl_segment->interp_time.term_add);
       interpl_segment->interp_time.term_cmd = (long)(1 + interpl_segment->dist / temp_vel);

       //?ó?ù1y3ì?ùDèμ??ü?úêy
       interpl_segment->interp_time.term_add = interpl_segment->interp_time.term_cmd;

       //??D?·′??êμ?ê?ù?è(ó?μ￥???ü?ú?úμ???ò?á?±í′?)
       temp_vel = interpl_segment->dist / (double)interpl_segment->interp_time.term_cmd;
      }
    
    //êμ?ê2?21?ù?è??ò?á?￡¨?ü?ú????ò?á?￡?
    interpl_segment->vel = temp_vel;
    
    interpl_segment->axis_dist[0] = PI/2.0 - interpl_segment->axis_dist[0];   //roate the axis
    
    //?????eê????èμ?èy??oˉêyoí?ü?ú??ò?á?μ?èy??êy
    interpl_segment->axis_vel[0] = cos(interpl_segment->axis_dist[0])*interpl_segment->axis_dist[2];
    interpl_segment->axis_vel[1] = sin(interpl_segment->axis_dist[0])*interpl_segment->axis_dist[2];
    interpl_segment->axis_vel[2]= 0.0;

    //????????2?21?ü?úμ??ó?ù?è·?á?￡¨ó?ê±??±íê?￡?
    interpl_segment->interp_time.time_delta = 1.0 / interpl_segment->interp_time.term_add;
    interpl_segment->interp_time.time_length = 0.0;         
    interpl_segment->interp_time.term_run = 0;
    interpl_segment->interp_time.term_rem = interpl_segment->interp_time.term_cmd + interpl_segment->interp_time.term_add;
	interpl_segment->interp_time.motion_stat = INP_MOTION_ACC; 
	interpl_segment->interp_time.motion_stat_next = INP_MOTION_ACC; 
	
	temp_vel = interpl_segment->axis_dist[0];
	temp_acc = interpl_segment->axis_dist[2];
	
	//??????±êμ?μ?×?±ê
	if(interpl_segment->basecmd == G_03)
	  {
	   temp_vel += interpl_segment->dist;
	  }
	else
	  {
	   temp_vel -= interpl_segment->dist;
	  }
	 
	//if(temp_vel >= PI_2) temp_vel -= PI_2;
	//if(temp_vel < 0)   temp_vel += PI_2;
	  
	interpl_segment->axis_dist[0] = cos(temp_vel)*temp_acc; 
	interpl_segment->axis_dist[1] = sin(temp_vel)*temp_acc; 
	interpl_segment->axis_dist[2] = 0.0;
	   
   } //end of if(interpl_segment->dist > MIN_DIST)
   
 else  //±??ü?ú′ú??2?1???￡??÷×?±ê?áμ???ò?2￠è??2êy??′??÷
   {
    //for(axis_idx = 0; axis_idx < AXIS_SUM; axis_idx ++)
    //   {
    //    mech_status.dist_tail[axis_idx] += interpl_segment->axis_dist[axis_idx];
    //   }      
    interpl_segment->interp_time.term_cmd = 0;
	interpl_segment->interp_time.term_add = 0;
    interpl_segment->interp_time.term_rem = 0;
    interpl_segment->interp_time.motion_stat = INP_MOTION_END;
    interpl_segment->interp_time.motion_stat_next = INP_MOTION_END;
   }
   
 interpl_segment->dist_out[0]= 0.0;
 interpl_segment->dist_out[1]= 0.0;
 interpl_segment->dist_out[2]= 0.0; 
 
}