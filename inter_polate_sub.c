/*****************************************************************************/
//�岹����ײ㺯��ģ��
/*****************************************************************************/
#include "math.h"
#include "string.h"

#include "key_code_def.h"
#include "inter_polate_def.h"

/*****************************************************************/
/* ����: ���������ٶȿ���                                        */
/* ���:                                                         */
/* ����ֵ:                                                       */
/* ע��:                                                         */
/*****************************************************************/
void linear_vel_ctrl(typ_interp_time * interp_time) 
{
 interp_time->motion_stat = interp_time->motion_stat_next;

 switch(interp_time->motion_stat)
   {
    case INP_MOTION_ACC:      //���ٽ׶�

         //�����Ѿ����е��������Լ�ʣ��������
  		 interp_time->term_run += 1; // interp_time->interp_increment;  
         interp_time->term_rem -= 1;

   		   //����һ���岹�������µ�ʱ�䳤��(�ٶ�)
         interp_time->time_length += interp_time->time_delta;
         if(interp_time->time_length >1.0) interp_time->time_length =1.0;

         //�жϼ��ٹ����Ƿ����
		   if(interp_time->term_run >= interp_time->term_add)
           {
            interp_time->motion_stat_next = INP_MOTION_UNI;
           }            
  		   if(interp_time->term_run >= interp_time->term_cmd)
           {
            interp_time->motion_stat_next = INP_MOTION_DEC;
           }
         break;
    case INP_MOTION_UNI:    //���ٽ׶�

         interp_time->term_run += 1;
         interp_time->term_rem -= 1;
  		   if(interp_time->term_run >= interp_time->term_cmd)
           {
            interp_time->motion_stat_next = INP_MOTION_DEC;
           }
         break;
    case INP_MOTION_DEC:    //���ٽ׶� 
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
    case INP_MOTION_END:   //�˶�ֹͣ
         interp_time->time_length = 0.0;
         interp_time->term_rem = 0;
 		 break;
    default:
         break;
   }//end of switch
}  //end of sub velocity_ctrl()

/*****************************************************************/
/*                     ֱ�߼ӹ��岹����                          */
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
/* Բ���岹                                                      */
/* ͨ���岹��dist_out[]���?Ҳ���ǲ岹��ʵ��Ϊ�˵õ���ȷ��       */
/* dist_out[]                                                    */
/* ���騺��:                                                        */
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
/* o����y: ?��??2?213��D��1???                     ֱ�߲岹�滮       */
/* ��??��:                                                         */
/* ����???��:                                                       */
/* ���騺��:                                                         */
/*****************************************************************/
void line_interp_plan(typ_interp_segment * interpl_segment)   
{
 unsigned int axis_idx;
 unsigned int add_num;
 double temp_vel;
 
 if(interpl_segment->dist > LINE_MIN_DIST)	 //��ǰ����ɲ岹
   { 
    if(interpl_segment->basecmd == G_00) temp_vel = user_para.swift_vel;
    else temp_vel = user_para.feed_vel;
                             
    //�����ǰ�������ٶ�
    temp_vel = temp_vel * INTERP_CYC * user_para.speed_scal / 100.0;			 //�ٶ�*����ת���޵�ϵ�� *�岹����/100 	   ò��һ�����ڵ��˶�����
    //���μƻ�����������
    interpl_segment->interp_time.term_cmd = (long)(1 + interpl_segment->dist / temp_vel);	 //���μƻ�����������
	//������������ڵ��˶�����
    temp_vel = interpl_segment->dist / (double)interpl_segment->interp_time.term_cmd;	  //��������������˶�����

    interpl_segment->interp_time.term_add = 3; //���������� 3

    for(axis_idx = 0; axis_idx < AXIS_SUM; axis_idx ++)
       {
	     //ʵ���ٶ��ڸ���ķ���,�з�����,=�������˶��ٶ�*��ǰ�θ���(��δ��ɵ�)�˶�����/��ǰ�οռ���λ��5-11
        interpl_segment->axis_vel[axis_idx] = temp_vel * interpl_segment->axis_dist[axis_idx] / interpl_segment->dist;
		 //	ʵ���ٶ��ڸ���ķ���,�з�����,�õ����岹���ڵ�λ������ʾ��=�������˶�����*��ǰ�θ��ᣨ��δ��ɵģ��˶�����/��ǰ�οռ���λ��	  ������
        interpl_segment->axis_seg[axis_idx] = 0.0;  //axis_seg clear 0
        // axis_vel*(freq^2)/����������ٶ� axis_vel����λ�Ʊ�ʾ���ٶ�,�ٶȳ�������ʱ�䵥λ��m/s2������
		// �õ���add_num�Ǽ��ٶȵ�������
        add_num = (unsigned int)(1 + fabs(interpl_segment->axis_vel[axis_idx]) * INTERP_FREQ_2 / mech_para.max_acc[axis_idx]);
        
        if(interpl_segment->interp_time.term_add < add_num)
          {
           interpl_segment->interp_time.term_add = add_num;//��������������
          }
       } 
           
    //�ж�������ٶ����ڱ���������Ҫ��   
    if(interpl_segment->interp_time.term_add > interpl_segment->interp_time.term_cmd)
      {
       //��Ŀ���ٶȽ�����,Ҳ������һ�����ٶȵ������,��Ҫ�ļ��ٶ����ھ�����,������Ҫ���һ����λ��,��Ҫ���������ھͳ���
       temp_vel *= sqrt((double)interpl_segment->interp_time.term_cmd / (double)interpl_segment->interp_time.term_add);
       interpl_segment->interp_time.term_cmd = (long)(1 + interpl_segment->dist / temp_vel);

       //�Լ��ٶ����ڽ��е���
       interpl_segment->interp_time.term_add = interpl_segment->interp_time.term_cmd;

       //��������ٶ�ֵ
       temp_vel = interpl_segment->dist / (double)interpl_segment->interp_time.term_cmd;

        //�����ǰ�ٶ��ڸ���ķ���   
       for(axis_idx = 0; axis_idx < AXIS_SUM; axis_idx ++)
          {     
           interpl_segment->axis_vel[axis_idx] = temp_vel * interpl_segment->axis_dist[axis_idx] / interpl_segment->dist;
          }
      }
      
    interpl_segment->vel = temp_vel;      
    
    //????????2?21?��?����??��?��?����?��?�ꡧ��?����??������?��?
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
/* ����: ����ι滮����                                          */
/* ���:                                                         */
/* ����ֵ:                                                       */
/* ע��:                                                         */
/*****************************************************************/
int segment_plan(typ_interp_segment *interpl_segment)   
{
 switch(interpl_segment->basecmd)
   {
    case G_00:
	     OpenLaser = 0;										  //�����ٶ�
         interpl_segment->vel = user_para.swift_vel;   
         line_interp_plan(interpl_segment); 
         return(1);   
    case G_01:
	     OpenLaser = 1;
         interpl_segment->vel = user_para.feed_vel; 	 //�����ٶ�
       	 line_interp_plan(interpl_segment);        
		 return(1);
	case G_02:	
    case G_03:
	     OpenLaser = 1;
    	 interpl_segment->vel = user_para.feed_vel;
       	 arc_interp_plan(interpl_segment);				   //Բ���岹
       	 return(1);
           
    case G_DWELL:
         interpl_segment->interp_time.term_rem 
                    = (long)(interpl_segment->dist / INTERP_CYC_MS);	
						//ʣ��岹���ڼ����� ����ǰ�οռ���λ�ƣ�mm������ʱʱ�䣨�룩/ msΪ��λ�Ĳ鲹���� 4ms
         return(1); 
         
    case G_04:
         return(1); 
         
    case PROGRAM_IDLE:
         interpl_segment->interp_time.motion_stat = INP_MOTION_END;	  //�˶�ֹͣ
         interpl_segment->interp_time.term_rem = 0;					  //ʣ��岹���ڼ�����
         return(1);
         
    case PROGRAM_END:
         interpl_segment->interp_time.motion_stat = INP_MOTION_END;
         interpl_segment->interp_time.term_rem = 0;
         return(1);
          //logic_cmd_treat();    //���������
    default: return(0);
   } // end of switch
  return(0); 
}


/**************************************************************************/
/* ����:����μӹ�����                                                    */
/* ���:                                                                  */
/* ����ֵ:                                                                */
/* ע��: ����һ���岹�����ڳ���ε����λ���� interpl_segment->dist_out[] */
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
/* ������:Բ���岹�滮                                           */
/* ��??��:                                                         */
/* ����???��:                                                       */
/* ���騺��:                                                        */
/*****************************************************************/
void arc_interp_plan(typ_interp_segment * interpl_segment)   
{
 unsigned int add_num;
 double temp_vel, temp_acc;
 
 if(interpl_segment->dist > ARC_MIN_ANGLE)
   {                          
    //�̡�?���???����?��?��(��?�̣�???��?��?����???��?��?������?)
    temp_vel = user_para.feed_vel;
        
    temp_vel = temp_vel * INTERP_CYC * user_para.speed_scal / 100.0;
    
    if(temp_vel > interpl_segment->axis_dist[1]) temp_vel = interpl_segment->axis_dist[1]; 
 
    temp_vel /= interpl_segment->axis_dist[2];        

    //3????????��?����y
    interpl_segment->interp_time.term_cmd = (long)(1 + interpl_segment->dist / temp_vel);

    //����??����?��?��?��(��?�̣�???��?��?����???��?��?������?)
    temp_vel = interpl_segment->dist / (double)interpl_segment->interp_time.term_cmd;

    //?????��?��?��?����?��?(��?�̣�???��?��?����???��?��?������?,��D��?o?��y)��??��?��?��1y3��?��D����??��?����y
    interpl_segment->interp_time.term_add = 3; //?��?��?��?����y��?D??�̨���?a3(?��?���䨮),????��???����1?��?1?��?1???����??
 
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
         
    //��?1??��?������??�䨮����??DD����??��??����??��?��?��?��?��o��??DD?��?����y�ꡧ?t???���̨���?    
    if(interpl_segment->interp_time.term_add > interpl_segment->interp_time.term_cmd)
      {
       //?��?a?��?����?��?��?��?�䨮?��?��(��?�̣�???��?��?����???��?��?������?)
       temp_vel *= sqrt((double)interpl_segment->interp_time.term_cmd / (double)interpl_segment->interp_time.term_add);
       interpl_segment->interp_time.term_cmd = (long)(1 + interpl_segment->dist / temp_vel);

       //?��?��1y3��?��D����??��?����y
       interpl_segment->interp_time.term_add = interpl_segment->interp_time.term_cmd;

       //??D?����??����?��?��?��(��?�̣�???��?��?����???��?��?������?)
       temp_vel = interpl_segment->dist / (double)interpl_segment->interp_time.term_cmd;
      }
    
    //����?��2?21?��?��??��?��?�ꡧ?��?��????��?��?��?
    interpl_segment->vel = temp_vel;
    
    interpl_segment->axis_dist[0] = PI/2.0 - interpl_segment->axis_dist[0];   //roate the axis
    
    //?????e��????����?��y??o����yo��?��?��??��?��?��?��y??��y
    interpl_segment->axis_vel[0] = cos(interpl_segment->axis_dist[0])*interpl_segment->axis_dist[2];
    interpl_segment->axis_vel[1] = sin(interpl_segment->axis_dist[0])*interpl_segment->axis_dist[2];
    interpl_segment->axis_vel[2]= 0.0;

    //????????2?21?��?����??��?��?����?��?�ꡧ��?����??������?��?
    interpl_segment->interp_time.time_delta = 1.0 / interpl_segment->interp_time.term_add;
    interpl_segment->interp_time.time_length = 0.0;         
    interpl_segment->interp_time.term_run = 0;
    interpl_segment->interp_time.term_rem = interpl_segment->interp_time.term_cmd + interpl_segment->interp_time.term_add;
	interpl_segment->interp_time.motion_stat = INP_MOTION_ACC; 
	interpl_segment->interp_time.motion_stat_next = INP_MOTION_ACC; 
	
	temp_vel = interpl_segment->axis_dist[0];
	temp_acc = interpl_segment->axis_dist[2];
	
	//??????������?��?��?����
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
   
 else  //��??��?���䨲??2?1???��??�¡�?����?����???��?2�騨??2��y??��??��
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