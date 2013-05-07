//**********************************************
#ifndef _KEY_CODE_DEF_H_
#define _KEY_CODE_DEF_H_

//**********************************************
//------控制命令码表(无参数命令)------------------------
//一级键码表,用于系统工作方式的设定----------------------
#define FIRST_CLASS          0xee    

 #define CMD_INIT            0xeb
 #define CMD_AUTO            0xea      
 #define CMD_MANU            0xe9
 
 #define STAT_FREE           0xe7

 #define RUN_MANNER_KEY      0xe7
//-------------------------------------------------------
 
 #define CMD_OFF_LINE_WORK   0xe5   //离线工作 
 #define CMD_ON_LINE_WORK    0xe4   //在线工作
 
 #define CMD_STOP_INSTANCE   0xe3   //立即停车

 #define CMD_STATION_SCAN_EN 0xe2   //限位使能
 #define CMD_STATION_SCAN_QU 0xe1   //限位撤销
 
 #define ASK_RECE_READY      0xe0   //询问接收状态

 #define STAT_DATA_REQ       0xdf   //状态数据请求(COM请求)
 #define STAT_FLAG_REQ       0xde   //状态标志检测(COM请求)

 #define STAT_DATA_REQUEST   0xdd   //状态数据请求(USB请求)
 #define STAT_FLAG_REQUIRE   0xdc   //状态标志检测(USB请求)

 #define CMD_SET_HOME        0xda   //设为机械原点
 #define CMD_SET_PHOME       0xd9   //设为程序原点
 #define CMD_SAVE_POSITION   0xd8   //保存当前位置 

 #define CMD_TOOL_SPEED_EN   0xd7   //对刀速度使能
 #define CMD_TOOL_SPEED_QU   0xd6   //对刀速度撤销 

 #define CMD_FIEL_PARA_LOAD  0xd5   //工作现场参数加载
 #define CMD_SET_PARA_LOAD   0xd4   //用户设定参数加载
 #define CMD_DEF_PARA_LOAD   0xd3   //默认参数加载

 #define CMD_CURR_PARA_SAVE  0xd1   //当前参数保存

 #define CMD_MECH_OFF        0xd0   //机床关闭 
//----------------------------------------------------------------------- 

//----------------------------------------------------------------------- 
//---------------------------------------------------
//二级键码表，用于控制机床的运动-----------------------  
#define SECOND_CLASS      0xcf      
 //---用于机床初始化--------------------
 #define CMD_BACK_HOME     0xcc      
 #define CMD_TOOL_X        0xcb
 #define CMD_TOOL_Y        0xca
 #define CMD_TOOL_Z        0xc9

#define MIN_INIT_KEY      0xc9

 //---用于自动运行----------------------
 #define CMD_SIMU          0xc8 
 #define CMD_RUN           0xc7       
 #define CMD_PAUSE         0xc6
 #define CMD_STOP          0xc5
 #define CMD_ESTOP         0xc4

#define MIN_AUTO_KEY      0xc4

 //---用于手动运行------------------------
 #define CMD_GO_MECH_HOME  0xc3       //返回机床原点
 #define CMD_GO_PROG_HOME  0xc2       //返回程序原点
 #define CMD_GO_LAST_POS   0xc1       //返回程序记忆的上一点
 #define CMD_GO_SET_DIST   0xc0       //运行指定位移(轴号和位移由fixed_dist变量指定)

 #define AXIS_XP           0xbf       //启动X轴正向运动
 #define AXIS_NXP          0xbe       //停止X轴正向运动
 #define AXIS_XN           0xbd       //启动X轴负向运动
 #define AXIS_NXN          0xbc       //停止X轴正向运动

 #define AXIS_YP           0xbb       //启动Y轴正向运动
 #define AXIS_NYP          0xba       //停止Y轴正向运动
 #define AXIS_YN           0xb9       //启动Y轴负向运动
 #define AXIS_NYN          0xb8       //停止Y轴正向运动

 #define AXIS_ZP           0xb7       //启动Z轴正向运动
 #define AXIS_NZP          0xb6       //停止Z轴正向运动
 #define AXIS_ZN           0xb5       //启动Z轴负向运动
 #define AXIS_NZN          0xb4       //停止Z轴正向运动

 #define SPINDLE_CW        0xb3
 #define SPINDLE_CC        0xb2
 #define SPINDLE_OFF       0xb1

 #define AXIS_STOP         0xb0
  
#define MIN_MANU_KEY       0xb0

//--------------------------------------------------------
//三级键码表
#define THIRD_CLASS          0xaf       //三级键所属的区域,所有三级键码均小于该值

 #define K_SPEED_INC         0xae       //进给修调系数增大
 #define K_SPEED_DEC         0xad       //进给修调系数减小
 #define K_SPINDLE_INC       0xac       //主轴修调系数增加
 #define K_SPINDLE_DEC       0xab       //主轴修调系数减小

 #define K_PAGE_UP           0xa7
 #define K_PAGE_DOWN         0xa6
 #define K_SELE_LEFT         0xa5
 #define K_SELE_RIGHT        0xa4

 #define PLANT_ID_SET        0xa1       //设置设备代码
 
#define MIN_KEY_CODE         0xa0       //最小的键码，小于该值即为非键码
//------------------------------------------------
#define KEY_NULL             0x00       //无键码*

//------------------------------------------------
//-------通讯信息标识码(带参数命令码)----------------
 #define EOT                   0xec   //通讯结束符

 #define WORK_PARA_TRANS       0x90   //运行参数传递
   #define MECH_PARA_SET       0x91   //系统默认配置信息
   #define USER_PARA_SET       0x92   //用户配置信息
   #define FIELD_PARA_SET      0x93   //工作现场信息 
 
 #define MECH_CODE_TRANS       0x95   //加工代码传输
 #define  CODE_DOWN_LOAD       0x94   //加工代码下载 


 #define SPEED_SET             0x96   //速度以及修调系数传递
 #define SPINDLE_SET           0x97   //主轴转速以及修调系数传递  
 #define POSITION_SET          0x98   //指定定长度运行的轴和距离(填充fixed_dist结构变量)

//-------通讯控制码-------------------------------- 
 #define CODE_REJECT           0x9a   //停止代码发送  mech_status.code_buf_stat
 #define CODE_PERMIT           0x9b   //请求代码发送
 
 //-------通讯状态码表------------------------------------
 #define STA_COMM_WAITING      0x9c      //数据通讯 mech_status.comu_flag
 #define STA_USB_COMM_OK       0x9d
 #define STA_USB_COMM_ERR      0x9e

 #define STA_TEST_REQUIRE      0x9f
 #define STA_PAUS_REQUIRE      0x8f
 #define STA_WAKE_REQUIRE      0x7f  
//********************************************************

//----------状态标识码定义(用于mech_status.stat)----------
//-------机器初始化函数相关状态码表-----------------------
 #define STA_HOME_START        0x70    //回机械原点开始
 #define STA_HOME_OK           0x71    //回机械原点完成
 #define STA_HOME_STOP         0x72    //回机械原点中止

 #define STA_TOOLX_SET         0x73    //对刀开始
 #define STA_TOOLY_SET         0x74    //对刀开始
 #define STA_TOOLZ_SET         0x75    //对刀开始
 #define STA_TOOL_OK           0x76    //对刀完成
 #define STA_TOOL_STOP         0x77    //对刀中止

 #define STA_MECH_HOME         0x78    //在机床原点
// #define STA_PROG_HOME         0x79    //在程序原点

//-------机器自动运行相关状态定义---------------------------
 #define STA_UNKNOWN           0x00
 #define STA_READY             0x80      //机器准备好
 #define STA_WORKING           0x81      //处于工作状态
 #define STA_PAUSING           0x82      //正在暂停
 #define STA_PAUSED            0x83      //处于暂停状态 
 #define STA_STOPPING          0x84      //正在停止 
 #define STA_ESTOPPING         0x85      //处于紧急停止状态 

//#define STA_SIMULATION        0x86
 #define STA_MANUAL            0x87
 
 #define STA_STOPPED           0x88      //处于停车状态

 #define STA_BROKEN            0x8a      //工作中断 
 #define STA_FINISHED          0x8b      //工作停止

 #define STA_STOP_INSTANCE     0x8c
 
//-------错误类型码相关定义-----------------------------------    
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
//-------CNC运行内部控制代码 -----------------------------
 #define PROGRAM_END     99          //程序结束
 #define G_DWELL         0x08          //运动暂停

 
//-------标准代码定义-------------------------------------
 #define G_00            0x00
 #define G_01            0x01
 #define G_02            0x02
 #define G_03            0x03
 #define G_04            0x04

#endif


