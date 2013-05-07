#ifndef _CF_CARD_CFG_H_
#define _CF_CARD_CFG_H_

//****操作地址定义*****************************************************

#define CF_DATA_REG  port4800            //offset 0x00
ioport unsigned int port4800;

#define CF_SECTOR_NO_CT_REG   port4802   //offset 0x02
ioport unsigned int port4802;

#define CF_CYLIND_HI_LO_REG   port4804   //offset 0x04
ioport unsigned int port4804;

#define CF_STAT_HEAD_REG  port4806       //offset 0x06
ioport unsigned int port4806;

#define CF_COMD_HEAD_REG  port4806       //offset 0x06
ioport unsigned int port4806;

//******************************************************
//寄存器偏移地址定义
//******************************************************
/*
#define CF_DATA_EVEN/ODD   0x00
#define CF_DATA_ODD        0x09
#define CF_ERROR_REG       0x01    //when read
#define CF_FEATURE_REG     0x01    //when write
#define CF_SECTOR_CT_REG   0x02
#define CF_SECTOR_NO_REG   0x03
#define CF_CYLIND_LO_REG   0x04
#define CF_CYLIND_HI_REG   0x05
#define CF_CARD_HEAD_REG   0x06
#define CF_STATUS_REG      0x07  //when read
#define CF_COMMAND_REG     0x07  //when write
*/
/*
 **********************************************************************
 当 CF卡采用16位操作模式时：
 1。片选CE1和CE2并联到主机的片选输出端；-REG可以固定接高，A10固定接低，
    只用A0/1/2/3地址线，WAIT引脚接DSP的READY引脚
 2。此时16位数据读写寄存器占用偏移地址00H
 3。扇区计数寄存器和扇区号寄存器组成一个16位操作字，其中扇区号寄存器占
    高8位，扇区计数寄存器占低8位，用偏移地址02H或03H均可
 4。柱面寄存器的高字节和低字节组成一个16位操作字，其中高字节寄存器占高8
    位，低字节寄存器占低8位，采用偏移地址04H或05H均可
 5。磁头寄存器和操作命令/状态寄存器组成一个16位操作字，其中命令/状态寄
    存器占高8位，磁头寄存器占低8位，采用偏移地址06H或07H均可
 **********************************************************************
*/ 

//CF卡每扇区的字节数
#define CF_BYTE_PER_SEC   256   //512

//CF卡状态查询的最大重复次数
#define CF_RETRY_TIMES    4000

//写命令寄存器后的等待时间－延时次数（等待信号转换）
#define CF_SIGNAL_DELAY   4000

#endif

//int cf_read_CIS(unsigned int *CIS_buf,unsigned int length);
//int cf_boot_read(unsigned int *data_buffer,unsigned int length);
//int cf_format_write(unsigned int *data_buffer);  

int cf_read_sector(unsigned long starting_sector,
                    unsigned int sec_num,
                    unsigned int *data_buffer) ;

int cf_write_sector(unsigned long starting_sector,
                     unsigned int sec_num,
                     unsigned int *data_buffer) ;
