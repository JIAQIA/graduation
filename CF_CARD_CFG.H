#ifndef _CF_CARD_CFG_H_
#define _CF_CARD_CFG_H_

//****������ַ����*****************************************************

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
//�Ĵ���ƫ�Ƶ�ַ����
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
 �� CF������16λ����ģʽʱ��
 1��ƬѡCE1��CE2������������Ƭѡ����ˣ�-REG���Թ̶��Ӹߣ�A10�̶��ӵͣ�
    ֻ��A0/1/2/3��ַ�ߣ�WAIT���Ž�DSP��READY����
 2����ʱ16λ���ݶ�д�Ĵ���ռ��ƫ�Ƶ�ַ00H
 3�����������Ĵ����������żĴ������һ��16λ�����֣����������żĴ���ռ
    ��8λ�����������Ĵ���ռ��8λ����ƫ�Ƶ�ַ02H��03H����
 4������Ĵ����ĸ��ֽں͵��ֽ����һ��16λ�����֣����и��ֽڼĴ���ռ��8
    λ�����ֽڼĴ���ռ��8λ������ƫ�Ƶ�ַ04H��05H����
 5����ͷ�Ĵ����Ͳ�������/״̬�Ĵ������һ��16λ�����֣���������/״̬��
    ����ռ��8λ����ͷ�Ĵ���ռ��8λ������ƫ�Ƶ�ַ06H��07H����
 **********************************************************************
*/ 

//CF��ÿ�������ֽ���
#define CF_BYTE_PER_SEC   256   //512

//CF��״̬��ѯ������ظ�����
#define CF_RETRY_TIMES    4000

//д����Ĵ�����ĵȴ�ʱ�䣭��ʱ�������ȴ��ź�ת����
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