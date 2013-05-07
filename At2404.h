#ifndef _AT24C04_H_
#define _AT24C04_H_

#define MECH_PARA_ADDR    0
#define USER_PARA_ADDR    128
#define FIELD_PARA_ADDR   256
#define PASS_WORD_ADDR    384
#define PLANT_ID_ADDR     480
#define MANUFACTUR_FLAG   496       

#define ROM_PAGE_SIZE      16
#define DELAY_P            15

/*
//------the control Pin define----------------- 
#define ROM_READ          *PFDATDIR &= 0xFDFF
#define ROM_WRITE         *PFDATDIR |= 0x0200
#define ROM_WPL_CLR       *PFDATDIR &= 0xFFFE
#define ROM_WPL_SET       *PFDATDIR |= 0x0001
#define ROM_SDA_CLR       *PFDATDIR &= 0xFFFD
#define ROM_SDA_SET       *PFDATDIR |= 0x0002
#define ROM_SCL_CLR       *PADATDIR &= 0xFFDF
#define ROM_SCL_SET       *PADATDIR |= 0x0020 
#define ROM_SDA_IN        (*PFDATDIR & 0x0002)
//---------------------------------------------
*/
#endif

int ROM_write(int addr, unsigned int *wrt_buf, int sum_byte);
int ROM_read(int addr, unsigned int *out_buf, int sum_byte);
