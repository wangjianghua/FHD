
#include "includes.h"


#if 1
const unsigned char lcd_disp_ram_welcome[LCD_DISP_RAM_SIZE]=
{
    /*-- ����x�߶�=128x64 --*/
    0xFF,0xFF,0xFF,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,
    0x07,0x07,0x07,0x87,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x87,0x87,
    0x87,0x07,0x07,0x07,0x07,0x87,0x87,0x87,0x87,0x87,0x07,0x07,0x07,0x07,0x07,0x07,
    0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x87,0x07,0x07,0x07,0x07,0x07,0x87,0x07,0x07,
    0x07,0x07,0x87,0x87,0x87,0x07,0x07,0x07,0x07,0x87,0x87,0x87,0x87,0x87,0x07,0x07,
    0x07,0x07,0x87,0x07,0x07,0x07,0x07,0x07,0x87,0x07,0x07,0x87,0x07,0x07,0x07,0x07,
    0x07,0x07,0x07,0x07,0x87,0x87,0x87,0x87,0x87,0x87,0x87,0x07,0x07,0x07,0x07,0x07,
    0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0xFF,0xFF,0xFF,
    0xFF,0xFF,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFC,0x03,0x00,0x00,
    0x00,0x01,0x86,0x00,0x00,0xFF,0x00,0x00,0x00,0x00,0x03,0xFC,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0x0F,0xF0,0x00,0xF0,0x0F,0xFF,0x00,0x00,
    0xFE,0x01,0x00,0x00,0x00,0x01,0xFE,0x00,0x00,0xFF,0x00,0x00,0x00,0x00,0x03,0xFC,
    0x00,0x00,0xFF,0x00,0x00,0x00,0x00,0x00,0xFF,0x00,0x00,0xFF,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0xFF,0x10,0x10,0x10,0x10,0x10,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,0xFF,
    0xFF,0xFF,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x07,0x04,0x04,0xC4,0xC4,0xC4,0xC4,0xC0,0x00,0x01,0xC2,0xC4,0xC4,
    0xC4,0xC2,0x01,0x00,0x00,0x07,0x04,0x04,0x84,0xC4,0xC3,0xC0,0x00,0x00,0xC0,0xC0,
    0xC0,0x80,0x80,0x00,0x00,0x00,0xC0,0xC7,0xC0,0xC1,0xC6,0xC1,0xC0,0x47,0x00,0x00,
    0x01,0x02,0x04,0x04,0x04,0x02,0x01,0xC0,0xC0,0xC7,0xC4,0xC4,0xC4,0xC4,0x03,0x00,
    0xC0,0xC0,0xC1,0xC2,0xC4,0x04,0x04,0x42,0xC1,0xC0,0xC0,0xC7,0xC4,0x84,0x04,0x04,
    0x04,0x44,0xC0,0xC0,0xC7,0x44,0x04,0x04,0x04,0x04,0x04,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,0xFF,
    0xFF,0xFF,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x07,0x01,0x80,0xE0,0xF0,0xFC,0xFF,0x7F,0x1F,0x07,
    0x03,0x00,0xC0,0x00,0x00,0x00,0xFC,0xFF,0xFF,0xFF,0xFF,0x00,0x00,0x00,0x00,0x01,
    0x8F,0xFF,0xFF,0xFF,0xFC,0x00,0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0xC0,0x00,0x00,0x00,0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0x1F,0x00,0x30,
    0x38,0xFC,0x00,0x87,0x8F,0x00,0x00,0x00,0xF8,0xF1,0x03,0x0F,0x1F,0x3F,0xFF,0xFE,
    0xF8,0xF0,0xE3,0x87,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,0xFF,
    0xFF,0xFF,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x80,0x00,0x80,0x00,0x8C,0x8E,0x0F,0x8F,0x0F,0x83,0x80,0x08,0x8C,0x0C,
    0x8E,0x0F,0x0F,0x00,0x00,0x00,0x80,0x83,0x87,0x87,0x0F,0x0E,0x8C,0x80,0x0C,0x0E,
    0x0F,0x07,0xF7,0x03,0x00,0x00,0x8C,0x8F,0x8F,0x0F,0x8F,0x8F,0x80,0x80,0x00,0x8C,
    0xEC,0x8C,0x8F,0x0F,0x00,0x80,0x80,0x88,0x0F,0x0F,0x8F,0x8F,0x8F,0x00,0xF0,0x88,
    0x8C,0x8C,0x0C,0x0F,0x0F,0x00,0x08,0x0C,0x0F,0x8F,0x8C,0x88,0x00,0x00,0x80,0x81,
    0x03,0x07,0x8F,0x8F,0x80,0x80,0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,0xFF,
    0xFF,0xFF,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x80,0xC1,0x07,0x00,0x07,0x01,0x01,0x07,0x80,0x47,0x41,0x41,0x87,0x00,0x07,
    0x01,0x00,0x84,0x40,0x40,0x40,0x84,0x06,0x05,0x44,0x80,0x03,0x04,0x04,0x03,0x00,
    0x80,0x40,0x07,0x00,0x80,0x43,0x45,0x45,0x45,0x80,0x07,0x00,0x00,0x07,0x00,0x00,
    0xC7,0xC4,0x04,0x00,0x03,0x05,0x05,0x05,0x00,0x03,0x04,0x04,0x02,0x00,0xC7,0x40,
    0x40,0x47,0x40,0x80,0x04,0x00,0x00,0x00,0x83,0x44,0x44,0x42,0x80,0x03,0x04,0x04,
    0x43,0x40,0x47,0xC0,0x47,0x40,0x47,0x00,0x00,0x80,0x40,0x40,0x40,0x40,0x80,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,0xFF,
    0xFF,0xFF,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x01,0x01,0xFF,0x00,0x00,0x00,0x00,0x00,0x03,0x80,0x40,0x20,0x10,0x08,0x07,0x00,
    0x00,0xE3,0x14,0x08,0x08,0x08,0x14,0xE3,0x00,0x00,0x00,0xC3,0x24,0x18,0x24,0xC3,
    0x00,0x00,0x00,0xFE,0x09,0x04,0x04,0x04,0x08,0xF1,0x00,0x00,0x60,0x58,0x44,0x43,
    0x40,0xFF,0x40,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0x00,
    0x00,0x00,0x00,0x81,0x7E,0x00,0x00,0xFF,0x00,0x00,0x00,0x00,0x00,0xFF,0x00,0x00,
    0x00,0x00,0x00,0xFF,0x00,0x00,0x00,0x00,0xC3,0x04,0x08,0x08,0x10,0x10,0x10,0xE1,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,0xFF,
    0xFF,0xFF,0xFF,0xE0,0xE0,0xE0,0xE0,0xE0,0xE0,0xE0,0xE0,0xE0,0xE0,0xE0,0xE0,0xE0,
    0xE0,0xE0,0xE3,0xE0,0xE0,0xE0,0xE0,0xE0,0xE3,0xE2,0xE2,0xE2,0xE2,0xE2,0xE2,0xE0,
    0xE0,0xE0,0xE1,0xE2,0xE2,0xE2,0xE1,0xE0,0xE0,0xE2,0xE1,0xE0,0xE0,0xE0,0xE0,0xE0,
    0xE1,0xE2,0xE0,0xE0,0xE1,0xE2,0xE2,0xE2,0xE1,0xE0,0xE0,0xE0,0xE0,0xE0,0xE0,0xE0,
    0xE0,0xE3,0xE0,0xE0,0xE0,0xE0,0xE0,0xE0,0xE0,0xE0,0xE0,0xE0,0xE0,0xE0,0xE3,0xE2,
    0xE2,0xE2,0xE2,0xE1,0xE0,0xE0,0xE0,0xE0,0xE1,0xE2,0xE2,0xE2,0xE1,0xE0,0xE0,0xE0,
    0xE0,0xE0,0xE0,0xE3,0xE0,0xE0,0xE0,0xE0,0xE0,0xE1,0xE2,0xE2,0xE2,0xE2,0xE1,0xE0,
    0xE0,0xE0,0xE0,0xE0,0xE0,0xE0,0xE0,0xE0,0xE0,0xE0,0xE0,0xE0,0xE0,0xFF,0xFF,0xFF,
};

#endif

#if 0
//**************delay************************************
void delay_nms(unsigned int n)
{
    OSTimeDly(n);
}


//***************send_data********************************
void data_send(unsigned char dat)
{
    unsigned char s,temp;
    int i;
    LCD_SCL_RST();
    s=dat;
    for(i=8;i>0;i--)
    { 
        LCD_SCL_RST();
        //NOP();
        //NOP();
        
        temp=s&0x80;
        if(temp)
        {
            LCD_SDA_SET();
        }
        else
        {
            LCD_SDA_RST();
        }

        LCD_SCL_SET();
        s=s<<1;
    }
}

//***************write command*****************************
void wr_com(unsigned char com)
{
    CPU_SR_ALLOC();
    CPU_INT_DIS();
    LCD_A0_RST();
    LCD_CS_RST();
    data_send(com);
    LCD_CS_SET();
    CPU_INT_EN();
}

//***************write data********************************
void wr_data(unsigned char dat)
{
    CPU_SR_ALLOC();
    CPU_INT_DIS();
    LCD_A0_SET();
    LCD_CS_RST();
    data_send(dat);
    LCD_CS_SET();
    CPU_INT_EN();
}

//**********************************************************
void display_kuang()
{
    unsigned char seg;
    unsigned char page;
    for(page=0xb0;page<0xb8;page++)
    {
        if(page==0xb0)
        {
            wr_com(page);
            wr_com(0x10);
            wr_com(0x00);
            for(seg=0;seg<128;seg++)
            {
                if(seg==0) wr_data(0xff);
                else if(seg==127) wr_data(0xff);
                else wr_data(0x01);
            }
        }
        else
        {
            if(page==0xb7)
            {
                wr_com(page);
                wr_com(0x10);
                wr_com(0x00);
                for(seg=0;seg<128;seg++)
                {
                    if(seg==0) wr_data(0xff);
                    else if(seg==127) wr_data(0xff);
                    else wr_data(0x80);
                }
            }

            else
            {
                wr_com(page);
                wr_com(0x10);
                wr_com(0x00);
                for(seg=0;seg<128;seg++)
                {
                    if(seg==0) wr_data(0xff);
                    else if(seg==127) wr_data(0xff);
                    else wr_data(0x00);
                }
            }
        }
    }
}


//**********************************************************
void display_lattice(unsigned char dat1,unsigned char dat2)
{
    unsigned char seg;
    unsigned char page;
    for(page=0xb0;page<0xb8;page++)
    {

        wr_com(page);
        wr_com(0x10);
        wr_com(0x00);
        for(seg=0;seg<64;seg++)
        { 
            wr_data(dat1);
            wr_data(dat2);
        }
    }

}


//**********************************************************
//**********************************************************
void display_map(unsigned char *p)
{
    unsigned char seg;
    unsigned char page;
    
    for(page=0xb0;page<0xb8;page++)
    {
        wr_com(page);
        wr_com(0x10);
        wr_com(0x00);

        for(seg=0;seg<128;seg++)
        { 
            wr_data(*p);
            ++p;
        }
    }

}

void lcd_init()
{
    LCM_CON_ON();
    //BLCD_CON_ON();
    delay_nms(100);
    LCD_RST();
    delay_nms(100);
    LCD_SET();
    delay_nms(100);

    //**********lcd inintial************
    wr_com(0xa0); //ADC normal
    wr_com(0xc8); //com normal
    wr_com(0xa3); //set partial display duty 1/65

    wr_com(0x2c); //vc on
    delay_nms(100);
    wr_com(0x2e); //vr on
    delay_nms(100);
    wr_com(0x2f); //internal booster,follower,divided on
    wr_com(0x25); //set rb/ra=5.29
    wr_com(0x81); //set reference voltage select
    wr_com(0x10);
    delay_nms(100);    
    wr_com(0xaf); //display on
    wr_com(0xa6); //display on
    wr_com(0x40); //set start line 00
    //BLCD_CON_OFF();
}
#else //����

/** 
 * @file     lcd.c
 * @brief    COG12864Һ������ʾ
 * @details  ST7565R����
 * @author   ����
 * @email    591881218@qq.com
 * @date     2014
 * @version  vX.XX
 * @par Copyright (c):  
 *           �����к϶����Ƽ����޹�˾
 * @par History:          
 *   version: author, date, desc\n 
 */

unsigned char lcd_disp_buf[LCD_YSIZE / 8][LCD_XSIZE];

void lcd_delay(unsigned int dly)
{
    OSTimeDly(dly);
}

void lcd_port_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;


    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);    

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);     
}

void lcd_send_byte(unsigned char byte)
{
    unsigned char i, temp;


    temp = byte;

    for(i = 0; i < 8; i++)
    {
        LCD_SCLK_RESET();

        if(temp & 0x80)
        {
            LCD_MOSI_SET();
        }
        else
        {
            LCD_MOSI_RESET();
        }

        LCD_SCLK_SET();

        temp <<= 1;
    }
}

void lcd_write_cmd(unsigned char cmd)
{   
    CPU_SR_ALLOC();
    

    CPU_INT_DIS();
    
    LCD_RS_RESET();
    LCD_CS_RESET();
    lcd_send_byte(cmd);
    LCD_CS_SET();

    CPU_INT_EN();
}

void lcd_write_data(unsigned char data)
{	
    CPU_SR_ALLOC();
    

    CPU_INT_DIS();
    
    LCD_RS_SET();
    LCD_CS_RESET();
    lcd_send_byte(data);
    LCD_CS_SET();

    CPU_INT_EN();
}

void lcd_clr_scr(void)
{
    unsigned char page, seg;

    
    for(page = 0xb0; page < 0xb8; page++)
    {
        lcd_write_cmd(page); //page
       
        lcd_write_cmd(0x10); //column
        
#if (REV_OPERATION_EN > 0U)
        lcd_write_cmd(0x04); //column
#else
        lcd_write_cmd(0x00); //column
#endif  

        for(seg = 0; seg < 128; seg++) //д128��
        {
            lcd_write_data(0x00);	
        }	
    }
}

void lcd_disp_map(unsigned char *pmap) //pmap��ͼƬ�����׵�ַ
{
    unsigned char page, seg;

    
    for(page = 0xb0; page < 0xb8; page++)
    {
        lcd_write_cmd(page); //page
       
        lcd_write_cmd(0x10); //column
        
#if (REV_OPERATION_EN > 0U)
        lcd_write_cmd(0x04); //column
#else
        lcd_write_cmd(0x00); //column
#endif 

        for(seg = 0; seg < 128; seg++) //д128��
        {
            lcd_write_data(*pmap++);	
        }	
    }
}

void lcd_init(void)
{
    lcd_port_init();

    LCM_POW_ON();
    LCD_On();    

    lcd_delay(100);

    LCD_RST_RESET();
    lcd_delay(100);
    LCD_RST_SET();
    lcd_delay(100);

    lcd_write_cmd(0xe2); /*����λ*/
    lcd_delay(100);
    lcd_write_cmd(0x2c); /*��ѹ����1*/
    lcd_delay(100);
    lcd_write_cmd(0x2e); /*��ѹ����2*/
    lcd_delay(100);
    lcd_write_cmd(0x2f); /*��ѹ����3*/
    lcd_delay(100);
    lcd_write_cmd(0x23); /*�ֵ��Աȶȣ������÷�Χ0x20��0x27*/
    lcd_write_cmd(0x81); /*΢���Աȶ�*/
    lcd_write_cmd(0x1a); /*΢���Աȶȵ�ֵ�������÷�Χ0x00��0x3f*/
    lcd_write_cmd(0xa3); /*1/7 ƫѹ�ȣ�bias��*/

#if (REV_OPERATION_EN > 0U)    
    lcd_write_cmd(0xc0); /*��ɨ��˳�򣺴��µ���*/
    lcd_write_cmd(0xa1); /*��ɨ��˳�򣺴��ҵ���*/
#else
    lcd_write_cmd(0xc8); /*��ɨ��˳�򣺴��ϵ���*/
    lcd_write_cmd(0xa0); /*��ɨ��˳�򣺴�����*/
#endif    

    lcd_write_cmd(0x40); /*��ʼ�У��ӵ�һ�п�ʼ*/
    lcd_write_cmd(0xaf); /*����ʾ*/

    lcd_delay(100);

    lcd_clr_scr();
}

#endif