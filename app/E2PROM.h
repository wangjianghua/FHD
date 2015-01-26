
#ifndef E2PPROM_H
#define E2PPROM_H
void E2promReadBuffer(  unsigned short nAddr,unsigned char *nContent, unsigned char nLen );
void E2promWriteBuffer(unsigned short nAddr,unsigned char *nContent, unsigned char nLen);

unsigned char ReadChar_BcdToHex(unsigned short add);
void _pcf8576_operation(unsigned char addr,unsigned char *Ldata,unsigned char len);
void _pcf8576_init(void);

//LCD
#define LCD_ADDR_PCF8576                     0x70
#define LCD_ADDR_START                         0x00
#define LCD_DISPLAY_OFF                        0xD0
#define LCD_DISPLAY_ON                          0xD8
#define LCD_PARTS_SELECT                      0xE0
#define LCD_FLASH_CON                           0x70


  /////////////////////////////
  // INSTRUCTION CODES
  ////////////////////////////
  #define WREN 0x06
  #define WRDI 0x04
  #define RDSR 0x05
  #define WRSR 0x01
  #define READ 0x03
  #define WRITE 0x02
  
  #define PROGRAM 0x02
  #define SECTOR_ERASE 0x52
  #define CHIP_ERASE 0x62
  #define RDID 0x15

#define PORT_NUMBER 4

#define PAGE_OF_AT24C16    8


#define CS_RIGHT    0x00
#define CS_WRONG    0x04;//数据不可靠


#define   AMOUNT_OF_DEGREE                2//总 正向  反向
#define   E2PROM_SPACE            0x0200    //AT240C16    
#define   AMOUNT_OF_HOURS_FROZEN_DEFREE            48    //AT240C16    

/*
空间分配方案

表号 电表常数   电量整数 电量小数

*/




#endif

