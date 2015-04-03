#include "includes.h"

#define RTC_IIC_ADDR 0xD0

#if 0
#define   RTC_SDA_PORT   GPIOA
#define   RTC_SDA_PIN   GPIO_Pin_12
#else //华兄

#define   RTC_SDA_PORT   GPIOB
#define   RTC_SDA_PIN    GPIO_Pin_7

#endif


void RTC_SomeNOP(void)
{

    unsigned char kk=50;

    while(kk--);

}//小于100KHz

  
void RTC_SomeNOP_Short(void)
{

    unsigned char kk=20;
    while(kk--);

}//小于100KHz


void RTC_SCLHighToLow(void)
{    

    RTC_SomeNOP_Short();
    RTC_SCL_SET();RTC_SomeNOP();
    RTC_SCL_RST();RTC_SomeNOP_Short();;
}



void RTC_Start(void)
{

    RTC_SDA_SET();  

    RTC_SomeNOP();

    RTC_SCL_SET();        

    RTC_SomeNOP();        

    RTC_SDA_RST();    

    RTC_SomeNOP();        

    RTC_SCL_RST();  

    RTC_SomeNOP();

}



 void RTC_Stop(void)

{

    RTC_SDA_RST(); 

    RTC_SomeNOP();

    RTC_SCL_SET();

    RTC_SomeNOP();    

    RTC_SDA_SET();     

    RTC_SomeNOP();        

    RTC_SCL_RST();

    RTC_SomeNOP();

}



void RTC_Ack(void)

{

    RTC_SDA_RST(); 

    RTC_SCLHighToLow();    

}

void RTC_NoAck(void)
{

    RTC_SDA_SET();     

    RTC_SCLHighToLow();

}



unsigned char RTC_Read8Bit(void)

{

    unsigned char temp=0, BitCounter=8;

    GPIO_InitTypeDef GPIO_InitStructure;



    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = RTC_SDA_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;

    GPIO_Init(RTC_SDA_PORT, &GPIO_InitStructure);

    RTC_SomeNOP();


    do{

        temp<<=1;

        RTC_SCL_SET();

        RTC_SomeNOP();

        if(GPIO_ReadInputDataBit(RTC_SDA_PORT, RTC_SDA_PIN)) temp++;        

        RTC_SCL_RST();

        RTC_SomeNOP();

    }while(--BitCounter);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;

    GPIO_Init(RTC_SDA_PORT, &GPIO_InitStructure);


    RTC_SomeNOP_Short();
    
    return(temp);

}



unsigned char RTC_Write8Bit(unsigned char data)
{

    unsigned int wait = 0, ErrIndication;
    unsigned char BitCounter = 8;
    GPIO_InitTypeDef GPIO_InitStructure;
    
    //E2P_SDA_DIR = 0;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = RTC_SDA_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;    

    do{    

        if(data&0x80)        
            RTC_SDA_SET();         
        else            
            RTC_SDA_RST();

        RTC_SCLHighToLow();
        data<<=1;
    
    }while(--BitCounter);

    
    //E2P_SDA_DIR = 1;    
    GPIO_Init(RTC_SDA_PORT, &GPIO_InitStructure);
        
    RTC_SCL_SET();    

    RTC_SomeNOP_Short();

    while(GPIO_ReadInputDataBit(RTC_SDA_PORT, RTC_SDA_PIN))
    {
        if(wait++ > 5000)
        {
            ErrIndication=1;
            break;
        }
    }

    RTC_SCL_RST();


    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;

    GPIO_Init(RTC_SDA_PORT, &GPIO_InitStructure);
    RTC_SDA_SET();

    RTC_SomeNOP_Short();
    

    return ErrIndication;

}


unsigned char RTC_ReadByte(unsigned short addr)
{

    unsigned char i;
    unsigned short j = 0;

    do
    {
        CLR_WatchDog();

        if(j++ > 1000)
        {
            return ERROR;

        }

        RTC_Start();

    }while(RTC_Write8Bit(RTC_IIC_ADDR));  
  

    RTC_Write8Bit((unsigned char)(addr));

    RTC_Start();

    RTC_Write8Bit(RTC_IIC_ADDR|0x01);

  
    i = RTC_Read8Bit(); 

    RTC_NoAck();

    RTC_Stop();    
        

    RTC_SCL_SET();
    RTC_SDA_SET();

    return i;

}

void RTC_WriteByte(unsigned short addr,unsigned char nContent)
{
    unsigned char i;
    unsigned short j=0;

    do
    {
        if(j++ > 1000)
        {
            return;
        }

        RTC_Start();

     }while(RTC_Write8Bit(RTC_IIC_ADDR));    


    RTC_Write8Bit((unsigned char)(addr));
    RTC_Write8Bit(nContent);

    RTC_Stop();

    RTC_SDA_SET();
    RTC_SCL_SET();

}


void RTC_ReadBuffer(unsigned short addr,unsigned char *data,unsigned char len)
{

    unsigned char i;
    unsigned short j = 0;

    if(len == 0)
        return;
    
    do
    {


        CLR_WatchDog();

        if(j++ > 1000)
        {
            return;

        }

        RTC_Start();

    }while(RTC_Write8Bit(RTC_IIC_ADDR));    

    RTC_Write8Bit((unsigned char)(addr));

    RTC_Start();

    RTC_Write8Bit(RTC_IIC_ADDR|0x01);

    for(i = 0; i < len - 1; i++)

    {             

        data[i]=RTC_Read8Bit();         

        //data++;

        RTC_Ack();            

    }    


    data[i]=RTC_Read8Bit(); 
    RTC_NoAck();
    RTC_Stop();    

      
    RTC_SCL_SET();
    RTC_SDA_SET();

      

}



void RTC_WriteBuffer(unsigned short addr,unsigned char *data,unsigned char len)
{

    unsigned char i;
    unsigned short j=0;

    if(len == 0)
        return; 
    
   
    do
    {
        if(j++ > 1000)
        {
            //E2P_WP_SET();
            return;

        }
        RTC_Start();       

     }while(RTC_Write8Bit(RTC_IIC_ADDR));    
       
    RTC_Write8Bit((unsigned char)(addr));   

    for(i=0;i<len;i++)
    {
        RTC_Write8Bit(data[i]);
    }    

    RTC_Stop();


    RTC_SDA_SET();
    RTC_SCL_SET();        

}



void RTC_ReadTime(unsigned char *time)
{
    if(time == NULL)
        return;

    RTC_ReadBuffer(0, time, 7);
    time[SEC_POS] &= 0x7f;
    time[MIN_POS] &= 0x7f;
    time[HOUR_POS] &= 0x3f;
    time[DAY_POS] &= 0x07;
    time[DATE_POS] &= 0x3f;
    time[MONTH_POS] &= 0x1f;
    //time[YEAR_POS] &= 0x7f;

}




void RTC_WriteTime(unsigned char *time)
{
    if(time == NULL)
        return;

    
    time[SEC_POS] &= 0x7f;
    time[MIN_POS] &= 0x7f;
    time[HOUR_POS] &= 0x3f;
    time[DAY_POS] &= 0x07;
    time[DATE_POS] &= 0x3f;
    time[MONTH_POS] &= 0x1f;
    //time[YEAR_POS] &= 0x7f;

    RTC_WriteBuffer(0, time, 7);

}


/** 
 * @file     rtc.c
 * @brief    RTC
 * @details  实时时钟
 * @author   华兄
 * @email    591881218@qq.com
 * @date     2014
 * @version  vX.XX
 * @par Copyright (c):  
 *           深圳市合尔凯科技有限公司
 * @par History:          
 *   version: author, date, desc\n 
 */

#if (DEBUG_INFO_PRINT_EN > 0U)
#define DEBUG_WARN(s)         printf s
#define DEBUG_PRINT(s)        printf s
#else
#define DEBUG_WARN(s)
#define DEBUG_PRINT(s)           
#endif

/**
* @brief  Configures the RTC.
* @param  None
* @retval None
*/
void RTC_Configuration(void)
{
    /* Enable PWR and BKP clocks */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);

    /* Allow access to BKP Domain */
    PWR_BackupAccessCmd(ENABLE);

    /* Reset Backup Domain */
    BKP_DeInit();

#if RTC_CLK_SRC_LSI > 0u
    /* Enable the LSI OSC */
    RCC_LSICmd(ENABLE);
    /* Wait till LSI is ready */
    while (RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET)
    {}

    /* Select the RTC Clock Source */
    RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);
#elif RTC_CLK_SRC_LSE > 0u
    /* Enable LSE */
    RCC_LSEConfig(RCC_LSE_ON);
    /* Wait till LSE is ready */
    while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET)
    {}

    /* Select LSE as RTC Clock Source */
    RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);      
#else
    /* Select HSE/128 as RTC Clock Source */
    RCC_RTCCLKConfig(RCC_RTCCLKSource_HSE_Div128);
#endif

    /* Enable RTC Clock */
    RCC_RTCCLKCmd(ENABLE);

    /* Wait for RTC registers synchronization */
    RTC_WaitForSynchro();

    /* Wait until last write operation on RTC registers has finished */
    RTC_WaitForLastTask();

    /* Enable the RTC Second */
    RTC_ITConfig(RTC_IT_SEC, ENABLE);

    /* Wait until last write operation on RTC registers has finished */
    RTC_WaitForLastTask();

#if RTC_CLK_SRC_LSI > 0u
    /* Set RTC prescaler: set RTC period to 1sec */
    RTC_SetPrescaler(32767); /* RTC period = RTCCLK/RTC_PR = (32.768 KHz)/(32767+1) */
#elif RTC_CLK_SRC_LSE > 0u
    /* Set RTC prescaler: set RTC period to 1sec */
    RTC_SetPrescaler(39999);
#else
    /* Set RTC prescaler: set RTC period to 1sec */
    RTC_SetPrescaler(62499);
#endif

    /* Wait until last write operation on RTC registers has finished */
    RTC_WaitForLastTask();
}

void rtc_adjust_time(void)
{
    u32 time;


    /* 换算成从2000年开始的秒数 */
    time = OS_mktime((struct RTCCounterValue *)g_rtc_time);

    /* Wait until last write operation on RTC registers has finished */
    RTC_WaitForLastTask();
    /* Change the current time */
    RTC_SetCounter(time);
    /* Wait until last write operation on RTC registers has finished */
    RTC_WaitForLastTask();    
}

u32 rtc_get_time(void)
{
    return RTC_GetCounter();
}

void rtc_config(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    
    
    /* Enable the RTC Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = RTC_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

#if 0
    if(BKP_ReadBackupRegister(BKP_DR1) != 0xA5A5)
#else
    if(1)
#endif
    {
        /* Backup data register value is not correct or not yet programmed (when
           the first time the program is executed) */
           
        DEBUG_WARN("\r\n\n RTC not yet configured....");

        /* RTC Configuration */
        RTC_Configuration();

        DEBUG_WARN("\r\n RTC configured....");

        /* Adjust time by values entered by the user on the hyperterminal */
        rtc_adjust_time();

        BKP_WriteBackupRegister(BKP_DR1, 0xA5A5);
    }
    else
    {
        /* Check if the Power On Reset flag is set */
        if (RCC_GetFlagStatus(RCC_FLAG_PORRST) != RESET)
        {
            DEBUG_WARN("\r\n\n Power On Reset occurred....");
        }
        /* Check if the Pin Reset flag is set */
        else if (RCC_GetFlagStatus(RCC_FLAG_PINRST) != RESET)
        {
            DEBUG_WARN("\r\n\n External Reset occurred....");
        }

        DEBUG_WARN("\r\n No need to configure RTC....");
        /* Wait for RTC registers synchronization */
        RTC_WaitForSynchro();

        /* Enable the RTC Second */
        RTC_ITConfig(RTC_IT_SEC, ENABLE);
        /* Wait until last write operation on RTC registers has finished */
        RTC_WaitForLastTask();
    }

    /* Clear reset flags */
    RCC_ClearFlag();  
}

