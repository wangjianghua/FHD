

#include "includes.h"

#include "e2prom.h"


#define   E2P_SDA_PORT   GPIOB //华兄
#define   E2P_SDA_PIN    GPIO_Pin_11


#if 1





void SomeNOP(void)

{

    unsigned char kk=20;  //10

    while(kk--);

}//小于100KHz

  
void SomeNOP_Short(void)
{

    unsigned char kk=10;  //5

    while(kk--);

}//小于100KHz





unsigned short E2PTemp1;

unsigned char E2PErrIndication=0;

unsigned char E2PTemp0;







void E2promSCLHighToLow(void)
{    

    SomeNOP_Short();
    E2P_SCL_SET();SomeNOP();
    E2P_SCL_RST();SomeNOP_Short();;
}



void E2promStart(void)
{

    E2P_SDA_SET();  

    SomeNOP();

    E2P_SCL_SET();        

    SomeNOP();        

    E2P_SDA_RST();    

    SomeNOP();        

    E2P_SCL_RST();  

    SomeNOP();

}



 void E2promStop(void)

{

    E2P_SDA_RST(); 

    SomeNOP();

    E2P_SCL_SET();

    SomeNOP();    

    E2P_SDA_SET();     

    SomeNOP();        

    E2P_SCL_RST();

    SomeNOP();

}



void E2promAck(void)

{

    E2P_SDA_RST(); 

    E2promSCLHighToLow();    

}



void E2promNoAck(void)

{

    E2P_SDA_SET();     

    E2promSCLHighToLow();

}



unsigned char E2promReadByte(void)

{

    unsigned char temp=0, BitCounter=8;

    GPIO_InitTypeDef GPIO_InitStructure;



    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_InitStructure.GPIO_Pin = E2P_SDA_PIN;

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;

    GPIO_Init(E2P_SDA_PORT, &GPIO_InitStructure);



    //Change to Input Mode

    //E2P_SDA_DIR = 1;

    //E2P_SDA_DIR |= (E2P_SDA);

    SomeNOP();

    

    do{

        temp<<=1;

        E2P_SCL_SET();

        SomeNOP();

        if(GPIO_ReadInputDataBit(E2P_SDA_PORT, E2P_SDA_PIN)) temp++;        

        E2P_SCL_RST();

        SomeNOP();

    }while(--BitCounter);

    

    //Change to Output Mode    

    //E2P_SDA_DIR = 0;

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;

    GPIO_Init(E2P_SDA_PORT, &GPIO_InitStructure);


    SomeNOP_Short();
    

    return(temp);

}



unsigned char E2promWriteByte(unsigned char data)
{

    unsigned int wait = 0;
    unsigned char BitCounter = 8;
    GPIO_InitTypeDef GPIO_InitStructure;
    
    //E2P_SDA_DIR = 0;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = E2P_SDA_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;    

    do{    

        if(data&0x80)        
            E2P_SDA_SET();         
        else            
            E2P_SDA_RST();

        E2promSCLHighToLow();
        data<<=1;
    
    }while(--BitCounter);

    
    //E2P_SDA_DIR = 1;    
    GPIO_Init(E2P_SDA_PORT, &GPIO_InitStructure);

    //SomeNOP();
        
    E2P_SCL_SET();    

    SomeNOP_Short();

    while(GPIO_ReadInputDataBit(E2P_SDA_PORT, E2P_SDA_PIN))
    {
        if(wait++ > 5000)
        {
            E2PErrIndication=1;
            break;
        }
    }

    //E2P_SDA_DIR = 0; 
    E2P_SCL_RST();
    
    //SomeNOP_Short();

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;

    GPIO_Init(E2P_SDA_PORT, &GPIO_InitStructure);
    E2P_SDA_SET();

    SomeNOP_Short();
    

    return E2PErrIndication;

}





unsigned char E2_ReadByte(unsigned short addr)
{

    unsigned char i;
    unsigned short j = 0;

    E2PTemp0=0xA0;  //|((addr&0x700)>>7);
    E2PTemp1=addr;

    do
    {

        E2PErrIndication = 0;
        CLR_WatchDog();

        if(j++ > 1000)
        {

            E2PErrIndication = 1;

            return ERROR;

        }

        E2promStart();

    }while(E2promWriteByte(E2PTemp0));  
  

    E2promWriteByte((unsigned char)(E2PTemp1));

    E2promStart();

    E2promWriteByte(E2PTemp0|0x01);

  
    i = E2promReadByte(); 

    E2promNoAck();

    E2promStop();    
        

    E2P_SCL_SET();
    E2P_SDA_SET();

    return i;

}

void E2_WriteByte(unsigned short addr,unsigned char nContent)
{
    unsigned char i;
    unsigned short j=0;

    E2P_WP_RST();

    E2PTemp0= 0xA0;  //|((addr&0x700)>>7);
    E2PTemp1=addr;  

    do
    {
        E2PErrIndication = 0;
        
        if(j++ > 1000)
        {
            E2PErrIndication = 1;
            E2P_WP_SET();
            return;
        }

        E2promStart();

     }while(E2promWriteByte(E2PTemp0));    


    E2promWriteByte((unsigned char)(E2PTemp1));
    E2promWriteByte(nContent);

    E2promStop();
    E2P_WP_SET();

    E2P_SDA_SET();
    E2P_SCL_SET();

}

void E2promCheckEnd()
{
    int j;
    do
    {

        E2PErrIndication = 0;

        //_clr_wdt();

        if(j++ > 1000)

        {

            E2PErrIndication = 1;

            E2P_WP_SET();

            return;

        }

        E2promStart();
        
     }while(E2promWriteByte(E2PTemp0));    

    E2promStop();
        
}
void E2promReadBuffer(unsigned short addr,unsigned char *data,unsigned char len)
{

    unsigned char i;

    unsigned short j = 0;



    if(len == 0)

        return;

    



    E2PTemp0=0xA0;  //|((addr&0x700)>>7);

    E2PTemp1=addr;



    do

    {

        E2PErrIndication = 0;

        CLR_WatchDog();

        if(j++ > 1000)

        {

            E2PErrIndication = 1;

            return;

        }

        E2promStart();

    }while(E2promWriteByte(E2PTemp0));    

  

    //E2promStart();

    //E2promWriteByte(E2PTemp0);

    E2promWriteByte((unsigned char)(E2PTemp1));

    E2promStart();

    E2promWriteByte(E2PTemp0|0x01);

    //if(len > 1)

    {

        //len--;

        for(i = 0; i < len - 1; i++)

        {             

            data[i]=E2promReadByte();         

            //data++;

            E2promAck();            

        }    

    }

    data[i]=E2promReadByte(); 

    E2promNoAck();

    E2promStop();    

        

    E2P_SCL_SET();

    E2P_SDA_SET();

   
}



void E2promWrite_Page(unsigned short addr,unsigned char *data,unsigned char len)
{

    unsigned char i;

    unsigned short j=0;

    if(len == 0)
        return;  

    

    E2PTemp0=0xA0; 
    E2PTemp1=addr;  

    do
    {

        E2PErrIndication = 0;

        //_clr_wdt();

        if(j++ > 1000)

        {

            E2PErrIndication = 1;

            //E2P_WP_SET();

            return;

        }

        E2promStart();
        
     }while(E2promWriteByte(E2PTemp0));    


    E2promWriteByte((unsigned char)(E2PTemp1));
    

    for(i=0;i<len;i++)
    {

        E2promWriteByte(data[i]);

    }    


    E2promStop();

    //E2P_WP_SET();



    E2P_SDA_SET();

    E2P_SCL_SET();

    OSTimeDly(5);

}





void E2promWriteBuffer(unsigned short nAddr,unsigned char *nContent, unsigned char nLen)
{

    unsigned char ii;  

    GPIO_SetBits(GPIOC, GPIO_Pin_9);

    E2P_WP_RST();

#if 1

    if( ( (unsigned char)(nAddr/PAGE_OF_AT24C16) ) !=  ((unsigned char)( (nAddr+nLen-1)/PAGE_OF_AT24C16))  )
    {

           ii=PAGE_OF_AT24C16 - (nAddr%PAGE_OF_AT24C16) ;

           E2promWrite_Page(nAddr ,nContent,ii);

           nLen-=ii;

           nAddr+=ii;

           nContent+=ii;

            while(nLen)
            {

                if( nLen>=PAGE_OF_AT24C16)
                {

                    ii=PAGE_OF_AT24C16;

                }

                else

                {

                    ii=nLen;

                }

                E2promWrite_Page(nAddr ,nContent,ii);

                nLen-=ii;

                nAddr+=ii;

                nContent+=ii;

                //WDTE = 0xAC;

           };

           //E2promCheckEnd();

    }
    else
    {

        E2promWrite_Page(nAddr ,nContent,nLen);

        //E2promCheckEnd();

    }

    GPIO_ResetBits(GPIOC, GPIO_Pin_9);

    E2P_WP_SET();
#endif

}





#else









void flash()

{

    uchar kk=10;

    while(kk--);

}





void start()

{

   E2P_SDA=1; flash(); E2P_SCL=1; flash(); E2P_SDA=0; flash(); E2P_SCL=0; flash();

}



void stop()

{

   E2P_SDA=0; flash(); E2P_SCL=1; flash(); E2P_SDA=1; flash();

}



void writex(uchar j)

{

   uchar i,temp;

   temp=j;

   for (i=0;i<8;i++){

      E2P_SCL=0; flash();

      if(temp&0x80)        

            E2P_SDA = 1;         

        else            

            E2P_SDA = 0; 

      flash(); 

      temp=temp<<1; 

      E2P_SCL=1; flash();

   }

   

   E2P_SCL=0; flash(); E2P_SDA=1; flash();

}



uchar readx()

{

   uchar i,j,k=0;

   E2P_SCL=0; flash(); 

   E2P_SDA_DIR=1;

   for (i=0;i<8;i++){

      flash(); E2P_SCL=1; flash();

      if (E2P_SDA==1)

        j=1;

      else 

        j=0;

      k=(k<<1)|j; E2P_SCL=0;

   }

   flash();



   E2P_SDA_DIR = 0;

   

   return(k);

}



void clock()

{

   uchar i=0;

   E2P_SDA_DIR = 1;

   E2P_SCL=1; flash();   

   while ((E2P_SDA==1)&&(i<255))i++;

   E2P_SDA_DIR = 0;

   E2P_SCL=0; flash();

}



uchar x24c02_read(uchar address)

{

   uchar i;

   start(); writex(0xa0);

   clock(); writex(address);

   clock(); start();

   writex(0xa1); clock();

   i=readx(); stop();

   delay_nms(10);

   return(i);

}



void x24c02_write(uchar address,uchar info)

{

   E2P_WP=0;

   start(); writex(0xa0);

   clock(); writex(address);

   clock(); writex(info);

   clock(); stop();

   E2P_WP=1;

   delay_nms(10);

}





#endif

