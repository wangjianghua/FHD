#include "includes.h"

static  OS_EVENT  *KeySem;
unsigned int KeyPressed;

void CLR_WatchDog()
{
    WWDG_SetCounter(0xFF);
    WWDG_ClearFlag();
}

#if 0
void TRIC1_OFF()
{
    GPIO_ResetBits(GPIOB, GPIO_Pin_12);
    GPIO_SetBits(GPIOB, GPIO_Pin_13);
}

void TRIC1_ON()
{
    GPIO_ResetBits(GPIOB, GPIO_Pin_13);
    GPIO_SetBits(GPIOB, GPIO_Pin_12);            
}

//���Ƶ���
void TRIC2_ON()
{
    GPIO_ResetBits(GPIOB, GPIO_Pin_15);
    GPIO_SetBits(GPIOB, GPIO_Pin_14);
}

void TRIC2_OFF()
{
    GPIO_ResetBits(GPIOB, GPIO_Pin_14);
    GPIO_SetBits(GPIOB, GPIO_Pin_15);    
}
#else //����

/* ���Ƶ������� */
void TRIC1_ON(void)
{
    GPIO_SetBits(GPIOA, GPIO_Pin_1); //TRIC1        
    GPIO_ResetBits(GPIOA, GPIO_Pin_2); //TRIC1EN            
}

void TRIC1_OFF(void)
{
    GPIO_SetBits(GPIOA, GPIO_Pin_2); //TRIC1EN    
    GPIO_ResetBits(GPIOA, GPIO_Pin_1); //TRIC1  
}

/* ���Ƶ��ݷ��� */
void TRIC3_ON(void)
{
    GPIO_SetBits(GPIOB, GPIO_Pin_8); //TRIC3    
    GPIO_ResetBits(GPIOB, GPIO_Pin_9); //TRIC3EN        
}

void TRIC3_OFF(void)
{  
    GPIO_SetBits(GPIOB, GPIO_Pin_9); //TRIC3EN
    GPIO_ResetBits(GPIOB, GPIO_Pin_8); //TRIC3    
}

/* ���Ƹ������� */
void TRIC4_ON(void)
{
    GPIO_SetBits(GPIOB, GPIO_Pin_14); //TRIC4           
}

void TRIC4_OFF(void)
{  
    GPIO_ResetBits(GPIOB, GPIO_Pin_14); //TRIC4    
}

/* ��������· */
void TRIC2_ON(void)
{
    GPIO_SetBits(GPIOC, GPIO_Pin_7); //TRIC2EN    
    GPIO_ResetBits(GPIOC, GPIO_Pin_6); //TRIC2           
}

void TRIC2_OFF(void)
{
    GPIO_SetBits(GPIOC, GPIO_Pin_6); //TRIC2      
    GPIO_ResetBits(GPIOC, GPIO_Pin_7); //TRIC2EN   
}

/* �������� */
void aux_res_on(void)
{
    TRIC4_ON();
}

void aux_res_off(void)
{
    TRIC4_OFF();
}

#endif

#if 0

void Relay_Keep_Mode(uint8_t mode )
{
    if(mode == RELAY_SWITCH_ON)
    {        
        
        TRIC1_OFF(); 
        //ow_tick_delay(250);
        TRIC2_ON();
        //OSTimeDly(1);
        
    }
    else
    {
        TRIC2_OFF();   
        TRIC1_ON();
        //OSTimeDly(1);                     
    }
}

#else //����

void Relay_Keep_Mode(uint8_t mode)
{
    if(RELAY_FORWARD_SWITCH_ON == mode) //�������򹩵�
    {        
        TRIC2_OFF(); //����·    
        //for(u16 i = 0; i < 5000; i++);
        TRIC3_OFF(); //���ݷ���
        TRIC1_ON(); //��������           

        g_relay_switch = RELAY_FORWARD_SWITCH_ON;
    }
    else if(RELAY_BACKWARD_SWITCH_ON == mode) //���ݷ��򹩵�
    {
        TRIC2_OFF(); //����·  
        //for(u16 i = 0; i < 5000; i++);
        TRIC1_OFF(); //��������
        TRIC3_ON(); //���ݷ��� 

        g_relay_switch = RELAY_BACKWARD_SWITCH_ON;
    }
    else //����·����
    {
        TRIC1_OFF(); //��������
        //for(u16 i = 0; i < 5000; i++);
        TRIC3_OFF(); //���ݷ���
        TRIC2_ON(); //����·   

        aux_res_off(); //�رո�������

        g_relay_switch = RELAY_SWITCH_OFF;
    }
}

#endif

void GUI_X_Init (void) 
{
    KeySem = OSSemCreate(0);
}

int  GUI_X_WaitKey (void) 
{
    int    r;
    INT8U  err;
    
    
Wait_next_key_event:
    
    if (KeyPressed == 0) {
        OSSemPend(KeySem, 0, &err);
    }

    if(KeyPressed == 0)
        goto Wait_next_key_event;

    r = KeyPressed;

    if(RESET != GPIO_ReadInputDataBit(GPIOC, r))
    {
        KeyPressed = 0;
        goto Wait_next_key_event;
    }

    OSTimeDly(30); //20ms

    if(RESET != GPIO_ReadInputDataBit(GPIOC, r))
    {
        KeyPressed = 0;
        goto Wait_next_key_event;
    }

    KeyPressed = 0;

    return (r);
}


void  GUI_X_StoreKey (unsigned int k) 
{
    KeyPressed = k;
    OSSemPost(KeySem);
}




