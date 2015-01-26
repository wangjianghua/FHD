/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.3.0
  * @date    04/16/2010
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2010 STMicroelectronics</center></h2>
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "includes.h"
#include "stm32f10x_adc.h"




/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/
static int s_it_key_val = 0;
//static int s_it_key_count = 0;
//static U32 s_it_key_time = 0;
INT32U   g_last_uptime, g_uptime_bak;

void  os_IntExit (void);


/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
    //IMPORT hard_fault_handler_c 
    //asm("TST LR, #4 \n" /* r2 = address of sFlag */
    //"ITE EQ \n" /* jump over constant */
    //"MRSEQ R0, MSP \n" /* address of sFlag */
    //"MRSNE R0, PSP \n" /* r3 = address of UART1_SR */
    //"B hard_fault_handler_c"); /* sFlag = r0 */
}

#define E2PROM_ERROR_ADDR   248

void hard_fault_handler_c(unsigned int * hardfault_args) 
{ 

unsigned int stacked_r0; 
unsigned int stacked_r1; 
unsigned int stacked_r2; 
unsigned int stacked_r3; 
unsigned int stacked_r12; 
unsigned int stacked_lr; 
unsigned int stacked_pc; 
unsigned int stacked_psr; 

stacked_r0 = ((unsigned long) hardfault_args[0]); 
stacked_r1 = ((unsigned long) hardfault_args[1]); 
stacked_r2 = ((unsigned long) hardfault_args[2]); 
stacked_r3 = ((unsigned long) hardfault_args[3]); 

stacked_r12 = ((unsigned long) hardfault_args[4]); 
stacked_lr = ((unsigned long) hardfault_args[5]); 
stacked_pc = ((unsigned long) hardfault_args[6]); 
stacked_psr = ((unsigned long) hardfault_args[7]); 

#if 0
g_sys_conf.error_count++;
g_sys_conf.error_sn  %=  MAX_ERROR_SN;
g_sys_conf.error_time[g_sys_conf.error_sn][0] = g_rtc_time[MIN_POS];
g_sys_conf.error_time[g_sys_conf.error_sn][1] = g_rtc_time[HOUR_POS];
g_sys_conf.error_time[g_sys_conf.error_sn][2] = g_rtc_time[DATE_POS];
g_sys_conf.error_time[g_sys_conf.error_sn][3] = g_rtc_time[MONTH_POS];
g_sys_conf.error_data[g_sys_conf.error_sn] = stacked_pc;
g_sys_conf.error_sn++;
#endif //华兄

//E2promWriteBuffer(OffsetOf(SYS_CONF, error_count), (unsigned char *)&g_sys_conf.error_count, 38);

#if 0
printf ("[Hard fault handler]\n"); 
printf ("R0 = %x\n", stacked_r0); 
printf ("R1 = %x\n", stacked_r1); 
printf ("R2 = %x\n", stacked_r2); 
printf ("R3 = %x\n", stacked_r3); 
printf ("R12 = %x\n", stacked_r12); 
printf ("LR = %x\n", stacked_lr); 
printf ("PC = %x\n", stacked_pc); 
printf ("PSR = %x\n", stacked_psr); 
printf ("BFAR = %x\n", (*((volatile unsigned long *)(0xE000ED38)))); 
printf ("CFSR = %x\n", (*((volatile unsigned long *)(0xE000ED28)))); 
printf ("HFSR = %x\n", (*((volatile unsigned long *)(0xE000ED2C)))); 
printf ("DFSR = %x\n", (*((volatile unsigned long *)(0xE000ED30)))); 
printf ("AFSR = %x\n", (*((volatile unsigned long *)(0xE000ED3C)))); 

exit(0); // terminate 
#endif

while(1);


return; 
} 

void HardFault_Handler(void) 
{ 
    //IMPORT hard_fault_handler_c 
    asm("TST LR, #4 \n" /* r2 = address of sFlag */
    "ITE EQ \n" /* jump over constant */
    "MRSEQ R0, MSP \n" /* address of sFlag */
    "MRSNE R0, PSP \n" /* r3 = address of UART1_SR */
    "B hard_fault_handler_c"); /* sFlag = r0 */

} 

#if 0
/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  EXP_PROC_SaveCnxt(en_exp_hardware);
  while (1)
  {
  }
}
#endif

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
    asm("TST LR, #4 \n" /* r2 = address of sFlag */
    "ITE EQ \n" /* jump over constant */
    "MRSEQ R0, MSP \n" /* address of sFlag */
    "MRSNE R0, PSP \n" /* r3 = address of UART1_SR */
    "B hard_fault_handler_c"); /* sFlag = r0 */
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
    asm("TST LR, #4 \n" /* r2 = address of sFlag */
    "ITE EQ \n" /* jump over constant */
    "MRSEQ R0, MSP \n" /* address of sFlag */
    "MRSNE R0, PSP \n" /* r3 = address of UART1_SR */
    "B hard_fault_handler_c"); /* sFlag = r0 */
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
    asm("TST LR, #4 \n" /* r2 = address of sFlag */
    "ITE EQ \n" /* jump over constant */
    "MRSEQ R0, MSP \n" /* address of sFlag */
    "MRSNE R0, PSP \n" /* r3 = address of UART1_SR */
    "B hard_fault_handler_c"); /* sFlag = r0 */
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
    OS_CPU_PendSVHandler();  
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
    mseconds++;
    if(mseconds > 999)
        mseconds = 0;

    g_msec_count++; //华兄
    
    OS_CPU_SysTickHandler();
}

void WWDG_IRQHandler(void)
{
    WWDG_SetCounter(0xFF);
    WWDG_ClearFlag();
    EXP_PROC_SaveCnxt(en_exp_dog);
}

unsigned short g_diff_num = 0;
unsigned short g_diff_array;
unsigned char g_diff_pos = 0;

unsigned short g_freq_count = 0; //华兄
unsigned char g_relay_switch = 0;
unsigned long g_aux_res_count = 0;

void TIM2_IRQHandler(void) 
{ 
    int cnt, res;


    g_freq_count++; //华兄，周期100us，计数0~200

    if(0 != g_aux_res_count)
    {
        g_aux_res_count--;

        if(0 == g_aux_res_count)
        {
            aux_res_off(); //关闭辅助电阻
        }
    }

#if 0
    if(g_freq_count > 200)
    {
        g_freq_count = 0;
    }
#endif

    if(g_power_state == SYS_POWER_STEADY)
    {
        cnt  = (AD_SAMPLE_TIMES_PER_PERIOD*2 - DMA1_Channel1->CNDTR);

        if(cnt >= AD_SAMPLE_TIMES_PER_PERIOD*2)
        {
            cnt = 0;
            if(TIM_GetITStatus(TIM2, TIM_IT_Update)==SET) 
            {         
                TIM_ClearITPendingBit(TIM2, TIM_IT_Update); 
            } 
            return;
        }
        else if(cnt >= AD_SAMPLE_TIMES_PER_PERIOD)
        {
            cnt -= AD_SAMPLE_TIMES_PER_PERIOD;
        }
        else if(cnt < 0)
        {
            cnt = 0;
            if(TIM_GetITStatus(TIM2, TIM_IT_Update)==SET) 
            {         
                TIM_ClearITPendingBit(TIM2, TIM_IT_Update); 
            } 
            return;
        }

        if(cnt > 0)
            cnt--;
        else
        {
#if 0            
            cnt = 99;
#else //华兄
            cnt = AD_SAMPLE_TIMES_PER_PERIOD - 1;
#endif
        }

        res = (INT16S)ADC_RegularConvertedValueTab[0][cnt] - (INT16S)ADC_RegularConvertedValueTab[1][cnt];

#if 0        
        if(res > 120 || res < -120)
#else //华兄
        if(res > 250 || res < -250)
#endif
        {
            g_diff_num++;
            g_diff_array |= (0x01 << g_diff_pos);            
        }
       
        g_diff_pos++;
        g_diff_pos &= 0x0f;
        
        if( g_diff_array & (0x01 << g_diff_pos))
        {
            //缓冲里最后一个数据超限
            g_diff_array &= (~(0x01 << g_diff_pos));
            if(g_diff_num)
                g_diff_num--;
        }

        if(g_diff_num >= g_break_pos)  //10 = 2ms
        {
            if(g_sys_conf.SysRunStatus & SYS_RELAY_ON_FLAG)
            {
                if((g_sys_conf.SysSwitch & (SYS_DROP_KEEP_MASK)) && //晃电保护开关打开
                   (FALSE == g_sys_conf.main_mos_broken) &&  //主MOS管正常
                   (RESET == MAIN_MOS_CHECK_JDQ_STAT())) //主电路继电器关闭
                {
                    //电容放电继电器吸合
                    g_sys_conf.SysRunStatus |= SYS_DROP_ACTION;

#if 0                        
                    //Relay_Keep_Mode(RELAY_SWITCH_ON);
                    
                    Relay_Keep_Mode(RELAY_BACKWARD_SWITCH_ON);
#else //华兄

#if (MULTI_MODE_EN > 0u)
                    if(enum_mode_1 == g_sys_conf.mode)
                    {
                        aux_res_on(); //打开辅助电阻
                        g_aux_res_count = 4000; //25us * 4000 = 100ms
                        
                        Relay_Keep_Mode(RELAY_FORWARD_SWITCH_ON); //电容正向放电
                    }
                    else
                    {
                        aux_res_on(); //打开辅助电阻
                        g_aux_res_count = 4000; //25us * 4000 = 100ms
                        
                        if((g_freq_count >= (200 - 5)) && (g_freq_count < (600 - 5))) //电容反向放电
                        {
                            Relay_Keep_Mode(RELAY_BACKWARD_SWITCH_ON);
                        }
                        else //电容正向放电
                        {
                            Relay_Keep_Mode(RELAY_FORWARD_SWITCH_ON);
                        }
                    }
#else
                    Relay_Keep_Mode(RELAY_FORWARD_SWITCH_ON); //电容正向放电
#endif

#endif

                    //g_dbg_disp = g_freq_count;
                }              
                    
                g_droping_power_on_time = 0;
                
                g_power_state = SYS_DROPING;
                break_off_count = 0;
                LCD_Off();
                g_droping_timestamp = OSTime;//OSTimeGet();
                LED_2_ON();
            }
        }   
    }
    
    if(TIM_GetITStatus(TIM2,TIM_IT_Update)==SET) 
    {         
        TIM_ClearITPendingBit(TIM2,TIM_IT_Update); 
    } 
} 

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles RTC global interrupt request.
  * @param  None
  * @retval None
  */
void RTC_IRQHandler(void)
{
    /* 华兄 */
    if(RTC_GetITStatus(RTC_IT_SEC) != RESET)
    {
        /* Clear the RTC Second interrupt */
        RTC_ClearITPendingBit(RTC_IT_SEC);

        /* Wait until last write operation on RTC registers has finished */
        RTC_WaitForLastTask();

        g_msec_count = 0;
    }    
}

void EXIT_key_verify(int Key_val)
{
   
}

void EXTI0_IRQHandler(void)
{   
    CPU_FNCT_VOID  isr;
    CPU_SR_ALLOC();


    CPU_CRITICAL_ENTER();                                       /* Tell the OS that we are starting an ISR            */

    OSIntEnter();

    CPU_CRITICAL_EXIT();
    
    if(EXTI_GetITStatus(EXTI_Line0) != RESET)
    {
        /* Toggle LED1 */

        if((g_power_state == SYS_POWER_ON) || (g_power_state == SYS_DROPED))
        {
            g_uptime_bak = g_last_uptime;
            g_last_uptime = OSTime;
        }
        else if(((OSTime - g_last_uptime)%20) < 2)
        {
            g_uptime_bak = g_last_uptime;
            g_last_uptime = OSTime;
            //g_pow_count = 30;
        }
        else if(((OSTime - g_uptime_bak)%20) < 2)
        {
            //g_uptime_bak = g_last_uptime;
            g_last_uptime = OSTime;
            //g_pow_count = 30;
        }
        else
        {
            g_uptime_bak = OSTime;
        }

        g_pow_count = 30;

        if(g_freq_count > 798)
        {
            g_freq_count = 0; //华兄
        }
        
        /* Clear the  EXTI line 0 pending bit */
        EXTI_ClearITPendingBit(EXTI_Line0);
    }

    OSIntExit(); 
 
    return;
}


void EXTI1_IRQHandler(void)
{   
    CPU_FNCT_VOID  isr;
    CPU_SR_ALLOC();


    CPU_CRITICAL_ENTER();                                       /* Tell the OS that we are starting an ISR            */

    OSIntEnter();

    CPU_CRITICAL_EXIT();
    
    if(EXTI_GetITStatus(EXTI_Line1) != RESET)
    {
        /* Toggle LED1 */        
        //g_ac_sb1_count = 30;
        
        /* Clear the  EXTI line 0 pending bit */
        EXTI_ClearITPendingBit(EXTI_Line1);
    }

    OSIntExit(); 
 
    return;
}

void EXTI2_IRQHandler(void)
{
    
    return;
}

void EXTI3_IRQHandler(void)
{    
    return;
}

void EXTI4_IRQHandler(void)
{
    
    return;
}

/******************************************************************************/
/*            STM32F10x Peripherals Interrupt Handlers                        */
/******************************************************************************/

/**
  * @brief  This function handles External lines 9 to 5 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI9_5_IRQHandler(void)
{    
    return;
}



/******************************************************************************/
/*            STM32F10x Peripherals Interrupt Handlers                        */
/******************************************************************************/

/**
  * @brief  This function handles External lines 9 to 5 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI15_10_IRQHandler(void)
{      
    CPU_FNCT_VOID  isr;
    CPU_SR_ALLOC();
    unsigned int exit_s, i;


    CPU_CRITICAL_ENTER();                                       /* Tell the OS that we are starting an ISR            */

    OSIntEnter();

    CPU_CRITICAL_EXIT();


    for(i = 0, exit_s = EXTI_Line10; i < 4; i++, exit_s <<= 1)
    {
        if(EXTI_GetITStatus(exit_s) != RESET)
        {
            /* Toggle LED1 */
            //g_last_uptime = OSTimeGet();
            /* Clear the  EXTI line 0 pending bit */

            GUI_X_StoreKey(exit_s);

            DEBUG_LED_OFF();

#if 0            
            if(LED_3_STATUS())
            {
                LED_3_OFF();
                //TRIC1_OFF();
            }
            else
            {
                LED_3_ON();
                //TRIC1_ON();
            }
#endif

            EXTI_ClearITPendingBit(exit_s);
        }
    }

    //EXTI_ClearITPendingBit(EXTI_Line0);

    OSIntExit(); 
 
    return;
}


void USART_IRQProc(UART_CCB  *uccb, USART_TypeDef * USARx)
{
  if(USART_GetITStatus(USARx, USART_IT_RXNE) != RESET)
  {
    /* Read one byte from the receive data register */    
    *(uccb->gpUartRxAddress) = USART_ReceiveData(USARx);
    uccb->gpUartRxAddress++;

    if(uccb->gpUartRxAddress == uccb->gpUartRxEndAddress)
    {
        //回头
        uccb->gpUartRxAddress = uccb->gpUartRxStartAddress;
    }
	
    if(uccb->gpUartRxReadAddress == uccb->gpUartRxAddress)
    {
        //可以考虑加错误统计
    } 
    
#ifndef DEBUG_LED
    LED_3_ON();
#endif
  }

  if(USART_GetITStatus(USARx, USART_IT_TXE) != RESET)
  {   
    /* Write one byte to the transmit data register */
    if( uccb->gUartTxCnt > 0 )
    {
        USART_SendData(USARx, *(uccb->gpUartTxAddress));
    	uccb->gpUartTxAddress++;   	
    	uccb->gUartTxCnt--;
	}
    else
    {        
        USART_ITConfig(USARx, USART_IT_TXE, DISABLE);
    }

#ifndef DEBUG_LED
    LED_3_ON();
#endif	
  }
}

/**
  * @brief  This function handles USARTx global interrupt request.
  * @param  None
  * @retval None
  */
void USART1_IRQHandler(void)
{
    
    USART_IRQProc(&g_uart_ccb[COM_PORT_485], USART1);

}


/**
  * @brief  This function handles DMA1 Channel 6 interrupt request.
  * @param  None
  * @retval None
  */
void DMA1_Channel1_IRQHandler(void)
{
  /* Test on DMA1 Channel6 Transfer Complete interrupt */
  if(DMA_GetITStatus(DMA1_IT_TC1))
  {
    /* Get Current Data Counter value after complete transfer */
    //CurrDataCounterEnd = DMA_GetCurrDataCounter(DMA1_Channel1);
    /* Clear DMA1 Channel6 Half Transfer, Transfer Complete and Global interrupt pending bits */
    //BSP_ADC_DMA_int_proc();
    DMA_ClearITPendingBit(DMA1_IT_GL1);
  }
}


/**
  * @brief  This function handles ADC1 and ADC2 global interrupts requests.
  * @param  None
  * @retval None
*/

void ADC1_2_IRQHandler(void)
{
  /* Reset PC.06 pin */
  //GPIO_WriteBit(GPIOC, GPIO_Pin_6, Bit_RESET);
}

//const struct armSysCtlReg * p_armSysCtl = (struct armSysCtlReg *)0x680FF000;

void EXP_PROC_SaveCnxt(char err_type)
{
   
    // save statistics below
    // 0x680FF040

}

void  os_IntExit (void)
{
#if OS_CRITICAL_METHOD == 3                                /* Allocate storage for CPU status register */
    OS_CPU_SR  cpu_sr = 0;
#endif

    if (OSRunning == OS_TRUE) {
        OS_ENTER_CRITICAL();
        if (OSIntNesting > 0) {                            /* Prevent OSIntNesting from wrapping       */
            OSIntNesting--;
        }        
        OS_EXIT_CRITICAL();
    }
}

/**
  * @}
  */ 


/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
