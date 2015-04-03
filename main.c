#include "includes.h"

static   OS_STK      App_TaskStartStk[APP_CFG_TASK_START_STK_SIZE];
static   OS_STK      App_TaskPollStk[APP_CFG_TASK_POLL_STK_SIZE];
static   OS_STK      App_TaskEndTickStk[APP_CFG_TASK_END_TICK_STK_SIZE];
static   OS_STK      App_TaskKeyStk[APP_CFG_TASK_KEY_STK_SIZE];

INT32U g_power_state;

INT32U g_pow_count;  //检测107
//INT32U g_ac_sb1_count;  //检测101市电

INT32U mseconds;

u32 g_msec_count = 0; //华兄，毫秒计数器
u32 g_dropping_msec = 0; //华兄，掉电时毫秒计数值
u32 g_power_on_msec = 0; //华兄，来电时毫秒计数值
u32 g_power_check_count = 0; //华兄，再启动计数器 
u32 g_power_check_flag = FALSE; //华兄，再启动标志

INT32U g_dbg_disp = 0; //华兄

INT32U g_rms_buf[MAX_RMS_NUM]; //华兄
INT8U g_rms_buf_index;

INT32U g_voltage = 22000;

INT32U g_droping_timestamp;
INT32U g_droping_power_on_time; //华兄

INT32U g_main_mos_check_jdq_on_time;
INT32U g_main_mos_delay_on_time;
INT32U g_main_mos_delay_off_time;
INT32U g_main_mos_broken_test = FALSE;
INT32U g_main_mos_broken_test_time;

INT32U diff_num, ac_rms, break_off_count, jdq_timer;
INT32U g_scount;

#if 0
#define AC_POWER_NORMAL_RMS   (22000*850)  //220 * 80%
#define AC_POWER_RECOVER_RMS   (22000*780)  //220 * 80%
#else //华兄
#define AC_POWER_NORMAL_RMS    (22000 * 900)
#define AC_POWER_RECOVER_RMS   (22000 * 700)
#endif

int displayTimerCounter;

extern unsigned char	OW_RomID[8];

unsigned char E2PROM_Test()
{
    int ii;
    unsigned char temp[10];

    ii = 11;   
    
    for(ii = 0; ii < 200; ii++)
    {
        temp[0] = ii+2;
        E2promWriteBuffer(ii, temp, 1);
    }

    for(ii = 0; ii < 200; ii++)
    {
        E2promReadBuffer(ii, temp, 1);
        if((ii+2) != temp[0])
            return ERROR;
            
    }  

    return OK;
}



unsigned char RTC_Test()
{
    int ii;
    unsigned char temp[8] = {0x10,25,0x23,0x04,0x21,0x07,0x11};
    
    RTC_WriteTime(temp);

    memset(temp,0 ,8);

    for(ii = 0; ii < 5; ii++)
    {
        RTC_ReadTime(temp);
        
        if(4 != temp[3])
            return ERROR;

         if(0x23 != temp[2])
            return ERROR;
            
    }   

    return OK;
}


void  App_TaskEndTick (void *p_arg)
{
    (void)p_arg;
    
    for (;;)
    {     
        if((SYS_DROPED != g_power_state) && (SYS_AUTH_ERROR != g_power_state))
        {
#ifndef DEBUG_LED
            LED_UART_OFF(); //华兄
#endif            

            End_tick_check();    
        }

        OSTimeDly(OS_TICKS_PER_SEC / 50);
    }
}


U32 send_test_cmd(UCHAR * cmd_buf, USHORT cmd_len)
{
    P_MSG_INFO p_Msg = NULL;

    if( !(p_Msg = (P_MSG_INFO)alloc_send_buffer(MSG_SHORT)) )
    {
        return ERROR;
    }

    strcpy(p_Msg->msg_buffer, cmd_buf);

    p_Msg->msg_header.msg_len = cmd_len;/* */

    p_Msg->msg_header.end_id = RS485_COM_PORT;

    p_Msg->msg_header.need_buffer_free = OK;/* FALSE 标识end 负责buffer 释放*/

    return End_send(p_Msg);
}

static void App_TaskKey(void *parg)
{
    unsigned int key_val;
    while(1)
    {
        key_val = GUI_X_WaitKey();
        if((g_power_state != SYS_DROPED) && (SYS_AUTH_ERROR != g_power_state))
        {
            displayTimerCounter = LCD_DISPLAY_TIME_SEC;
            LCD_On(); 
            GUI_Key_Proc(key_val);
        }
    }
}

#if 0
int App_Check_Self_Power()
{

    int crc;
    U32 t1,t2;
    crc = MEM_Cal_Time_Crc(&g_sys_conf.lastBreakTimeStamp[0]);

    if(crc != g_sys_conf.lastBreakTimeStamp[7])
    {
        return ERROR;
    }

    t1 = OS_mktime((struct RTCCounterValue *)&g_sys_conf.lastBreakTimeStamp[0]);
    t2 = OS_mktime((struct RTCCounterValue *)g_rtc_time);

#if 0                
    if(t1 >= t2)
    {
        return ERROR;
    }
#endif

    if(((t1 + g_sys_conf.SelfPowerTime)== (t2)) || ((t1 + g_sys_conf.SelfPowerTime+1)== (t2)))
    {
        return OK;
    }
    
    if((t1 + g_sys_conf.SelfPowerTime + 1)> (t2))
    {
        return (OK+1);    
    }

#if 0 //华兄
    if((t2) >= (t1 + g_sys_conf.SelfPowerTime))
    {
        return (OK);    
    }
#endif

    return ERROR;
}
#else //华兄

#if (SELF_POWER_EN > 0u)
int App_Check_Self_Power(void)
{
    int crc;
    U32 t1, t2;

    
    crc = MEM_Cal_Time_Crc(&g_sys_conf.lastBreakTimeStamp[0]);

    if(crc != g_sys_conf.lastBreakTimeStamp[7])
    {
        return ERROR;
    }

    t1 = OS_mktime((struct RTCCounterValue *)&g_sys_conf.lastBreakTimeStamp[0]);
    t2 = OS_mktime((struct RTCCounterValue *)g_rtc_time);

    if((t1 + g_sys_conf.SelfPowerDeadlineHigh) == (t2))
    {
        if((g_dropping_msec + g_sys_conf.SelfPowerDeadlineLow * 100) > g_msec_count)
        {
            return (OK);
        }
        else
        {
            return (ERROR);
        }
    }
    else if((t1 + g_sys_conf.SelfPowerDeadlineHigh) > (t2))
    {
        return (OK);
    }
    else
    {
        return (ERROR);
    }
}
#endif

#endif

static  void  App_TaskPoll (void *p_arg)
{
    while(1)
    {        
               
        //LCD_Time_Refreash();
        OSTimeDlyHMSM(0,0,0,1000);

        RTC_ReadTime(g_rtc_time);

#if (SELF_POWER_EN > 0u)
        rtc_adjust_time(); //华兄
#endif

        if((Bcd2HexChar(g_rtc_time[HOUR_POS])%8) == 0 )
        {
            if((g_rtc_time[MIN_POS] == 1) && (g_rtc_time[SEC_POS] == 0x17))
                AUTO_Test();
        }

        if(AUTO_Getsar() != 0x03)
        {
            g_sys_conf.SysSwitch |= SYS_WARNING_MASK; //华兄
                
            g_scount++;
            if(g_scount > (3600 * 21))
            {
                while(1)
                {
                    OSTimeDlyHMSM(0,0,5,0);

                    RTC_ReadTime(g_rtc_time);

                    g_power_state = SYS_AUTH_ERROR;

#ifndef DEBUG_LED
                    LED_UART_TOGGLE();
#endif                    

                    AUTO_Test();

                    if(AUTO_Getsar() == 0x03)
                    {
                        g_power_state = SYS_POWER_ON;
                        g_scount = 0;
                        CPU_IntDis();
                        while(1);
                        break;
                    }
                }
            }
        }
        else
        {
            g_sys_conf.SysSwitch &= ~SYS_WARNING_MASK; //华兄
                
            g_scount = 0;
        }

        if(SYS_DROPED != g_power_state)
        {

            //send_test_cmd("Hello world", 11);

            GUI_Sec_Refresh();

            LED_RUN_TOGGLE();

            if(displayTimerCounter)
            {
                displayTimerCounter--;
                if(displayTimerCounter == 0)
                {
                    LCD_Off();
                }
            }

            //if(g_rtc_time[YEAR_POS] >= 0x12)
            {
            //    while(1);
            }
            
        }

        
    }
}

/*
*********************************************************************************************************
*                                      CREATE APPLICATION TASKS
*
* Description:  This function creates the application tasks.
*
* Arguments  :  none
*
* Returns    :  none
*********************************************************************************************************
*/

static  void  App_TaskCreate (void)
{
    OSTaskCreateExt( App_TaskPoll,                               
                    (void *)0,
                    (OS_STK *)&App_TaskPollStk[APP_CFG_TASK_POLL_STK_SIZE - 1],
                     APP_CFG_TASK_POLL_PRIO,
                     APP_CFG_TASK_POLL_PRIO,
                    (OS_STK *)&App_TaskPollStk[0],
                     APP_CFG_TASK_POLL_STK_SIZE,
                    (void *)0,
                     OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR);

    OSTaskCreateExt( App_TaskEndTick,                               
                    (void *)0,
                    (OS_STK *)&App_TaskEndTickStk[APP_CFG_TASK_END_TICK_STK_SIZE - 1],
                     APP_CFG_TASK_END_TICK_PRIO,
                     APP_CFG_TASK_END_TICK_PRIO,
                    (OS_STK *)&App_TaskEndTickStk[0],
                     APP_CFG_TASK_END_TICK_STK_SIZE,
                    (void *)0,
                     OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR);

    OSTaskCreateExt( App_TaskKey,                               
                    (void *)0,
                    (OS_STK *)&App_TaskKeyStk[APP_CFG_TASK_KEY_STK_SIZE - 1],
                     APP_CFG_TASK_KEY_PRIO,
                     APP_CFG_TASK_KEY_PRIO,
                    (OS_STK *)&App_TaskKeyStk[0],
                     APP_CFG_TASK_KEY_STK_SIZE,
                    (void *)0,
                     OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR); 
}


CPU_INT32U  BSP_CPU_ClkFreq (void)
{
    RCC_ClocksTypeDef  rcc_clocks;

    RCC_GetClocksFreq(&rcc_clocks);

    return ((CPU_INT32U)rcc_clocks.HCLK_Frequency);
}

int ADC_get_diff_num()
{
    int i = AD_SAMPLE_TIMES_PER_PERIOD, res, num = 0;
    while(i--)
    {
        res = (INT16S)ADC_RegularConvertedValueTab[0][i];
        res -= (INT16S)ADC_RegularConvertedValueTab[1][i];
        if(res > 50 || res < -50)
        {
            num++;
        }
    }

    return num;
}

#if 0
INT16U g_adc_buf[3200];
INT16U g_adc_buf_index = 0;
#endif //华兄

int ADC_get_ac_rms()
{
#if 0    
    int i = AD_SAMPLE_TIMES_PER_PERIOD*2;
#else //华兄
    int i = AD_SAMPLE_TIMES_PER_PERIOD;
#endif
    unsigned int res = 0;
    INT16U* ad_addr = (INT16U*)ADC_RegularConvertedValueTab;

    
    while(i--)
    {
#if 0
        g_adc_buf[g_adc_buf_index] = *ad_addr;
        g_adc_buf_index++;

        if(g_adc_buf_index >= 3200)
        {
            g_adc_buf_index = 0;
        }
#endif //华兄      

        res += *ad_addr;
        ad_addr++;        
    }

#if 0
    return res/(AD_SAMPLE_TIMES_PER_PERIOD*2);
#else //华兄
    return (res / AD_SAMPLE_TIMES_PER_PERIOD);
#endif
}

/*
*********************************************************************************************************
*                                          STARTUP TASK
*
* Description : This is an example of a startup task.  As mentioned in the book's text, you MUST
*               initialize the ticker only once multitasking has started.
*
* Arguments   : p_arg   is the argument passed to 'App_TaskStart()' by 'OSTaskCreate()'.
*
* Returns     : none
*
* Notes       : 1) The first line of code is used to prevent a compiler warning because 'p_arg' is not
*                  used.  The compiler should not generate any code for this statement.
*********************************************************************************************************
*/

#if 1
unsigned short g_break_pos = 2;
#else //华兄
unsigned short g_break_pos = 3;
#endif

unsigned char g_hard_v_f;

static  void  App_TaskStart (void *p_arg)
{
    INT32U  hclk_freq;
    INT32U  cnts;
#if OS_CRITICAL_METHOD == 3                                     /* Allocate storage for CPU status register                 */
    OS_CPU_SR  cpu_sr = 0;
#endif    
    INT8U  i;
    INT32U  sum, average, g_rms_discrete, count = 0;

       
    (void)p_arg;
   
    BSP_Init();                                                 /* Init BSP fncts.                                          */
    CPU_IntEn();  

    /* Init CPU name & int. dis. time measuring fncts. */    
    hclk_freq = BSP_CPU_ClkFreq();                              /* Determine SysTick reference freq.                        */
    cnts  = hclk_freq / (CPU_INT32U)OS_TICKS_PER_SEC;           /* Determine nbr SysTick increments in OS_TICKS_PER_SEC.    */
    OS_CPU_SysTickInit(cnts);                                   /* Init uC/OS periodic time src (SysTick).                  */

    //等待系统稳定
    //OSTimeDly(1000);

    LED_ON(); //华兄

    GUI_Init();
    GUI_X_Init();  

    End_Init();

    RTC_ReadTime(g_rtc_time);

#if (SELF_POWER_EN > 0u)
    rtc_adjust_time(); //华兄
#endif      

#if (E2PROM_TEST_EN > 0u)
    E2prom_Test();
#endif

    //Mem_Init();                                                 /* Init mem mgmt module.                                    */
    MEM_Init();

    g_pow_count = 0;
    //g_ac_sb1_count = 0;
    jdq_timer = 0;

    JDQ_OFF(); //华兄 

#if 0 //华兄
    lcd_init();
#endif

    //GUI_Context.DrawMode = LCD_DRAWMODE_REV;
    //GUI_Context.Color = GUI_BLACK;

    
    //display_map((unsigned char *)lcd_disp_buf_welcome);
    //GUI_DispStringAt("晃电次数: ABCDE", 0, 16);
    //display_map((unsigned char *)lcd_disp_buf);

    //while(1)
    {
    //    OSTimeDly(1);
    }

#if (OS_TASK_STAT_EN > 0)
    OSStatInit();                                               /* Determine CPU capacity                                   */
#endif

    App_TaskCreate();                                            /* Create application tasks                                 */

    Relay_Keep_Mode(RELAY_SWITCH_OFF);
    
#if 1 //华兄   
    MAIN_MOS_CHECK_JDQ_ON(); 

    MAIN_MOS_OFF();
#endif

    //RTC_Test();
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); 

    LCD_disp_main_form(0, FORM_MSG_RERRESH);
   
    //TRIC1_OFF();
    EXTI15_10_Config();
    //while(1)OSTimeDly(1);

    EXTI0_Config();

#if 0 //华兄
    EXTI1_Config();
#endif
    
    /* TIM1 counter enable */
    TIM_Cmd(TIM1, ENABLE);
    /* TIM1 main Output Enable */
    TIM_CtrlPWMOutputs(TIM1, ENABLE); 

    BSP_TIM6_20msTimer_start();
    
    g_power_state = SYS_POWER_ON;

    g_scount = 0;

#if 1 //华兄
    //g_sys_conf.voltageFixCoe = 11000;

    DEBUG_LED_OFF();
#endif

    AUTO_Test();

#if 0
    if(AUTO_Getsar() == 0x03)
    {
        if((OK+2) <= (g_hard_v_f = magic_verify(OW_RomID)))
        {
            while(1)
            {
                OSTimeDlyHMSM(0,0,2,0);

                //RTC_ReadTime(g_rtc_time);

                g_power_state = SYS_AUTH_ERROR;

                LED_HD_TOGGLE();

                //AUTO_Test();

                g_power_state = SYS_DROPED;

                //CPU_IntDis();
                //while(1);              
            }
        }
    }
#else
    if(0x03 == AUTO_Getsar())
    {
        g_sys_conf.SysSwitch &= ~SYS_WARNING_MASK;
    }
    else
    {
        g_sys_conf.SysSwitch |= SYS_WARNING_MASK;
    }
#endif

    LED_OFF(); //华兄
    LCD_Off();
    
    while (DEF_TRUE) 
    {
#if 0 //华兄        
        //电路接通
        if(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_6)==0)
        {
            
            g_sys_conf.SysRunStatus |= SYS_RELAY_ON_FLAG;
            //if(jdq_timer)
            {            
            //    jdq_timer = 0;
            //    JDQ_OFF();
            }
        }
        else
        {
            
            g_sys_conf.SysRunStatus &= (~SYS_RELAY_ON_FLAG);
        }
#else

#if 0
        /* 检测接触器状态 */
        if(g_pow_count) //华兄
        {
            g_sys_conf.SysRunStatus |= SYS_RELAY_ON_FLAG;
        }
        else
        {
            g_sys_conf.SysRunStatus &= (~SYS_RELAY_ON_FLAG);
        }
#endif

#endif

        if(jdq_timer)
        {            
            jdq_timer--;
            if(jdq_timer == 0)
            {
                //DEBUG_LED_OFF();
                
                JDQ_OFF();
            }
        }

        if(JDQ_ST())
            g_sys_conf.SysRunStatus |= SYS_JDQ_ON_FLAG;
        else
            g_sys_conf.SysRunStatus &= (~SYS_JDQ_ON_FLAG);

        ac_rms = ADC_get_ac_rms();

        g_dbg_disp = ac_rms;
        
        switch(g_power_state)
        {
        case SYS_POWER_ON:

#if 1
            /* 检测接触器状态 */
            if(g_pow_count) //华兄
            {
                g_sys_conf.SysRunStatus |= SYS_RELAY_ON_FLAG;
            }
            else
            {
                g_sys_conf.SysRunStatus &= (~SYS_RELAY_ON_FLAG);
            }
#endif

            diff_num = ADC_get_diff_num();
                        
            LED_PWR_OFF();
            LED_HD_OFF();
            
            if((diff_num == 0) && ((g_sys_conf.voltageFixCoe*ac_rms) > AC_POWER_NORMAL_RMS))
            {
                g_diff_num = 0;
                g_diff_array = 0;
                g_diff_pos = 0;
                g_power_state = SYS_POWER_STEADY;    
                
                LED_PWR_ON();    

#if (SELF_POWER_EN > 0u)                
                if(g_sys_conf.SysSwitch & (SYS_SELF_POWER_MASK))
                {
                    if(!(g_sys_conf.SysRunStatus & SYS_RELAY_ON_FLAG))
                    {
                        g_sys_conf.SysRunStatus |= SYS_POWER_CHECK_FLAG;

                        if(OK == App_Check_Self_Power()) //华兄，须校准时间日期，方有效
                        {
                            g_power_check_flag = TRUE;
                            g_power_check_count = (g_sys_conf.SelfPowerActionTimeHigh * 1000) + (g_sys_conf.SelfPowerActionTimeLow * 100);
                        }
                        else
                        {
                            g_power_check_flag = FALSE;
                            g_power_check_count = 0;
                        }                        
                    }
                }
#endif                
            }
            else if(diff_num == 0)
            {
                if((g_sys_conf.voltageFixCoe*ac_rms) > AC_POWER_RECOVER_RMS)
                {
                    count++;
                    
                    if(count >= 10)
                    {
                        count = 0;
                        
                        g_rms_buf[g_rms_buf_index] = ac_rms;
                        g_rms_buf_index++;
                    }
                    
                    if(MAX_RMS_NUM == g_rms_buf_index)
                    {
                        sum = 0;
                        
                        for(i = 0; i < MAX_RMS_NUM; i++)
                        {
                            sum += g_rms_buf[i];
                        }
                    
                        average = sum / MAX_RMS_NUM;
                    
                        sum = 0;
                    
                        for(i = 0; i < MAX_RMS_NUM; i++)
                        {
                            sum += pow(abs((int)g_rms_buf[i] - (int)average), 2);
                        }
                    
                        g_rms_discrete = sqrt(sum) * 100;
                        //g_dbg_disp = g_rms_discrete;

                        count = 0;
                        g_rms_buf_index = 0;

                        if(g_rms_discrete < MAX_RMS_DISCRETE)
                        {
                            g_diff_num = 0;
                            g_diff_array = 0;
                            g_diff_pos = 0;
                            g_power_state = SYS_POWER_STEADY;   
                            
                            LED_PWR_ON();    
            
#if (SELF_POWER_EN > 0u)                
                            if(g_sys_conf.SysSwitch & (SYS_SELF_POWER_MASK))
                            {
                                if(!(g_sys_conf.SysRunStatus & SYS_RELAY_ON_FLAG))
                                {
                                    g_sys_conf.SysRunStatus |= SYS_POWER_CHECK_FLAG;
            
                                    if(OK == App_Check_Self_Power()) //华兄，须校准时间日期，方有效
                                    {
                                        g_power_check_flag = TRUE;
                                        g_power_check_count = (g_sys_conf.SelfPowerActionTimeHigh * 1000) + (g_sys_conf.SelfPowerActionTimeLow * 100);
                                    }
                                    else
                                    {
                                        g_power_check_flag = FALSE;
                                        g_power_check_count = 0;
                                    }                        
                                }
                            }
#endif                
                        }
                    }
                }
                else
                {
                    count = 0;
                    g_rms_buf_index = 0;
                }
            }            
            else
            {
                count = 0;
                g_rms_buf_index = 0;
            }                        
            
            break;
            
        case SYS_POWER_STEADY:
            LED_HD_OFF();
            
            count = 0;
            g_rms_buf_index = 0;

#if 1
            /* 检测接触器状态 */
            if(g_pow_count) //华兄
            {
                g_sys_conf.SysRunStatus |= SYS_RELAY_ON_FLAG;

                g_main_mos_delay_off_time = 0;

                if((0 == g_main_mos_check_jdq_on_time) && (g_main_mos_delay_on_time < 15)) //500ms
                {
                    MAIN_MOS_ON();

                    g_main_mos_delay_on_time++;

                    if(15 == g_main_mos_delay_on_time) //15ms
                    {
                        if(FALSE == g_sys_conf.main_mos_broken) //主MOS管坏了，不断开主电路继电器。否则，会断开 
                        {
                            MAIN_MOS_CHECK_JDQ_OFF();
                        }

                        g_main_mos_broken_test = TRUE;
                        g_main_mos_broken_test_time = 100; //100ms
                    }
                }
            }
            else
            {
                g_sys_conf.SysRunStatus &= (~SYS_RELAY_ON_FLAG);

                g_main_mos_check_jdq_on_time = 500;
                g_main_mos_delay_on_time = 0;

                if(g_main_mos_delay_off_time < 100)
                {                   
                    g_main_mos_delay_off_time++;
                    
                    if(100 == g_main_mos_delay_off_time)
                    {
                        MAIN_MOS_CHECK_JDQ_ON();
                        MAIN_MOS_OFF();
                    }
                }
            }            

            /* 测试主MOS管好坏 */
            if(TRUE == g_main_mos_broken_test)
            {
                if(g_main_mos_broken_test_time)
                {
                    g_main_mos_broken_test_time--;

                    if(0 == g_main_mos_broken_test_time)
                    {
                        g_main_mos_broken_test = FALSE;
                        
                        if(0 == g_pow_count)
                        {
                            g_sys_conf.main_mos_broken = TRUE;

                            /* 写E2PROM记录 */
                            
                        }
                    }
                }
            }          
#endif

            //diff_num = ADC_get_diff_num();  
            //ac_rms = ADC_get_ac_rms();      

#if (SELF_POWER_EN > 0u)            
            if(g_sys_conf.SysSwitch & (SYS_SELF_POWER_MASK))
            {
                if(g_sys_conf.SysRunStatus & SYS_POWER_CHECK_FLAG)
                {
#if 0                    
                    if(OK == App_Check_Self_Power()) //须校准时间日期，方有效
                    {
                        if(1) //华兄
                        {   
                            //DEBUG_LED_ON();
                        
                            JDQ_ON();
                            jdq_timer = JDO_ON_ACT_TIME;
                            MEM_SaveSelfEvent(0);
                        }

                        g_sys_conf.SysRunStatus &= (~SYS_POWER_CHECK_FLAG);                        
                    }
                    else if(ERROR == App_Check_Self_Power())
                    {
                        g_sys_conf.SysRunStatus &= (~SYS_POWER_CHECK_FLAG);  
                    }
#else //华兄
                    if(TRUE == g_power_check_flag)
                    {
                        if(0 == g_power_check_count)
                        {
                            g_power_check_flag = FALSE;
                            
                            //DEBUG_LED_ON();
                            
                            JDQ_ON();
                            jdq_timer = JDO_ON_ACT_TIME;
                            MEM_SaveSelfEvent(0);
                            
                            g_sys_conf.SysRunStatus &= (~SYS_POWER_CHECK_FLAG);   
                        }
                    }
                    else
                    {
                        g_sys_conf.SysRunStatus &= (~SYS_POWER_CHECK_FLAG);  
                    }
#endif
                }
            }
#endif

            break;

        case SYS_DROPING:
            
            //ac_rms = ADC_get_ac_rms();
           
            if((g_sys_conf.voltageFixCoe*ac_rms) > AC_POWER_RECOVER_RMS)
            {
                g_droping_power_on_time++;
                if((g_droping_power_on_time > 25 ) /*&& g_ac_sb1_count*/) //华兄
                {
                    if(g_sys_conf.SysRunStatus & SYS_DROP_ACTION)
                    {
#if 1
                        /* 检测接触器状态 */
                        if(g_pow_count) //华兄
                        {
                            g_sys_conf.SysRunStatus |= SYS_RELAY_ON_FLAG;
                        }
                        else
                        {
                            g_sys_conf.SysRunStatus &= (~SYS_RELAY_ON_FLAG);
                        }
#endif
                        
                        //继电器有投切动作
                        if(g_sys_conf.SysRunStatus & SYS_RELAY_ON_FLAG)
                        {
                                                    
                            //继电器仍然在合闸
                            cnts = g_last_uptime;
                            if(OSTimeGet() < cnts)
                            {
                                break;
                            }

                            hclk_freq = (OSTimeGet()- cnts);

                            /* 华兄 */
                            if(RELAY_FORWARD_SWITCH_ON == g_relay_switch)
                            {
                                if((hclk_freq > 0) && (hclk_freq <= 2))
                                {
                                    //DEBUG_LED_ON();
                                    
                                    //电源恢复正常
                                    Relay_Keep_Mode(RELAY_SWITCH_OFF);
                                    g_sys_conf.SysRunStatus &= (~SYS_DROP_ACTION);
                                    g_power_state = SYS_POWER_ON;
                                    LED_HD_OFF();
                                    MEM_SaveDropEvent(OK);
                                    break;
                                }
                            }
                            else
                            {
                                if((hclk_freq > 10) && (hclk_freq <= 12))
                                {
                                    //DEBUG_LED_ON();
                                    
                                    //电源恢复正常
                                    Relay_Keep_Mode(RELAY_SWITCH_OFF);
                                    g_sys_conf.SysRunStatus &= (~SYS_DROP_ACTION);
                                    g_power_state = SYS_POWER_ON;
                                    LED_HD_OFF();
                                    MEM_SaveDropEvent(OK);
                                    break;
                                }
                            }

#ifdef DEBUG_LED
                            if(g_droping_power_on_time > 35) //华兄
                            {
                                //DEBUG_LED_ON();
                            }
#endif                            
                        }
                        else
                        {
                            //继电器已经分开
                            //电源恢复正常
                            Relay_Keep_Mode(RELAY_SWITCH_OFF);
                            g_power_state = SYS_POWER_ON;
                            LED_HD_OFF();
                            
                            MEM_SaveDropTime();
                            g_dropping_msec = g_msec_count; //华兄
                            MEM_SaveDropEvent(ERROR);
                            g_sys_conf.SysRunStatus &= (~SYS_DROP_ACTION);                            
                            
                            break;
                        }
                    }
                    else
                    {
                        //用户工作继电器没有投入工作
                        //电源恢复正常
                        Relay_Keep_Mode(RELAY_SWITCH_OFF);
                        g_power_state = SYS_POWER_ON;
                        LED_HD_OFF();
                        //MEM_SaveDropEvent(OK);
                        break;
                    }
                }
            }
            else
            {
                g_droping_power_on_time = 0;
            }

            
            if((OSTimeGet()) > (g_droping_timestamp + g_sys_conf.PowerDropKeepTime))
            {          
#if 0 //华兄               
TASK_DROPED_PROC:            
#endif                   

                 g_power_state = SYS_DROPED;
                 g_droping_power_on_time = 0;
                 //g_ac_sb1_count  = 0; //华兄
                 g_pow_count = 0; 

                 if(g_sys_conf.SysRunStatus & SYS_DROP_ACTION)
                 {
                    //OS_ENTER_CRITICAL();
                    MEM_SaveDropTime();
                    g_dropping_msec = g_msec_count; //华兄
                    //OS_EXIT_CRITICAL();
                    g_sys_conf.SysRunStatus &= (~SYS_DROP_ACTION);                
                 }    

                 Relay_Keep_Mode(RELAY_SWITCH_OFF);
                 LED_HD_OFF();
                    
                 break;
            }

#if 0 //华兄
            if(!(g_sys_conf.SysRunStatus & SYS_RELAY_ON_FLAG))
            {
                //如果继电器已经断开
                break_off_count++;
                if(break_off_count > 4)
                {                    
                    goto TASK_DROPED_PROC;
                }
            }
            else
            {
                break_off_count = 0;
            }
#endif

            break;
        case SYS_DROPED:
        {
#if 0 //华兄            
            if(g_ac_sb1_count)
#endif

            if((g_sys_conf.voltageFixCoe * ac_rms) > AC_POWER_RECOVER_RMS)
            {
                g_droping_power_on_time++;
                if(g_droping_power_on_time > 60)
                {
                    g_power_state = SYS_POWER_ON;
                    
                    lcd_init();                  
                    //MEM_Init();
                }
            }
            else
            {
                LED_RUN_OFF();
                LED_PWR_OFF();
                LED_HD_OFF();
                g_droping_power_on_time = 0;
            }
            
            break;
        }
        case SYS_AUTH_ERROR:
            LCM_PWR_OFF();
            LCD_Off();
            break;            
        }

        /* 华兄 */
        if(TRUE == g_sys_conf.main_mos_broken)
        {
            MAIN_MOS_BROKEN_ALRAM_ON();
        }         
              
        OSTimeDly(1);
    }
}

int  main(void)
{
    CPU_INT08U  err;
    
    
    CPU_IntDis();

    /* Set the Vector Table base location at APPLICATION_ADDRESS */ 
    NVIC_SetVectorTable(NVIC_VectTab_FLASH, APPLICATION_ADDRESS & (~NVIC_VectTab_FLASH)); //华兄

    OSInit();                                                   /* Initialize "uC/OS-II, The Real-Time Kernel"              */

    OSTaskCreateExt( App_TaskStart,                             /* Create the start task                                    */
                    (void *)0,
                    (OS_STK *)&App_TaskStartStk[APP_CFG_TASK_START_STK_SIZE - 1],
                     APP_CFG_TASK_START_PRIO,
                     APP_CFG_TASK_START_PRIO,
                    (OS_STK *)&App_TaskStartStk[0],
                     APP_CFG_TASK_START_STK_SIZE,
                    (void *)0,
                     OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR);

    OSStart();         
}

