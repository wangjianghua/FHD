#include "includes.h"


void MEM_Para_Init()
{
    memset(&g_sys_conf, 0, sizeof(SYS_CONF));

    g_sys_conf.initStatusWord = SYS_MAGIC_WORD;

    MEM_ClearPassword(); //华兄
    
    g_sys_conf.PowerDropKeepTime = DEFAULT_DROP_KEEP_TIME;
        
    g_sys_conf.SelfPowerTime = DEFAULT_SELF_POWER_TIME;

#if (SELF_POWER_EN > 0u)
    g_sys_conf.SelfPowerDeadlineHigh = DEFAULT_SELF_POWER_DEADLINE_HIGH;
    g_sys_conf.SelfPowerDeadlineLow = DEFAULT_SELF_POWER_DEADLINE_LOW;
    g_sys_conf.SelfPowerActionTimeHigh = DEFAULT_SELF_POWER_ACTION_TIME_HIGH;
    g_sys_conf.SelfPowerActionTimeLow = DEFAULT_SELF_POWER_ACTION_TIME_LOW;
#endif    

    g_sys_conf.break_points = 2; //华兄
    g_sys_conf.adc_diff = 250;
    
    g_sys_conf.SysSwitch = SYS_DROP_KEEP_MASK;

#if 0    
    g_sys_conf.selfPowerLimit = 90;
#else //华兄
    g_sys_conf.selfPowerLimit = 0;
#endif

#if 0    
    g_sys_conf.SelfPowerValidTime = 3;
#else //华兄
    g_sys_conf.SelfPowerValidTime = 0;
#endif

    g_sys_conf.voltageFixCoe = 11000;
    g_sys_conf.dev_addr[0] = 1;

#if (MAIN_MOS_CHECK_EN > 0u)
    g_sys_conf.main_mos_broken = FALSE; //华兄
#endif    

#if (MULTI_MODE_EN > 0u)    
    g_sys_conf.mode = enum_mode_0; //华兄
#endif    
}

void MEM_Init()
{
    int i = 3;


    while(i > 0)
    {    
        E2promReadBuffer(0, (unsigned char *)&g_sys_conf, sizeof(SYS_CONF));
        if(g_sys_conf.initStatusWord == SYS_MAGIC_WORD)
        {
            break;
        }
        OSTimeDly(500);
        i--;

        clr_wdt(); //华兄
    }

    if(0 == i)
    {
        MEM_Para_Init();
        g_rtc_time[YEAR_POS] = 0x15;
        g_rtc_time[MONTH_POS] = 0x01;
        g_rtc_time[DATE_POS] = 0x01;
        g_rtc_time[DAY_POS] = 0x04;
        g_rtc_time[HOUR_POS] = 0x00;
        g_rtc_time[MIN_POS] = 0x00;
        g_rtc_time[SEC_POS] = 0x00;                
        RTC_WriteTime(g_rtc_time);
        E2promWriteBuffer(0, (unsigned char *)&g_sys_conf, sizeof(SYS_CONF));
        
        OSTimeDly(500);
        E2promReadBuffer(0, (unsigned char *)&g_sys_conf, sizeof(SYS_CONF));
        if(g_sys_conf.initStatusWord != SYS_MAGIC_WORD)
        {
            MEM_Para_Init();
            g_sys_conf.SysRunStatus |= SYS_ERROR_EEPROM_RW;
        }

        clr_wdt(); //华兄
    }
}

unsigned char MEM_Cal_Time_Crc(unsigned char *timep)
{
    int cac;
    
    if(timep == 0)
        return 0;

    cac = timep[0] + timep[1] + timep[2] + timep[3] + timep[4] + timep[5] + timep[6];

    cac += 0xB6;

    return cac;
}

void MEM_SaveSelfEvent(unsigned char ok_flag)
{

#if 1
    int save_pos;

    unsigned char temp[8];
    
    //OSSchedLock();
    //RTC_ReadTime(temp);
    //OSSchedUnlock();

    save_pos = g_sys_conf.SelfPowerOnCount % DROP_HISTORY_MAX_COUNT;
    memcpy(&g_sys_conf.SelfPowerOnTime[save_pos], g_rtc_time, 7);

    g_sys_conf.SelfPowerOnTime[save_pos][SYS_SAVE_TIME_LEN-1] = MEM_Cal_Time_Crc((unsigned char*)&g_sys_conf.SelfPowerOnTime[save_pos][0]);
   
    
    //OSSchedLock();    
    E2promWriteBuffer(OffsetOf(SYS_CONF, SelfPowerOnTime[0][0]) + save_pos*SYS_SAVE_TIME_LEN,
                        &g_sys_conf.SelfPowerOnTime[save_pos][0], 
                        SYS_SAVE_TIME_LEN);

    
    
    g_sys_conf.SelfPowerOnCount++;
    E2promWriteBuffer(OffsetOf(SYS_CONF, SelfPowerOnCount),
                        (unsigned char*)&g_sys_conf.SelfPowerOnCount, 
                        SizeOf(SYS_CONF, SelfPowerOnCount));
    //OSSchedUnlock();
#endif    
}

void MEM_SaveDropEvent(unsigned char ok_flag)
{

#if 1
    int save_pos;

    unsigned short st;

    //unsigned char temp[8];
    
    //OSSchedLock();
    //RTC_ReadTime(temp);
    //OSSchedUnlock();

    st  = (OSTimeGet() - g_droping_timestamp );

    if(st < 120)
        return;

    save_pos = g_sys_conf.PowerDropCount % DROP_HISTORY_MAX_COUNT;
    memcpy(&g_sys_conf.PowerDropTime[save_pos], g_rtc_time, 7);

    g_sys_conf.PowerDropTime[save_pos][SYS_SAVE_TIME_LEN-1] = MEM_Cal_Time_Crc((unsigned char*)&g_sys_conf.PowerDropTime[save_pos][0]);
    if(ok_flag == OK)        
        g_sys_conf.PowerDropTime[save_pos][SYS_SAVE_TIME_LEN-1] &= (~0X01);
    else
        g_sys_conf.PowerDropTime[save_pos][SYS_SAVE_TIME_LEN-1] |= (0X01);
    
    //OSSchedLock();    
    E2promWriteBuffer(OffsetOf(SYS_CONF, PowerDropTime[0][0]) + save_pos*SYS_SAVE_TIME_LEN,
                        &g_sys_conf.PowerDropTime[save_pos][0], 
                        SYS_SAVE_TIME_LEN);

   
    memcpy(&g_sys_conf.PowerDropPeriod[save_pos], &st, 2);
    E2promWriteBuffer(OffsetOf(SYS_CONF, PowerDropPeriod[0]) + save_pos*2,
                        (unsigned char*)&g_sys_conf.PowerDropPeriod[save_pos], 
                        2);

    
    g_sys_conf.PowerDropCount++;
    E2promWriteBuffer(OffsetOf(SYS_CONF, PowerDropCount),
                        (unsigned char*)&g_sys_conf.PowerDropCount, 
                        SizeOf(SYS_CONF, PowerDropCount));
    //OSSchedUnlock();
#endif    
}


void MEM_SaveDropTime()
{

#if 1
    unsigned char temp[8];

    //GPIO_SetBits(GPIOB, GPIO_Pin_8);
    
    //OSSchedLock();
    //RTC_ReadTime(temp);
   
    
    memcpy(&g_sys_conf.lastBreakTimeStamp, g_rtc_time, 7);

    g_sys_conf.lastBreakTimeStamp[7] = MEM_Cal_Time_Crc(&g_sys_conf.lastBreakTimeStamp[0]);
                
    E2promWriteBuffer(OffsetOf(SYS_CONF, lastBreakTimeStamp[0]),
                        (unsigned char*)&g_sys_conf.lastBreakTimeStamp[0], 
                        SYS_SAVE_TIME_LEN);
    //OSSchedUnlock();

    //GPIO_ResetBits(GPIOB, GPIO_Pin_8);
#endif    
}

// 华兄
unsigned char MEM_Cal_Pwd_Crc(unsigned char *ptr)
{
    int crc;
    
    
    if(NULL == ptr)
    {
        return (0);
    }

    crc = ptr[0] + ptr[1] + ptr[2] + ptr[3] + ptr[4] + ptr[5];

    crc += 0x88;

    return (crc);
}

void MEM_SavePassword(void)
{
    memcpy(&g_sys_conf.password[0], g_new_pwd, 6);

    g_sys_conf.password[6] = MEM_Cal_Pwd_Crc(&g_sys_conf.password[0]);

    E2promWriteBuffer( OffsetOf(SYS_CONF, password[0]),
                      (unsigned char *)&g_sys_conf.password[0],
                       SYS_PWD_LEN);
}

void MEM_RestorePassword(void)
{
    u8 crc;

    
    E2promReadBuffer( OffsetOf(SYS_CONF, password[0]),
                     (unsigned char *)&g_sys_conf.password[0],
                      SYS_PWD_LEN);

    crc = MEM_Cal_Pwd_Crc(&g_sys_conf.password[0]);

    if(g_sys_conf.password[6] != crc)
    {
        return;
    }

    memcpy(g_pwd, &g_sys_conf.password[0], 6);
}

void MEM_ClearPassword(void)
{
    memset(&g_sys_conf.password[0], 0, SYS_PWD_LEN);

    g_sys_conf.password[6] = MEM_Cal_Pwd_Crc(&g_sys_conf.password[0]);

    E2promWriteBuffer( OffsetOf(SYS_CONF, password[0]),
                      (unsigned char *)&g_sys_conf.password[0],
                       SYS_PWD_LEN);
}

