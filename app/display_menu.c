#include "includes.h"

SYS_CONF g_sys_conf;

unsigned char form_id = FORM_ID_MAIN_FORM;

unsigned char g_rtc_time[7];



extern unsigned char g_hard_v_f;

/*days for 12 months*/
const unsigned short days_per_month[] =
{
    0, 31, 60, 91, 121, 152,
    182, 213, 244, 274, 305, 335,
};

const char monthDays[]= {31,28,31,30,31,30,31,31,30,31,30,31};

const char SYS_HARDWARE_VER[] = {0x15, 0x01, 0x25, 0x20, 0x00};

const char SYS_SOFTWARE_VER[] = {0x15, 0x06, 0x24, 0x26, 0x00};

const LCD_FORM form_list[MAX_FORM_NUM] =
{
    {LCD_disp_main_form},
    
    {LCD_disp_keep_form},

#if (SELF_POWER_EN > 0u)
    {LCD_disp_self_form},
#endif

    {LCD_disp_about_form},
    
    {LCD_disp_setting_form},
        
};

BOOL g_pwd_flag = FALSE; //华兄
char g_pwd_sn = 0, g_enter_pwd[6] = {0}, g_new_pwd[7] = {0}, g_pwd[6] = {0};
const char SYS_PWD_DEFAULT[] = {8, 8, 1, 2, 1, 8};

/* 输入:  BCD CHAR    /输出: 返回Hex UCHAR */
unsigned char Bcd2HexChar(unsigned char bcd)
{
    return((bcd>>4)*10+(bcd&0x0F));
}

/* 输入:  Hex UCHAR    /输出: 返回BCD CHAR     */
unsigned char Hex2BcdChar(unsigned char hex)
{
    return((hex%10)|(((unsigned char)(hex/10))<<4));
}

void Bcd2HexBuffer(unsigned char *bcd, unsigned short len)
{
    if(len == 0)
        return;

    while(len--)
    {
        *bcd = Bcd2HexChar(*bcd);
        bcd++;
    }
}


// validate the tm structure
static void CheckTime(struct RTCCounterValue *timeptr)
{
    // we could do some normalization here, e.g.
    // change 40 october to 9 november

    if (timeptr->Sec>59) timeptr->Sec=59;
    if (timeptr->Min>59) timeptr->Min=59;
    if (timeptr->Hour>23) timeptr->Hour=23;
    if (timeptr->Week>7) timeptr->Week=7;
    if (timeptr->Day<1) timeptr->Day=1;
    else if (timeptr->Day>31) timeptr->Day=31;
    if (timeptr->Month > 12) timeptr->Month=12;
    if (timeptr->Month == 0) timeptr->Month=1;
    if (timeptr->Year<0) timeptr->Year=0;
    if (timeptr->Year>99) timeptr->Year=99;
}


// convert broken time to calendar time (seconds since 2000)
U32 OS_mktime(struct RTCCounterValue *tmp)
{

#define TIME_CAL_START_YEAR   2000L
    
    int year, month, i;
    U32 seconds;
    struct RTCCounterValue timeptr;

    memcpy(&timeptr, tmp, sizeof(struct RTCCounterValue));
    Bcd2HexBuffer((unsigned char*) &timeptr, sizeof(struct RTCCounterValue));

    CheckTime(&timeptr);

    year = timeptr.Year + TIME_CAL_START_YEAR;

    month = timeptr.Month;

    // seconds from 2000 till 1 jan 00:00:00 this year
    seconds= (year - TIME_CAL_START_YEAR)*(60*60*24L*365);

    // add extra days for leap years
    for (i=TIME_CAL_START_YEAR; i<year; i++)
    {
        if (LEAP_YEAR(i))
        {
            seconds+= 60*60*24L;
        }
    }

    // add days for this year
    for (i=0; i<(month-1); i++)
    {
        if (i==1 && LEAP_YEAR(year))
        {
            seconds+= 60L*60*24L*29;
        }
        else
        {
            seconds+= 60*60*24L*monthDays[i];
        }
    }

    seconds+= (timeptr.Day-1)*60L*60*24L;
    seconds+= timeptr.Hour*60L*60;
    seconds+= timeptr.Min*60L;
    seconds+= timeptr.Sec;

    return seconds;
}

unsigned char get_max_day_in_month(unsigned char year_bcd, unsigned month_bcd)
{
    unsigned char month= Bcd2HexChar(month_bcd);
    if((month >= 13) ||(month == 0))
        return 30;

    if(((Bcd2HexChar(year_bcd)%4) == 0) && (month == 2))
        return (monthDays[month - 1] + 1);
    
    return (monthDays[month - 1]);
        
}

void LCD_Clear_All()
{
    char em_s[21];
    memset(em_s, ' ', 20);
    em_s[20] = 0;
    GUI_DispStringAt(em_s, 0, 0); 
    GUI_DispStringAt(em_s, 0, 16); 
    GUI_DispStringAt(em_s, 0, 32); 
    GUI_DispStringAt(em_s, 0, 48); 
}


void LCD_Clear_WorkWindow()
{
    char em_s[21];
    memset(em_s, ' ', 20);
    em_s[20] = 0;
    GUI_DispStringAt(em_s, 0, 16); 
    GUI_DispStringAt(em_s, 0, 32); 
    GUI_DispStringAt(em_s, 0, 48); 
}


int LCD_find_next_self_sn(unsigned char *sn, unsigned char * sp, unsigned char dir)
{
    unsigned char s_p,crc, res, count = 0, new_sn = *sn;   

    if(!g_sys_conf.SelfPowerOnCount)
        return ERROR;

MAIN_FORM_NEXT_SN:

    if(count >= 10)
        return ERROR;

    if(dir == FIND_UP)
        new_sn = (new_sn+1)%10;
    else
        new_sn = (new_sn+9)%10;
        
    if((g_sys_conf.SelfPowerOnCount > new_sn) && (new_sn  < DROP_HISTORY_MAX_COUNT))
    {
        s_p = (((g_sys_conf.SelfPowerOnCount + 9)%10) + 10 - new_sn)%10;

        crc = MEM_Cal_Time_Crc((unsigned char*)&g_sys_conf.SelfPowerOnTime[s_p][0]);

        if((crc) != ((g_sys_conf.SelfPowerOnTime[s_p][7])))
        {            
            count++;
            goto MAIN_FORM_NEXT_SN;
        }

        *sn = new_sn;
        *sp = s_p;
        res = OK;
    }
    else
    {
             
          if(dir == FIND_UP)
          {
                new_sn = 9;
          }
          else
          {
             if(g_sys_conf.SelfPowerOnCount >= 10)
             {
                new_sn = 9;
             }
             else
             {
                new_sn = ((g_sys_conf.SelfPowerOnCount));
             }
          }
          goto MAIN_FORM_NEXT_SN;
    }

    return res;

}

int LCD_find_next_drop_sn(unsigned char *sn, unsigned char * sp, unsigned char dir)
{
    unsigned char s_p,crc, res, count = 0, new_sn = *sn;   

    if(!g_sys_conf.PowerDropCount)
        return ERROR;

MAIN_FORM_NEXT_SN:

    if(count >= 10)
        return ERROR;

    if(dir == FIND_UP)
        new_sn = (new_sn+1)%10;
    else
        new_sn = (new_sn+9)%10;
        
    if((g_sys_conf.PowerDropCount > new_sn) && (new_sn  < DROP_HISTORY_MAX_COUNT))
    {
        s_p = (((g_sys_conf.PowerDropCount + 9)%10) + 10 - new_sn)%10;

        crc = MEM_Cal_Time_Crc((unsigned char*)&g_sys_conf.PowerDropTime[s_p][0]);

        if((crc&0xfe) != ((g_sys_conf.PowerDropTime[s_p][7])&0xfe))
        {            
            count++;
            goto MAIN_FORM_NEXT_SN;
        }

        *sn = new_sn;
        *sp = s_p;
        res = OK;
    }
    else
    {
             
          if(dir == FIND_UP)
          {
                new_sn = 9;
          }
          else
          {
             if(g_sys_conf.PowerDropCount >= 10)
             {
                new_sn = 9;
             }
             else
             {
                new_sn = ((g_sys_conf.PowerDropCount));
             }
          }
          goto MAIN_FORM_NEXT_SN;
    }

    return res;

}

int LCD_disp_main_form(unsigned int key_event, unsigned int form_msg)
{
    static unsigned char sn = 0;
    
    unsigned char temp[8], Headline_disp_buf[18], res = OK;
    unsigned char s_p, crc, t_sn;
   
    if(form_msg == FORM_MSG_RERRESH)
    {        
        LCD_Time_Refreash();
    }
    else if(form_msg == FORM_MSG_KEY)
    {
        switch(key_event)
        {
        case KEY_VAL_OK:   
            //form_id = FORM_ID_KEEP_FORM;
            form_id = (form_id + 1)%(MAX_FORM_NUM - 1);
            return FORM_MSG_DATA;
        case KEY_VAL_SET:
            form_id = FORM_ID_SETTING_FORM;
            return FORM_MSG_RERRESH;
        }
    }
    else
    {
        return FORM_MSG_NON;
    }
     

    sprintf(Headline_disp_buf, "晃电次数:%07d",  g_sys_conf.PowerDropCount);
    GUI_DispStringAt(Headline_disp_buf, 0, 16); 

#if (SELF_POWER_EN > 0u)    
    sprintf(Headline_disp_buf, "再启次数:%07d",  g_sys_conf.SelfPowerOnCount);
#else //华兄
    sprintf(Headline_disp_buf, "保护时长:%04d ms",  g_sys_conf.PowerDropKeepTime);
#endif
    GUI_DispStringAt(Headline_disp_buf, 0, 32);

    crc = MEM_Cal_Time_Crc((unsigned char*)&g_sys_conf.lastBreakTimeStamp[0]);

    if((crc&0xfe) != ((g_sys_conf.PowerDropTime[s_p][7])&0xfe))
    {            
        sprintf(Headline_disp_buf, "T:%02X-%02X-%02X-%02X-%02X",  
                                    g_sys_conf.lastBreakTimeStamp[5],
                                    g_sys_conf.lastBreakTimeStamp[4],
                                    g_sys_conf.lastBreakTimeStamp[2],
                                    g_sys_conf.lastBreakTimeStamp[1],
                                    g_sys_conf.lastBreakTimeStamp[0]);
    }
    else
    {
#if 1        
        sprintf(Headline_disp_buf, "T:%02X-%02X-%02X-%02X-%02X",  
                                    g_sys_conf.lastBreakTimeStamp[5],
                                    g_sys_conf.lastBreakTimeStamp[4],
                                    g_sys_conf.lastBreakTimeStamp[2],
                                    g_sys_conf.lastBreakTimeStamp[1],
                                    g_sys_conf.lastBreakTimeStamp[0]);
#else        
        crc = 0;
        sprintf(Headline_disp_buf, "T:%02d-%02d-%02d-%02d-%02d",  
                                        crc,
                                        crc,
                                        crc,
                                        crc,
                                        crc);
#endif            
    }
                                                                
    GUI_DispStringAt(Headline_disp_buf, 0, 48);

    return FORM_MSG_NON;



    
}

int LCD_disp_self_form(unsigned int key_event, unsigned int form_msg)
{
    static unsigned char sn = 0;
    
    unsigned char temp[8], Headline_disp_buf[18], res = OK;
    unsigned char s_p, crc, t_sn;
   
    if(form_msg == FORM_MSG_RERRESH)
    {        
        LCD_Time_Refreash();        
        return FORM_MSG_NON;
    }

    if(FORM_MSG_DATA == form_msg)
    {
        sn = 1;
        res = LCD_find_next_self_sn(&sn, &s_p, FIND_DOWN);
        if(res != OK)
            sn = 0;
    }

    if(form_msg == FORM_MSG_KEY)
    {
        switch(key_event)
        {
        case KEY_VAL_UP:            
            res = LCD_find_next_self_sn(&sn, &s_p,  FIND_UP);           
            break;
        case KEY_VAL_DOWN:            
            res = LCD_find_next_self_sn(&sn, &s_p, FIND_DOWN);              
            break;
        case KEY_VAL_OK:            
            //form_id = FORM_ID_MAIN_FORM;
            form_id = (form_id + 1)%(MAX_FORM_NUM - 1);
            return FORM_MSG_DATA;
        case KEY_VAL_SET:
            form_id = FORM_ID_SETTING_FORM;
            return FORM_MSG_RERRESH;
        }
    }      

    
    if(res == OK)
    {        
  
        sprintf(Headline_disp_buf, "上%02d次再启动    ", (sn+1) );

        GUI_DispStringAt(Headline_disp_buf, 0, 16);

                
        sprintf(Headline_disp_buf, "T:%02X-%02X-%02X-%02X-%02X",  
                                        g_sys_conf.SelfPowerOnTime[s_p][5],
                                        g_sys_conf.SelfPowerOnTime[s_p][4],
                                        g_sys_conf.SelfPowerOnTime[s_p][2],
                                        g_sys_conf.SelfPowerOnTime[s_p][1],
                                        g_sys_conf.SelfPowerOnTime[s_p][0]);
                                        //g_sys_conf.PowerDropTime[s_p][0]);
        GUI_DispStringAt(Headline_disp_buf, 0, 32);
    }
    else
    {       
    
        sprintf(Headline_disp_buf, "上%02d次再启动无    ", (sn+1) );

        GUI_DispStringAt(Headline_disp_buf, 0, 16);
        
        crc = 0;
        sprintf(Headline_disp_buf, "T:%02d-%02d-%02d-%02d-%02d",  
                                        crc,
                                        crc,
                                        crc,
                                        crc,
                                        crc);
        GUI_DispStringAt(Headline_disp_buf, 0, 32);
    }

    sprintf(Headline_disp_buf, "                ");
    sprintf(Headline_disp_buf, "     %06d     ",ac_rms);
    GUI_DispStringAt(Headline_disp_buf, 0, 48);

    return FORM_MSG_NON;
                
}

int LCD_disp_about_form(unsigned int key_event, unsigned int form_msg)
{
    static unsigned char sn = 0;
    unsigned char temp[8], Headline_disp_buf[18], res = OK;
    unsigned char s_p, crc, t_sn;
    unsigned char ver_ftr;
    
   
    if(form_msg == FORM_MSG_RERRESH)
    {        
        LCD_Time_Refreash();        
        return FORM_MSG_NON;
    }


    if(form_msg == FORM_MSG_KEY)
    {
        switch(key_event)
        {
        case KEY_VAL_UP:            
            //res = LCD_find_next_drop_sn(&sn, &s_p,  FIND_UP);            
            break;
        case KEY_VAL_DOWN:            
            //res = LCD_find_next_drop_sn(&sn, &s_p, FIND_DOWN);              
            break;
        case KEY_VAL_OK:            
            //form_id = FORM_ID_MAIN_FORM;
            form_id = (form_id + 1)%(MAX_FORM_NUM - 1);
            return FORM_MSG_DATA;
        case KEY_VAL_SET:
            form_id = FORM_ID_SETTING_FORM;
            return FORM_MSG_RERRESH;
        }
    }      

    sprintf(Headline_disp_buf, "Soft:%02X%02X%02X-%02X%02X",  
                SYS_SOFTWARE_VER[0],
                SYS_SOFTWARE_VER[1],
                SYS_SOFTWARE_VER[2],
                SYS_SOFTWARE_VER[3],
                SYS_SOFTWARE_VER[4]);

    ver_ftr = get_ver_ftr();

    sprintf(&Headline_disp_buf[14], "%02x", ver_ftr); //华兄
    
    GUI_DispStringAt(Headline_disp_buf, 0, 16); 
    
    sprintf(Headline_disp_buf, "Hard:%02X%02X%02X-%02X%02X",  
                SYS_HARDWARE_VER[0],
                SYS_HARDWARE_VER[1],
                SYS_HARDWARE_VER[2],
                SYS_HARDWARE_VER[3],
                SYS_HARDWARE_VER[4]);

#if 0 //华兄
    Headline_disp_buf[15] = 0x30 + AUTO_Getsar();
    Headline_disp_buf[14] = 0x30 + g_hard_v_f;
#endif    

    GUI_DispStringAt(Headline_disp_buf, 0, 32);

    sprintf(Headline_disp_buf, "   20%02X-%02X-%02X   ", g_rtc_time[YEAR_POS], g_rtc_time[MONTH_POS], g_rtc_time[DATE_POS]);
    GUI_DispStringAt(Headline_disp_buf, 0, 48);


    return FORM_MSG_NON;
                
}


int LCD_disp_keep_form(unsigned int key_event, unsigned int form_msg)
{
    static unsigned char sn = 0;
    
    unsigned char temp[8], Headline_disp_buf[18], res = OK;
    unsigned char s_p, crc, t_sn;
   
    if(form_msg == FORM_MSG_RERRESH)
    {        
        LCD_Time_Refreash();        
        return FORM_MSG_NON;
    }

    if(FORM_MSG_DATA == form_msg)
    {
        sn = 1;
        res = LCD_find_next_drop_sn(&sn, &s_p, FIND_DOWN);
        if(res != OK)
            sn = 0;
    }

    if(form_msg == FORM_MSG_KEY)
    {
        switch(key_event)
        {
        case KEY_VAL_UP:            
            res = LCD_find_next_drop_sn(&sn, &s_p,  FIND_UP);
            
            break;
        case KEY_VAL_DOWN:            
            res = LCD_find_next_drop_sn(&sn, &s_p, FIND_DOWN);              
            break;
        case KEY_VAL_OK:            
            //form_id = FORM_ID_MAIN_FORM;
            form_id = (form_id + 1)%(MAX_FORM_NUM - 1);
            return FORM_MSG_DATA;
        case KEY_VAL_SET:
            form_id = FORM_ID_SETTING_FORM;
            return FORM_MSG_RERRESH;
        }
    }      

    
    if(res == OK)
    {        
        if((g_sys_conf.PowerDropTime[s_p][7]&0x01))
            sprintf(Headline_disp_buf, "上%02d次防晃电失效", (sn+1) );
        else
            sprintf(Headline_disp_buf, "上%02d次防晃电成功", (sn+1) );

        GUI_DispStringAt(Headline_disp_buf, 0, 16);

        
        sprintf(Headline_disp_buf, "晃电时长:%04d ms",  g_sys_conf.PowerDropPeriod[s_p]);
        GUI_DispStringAt(Headline_disp_buf, 0, 32);
        sprintf(Headline_disp_buf, "T:%02X-%02X-%02X-%02X-%02X",  
                                        g_sys_conf.PowerDropTime[s_p][5],
                                        g_sys_conf.PowerDropTime[s_p][4],
                                        g_sys_conf.PowerDropTime[s_p][2],
                                        g_sys_conf.PowerDropTime[s_p][1],
                                        g_sys_conf.PowerDropTime[s_p][0]);
                                        //g_sys_conf.PowerDropTime[s_p][0]);
        GUI_DispStringAt(Headline_disp_buf, 0, 48);
    }
    else
    {       
    
        sprintf(Headline_disp_buf, "上%02d次防晃电无  ", (sn+1) );


        GUI_DispStringAt(Headline_disp_buf, 0, 16);

        
        sprintf(Headline_disp_buf, "晃电时长:0000 ms");
        GUI_DispStringAt(Headline_disp_buf, 0, 32);
        
        crc = 0;
        sprintf(Headline_disp_buf, "T:%02d-%02d-%02d-%02d-%02d",  
                                        crc,
                                        crc,
                                        crc,
                                        crc,
                                        crc);
        GUI_DispStringAt(Headline_disp_buf, 0, 48);
    }

    return FORM_MSG_NON;
                
}


int LCD_disp_setting_form(unsigned int key_event, unsigned int form_msg)
{
    static EN_SET_ITEM set_sn = 0, date_sn, reset, new_time[7];

    static char self_power_sn = 0;
    
    unsigned char temp[8], Headline_disp_buf[18], res;
    unsigned char time_t;

    LCD_Clear_All();
    
    if(FALSE == g_pwd_flag) //华兄
    {
        set_sn = enum_menu_password;
    }    
    
    if(form_msg == FORM_MSG_RERRESH)
    {
        //LCD_Time_Refreash();
    }
    else if(form_msg == FORM_MSG_KEY)
    {
        switch(key_event)
        {
        case KEY_VAL_SET: 
            if(enum_menu_password == set_sn) //华兄
            {
                g_pwd_sn = (g_pwd_sn + 1) % enum_pwd_button;
            }
            else if(enum_menu_set_password == set_sn) //华兄
            {
                g_pwd_sn = (g_pwd_sn + 1) % enum_pwd_button;
            }
            
#if (SELF_POWER_EN > 0u)            
            else if(en_menu_set_selfpowerdeadline == set_sn) //华兄
            {
                self_power_sn = (self_power_sn + 1) % 2;
            }
            else if(en_menu_set_selfpoweractiontime == set_sn) //华兄
            {
                self_power_sn = (self_power_sn + 1) % 2;
            }
#endif            

            else if(set_sn == en_menu_set_date)
            {
                //RTC_ReadTime(new_time);
                //date_sn = YEAR_POS;
                date_sn = (date_sn + 1)% 7;
            }
            else if(set_sn == en_menu_set_reset)
            {
                reset = !reset;
            }
            else
            {
                set_sn = (set_sn + 1) % en_menu_set_butt; 

#if (SELF_POWER_EN > 0u)
                if(en_menu_set_selfpowerdeadline == set_sn) //华兄
                {
                    self_power_sn = enum_self_power_0;
                }
#else
                if(0)
                {

                }
#endif
                else if(set_sn == en_menu_set_date)
                {
                    RTC_ReadTime(new_time);
                    date_sn = en_year_set;
                }
                else if(set_sn == en_menu_set_date)
                    reset = FALSE;
            }
            break;
        case KEY_VAL_DOWN:
            if(enum_menu_password == set_sn) //华兄
            {
                switch(g_pwd_sn)
                {
                case enum_pwd_0:
                    if(0 == g_enter_pwd[enum_pwd_0])
                    {
                        g_enter_pwd[enum_pwd_0] = 9;
                    }
                    else
                    {
                        g_enter_pwd[enum_pwd_0] = g_enter_pwd[enum_pwd_0] - 1;
                    }
                    break;

                case enum_pwd_1:
                    if(0 == g_enter_pwd[enum_pwd_1])
                    {
                        g_enter_pwd[enum_pwd_1] = 9;
                    }
                    else
                    {
                        g_enter_pwd[enum_pwd_1] = g_enter_pwd[enum_pwd_1] - 1;
                    }
                    break;

                case enum_pwd_2:
                    if(0 == g_enter_pwd[enum_pwd_2])
                    {
                        g_enter_pwd[enum_pwd_2] = 9;
                    }
                    else
                    {
                        g_enter_pwd[enum_pwd_2] = g_enter_pwd[enum_pwd_2] - 1;
                    }
                    break;

                case enum_pwd_3:
                    if(0 == g_enter_pwd[enum_pwd_3])
                    {
                        g_enter_pwd[enum_pwd_3] = 9;
                    }
                    else
                    {
                        g_enter_pwd[enum_pwd_3] = g_enter_pwd[enum_pwd_3] - 1;
                    }
                    break;

                case enum_pwd_4:
                    if(0 == g_enter_pwd[enum_pwd_4])
                    {
                        g_enter_pwd[enum_pwd_4] = 9;
                    }
                    else
                    {
                        g_enter_pwd[enum_pwd_4] = g_enter_pwd[enum_pwd_4] - 1;
                    }
                    break;      

                case enum_pwd_5:
                    if(0 == g_enter_pwd[enum_pwd_5])
                    {
                        g_enter_pwd[enum_pwd_5] = 9;
                    }
                    else
                    {
                        g_enter_pwd[enum_pwd_5] = g_enter_pwd[enum_pwd_5] - 1;
                    }
                    break;                     
                }
            }
            else if(enum_menu_set_password == set_sn) //华兄
            {
                switch(g_pwd_sn)
                {
                case enum_pwd_0:
                    if(0 == g_new_pwd[enum_pwd_0])
                    {
                        g_new_pwd[enum_pwd_0] = 9;
                    }
                    else
                    {
                        g_new_pwd[enum_pwd_0] = g_new_pwd[enum_pwd_0] - 1;
                    }
                    break;

                case enum_pwd_1:
                    if(0 == g_new_pwd[enum_pwd_1])
                    {
                        g_new_pwd[enum_pwd_1] = 9;
                    }
                    else
                    {
                        g_new_pwd[enum_pwd_1] = g_new_pwd[enum_pwd_1] - 1;
                    }
                    break;

                case enum_pwd_2:
                    if(0 == g_new_pwd[enum_pwd_2])
                    {
                        g_new_pwd[enum_pwd_2] = 9;
                    }
                    else
                    {
                        g_new_pwd[enum_pwd_2] = g_new_pwd[enum_pwd_2] - 1;
                    }
                    break;

                case enum_pwd_3:
                    if(0 == g_new_pwd[enum_pwd_3])
                    {
                        g_new_pwd[enum_pwd_3] = 9;
                    }
                    else
                    {
                        g_new_pwd[enum_pwd_3] = g_new_pwd[enum_pwd_3] - 1;
                    }
                    break;

                case enum_pwd_4:
                    if(0 == g_new_pwd[enum_pwd_4])
                    {
                        g_new_pwd[enum_pwd_4] = 9;
                    }
                    else
                    {
                        g_new_pwd[enum_pwd_4] = g_new_pwd[enum_pwd_4] - 1;
                    }
                    break;      

                case enum_pwd_5:
                    if(0 == g_new_pwd[enum_pwd_5])
                    {
                        g_new_pwd[enum_pwd_5] = 9;
                    }
                    else
                    {
                        g_new_pwd[enum_pwd_5] = g_new_pwd[enum_pwd_5] - 1;
                    }
                    break;                     
                }
            }            
            else if(set_sn ==  en_menu_set_droptimelength)
            {
                if(g_sys_conf.PowerDropKeepTime >= DROP_TIME_ADJUST_STEP*2)
                {
                   g_sys_conf.PowerDropKeepTime -= DROP_TIME_ADJUST_STEP;
                } 
                
                //if(g_break_pos > 1)
                //    g_break_pos--;
            }
            
#if (SELF_POWER_EN > 0u)            
            else if(en_menu_set_selfpowerdeadline == set_sn)
            {
                switch(self_power_sn)
                {
                case enum_self_power_0:
                    if(g_sys_conf.SelfPowerDeadlineHigh >= SELF_TIME_ADJUST_STEP * 2)
                    {
                       g_sys_conf.SelfPowerDeadlineHigh -= SELF_TIME_ADJUST_STEP;
                    } 
                    break;

                case enum_self_power_1:
                    if(g_sys_conf.SelfPowerDeadlineLow >= SELF_TIME_ADJUST_STEP)
                    {
                       g_sys_conf.SelfPowerDeadlineLow -= SELF_TIME_ADJUST_STEP;
                    }                     
                    break;
                }                         
            }            
            else if(en_menu_set_selfpoweractiontime == set_sn)
            {
#if 0                
                if(g_sys_conf.SelfPowerTime >= SELF_TIME_ADJUST_STEP*2)
                {
                   g_sys_conf.SelfPowerTime -= SELF_TIME_ADJUST_STEP;
                }    
#else //华兄
                switch(self_power_sn)
                {
                case enum_self_power_0:
                    if(g_sys_conf.SelfPowerActionTimeHigh >= SELF_TIME_ADJUST_STEP * 2)
                    {
                       g_sys_conf.SelfPowerActionTimeHigh -= SELF_TIME_ADJUST_STEP;
                    } 
                    break;

                case enum_self_power_1:
                    if(g_sys_conf.SelfPowerActionTimeLow >= SELF_TIME_ADJUST_STEP)
                    {
                       g_sys_conf.SelfPowerActionTimeLow -= SELF_TIME_ADJUST_STEP;
                    }                     
                    break;
                }            
#endif                
            }
#endif

            else if(set_sn == en_menu_set_hdswitch)
            {
                 if(g_sys_conf.SysSwitch & (SYS_DROP_KEEP_MASK))
                 {
                    g_sys_conf.SysSwitch &= (~SYS_DROP_KEEP_MASK);
                 }
                 else
                 {
                    g_sys_conf.SysSwitch |= (SYS_DROP_KEEP_MASK);
                 }
            }

#if (SELF_POWER_EN > 0u)            
            else if(set_sn == en_menu_set_selfswitch)
            {
                 if(g_sys_conf.SysSwitch & (SYS_SELF_POWER_MASK))
                 {
                    g_sys_conf.SysSwitch &= (~SYS_SELF_POWER_MASK);
                 }
                 else
                 {
                    g_sys_conf.SysSwitch |= (SYS_SELF_POWER_MASK);
                 }
            }   
#endif

#if (MULTI_MODE_EN > 0u)
            else if(en_menu_set_mode == set_sn)
            {
                if(enum_mode_0 == g_sys_conf.mode)
                {
                    g_sys_conf.mode = enum_mode_1;
                }
                else
                {
                    g_sys_conf.mode--;
                }
            }
#endif

            else if(set_sn == en_menu_set_addr)
            {
                 g_sys_conf.dev_addr[0]--;
                 if(g_sys_conf.dev_addr[0] == 0)
                    g_sys_conf.dev_addr[0] = 255;
                    
            }       
            else if(set_sn == en_menu_set_reset)                
            {
                reset = !reset;
            }                 
            else if(set_sn == en_menu_set_date)
            {
                switch(date_sn)
                {
                case en_year_set:
                    time_t = Bcd2HexChar(new_time[YEAR_POS])+99;
                    time_t %= 100;
                    new_time[YEAR_POS] = Hex2BcdChar(time_t);
                    break;
                case en_month_set:
#if 0                       
                    time_t = Bcd2HexChar(new_time[MONTH_POS])+11;
                 
                    if(time_t > 12)
                        time_t = 1;
#else //华兄
                    time_t = Bcd2HexChar(new_time[MONTH_POS]);

                    if(time_t > 1)
                    {
                        time_t--;
                    }
                    else
                    {
                        time_t = 12;
                    }
#endif
                    new_time[MONTH_POS] = Hex2BcdChar(time_t);
                    break;
                case en_day_set:
                    time_t = Bcd2HexChar(new_time[DATE_POS]);
                    if(time_t > 1)
                        time_t--;
                    else
                        time_t = get_max_day_in_month(new_time[YEAR_POS], new_time[MONTH_POS]);                                   
                    new_time[DATE_POS] = Hex2BcdChar(time_t);
                    break;
                case en_hour_set:
                    time_t = Bcd2HexChar(new_time[HOUR_POS])+23;
                    time_t %= 24;  
                    new_time[HOUR_POS] = Hex2BcdChar(time_t);
                    break;
                case en_min_set:
                    time_t = Bcd2HexChar(new_time[MIN_POS])+59;
                    time_t %= 60;  
                    new_time[MIN_POS] = Hex2BcdChar(time_t);
                    break;
                case en_sec_set:
                    time_t = Bcd2HexChar(new_time[SEC_POS])+59;
                    time_t %= 60;  
                    new_time[SEC_POS] = Hex2BcdChar(time_t);
                    break;
                }
                
            }

            break;
        case KEY_VAL_UP:
            if(enum_menu_password == set_sn) //华兄
            {
                switch(g_pwd_sn)
                {
                case enum_pwd_0:
                    g_enter_pwd[enum_pwd_0] = (g_enter_pwd[enum_pwd_0] + 1) % 10;
                    break;

                case enum_pwd_1:
                    g_enter_pwd[enum_pwd_1] = (g_enter_pwd[enum_pwd_1] + 1) % 10;
                    break;

                case enum_pwd_2:
                    g_enter_pwd[2] = (g_enter_pwd[enum_pwd_2] + 1) % 10;
                    break;

                case enum_pwd_3:
                    g_enter_pwd[enum_pwd_3] = (g_enter_pwd[enum_pwd_3] + 1) % 10;
                    break;

                case enum_pwd_4:
                    g_enter_pwd[enum_pwd_4] = (g_enter_pwd[enum_pwd_4] + 1) % 10;
                    break;      

                case enum_pwd_5:
                    g_enter_pwd[enum_pwd_5] = (g_enter_pwd[enum_pwd_5] + 1) % 10;
                    break;                     
                }
            }
            else if(enum_menu_set_password == set_sn) //华兄
            {
                switch(g_pwd_sn)
                {
                case enum_pwd_0:
                    g_new_pwd[enum_pwd_0] = (g_new_pwd[enum_pwd_0] + 1) % 10;
                    break;

                case enum_pwd_1:
                    g_new_pwd[enum_pwd_1] = (g_new_pwd[enum_pwd_1] + 1) % 10;
                    break;

                case enum_pwd_2:
                    g_new_pwd[2] = (g_new_pwd[enum_pwd_2] + 1) % 10;
                    break;

                case enum_pwd_3:
                    g_new_pwd[enum_pwd_3] = (g_new_pwd[enum_pwd_3] + 1) % 10;
                    break;

                case enum_pwd_4:
                    g_new_pwd[enum_pwd_4] = (g_new_pwd[enum_pwd_4] + 1) % 10;
                    break;      

                case enum_pwd_5:
                    g_new_pwd[enum_pwd_5] = (g_new_pwd[enum_pwd_5] + 1) % 10;
                    break;                     
                }
            }                        
            else if(set_sn ==  en_menu_set_droptimelength)
            {
                g_sys_conf.PowerDropKeepTime += DROP_TIME_ADJUST_STEP;
                if(g_sys_conf.PowerDropKeepTime > MAX_DROP_TIME)
                {
                   g_sys_conf.PowerDropKeepTime = MAX_DROP_TIME;
                }

                //if(g_break_pos > 1)
                //    g_break_pos++;
            }

#if (SELF_POWER_EN > 0u)            
            else if(en_menu_set_selfpowerdeadline == set_sn)
            {
                switch(self_power_sn)
                {
                case enum_self_power_0:
                    g_sys_conf.SelfPowerDeadlineHigh += SELF_TIME_ADJUST_STEP;
                    if(g_sys_conf.SelfPowerDeadlineHigh > SELF_POWER_ON_TIME_HIGH)
                    {
                       g_sys_conf.SelfPowerDeadlineHigh = SELF_POWER_ON_TIME_HIGH;
                    }
                    break;

                case enum_self_power_1:
                    g_sys_conf.SelfPowerDeadlineLow += SELF_TIME_ADJUST_STEP;
                    if(g_sys_conf.SelfPowerDeadlineLow > SELF_POWER_ON_TIME_LOW)
                    {
                       g_sys_conf.SelfPowerDeadlineLow = SELF_POWER_ON_TIME_LOW;
                    }                 
                    break;
                }                  
            }            
            else if(en_menu_set_selfpoweractiontime == set_sn)
            {
#if 0                
                g_sys_conf.SelfPowerTime += SELF_TIME_ADJUST_STEP;
                if(g_sys_conf.SelfPowerTime > SELF_POWER_ON_TIME)
                {
                   g_sys_conf.SelfPowerTime = SELF_POWER_ON_TIME;
                }
#else //华兄
                switch(self_power_sn)
                {
                case enum_self_power_0:
                    g_sys_conf.SelfPowerActionTimeHigh += SELF_TIME_ADJUST_STEP;
                    if(g_sys_conf.SelfPowerActionTimeHigh > SELF_POWER_ON_TIME_HIGH)
                    {
                       g_sys_conf.SelfPowerActionTimeHigh = SELF_POWER_ON_TIME_HIGH;
                    }
                    break;

                case enum_self_power_1:
                    g_sys_conf.SelfPowerActionTimeLow += SELF_TIME_ADJUST_STEP;
                    if(g_sys_conf.SelfPowerActionTimeLow > SELF_POWER_ON_TIME_LOW)
                    {
                       g_sys_conf.SelfPowerActionTimeLow = SELF_POWER_ON_TIME_LOW;
                    }                 
                    break;
                }          
#endif                
            }
#endif

            else if(set_sn == en_menu_set_hdswitch)
            {
                 if(g_sys_conf.SysSwitch & (SYS_DROP_KEEP_MASK))
                 {
                    g_sys_conf.SysSwitch &= (~SYS_DROP_KEEP_MASK);
                 }
                 else
                 {
                    g_sys_conf.SysSwitch |= (SYS_DROP_KEEP_MASK);
                 }
            }

#if (SELF_POWER_EN > 0u)            
            else if(set_sn == en_menu_set_selfswitch)
            {
                 if(g_sys_conf.SysSwitch & (SYS_SELF_POWER_MASK))
                 {
                    g_sys_conf.SysSwitch &= (~SYS_SELF_POWER_MASK);
                 }
                 else
                 {
                    g_sys_conf.SysSwitch |= (SYS_SELF_POWER_MASK);
                 }
            }
#endif

#if (MULTI_MODE_EN > 0u)
            else if(en_menu_set_mode == set_sn)
            {
                g_sys_conf.mode++;

                g_sys_conf.mode %= enum_mode_button;
            }
#endif

            else if(set_sn == en_menu_set_addr)
            {
                 g_sys_conf.dev_addr[0]++;
                 if(g_sys_conf.dev_addr[0] == 0)
                    g_sys_conf.dev_addr[0] = 1;
            }  
            else if(set_sn == en_menu_set_reset)                
            {
                reset = !reset;
            }     
            else if(set_sn == en_menu_set_date)
            {
                switch(date_sn)
                {
                case en_year_set:
                    time_t = Bcd2HexChar(new_time[YEAR_POS])+1;
                    time_t %= 100;
                    new_time[YEAR_POS] = Hex2BcdChar(time_t);
                    break;
                case en_month_set:
                    time_t = Bcd2HexChar(new_time[MONTH_POS])+1;
                    if(time_t > 12)
                        time_t = 1;
                    new_time[MONTH_POS] = Hex2BcdChar(time_t);
                    break;
                case en_day_set:
                    time_t = Bcd2HexChar(new_time[DATE_POS])+1;
                    if(time_t > get_max_day_in_month(new_time[YEAR_POS], new_time[MONTH_POS]))
                       time_t = 1;                    
                    new_time[DATE_POS] = Hex2BcdChar(time_t);
                    break;
                case en_hour_set:
                    time_t = Bcd2HexChar(new_time[HOUR_POS])+1;
                    time_t %= 24;  
                    new_time[HOUR_POS] = Hex2BcdChar(time_t);
                    break;
                case en_min_set:
                    time_t = Bcd2HexChar(new_time[MIN_POS])+1;
                    time_t %= 60;  
                    new_time[MIN_POS] = Hex2BcdChar(time_t);
                    break;
                case en_sec_set:
                    time_t = Bcd2HexChar(new_time[SEC_POS])+1;
                    time_t %= 60;  
                    new_time[SEC_POS] = Hex2BcdChar(time_t);
                    break;
                }
            }

            break;
        
        case KEY_VAL_OK:
            if(enum_menu_password == set_sn) //华兄
            {
                if(enum_pwd_ok == g_pwd_sn)
                {
                    MEM_RestorePassword();
                    
                    if(((g_pwd[enum_pwd_0] == g_enter_pwd[enum_pwd_0]) &&
                        (g_pwd[enum_pwd_1] == g_enter_pwd[enum_pwd_1]) &&
                        (g_pwd[enum_pwd_2] == g_enter_pwd[enum_pwd_2]) &&
                        (g_pwd[enum_pwd_3] == g_enter_pwd[enum_pwd_3]) &&
                        (g_pwd[enum_pwd_4] == g_enter_pwd[enum_pwd_4]) &&
                        (g_pwd[enum_pwd_5] == g_enter_pwd[enum_pwd_5])) 

                        || 
                        
                       ((SYS_PWD_DEFAULT[0] == g_enter_pwd[enum_pwd_0]) &&
                        (SYS_PWD_DEFAULT[1] == g_enter_pwd[enum_pwd_1]) &&
                        (SYS_PWD_DEFAULT[2] == g_enter_pwd[enum_pwd_2]) &&
                        (SYS_PWD_DEFAULT[3] == g_enter_pwd[enum_pwd_3]) &&
                        (SYS_PWD_DEFAULT[4] == g_enter_pwd[enum_pwd_4]) &&
                        (SYS_PWD_DEFAULT[5] == g_enter_pwd[enum_pwd_5])))
                    {
                        g_pwd_flag = TRUE;

                        set_sn = en_menu_set_droptimelength;
                        g_pwd_sn = enum_pwd_0;
                        
                        memset(g_new_pwd, 0, sizeof(g_new_pwd));
                    }
                    else
                    {
                        g_pwd_flag = FALSE;
                    }
                }
                else if(enum_pwd_cancel == g_pwd_sn)
                {
                    E2promWriteBuffer(0, (unsigned char *)&g_sys_conf, OffsetOf(SYS_CONF, PowerDropCount));
                    
                    form_id = FORM_ID_MAIN_FORM;
                    
                    return (FORM_MSG_RERRESH);
                }

                break;
            }

            if(enum_menu_set_password == set_sn) //华兄
            {
                if(enum_pwd_ok == g_pwd_sn)
                {
                    MEM_SavePassword();
                    
                    g_pwd_flag = FALSE;
                    
                    set_sn = enum_menu_password;
                    
                }
                else if(enum_pwd_cancel == g_pwd_sn)
                {
                    set_sn = en_menu_set_droptimelength;             
                }

                g_pwd_sn = enum_pwd_0;
                memset(g_enter_pwd, 0, sizeof(g_enter_pwd));     

                break;
            }                        

#if (SELF_POWER_EN > 0u)
            if(en_menu_set_selfpowerdeadline == set_sn) //华兄
            {
                set_sn = en_menu_set_selfpoweractiontime;

                self_power_sn = enum_self_power_0;
 
                break;
            } 
            else if(en_menu_set_selfpoweractiontime == set_sn) //华兄
            {
                set_sn = en_menu_set_hdswitch;
                
                break;
            }            
#else
            if(0)
            {

            }
#endif
            else if(set_sn == en_menu_set_date)
            {
                if(date_sn == en_ok_set)
                {
                    RTC_WriteTime(new_time);                                      
                }
                
                set_sn = en_menu_set_reset;
                
                break;
            }
            else if(set_sn == en_menu_set_reset)                
            {
                if(reset == TRUE) 
                {
                    //OSSchedLock();
                    MEM_Para_Init();
                    E2promWriteBuffer(0, (unsigned char *)&g_sys_conf, sizeof(SYS_CONF));
                    //OSSchedUnlock();                        
                } 
                 
               
                set_sn = enum_menu_set_password; //华兄
                g_pwd_sn = enum_pwd_0;
                
                break;
                
            }
            
            E2promWriteBuffer(0, (unsigned char *)&g_sys_conf, OffsetOf(SYS_CONF, PowerDropCount));
            
            form_id = FORM_ID_MAIN_FORM;
            return FORM_MSG_RERRESH;
        }
    }
    else
    {
        return FORM_MSG_NON;
    }

    switch(set_sn)
    {
    case enum_menu_password: //华兄
        sprintf(Headline_disp_buf, "密码:");
        GUI_DispStringAt(Headline_disp_buf, 0, 16);    
        sprintf(Headline_disp_buf, "******");
        GUI_DispStringAt(Headline_disp_buf, 40, 32);    
        sprintf(Headline_disp_buf, "确定  取消"); //5个汉字宽度
        GUI_DispStringAt(Headline_disp_buf, 24, 48);       

        switch(g_pwd_sn)
        {
        case enum_pwd_0:
            sprintf(Headline_disp_buf, "%x", g_enter_pwd[enum_pwd_0]); 
            GUI_DispRevStringAt(Headline_disp_buf, 40, 32);
            break;

        case enum_pwd_1:
            sprintf(Headline_disp_buf, "%x", g_enter_pwd[enum_pwd_1]); 
            GUI_DispRevStringAt(Headline_disp_buf, 48, 32);
            break;

        case enum_pwd_2:
            sprintf(Headline_disp_buf, "%x", g_enter_pwd[enum_pwd_2]); 
            GUI_DispRevStringAt(Headline_disp_buf, 56, 32);
            break;

        case enum_pwd_3:
            sprintf(Headline_disp_buf, "%x", g_enter_pwd[enum_pwd_3]); 
            GUI_DispRevStringAt(Headline_disp_buf, 64, 32);
            break;    

        case enum_pwd_4:
            sprintf(Headline_disp_buf, "%x", g_enter_pwd[enum_pwd_4]); 
            GUI_DispRevStringAt(Headline_disp_buf, 72, 32);
            break;

        case enum_pwd_5:
            sprintf(Headline_disp_buf, "%x", g_enter_pwd[enum_pwd_5]); 
            GUI_DispRevStringAt(Headline_disp_buf, 80, 32);
            break;       

        case enum_pwd_ok:
            sprintf(Headline_disp_buf, "确定"); 
            GUI_DispRevStringAt(Headline_disp_buf, 24, 48);            
            break;

        case enum_pwd_cancel:
            sprintf(Headline_disp_buf, "取消"); 
            GUI_DispRevStringAt(Headline_disp_buf, 72, 48);            
            break;            
        }
        
        break;

    case enum_menu_set_password: //华兄
        sprintf(Headline_disp_buf, "新密码:");
        GUI_DispStringAt(Headline_disp_buf, 0, 16);        
        sprintf(Headline_disp_buf, "%x%x%x%x%x%x", g_new_pwd[enum_pwd_0], g_new_pwd[enum_pwd_1], g_new_pwd[enum_pwd_2], 
                                                   g_new_pwd[enum_pwd_3], g_new_pwd[enum_pwd_4], g_new_pwd[enum_pwd_5]);
        GUI_DispStringAt(Headline_disp_buf, 40, 32);           
        sprintf(Headline_disp_buf, "保存  取消"); //5个汉字宽度
        GUI_DispStringAt(Headline_disp_buf, 24, 48);       

        switch(g_pwd_sn)
        {
        case enum_pwd_0:
            sprintf(Headline_disp_buf, "%x", g_new_pwd[enum_pwd_0]); 
            GUI_DispRevStringAt(Headline_disp_buf, 40, 32);
            break;

        case enum_pwd_1:
            sprintf(Headline_disp_buf, "%x", g_new_pwd[enum_pwd_1]); 
            GUI_DispRevStringAt(Headline_disp_buf, 48, 32);
            break;

        case enum_pwd_2:
            sprintf(Headline_disp_buf, "%x", g_new_pwd[enum_pwd_2]); 
            GUI_DispRevStringAt(Headline_disp_buf, 56, 32);
            break;

        case enum_pwd_3:
            sprintf(Headline_disp_buf, "%x", g_new_pwd[enum_pwd_3]); 
            GUI_DispRevStringAt(Headline_disp_buf, 64, 32);
            break;    

        case enum_pwd_4:
            sprintf(Headline_disp_buf, "%x", g_new_pwd[enum_pwd_4]); 
            GUI_DispRevStringAt(Headline_disp_buf, 72, 32);
            break;

        case enum_pwd_5:
            sprintf(Headline_disp_buf, "%x", g_new_pwd[enum_pwd_5]); 
            GUI_DispRevStringAt(Headline_disp_buf, 80, 32);
            break;       

        case enum_pwd_ok:
            sprintf(Headline_disp_buf, "保存"); 
            GUI_DispRevStringAt(Headline_disp_buf, 24, 48);            
            break;

        case enum_pwd_cancel:
            sprintf(Headline_disp_buf, "取消"); 
            GUI_DispRevStringAt(Headline_disp_buf, 72, 48);            
            break;            
        }
        
        break;        
        
    case en_menu_set_droptimelength:
        sprintf(Headline_disp_buf, "晃电保护时长:");
        GUI_DispStringAt(Headline_disp_buf, 0, 16);    
            
        sprintf(Headline_disp_buf, "%04d ms", g_sys_conf.PowerDropKeepTime);
        GUI_DispRevStringAt(Headline_disp_buf, 36, 32);  

        //sprintf(Headline_disp_buf, "%04d po", g_break_pos);
        //GUI_DispRevStringAt(Headline_disp_buf, 48, 48);  
        
            
        break;
        
#if (SELF_POWER_EN > 0u)        
        case en_menu_set_selfpowerdeadline:
            sprintf(Headline_disp_buf, "再启动截止时间:");
            GUI_DispStringAt(Headline_disp_buf, 0, 16);    
    
#if 0
            sprintf(Headline_disp_buf, "%04d s", g_sys_conf.SelfPowerTime); 
            GUI_DispRevStringAt(Headline_disp_buf, 40, 32);
#else //华兄
            sprintf(Headline_disp_buf, "%02d.%d s", g_sys_conf.SelfPowerDeadlineHigh, g_sys_conf.SelfPowerDeadlineLow);
            GUI_DispStringAt(Headline_disp_buf, 40, 32);      
#endif        
    
            switch(self_power_sn)
            {
            case enum_self_power_0:
                sprintf(Headline_disp_buf, "%02d", g_sys_conf.SelfPowerDeadlineHigh); 
                GUI_DispRevStringAt(Headline_disp_buf, 40, 32);
                break;
    
            case enum_self_power_1:
                sprintf(Headline_disp_buf, "%d", g_sys_conf.SelfPowerDeadlineLow); 
                GUI_DispRevStringAt(Headline_disp_buf, 64, 32);            
                break;
            }    
    
            break;
    case en_menu_set_selfpoweractiontime:
        sprintf(Headline_disp_buf, "再启动动作时间:");
        GUI_DispStringAt(Headline_disp_buf, 0, 16);    

#if 0
        sprintf(Headline_disp_buf, "%04d s", g_sys_conf.SelfPowerTime); 
        GUI_DispRevStringAt(Headline_disp_buf, 40, 32);
#else //华兄
        sprintf(Headline_disp_buf, "%02d.%d s", g_sys_conf.SelfPowerActionTimeHigh, g_sys_conf.SelfPowerActionTimeLow);
        GUI_DispStringAt(Headline_disp_buf, 40, 32);      
#endif        

        switch(self_power_sn)
        {
        case enum_self_power_0:
            sprintf(Headline_disp_buf, "%02d", g_sys_conf.SelfPowerActionTimeHigh); 
            GUI_DispRevStringAt(Headline_disp_buf, 40, 32);
            break;

        case enum_self_power_1:
            sprintf(Headline_disp_buf, "%d", g_sys_conf.SelfPowerActionTimeLow); 
            GUI_DispRevStringAt(Headline_disp_buf, 64, 32);            
            break;
        }    

        break;
#endif

    case en_menu_set_hdswitch:
        sprintf(Headline_disp_buf, "晃电保护开关:");
        GUI_DispStringAt(Headline_disp_buf, 0, 16);    

        if(g_sys_conf.SysSwitch & (SYS_DROP_KEEP_MASK))
        {                 
            sprintf(Headline_disp_buf, "开");
        }
        else
        {
            sprintf(Headline_disp_buf, "关");
        }
         
        GUI_DispRevStringAt(Headline_disp_buf, 56, 32); 
        break;

#if (SELF_POWER_EN > 0u)        
    case en_menu_set_selfswitch:

        sprintf(Headline_disp_buf, "再启动保护开关:");
        GUI_DispStringAt(Headline_disp_buf, 0, 16); 

        if(g_sys_conf.SysSwitch & (SYS_SELF_POWER_MASK))
        {                 
            sprintf(Headline_disp_buf, "开");
        }
        else
        {
            sprintf(Headline_disp_buf, "关");
        }
         
        GUI_DispRevStringAt(Headline_disp_buf, 56, 32); 
        
        break;
#endif

#if (MULTI_MODE_EN > 0u)
    case en_menu_set_mode:

        sprintf(Headline_disp_buf, "工作模式:");
        GUI_DispStringAt(Headline_disp_buf, 0, 16); 

        switch(g_sys_conf.mode)
        {
        case enum_mode_0:
            sprintf(Headline_disp_buf, "模式1");
            break;

        case enum_mode_1:
            sprintf(Headline_disp_buf, "模式2");
            break;

        default:
            sprintf(Headline_disp_buf, "模式1");
            break;
        }

        GUI_DispRevStringAt(Headline_disp_buf, 44, 32); 
        
        break;
#endif

    case en_menu_set_addr:
        sprintf(Headline_disp_buf, "Modbus地址:");
        GUI_DispStringAt(Headline_disp_buf, 0, 16); 
        
        sprintf(Headline_disp_buf, "%03d", g_sys_conf.dev_addr[0]);                 
        GUI_DispRevStringAt(Headline_disp_buf, 52, 32); 
          
        break;
    case en_menu_set_reset:
        
        sprintf(Headline_disp_buf, "  系统数据复位  ");
        GUI_DispStringAt(Headline_disp_buf, 0, 16); 
        sprintf(Headline_disp_buf, "历史数据将被清除!");
        GUI_DispStringAt(Headline_disp_buf, 0, 32); 

        sprintf(Headline_disp_buf, "确定  取消"  );
        //GUI_DispStringAt(Headline_disp_buf, 40, 48);   
        GUI_DispStringAt(Headline_disp_buf, 24, 48); //华兄
        
        if(reset == TRUE)
        {
            sprintf(Headline_disp_buf, "确定"); 
            //GUI_DispRevStringAt(Headline_disp_buf, 40, 48);     
            GUI_DispRevStringAt(Headline_disp_buf, 24, 48); //华兄
        }
        else
        {
            sprintf(Headline_disp_buf, "取消"); 
            //GUI_DispRevStringAt(Headline_disp_buf, 88, 48);
            GUI_DispRevStringAt(Headline_disp_buf, 72, 48); //华兄
        }
        break;
    case en_menu_set_date:

        sprintf(Headline_disp_buf, "Y:%02X,M:%02X,D:%02X", new_time[YEAR_POS],new_time[MONTH_POS],new_time[DATE_POS] );
        GUI_DispStringAt(Headline_disp_buf, 9, 16);    
        sprintf(Headline_disp_buf, "H:%02X,M:%02X,S:%02X", new_time[HOUR_POS],new_time[MIN_POS],new_time[SEC_POS] );
        GUI_DispStringAt(Headline_disp_buf, 9, 32);    
        sprintf(Headline_disp_buf, "保存"  );
        GUI_DispStringAt(Headline_disp_buf, 48, 48);    

        switch(date_sn)
        {
        case en_year_set:
            sprintf(Headline_disp_buf, "%02X", new_time[YEAR_POS]); 
            GUI_DispRevStringAt(Headline_disp_buf, 25, 16);
            break;
        case en_month_set:
            sprintf(Headline_disp_buf, "%02X", new_time[MONTH_POS]); 
            GUI_DispRevStringAt(Headline_disp_buf, 65, 16);           
            break;
        case en_day_set:
            sprintf(Headline_disp_buf, "%02X", new_time[DATE_POS]); 
            GUI_DispRevStringAt(Headline_disp_buf, 105, 16);                       
            break;
        case en_hour_set:
            sprintf(Headline_disp_buf, "%02X", new_time[HOUR_POS]); 
            GUI_DispRevStringAt(Headline_disp_buf, 25, 32);            
            break;
        case en_min_set:
            sprintf(Headline_disp_buf, "%02X", new_time[MIN_POS]); 
            GUI_DispRevStringAt(Headline_disp_buf, 65, 32);            
            break;
        case en_sec_set:
            sprintf(Headline_disp_buf, "%02X", new_time[SEC_POS]); 
            GUI_DispRevStringAt(Headline_disp_buf, 105, 32);   
            break;
        case en_ok_set:
            sprintf(Headline_disp_buf, "保存"); 
            GUI_DispRevStringAt(Headline_disp_buf, 48, 48);
            break;
        }

        
        break;
    }

    return FORM_MSG_NON;
}

void LCD_Time_Refreash()
{   
    INT8U Headline_disp_buf[18];

#if 1    
    INT32U i, verify_voltage = 0, n_ac_rms = 0;

    
    for(i = 0; i < 32; i++)
    {
        n_ac_rms += ADC_get_ac_rms();
    }

    n_ac_rms >>= 5;
    
    verify_voltage = (g_sys_conf.voltageFixCoe * n_ac_rms) / 1000 / 100;    

    sprintf(Headline_disp_buf, "   %03dV %02x:%02x:%02x", verify_voltage, g_rtc_time[2], g_rtc_time[1], g_rtc_time[0]);
#endif

#if 0
    sprintf(Headline_disp_buf, "   %04d %02x:%02x:%02x", ac_rms, g_rtc_time[2], g_rtc_time[1], g_rtc_time[0]);
#endif

#if 0
    sprintf(Headline_disp_buf, "        %02x:%02x:%02x", g_rtc_time[2], g_rtc_time[1], g_rtc_time[0]);
#endif

#if (SELF_POWER_EN > 0u)
    if(g_sys_conf.SysSwitch & SYS_SELF_POWER_MASK)
    {
        Headline_disp_buf[2] = 'Z';
    }
#endif

#if 0
    if(g_sys_conf.SysSwitch & SYS_WARNING_MASK)
    {
        Headline_disp_buf[2] = 'W';
    }
#endif

#if 1
    if(g_sys_conf.SysSwitch & SYS_DROP_KEEP_MASK)
    {
        Headline_disp_buf[0] = 'H';
    }
#else
    if(g_sys_conf.SysSwitch & SYS_DROP_KEEP_MASK)
    {
        sprintf(Headline_disp_buf, "开 %04d %02X:%02X:%02X", g_dbg_disp, g_rtc_time[2],g_rtc_time[1],g_rtc_time[0]);
    }
    else
    {
        sprintf(Headline_disp_buf, "关 %04d %02X:%02X:%02X", g_dbg_disp, g_rtc_time[2],g_rtc_time[1],g_rtc_time[0]);
    }
#endif

    Headline_disp_buf[16] = 0;
    
    GUI_DispStringAt(Headline_disp_buf, 0, 0);
}

void GUI_Sec_Refresh()
{
    
    
    if(form_id ==  FORM_ID_SETTING_FORM)
    {
        return;
    }
    
    if(form_list[form_id].event_proc != 0)
    {
        (*form_list[form_id].event_proc)(0, FORM_MSG_RERRESH);
   
    }
}

void GUI_Key_Proc(unsigned int k)
{
    int rmsg;
    if(form_id >=  MAX_FORM_NUM)
    {
        form_id = FORM_ID_MAIN_FORM;
    }
    
    if(form_list[form_id].event_proc != 0)
    {
        if(FORM_MSG_NON != (rmsg  = (*form_list[form_id].event_proc)(k, FORM_MSG_KEY)))
        {
            g_pwd_flag = FALSE;
            
            g_pwd_sn = enum_pwd_0;
            memset(g_enter_pwd, 0, sizeof(g_enter_pwd));
            memset(g_new_pwd, 0, sizeof(g_new_pwd));
            
            (*form_list[form_id].event_proc)(k, rmsg);
        }
    }
    
}




