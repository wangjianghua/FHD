/******************************************************************************
* 版权所有： 深圳普元电力技术有限公司
* 文 件 名： register.c
* 版    本： 1.0
* 作    者： gooseli@163.com
* 创建日期： 2008.4.23
* 描    述： 通讯接口函数
* 其    他：
* 历    史：
******************************************************************************/

#include	"includes.h"

#define	REG_GRP_SIZE	48



static
RANGE	grp_val[REG_GRP_SIZE] = { 0 };
extern INT32U jdq_timer;
unsigned short msg_return_len;

/******************************************************************************
* 函 数 名： rtt_getday
* 描    述： 获取星期，Zeller:w=y+[y/4]+[c/4]-2c+[26(m+1)/10]+d-1
* 输    入： y，年份；m，月份
* 输    出： 无
* 返 回 值： 最大天数
* 全局变量： 无
* 调用模块： 无
* 作    者： gooseli@163.com
* 创建日期： 2008.11.13
* 其    他：
* 历    史：
******************************************************************************/
int32 get_week(int32 year, int32 month, int32 day)
{
	int32	century;
	int32	week;

	// 1..12 -> 13,14,1..10
    if (month <= 2)
    {
		month += 12;
		year -= 1;
    }
	
	century = year / 100;
	year = year % 100;
	week = century / 4 - 2 * century + year + year / 4 + 26 * (month + 1) / 10 + day - 1;
	week = week % 7;
	if(week < 0)
	{ 
		week += 7;
	}
	
	return week;
}


int32 is_leap(int32 y)
{
	// 能被4整除但不能被被100整除；能被四百整除
	return ( ( ((y)%4)==0 && ((y)%100)!=0 ) || ((y)%400)==0 );
}


/******************************************************************************
* 函 数 名： rtt_getday
* 描    述： 获取最大天数
* 输    入： y，年份；m，月份
* 输    出： 无
* 返 回 值： 最大天数
* 全局变量： 无
* 调用模块： 无
* 作    者： gooseli@163.com
* 创建日期： 2008.11.13
* 其    他：
* 历    史：
******************************************************************************/
int32 get_day(int32 year, int32 month)
{
	int32	day = 0;
	
	if(month == 1 || month == 3 || month == 5 || month == 7 ||
	   month == 8 || month == 10 || month == 12)
	{
		day = 31;
	}
	else if(month == 4 || month == 6 || month == 9 || month == 11)
	{
		day = 30;
	}
	else if(month == 2)
	{
		if(is_leap(year) == TRUE)
		{
			day = 29;
		}
		else
		{
			day = 28;
		}
	}
	
	return day;
}

void set_get_time_rang(RANGE time_val[])
{
	int32 temp;

	temp = get_day(time_val[TIME_YEAR].val, time_val[TIME_MON].val);
	time_val[TIME_DAY].min = 1;
	time_val[TIME_DAY].max = temp;

	time_val[TIME_WEEK].val = get_week(time_val[TIME_YEAR].val, time_val[TIME_MON].val, time_val[TIME_DAY].val);
}


void set_get_time(RANGE time_val[])
{
	

	time_val[TIME_YEAR].val = 2000 + Bcd2HexChar(g_rtc_time[YEAR_POS]);
	time_val[TIME_YEAR].min = 2000;
	time_val[TIME_YEAR].max = 2099;

	time_val[TIME_MON].val = Bcd2HexChar(g_rtc_time[MONTH_POS]);
	time_val[TIME_MON].min = 1;
	time_val[TIME_MON].max = 12;

	time_val[TIME_DAY].val = Bcd2HexChar(g_rtc_time[DATE_POS]);
	time_val[TIME_DAY].min = 1;
	time_val[TIME_DAY].max = 31;

	time_val[TIME_WEEK].val = Bcd2HexChar(g_rtc_time[DAY_POS]);
	time_val[TIME_WEEK].min = 0;
	time_val[TIME_WEEK].max = 255;

	time_val[TIME_HOUR].val = Bcd2HexChar(g_rtc_time[HOUR_POS]);
	time_val[TIME_HOUR].min = 0;
	time_val[TIME_HOUR].max = 23;

	time_val[TIME_MIN].val = Bcd2HexChar(g_rtc_time[MIN_POS]);
	time_val[TIME_MIN].min = 0;
	time_val[TIME_MIN].max = 59;

	time_val[TIME_SEC].val = Bcd2HexChar(g_rtc_time[SEC_POS]);
	time_val[TIME_SEC].min = 0;
	time_val[TIME_SEC].max = 59;

	time_val[TIME_MSEL].val = mseconds;
	time_val[TIME_MSEL].min = 0;
	time_val[TIME_MSEL].max = 999;

	set_get_time_rang(time_val);
}

void set_set_time(RANGE time_val[])
{
	//DATE_TIME time = { 0 };
	
    unsigned char new_time[8];
    
	new_time[YEAR_POS] = Hex2BcdChar(time_val[TIME_YEAR].val-2000);
	new_time[MONTH_POS] = Hex2BcdChar(time_val[TIME_MON].val);
	new_time[DATE_POS] = Hex2BcdChar(time_val[TIME_DAY].val);
	new_time[DAY_POS] = Hex2BcdChar(time_val[TIME_WEEK].val);
	new_time[HOUR_POS] = Hex2BcdChar(time_val[TIME_HOUR].val);
	new_time[MIN_POS] = Hex2BcdChar(time_val[TIME_MIN].val);
	new_time[SEC_POS] = Hex2BcdChar(time_val[TIME_SEC].val);
    
	mseconds = time_val[TIME_MSEL].val;
    
	RTC_WriteTime(new_time); 

	//event_log(CHG_TIME, 0);
}


int32 reg_gpm650_ctrl_operation(int32 reg_addr, int32 reg_val)
{
	int32	is_success;
	int32	error_code;
	int32	value = reg_val;
	
	is_success = FALSE;
	error_code = MODBUS_NO_ERR;

#if 1
	
	// 首先判断是否数据错误
	if((reg_addr >= 0) && ((reg_addr == 10 || reg_addr == 12)))
	{	
		// 执行操作
        
		if(reg_addr == 12)
		{			
		    is_success = TRUE;
		    if(value == 0X5500) 
            {				    
                JDQ_ON();
                jdq_timer = JDO_ON_ACT_TIME;
		    }
            else if(value == 0Xff00) 
            {				    
                JDQ_ON();
                //jdq_timer = JDO_ON_ACT_TIME;
		    }
            else if(value == 0X0) 
            {				    
                JDQ_OFF();
                //jdq_timer = JDO_ON_ACT_TIME;
		    }
            else if(value == 0XAA00) 
            {				    
                JDQ_OFF();
                //Delay(__IO uint32_t nCount)
                JDQ_ON();
                //jdq_timer = JDO_ON_ACT_TIME;
		    }


			
		}
		else if(reg_addr == 10)
		{
			if(value == 0XFF00) {				
				is_success = TRUE;
                MEM_para_init();
                E2promWriteBuffer(0, (unsigned char *)&g_sys_conf, sizeof(SYS_CONF));
			}
		}        

		// 然后判断是否操作错误
		if(is_success == FALSE)
		{
			error_code = MODBUS_OP_ERR;
		}
	}
	else
	{
		error_code = MODBUS_ADDR_ERR;
	}

#endif

    //error_code = MODBUS_ADDR_ERR;

	return error_code;
}

int32 reg_gpm650_fast_read(int32 reg_addr, int32 reg_num, uint8 resp[])
{
	uint8	*presp_buf;
	int32	resp_num;
	int32	error_code;
	int32	idx;
	int32	num;
	int32	i;

	//减去基址，得到相对地址			
	presp_buf = resp;
	resp_num = 0;
	error_code = MODBUS_NO_ERR;

#if 0
	if(reg_addr >= 0 && reg_addr + reg_num <= POWER_NUM * 2 + 2 * 4 + 14 * 2) {
		//读取浮点数测量数据，要额外判断数据地址和个数是否为偶数
		idx = (reg_addr & (~0x01));
		num = (reg_num & (~0x01));

		if(idx < POWER_NUM * 2) {
			for(i = idx / 2; resp_num < num * 2 && i < POWER_NUM; i ++)
			{
				resp_num += mb_float_to_byte(presp_buf + resp_num, power_avg[i]);
				idx += 2;
			}	
		} 		 
		if(idx < POWER_NUM * 2 + 2 * 4) {
			for(i = (idx - POWER_NUM * 2) / 2; resp_num < num * 2 && i < 2; i ++)
			{
				resp_num += mb_double_to_byte(presp_buf + resp_num, energy_val[i]);
				idx += 4;
			}
		}
		if(idx < POWER_NUM * 2 + 2 * 4 + 14 * 2) {
			for(i = (idx - POWER_NUM * 2 - 2 * 4) / 2; resp_num < num * 2 && i < 14; i ++)
			{
				if(i == 0) {
					resp_num += mb_int_to_byte(presp_buf + resp_num, di_val);
				}else if(i == 1) {														 
					resp_num += mb_int_to_byte(presp_buf + resp_num, do_val);	
				}else if(i == 2) {
					resp_num += mb_float_to_byte(presp_buf + resp_num, ai_val);
				}else if(i == 3) {
					resp_num += mb_float_to_byte(presp_buf + resp_num, ao_val[AO_1]);
				}else if(i == 4) {
					resp_num += mb_float_to_byte(presp_buf + resp_num, unbln_val[UNBLN_U]);
				}else if(i == 5) {
					resp_num += mb_float_to_byte(presp_buf + resp_num, unbln_val[UNBLN_I]);
				}else if(i == 6) {
					resp_num += mb_float_to_byte(presp_buf + resp_num, harm_val[SAMP_UA][HARM_THD]);
				}else if(i == 7) {
					resp_num += mb_float_to_byte(presp_buf + resp_num, harm_val[SAMP_UB][HARM_THD]);
				}else if(i == 8) {
					resp_num += mb_float_to_byte(presp_buf + resp_num, harm_val[SAMP_UC][HARM_THD]);
				}else if(i == 9) {
					resp_num += mb_float_to_byte(presp_buf + resp_num, harm_val[SAMP_IA][HARM_THD]);
				}else if(i == 10) {
					resp_num += mb_float_to_byte(presp_buf + resp_num, harm_val[SAMP_IB][HARM_THD]);
				}else if(i == 11) {
					resp_num += mb_float_to_byte(presp_buf + resp_num, harm_val[SAMP_IC][HARM_THD]);
				}else if(i == 12) {
					resp_num += mb_float_to_byte(presp_buf + resp_num, harm_val[SAMP_UN][HARM_THD]);
				}else if(i == 13) {
					resp_num += mb_float_to_byte(presp_buf + resp_num, harm_val[SAMP_IN][HARM_THD]);
				}else if(i == 14) {
					resp_num += mb_float_to_byte(presp_buf + resp_num, ao_val[AO_2]);
				}
                
				idx += 2;
			}
		}		

		//MODBUS寄存器以字为单位，2个字节
		if(resp_num == 0 || resp_num != num * 2) {
			error_code = MODBUS_OP_ERR;
		}
	}else {
		error_code = MODBUS_ADDR_ERR;		
	}
    
#else

    error_code = MODBUS_ADDR_ERR;	

#endif
	return error_code;
}

int32 reg_gpm650_grp_read(int32 reg_addr, int32 reg_num, uint8 resp[])
{
	
	uint8	*presp_buf;
	int32	resp_num;
	int32	error_code;
	int32	idx;
	int32	num;
	int32	i;
	
	//减去基址，得到相对地址			
	presp_buf = resp;
	resp_num = 0;
	error_code = MODBUS_NO_ERR;
	
	
    error_code = MODBUS_ADDR_ERR;		

	
	return error_code;
}


int32 reg_gpm650_grp_write(int32 reg_addr, int32 reg_num, const uint8 req[])
{
	uint8	*preq_buf;
	int32	req_num;
	int32	error_code;
	int32	idx;
	int32	num;
	int32	i;

	//flag_set_lock(LOCK_PROTO);
	
	//减去基址，得到相对地址			
	preq_buf = (uint8 *)req;
	req_num = 0;
	error_code = MODBUS_NO_ERR;
	
	error_code = MODBUS_ADDR_ERR;		

	
	return error_code;
}


int32 reg_gpm650_time_read(int32 reg_addr, int32 reg_num, uint8 resp[])
{
	uint8	*presp_buf;
	int32	resp_num;
	int32	error_code;
	int32	i;
	
	//减去基址，得到相对地址			
	presp_buf = resp;
	resp_num = 0;
	error_code = MODBUS_NO_ERR;
	
	//判断地址是否合法,必须连续写的寄存器数据
	if((reg_addr == MODBUS_TIME0_ADDR) && (reg_addr + reg_num == MODBUS_TIME0_ADDR + 5))
	{
		set_get_time(grp_val);	

		for(i = 0; i < TIME_SIZE; i ++) { 
			if(i == TIME_YEAR || i == TIME_MSEL) {
				resp_num += mb_short_to_byte(presp_buf + resp_num, grp_val[i].val);
			}else {
				resp_num += mb_char_to_byte(presp_buf + resp_num, grp_val[i].val);
			}
		}
		
		//MODBUS寄存器以字为单位，2个字节
		if(resp_num == 0 )
		{
			error_code = MODBUS_OP_ERR;
		}
	}
	else
	{
		error_code = MODBUS_ADDR_ERR;
	}	
	
	return error_code;
}

int32 reg_gpm650_time_write(int32 reg_addr, int32 reg_num, const uint8 req[])
{
	uint8	*preq_buf;
	int32	req_num;
	int32	error_code;
	int32	i;
	
	//减去基址，得到相对地址			
	preq_buf = (uint8 *)req;
	req_num = 0;
	error_code = MODBUS_NO_ERR;
	
	//判断地址是否合法,必须连续写的寄存器数据
	if(reg_addr == MODBUS_TIME0_ADDR && reg_addr + reg_num == MODBUS_TIME0_ADDR + 5)
	{
		set_get_time(grp_val);

		for(i = 0; i < TIME_SIZE; i ++) { 
			if(i == TIME_YEAR || i == TIME_MSEL) {
				req_num += mb_byte_to_short(preq_buf + req_num, (uint16 *)&grp_val[i].val);
			}else {
				req_num += mb_byte_to_char(preq_buf + req_num, (uint8 *)&grp_val[i].val);
			}
			if(grp_val[i].val > grp_val[i].max || grp_val[i].val < grp_val[i].min) {
				error_code = MODBUS_VALUE_ERR;
				
				break;
			}

			if(i == TIME_YEAR || i == TIME_MON) {
				set_get_time_rang(grp_val);
			}
		}

		//MODBUS寄存器以字为单位，2个字节
		if(req_num != reg_num * 2 && error_code == MODBUS_NO_ERR)
		{
			error_code = MODBUS_OP_ERR;
		}
		else if(error_code == MODBUS_NO_ERR)
		{
		    grp_val[TIME_WEEK].val = get_week(grp_val[TIME_YEAR].val, grp_val[TIME_MON].val, grp_val[TIME_DAY].val);
			set_set_time(grp_val);
		}
	}
	else
	{
		error_code = MODBUS_ADDR_ERR;
	}	
	
	return error_code;
}


int32 reg_gpm650_ext_grp_read(int32 reg_addr, int32 reg_num, uint8 resp[])
{
	uint8	*presp_buf;
	int32	resp_num;
	int32	error_code;
	int32	idx;
	int32	num;
	int32	i;
	
	//减去基址，得到相对地址			
	presp_buf = resp;
	resp_num = 0;
	error_code = MODBUS_NO_ERR;
#if 0	
	//判断地址在那个组
	if(reg_addr >= MODBUS_EXT_GRP0_ADDR && reg_addr + reg_num <= MODBUS_EXT_GRP0_ADDR + ADJ_NUM * (ADJ_SIZE - 1) * 2) {
		//读取浮点数测量数据，要额外判断数据地址和个数是否为偶数
		idx = ((reg_addr & (~0x01)) - MODBUS_EXT_GRP0_ADDR) / 2;
		num = (reg_num & (~0x01)) / 2;

		//读取校准系数
		set_get_adjs(adj_val);

		for(i = idx; resp_num < num * 4 && i < ADJ_NUM * (ADJ_SIZE - 1); i ++)
		{
			resp_num += mb_float_to_byte(presp_buf + resp_num, adj_val[i]);
		}

		//MODBUS寄存器以字为单位，2个字节
		if(resp_num == 0 || resp_num != num * 4) {
			error_code = MODBUS_OP_ERR;
		}
	}else {
		error_code = MODBUS_ADDR_ERR;		
	}
#else
    error_code = MODBUS_ADDR_ERR;		

#endif
	return error_code;
}



int32 reg_gpm650_ext_grp_write(int32 reg_addr, int32 reg_num, const uint8 req[])
{
	uint8	*preq_buf;
	int32	req_num;
	int32	error_code;
	int32	idx;
	int32	num;
	int32	i;
	int32	j;

	//flag_set_lock(LOCK_PROTO);
	
	//减去基址，得到相对地址			
	preq_buf = (uint8 *)req;
	req_num = 0;
	error_code = MODBUS_NO_ERR;
	
	error_code = MODBUS_ADDR_ERR;		

	
	return error_code;

}


int32 reg_gpm650_conf_read(int32 reg_addr, int32 reg_num, uint8 resp[])
{
	uint8	*presp_buf;
	int32	resp_num;
	int32	error_code;
	int32	idx;
	int32	num;
	int32	i,n,crc;
	uint16 * p_lognum, log_n;
	//减去基址，得到相对地址			
	presp_buf = resp;
	resp_num = 0;
	error_code = MODBUS_NO_ERR;
	
	//判断地址是否合法,必须连续写的寄存器数据
	if((reg_addr == MODBUS_CONF0_ADDR) && (reg_num == 7))
	{		
	    resp_num += mb_int_to_byte(presp_buf + resp_num, g_sys_conf.SysSwitch);
        resp_num += mb_short_to_byte(presp_buf + resp_num, g_sys_conf.PowerDropKeepTime);
#if 1 
        resp_num += mb_short_to_byte(presp_buf + resp_num, g_sys_conf.SelfPowerTime);
#else  //华兄
#if (SELF_POWER_EN > 0u)
        resp_num += mb_char_to_byte(presp_buf + resp_num, g_sys_conf.SelfPowerActionTimeHigh);
        resp_num += mb_char_to_byte(presp_buf + resp_num, g_sys_conf.SelfPowerActionTimeLow);
#endif        
#endif
        resp_num += mb_short_to_byte(presp_buf + resp_num, g_sys_conf.selfPowerLimit);	
        resp_num += mb_short_to_byte(presp_buf + resp_num, g_sys_conf.SelfPowerValidTime);	
        resp_num += mb_short_to_byte(presp_buf + resp_num, g_sys_conf.dev_addr[0]);	        		
		//MODBUS寄存器以字为单位，2个字节
		if(resp_num == 0 )
		{
			error_code = MODBUS_OP_ERR;
		}
	}
	else if((reg_addr == MODBUS_CONF1_ADDR) && (reg_num == 6))
	{	            
        
	    resp_num += mb_int_to_byte(presp_buf + resp_num, g_sys_conf.SysRunStatus);  
        g_voltage = (ac_rms * g_sys_conf.voltageFixCoe)/1000;
        resp_num += mb_int_to_byte(presp_buf + resp_num, g_voltage);
        resp_num += mb_short_to_byte(presp_buf + resp_num, g_sys_conf.PowerDropCount);
        resp_num += mb_short_to_byte(presp_buf + resp_num, g_sys_conf.SelfPowerOnCount);
	}	
    else if((reg_addr == MODBUS_CONF2_ADDR)&& (reg_num == 46) )
    {
        num = g_sys_conf.PowerDropCount % DROP_HISTORY_MAX_COUNT;
        p_lognum = (uint16*)(presp_buf + resp_num);
        log_n = 0;
        resp_num += mb_short_to_byte(presp_buf + resp_num, 0);
        
        if(g_sys_conf.PowerDropCount != 0)
        {                    
            for(i = 0; i < DROP_HISTORY_MAX_COUNT; i++)
            {                
                crc = MEM_Cal_Time_Crc((unsigned char*)&g_sys_conf.PowerDropTime[num][0]);

                if((crc&0xfe) == ((g_sys_conf.PowerDropTime[num][7])&0xfe))
                {     
                    for(n = 0; n < (SYS_SAVE_TIME_LEN-1); n++)
                    {   
                        resp_num += mb_char_to_byte(presp_buf + resp_num, g_sys_conf.PowerDropTime[num][n]);	
                        
                    }
                    log_n++;
                    resp_num += mb_short_to_byte(presp_buf + resp_num, g_sys_conf.PowerDropPeriod[num]);	
                }
                num++;
                num = num % DROP_HISTORY_MAX_COUNT;                
            }
        }

        *p_lognum = log_n;
        //else
        //    resp_num += mb_char_to_byte(presp_buf + resp_num, 0xff);	
    }
    else if((reg_addr == MODBUS_CONF3_ADDR) && (reg_num == 36))
    {
        num = g_sys_conf.PowerDropCount % DROP_HISTORY_MAX_COUNT;

        p_lognum = (uint16*)(presp_buf + resp_num);
        log_n = 0;
        resp_num += mb_short_to_byte(presp_buf + resp_num, 0);
        
        if(g_sys_conf.PowerDropCount != 0)
        {                    
            for(i = 0; i < DROP_HISTORY_MAX_COUNT; i++)
            {                
                crc = MEM_Cal_Time_Crc((unsigned char*)&g_sys_conf.SelfPowerOnTime[num][0]);

                if((crc) == ((g_sys_conf.SelfPowerOnTime[num][7])))
                {     
                    for(n = 0; n < (SYS_SAVE_TIME_LEN-1); n++)
                    {   
                        resp_num += mb_char_to_byte(presp_buf + resp_num, g_sys_conf.SelfPowerOnTime[num][n]);	
                    }
                    log_n++;
                }
                num++;
                num = num % DROP_HISTORY_MAX_COUNT;                
            }
        }

        *p_lognum = log_n;
        
        //else
        //    resp_num += mb_char_to_byte(presp_buf + resp_num, 0xff);	
    }
    else if((reg_addr == MODBUS_CONF4_ADDR) && (reg_num == 2))
    {
        	
        INT32U g_verify_voltage, n_ac_rms;        
        //req_num += mb_byte_to_float(preq_buf + req_num, &g_verify_voltage);	
        n_ac_rms = ADC_get_ac_rms();
        n_ac_rms += ADC_get_ac_rms();
        n_ac_rms += ADC_get_ac_rms();
        n_ac_rms += ADC_get_ac_rms();
        n_ac_rms >>= 2;
        g_verify_voltage = (g_sys_conf.voltageFixCoe * n_ac_rms)/1000;
        resp_num += mb_int_to_byte(presp_buf + resp_num, g_verify_voltage);
    }
    else
	{
		error_code = MODBUS_ADDR_ERR;
	}
        
	
	return error_code;
}

int32 reg_gpm650_conf_write(int32 reg_addr, int32 reg_num, const uint8 req[])
{
	uint8	*preq_buf;
	int32	req_num;
	int32	error_code;
	int32	idx;
	int32	num;
	int32	i;

	//flag_set_lock(LOCK_PROTO);
	
	//减去基址，得到相对地址			
	preq_buf = (uint8 *)req;
	req_num = 0;
	error_code = MODBUS_NO_ERR;
	
	//判断地址是否合法,必须连续写的寄存器数据
	if(reg_addr == MODBUS_CONF0_ADDR )
	{
		req_num += mb_byte_to_int(preq_buf + req_num, &g_sys_conf.SysSwitch);
        req_num += mb_byte_to_short(preq_buf + req_num, &g_sys_conf.PowerDropKeepTime);
#if 1        
        req_num += mb_byte_to_short(preq_buf + req_num, &g_sys_conf.SelfPowerTime);	
#else //华兄
#if (SELF_POWER_EN > 0u)
        req_num += mb_byte_to_char(preq_buf + req_num, &g_sys_conf.SelfPowerActionTimeHigh);
        req_num += mb_byte_to_char(preq_buf + req_num, &g_sys_conf.SelfPowerActionTimeLow);
#endif        
#endif
        req_num += mb_byte_to_short(preq_buf + req_num, &g_sys_conf.selfPowerLimit);
        req_num += mb_byte_to_short(preq_buf + req_num, &g_sys_conf.SelfPowerValidTime);
        req_num += mb_byte_to_short(preq_buf + req_num, &g_sys_conf.dev_addr[0]);	
					
		
		//MODBUS寄存器以字为单位，2个字节
		if(error_code == MODBUS_NO_ERR)
		{				
			E2promWriteBuffer(0, (unsigned char *)&g_sys_conf, OffsetOf(SYS_CONF, PowerDropCount));
		}
	} 
    else if((reg_addr == MODBUS_CONF4_ADDR) )
    {
        INT32U i, g_verify_voltage = 0, n_ac_rms = 0;

        
        req_num += mb_byte_to_float(preq_buf + req_num, &g_verify_voltage);	

        for(i = 0; i < 32; i++)
        {
            n_ac_rms += ADC_get_ac_rms();
        }

        n_ac_rms >>= 5;
        
        g_sys_conf.voltageFixCoe = (g_verify_voltage*1000)/n_ac_rms;

        E2promWriteBuffer(OffsetOf(SYS_CONF, voltageFixCoe), (unsigned char *)&g_sys_conf.voltageFixCoe, sizeof(g_sys_conf.voltageFixCoe));
        
        //resp_num += mb_short_to_byte(presp_buf + resp_num, g_sys_conf.voltageFixCoe);	
    }
	else
	{
		error_code = MODBUS_ADDR_ERR;
	}	

	//flag_set_unlock(LOCK_PROTO);
	
	return error_code;
}



/******************************************************************************
* 结束
******************************************************************************/


