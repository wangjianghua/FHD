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


#define REGISTER_DVC

int32 register_ctrl_operation(int32 reg_addr, int32 reg_val)
{
	int32	error_code = MODBUS_NO_ERR;

#if	defined(REGISTER_DVC)	
	error_code =  reg_gpm650_ctrl_operation(reg_addr, reg_val);
#endif	
	
	return error_code;
}

int32 register_fast_read(int32 reg_addr, int32 reg_num, uint8 resp[])
{
	int32	error_code = MODBUS_NO_ERR;

#if	defined(REGISTER_DVC)	
	error_code =  reg_gpm650_fast_read(reg_addr, reg_num, resp);
#endif
	
	return error_code;
}

int32 register_group_read(int32 reg_addr, int32 reg_num, uint8 resp[])
{
	int32	error_code = MODBUS_NO_ERR;

#if	defined(REGISTER_DVC)	
	error_code =  reg_gpm650_grp_read(reg_addr, reg_num, resp);
#endif
	
	return error_code;
}

int32 register_group_write(int32 reg_addr, int32 reg_num, const uint8 req[])
{
	int32	error_code = MODBUS_NO_ERR;
	
#if	defined(REGISTER_DVC)	
	error_code =  reg_gpm650_grp_write(reg_addr, reg_num, req);
#endif
	
	return error_code;
}


int32 register_time_read(int32 reg_addr, int32 reg_num, uint8 resp[])
{
	int32	error_code = MODBUS_NO_ERR;

#if	defined(REGISTER_DVC)	
	error_code =  reg_gpm650_time_read(reg_addr, reg_num, resp);
#endif

	return error_code;
}

int32 register_time_write(int32 reg_addr, int32 reg_num, const uint8 req[])
{
	int32	error_code = MODBUS_NO_ERR;

#if	defined(REGISTER_DVC)	
	error_code =  reg_gpm650_time_write(reg_addr, reg_num, req);
#endif

	return error_code;
}


int32 register_extern_group_read(int32 reg_addr, int32 reg_num, uint8 resp[])
{
	int32	error_code = MODBUS_NO_ERR;

#if	defined(REGISTER_DVC)	
	error_code =  reg_gpm650_ext_grp_read(reg_addr, reg_num, resp);
#endif
	
	return error_code;
}

int32 register_extern_group_write(int32 reg_addr, int32 reg_num, const uint8 req[])
{
	int32	error_code = MODBUS_NO_ERR;
	
#if	defined(REGISTER_DVC)	
	error_code =  reg_gpm650_ext_grp_write(reg_addr, reg_num, req);
#endif
	
	return error_code;
}


int32 register_conf_read(int32 reg_addr, int32 reg_num, uint8 resp[])
{
	int32	error_code = MODBUS_NO_ERR;

#if	defined(REGISTER_DVC)	
	error_code =  reg_gpm650_conf_read(reg_addr, reg_num, resp);
#endif

	return error_code;
}

int32 register_conf_write(int32 reg_addr, int32 reg_num, const uint8 req[])
{
	int32	error_code = MODBUS_NO_ERR;

#if	defined(REGISTER_DVC)	
	error_code =  reg_gpm650_conf_write(reg_addr, reg_num, req);
#endif

	return error_code;
}


/******************************************************************************
* 结束
******************************************************************************/


