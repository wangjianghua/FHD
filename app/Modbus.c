#include "includes.h"


/******************************************************************************
* 函 数 名： modbus_check
* 描    述： MODBUS地址和数值范围判断
* 输    入： 无
* 输    出： 无
* 返 回 值： 无
* 全局变量： 无
* 调用模块： 无
* 作    者： gooseli@163.com
* 创建日期： 2008.4.25
* 其    他：
* 历    史：
******************************************************************************/
int32 modbus_ctrl_operate(int32 reg_addr, const uint8 req[], uint8 resp[], int32 *perror_code)
{
	uint16	reg_val;
	int32	resp_num;
	
	reg_val = (req[0] << 8) | req[1];
	
	//执行操作,十六进制值0XFF00请求线圈为ON。十六进制值0X0000请求线圈为OFF
	if(reg_val == 0xFF00 || reg_val == 0x0000 || (reg_val == 0x5500 || reg_val == 0xAA00))
	{
		//遥控
		if(reg_addr >= MODBUS_CTRL_START_ADDR && reg_addr < MODBUS_CTRL_END_ADDR)
		{
			reg_addr -= MODBUS_CTRL_START_ADDR;
			*perror_code = register_ctrl_operation(reg_addr, reg_val);
		}
		else
		{
			*perror_code = MODBUS_ADDR_ERR;
		}
	}
	else
	{
		*perror_code = MODBUS_VALUE_ERR;
	}
	
	//判断操作是否失败
	if(*perror_code == MODBUS_NO_ERR)
	{
		resp[0] = req[0];
		resp[1] = req[1];
		
		resp_num = 2;
	}
	else
	{
		resp_num = 0;
	}
	
	return resp_num;
}


int32 modbus_read_operate(int32 reg_addr, int32 reg_num, uint8 resp[], int32 *perror_code)
{
	int32	resp_num;
	
	if(reg_num >= 0x01 && reg_num <= 0x7D)
	{
		//快速
		if(reg_addr >= MODBUS_FAST_START_ADDR && reg_addr + reg_num < MODBUS_FAST_END_ADDR)	
		{
			reg_addr -= MODBUS_FAST_START_ADDR;
			*perror_code = register_fast_read(reg_addr, reg_num, resp);
		}
		//遥测+定值
		else if(reg_addr >= MODBUS_GRP_START_ADDR && reg_addr + reg_num < MODBUS_GRP_END_ADDR)	
		{
			reg_addr -= MODBUS_GRP_START_ADDR;
			*perror_code = register_group_read(reg_addr, reg_num, resp);
		}		
		//对时
		else if(reg_addr >= MODBUS_TIME_START_ADDR && reg_addr + reg_num < MODBUS_TIME_END_ADDR)	
		{
			reg_addr -= MODBUS_TIME_START_ADDR;
			*perror_code = register_time_read(reg_addr, reg_num, resp);
		}		
		else
		{
			*perror_code = MODBUS_ADDR_ERR;
		}
	}
	else
	{
		*perror_code = MODBUS_VALUE_ERR;			
	}
	
	//判断操作是否失败
	if(*perror_code == MODBUS_NO_ERR)
	{		
		resp_num = reg_num * 2;
	}
	else
	{
		resp_num = 0;
	}
	
	return resp_num;
}


int32 modbus_extern_read_operate(int32 reg_addr, int32 reg_num, uint8 resp[], int32 *perror_code)
{
	int32	resp_num;
	
	if(reg_num >= 0x01 && reg_num <= 0x7D)
	{
		if(reg_addr >= MODBUS_CONF_START_ADDR && reg_addr + reg_num < MODBUS_CONF_END_ADDR)	
		{
			reg_addr -= MODBUS_CONF_START_ADDR;
			*perror_code = register_conf_read(reg_addr, reg_num, resp);
		}
		else
		{
			*perror_code = MODBUS_ADDR_ERR;
		}
	}
	else
	{
		*perror_code = MODBUS_VALUE_ERR;			
	}
	
	//判断操作是否失败
	if(*perror_code == MODBUS_NO_ERR)
	{		
		resp_num = reg_num * 2;
	}
	else
	{
		resp_num = 0;
	}
	
	return resp_num;
}

int32 modbus_write_operate(int32 reg_addr, int32 reg_num, int32 byte_num, const uint8 req[], int32 *perror_code)
{
	int32	is_success;
	
	if(reg_num >= 0x01 && reg_num <= 0x7B && byte_num == reg_num * 2)
	{		
		//定值
		if(reg_addr >= MODBUS_GRP_START_ADDR && reg_addr + reg_num < MODBUS_GRP_END_ADDR)	
		{
			reg_addr -= MODBUS_GRP_START_ADDR;
			*perror_code = register_group_write(reg_addr, reg_num, req);
		}
		//对时
		else if(reg_addr >= MODBUS_TIME_START_ADDR && reg_addr + reg_num < MODBUS_TIME_END_ADDR)	
		{
			reg_addr -= MODBUS_TIME_START_ADDR;
			*perror_code = register_time_write(reg_addr, reg_num, req);
		}
		else
		{
			*perror_code = MODBUS_ADDR_ERR;
		}
	}
	else
	{
		*perror_code = MODBUS_VALUE_ERR;			
	}
	
	//判断操作是否失败
	if(*perror_code == MODBUS_NO_ERR)
	{		
		is_success = TRUE;
	}
	else
	{
		is_success = FALSE;
	}
	
	return is_success;
}

int32 modbus_extern_write_operate(int32 reg_addr, int32 reg_num, int32 byte_num, const uint8 req[], int32 *perror_code)
{
	int32	is_success;
	
	if(reg_num >= 0x01 && reg_num <= 0x7B && byte_num == reg_num * 2)
	{		
		//秘密定值
		if(reg_addr >= MODBUS_CONF_START_ADDR && reg_addr + reg_num < MODBUS_CONF_END_ADDR)	
		{
			reg_addr -= MODBUS_CONF_START_ADDR;
			*perror_code = register_conf_write(reg_addr, reg_num, req);
		}
		else
		{
			*perror_code = MODBUS_ADDR_ERR;
		}
	}
	else
	{
		*perror_code = MODBUS_VALUE_ERR;			
	}
	
	//判断操作是否失败
	if(*perror_code == MODBUS_NO_ERR)
	{		
		is_success = TRUE;
	}
	else
	{
		is_success = FALSE;
	}
	
	return is_success;
}

int32 modbus_broad_time_write_operate(int32 reg_addr, int32 reg_num, int32 byte_num, const uint8 req[], int32 *perror_code)
{
	int32	is_success;
	
	if(reg_num >= 0x01 && reg_num <= 0x7B && byte_num == reg_num * 2)
	{		
		//广播对时
		if(reg_addr >= MODBUS_TIME_START_ADDR && reg_addr + reg_num < MODBUS_TIME_END_ADDR)	
		{
			reg_addr -= MODBUS_TIME_START_ADDR;
			*perror_code = register_time_write(reg_addr, reg_num, req);
		}
		else
		{
			*perror_code = MODBUS_ADDR_ERR;
		}
	}
	else
	{
		*perror_code = MODBUS_VALUE_ERR;			
	}
	
	//判断操作是否失败
	if(*perror_code == MODBUS_NO_ERR)
	{		
		is_success = TRUE;
	}
	else
	{
		is_success = FALSE;
	}
	
	return is_success;
}

int32 modbus_broad_conf_write_operate(int32 reg_addr, int32 reg_num, int32 byte_num, const uint8 req[], int32 *perror_code)
{
	int32	is_success;
	
	if(reg_num >= 0x01 && reg_num <= 0x7B && byte_num == reg_num * 2)
	{		
		//广播配置
		if(reg_addr >= MODBUS_CONF_START_ADDR && reg_addr + reg_num < MODBUS_CONF_END_ADDR)	
		{
			reg_addr -= MODBUS_CONF_START_ADDR;
			*perror_code = register_conf_write(reg_addr, reg_num, req);
		}
		else
		{
			*perror_code = MODBUS_ADDR_ERR;
		}
	}
	else
	{
		*perror_code = MODBUS_VALUE_ERR;			
	}
	
	//判断操作是否失败
	if(*perror_code == MODBUS_NO_ERR)
	{		
		is_success = TRUE;
	}
	else
	{
		is_success = FALSE;
	}
	
	return is_success;
}

/******************************************************************************
* 函 数 名： modbus_process
* 描    述： MODBUS协议解析
* 输    入： data，PDU，先高字节后低字节
*            data_len，数据域长度
* 输    出： 无
* 返 回 值： 返回回应数据包长度，1表示异常响应，即异常代码
* 全局变量： 无
* 调用模块： 无
* 作    者： gooseli@163.com
* 创建日期： 2008.4.23
* 其    他：
* 历    史：
******************************************************************************/
int32 modbus_error_process(uint8 send_buf[], int32 error_code)
{
	MODBUS_ERR_HEADER	*perror_header;
	int32	send_len;
	
	perror_header = (MODBUS_ERR_HEADER *)(send_buf);
	
	//异常响应只有异常代码没有数据
	perror_header->error_code = error_code;
	send_len = MODBUS_ERR_OFFSET;
	
	return send_len;
}

int32 modbus_ctrl_process(uint8 recv_buf[], int32 recv_len, uint8 send_buf[], int32 *perror_code)
{
	MODBUS_CTRL_HEADER		*preq_header;	
	MODBUS_CTRL_HEADER		*presp_header;	
	int32	addr;
	
	uint8	*preq_value;
	uint8	*presp_value;
	int32	req_value_num;
	int32	resp_value_num;
	int32	send_data_len;
	
	preq_header = (MODBUS_CTRL_HEADER *)(recv_buf);
	presp_header = (MODBUS_CTRL_HEADER *)(send_buf);
	preq_value = (uint8 *)(recv_buf + MODBUS_CTRL_OFFSET);
	presp_value = (uint8 *)(send_buf + MODBUS_CTRL_OFFSET);
	req_value_num = recv_len - MODBUS_CTRL_OFFSET;
	resp_value_num = 0;
	send_data_len = 0;
	
	//调整16进制数据的字节顺序
	addr = mb_swap(preq_header->reg_addr);
	
	//总长度等于8，PDU长度为5，数据长度为2
	if(req_value_num == 2)
	{
		resp_value_num = modbus_ctrl_operate(addr, preq_value, presp_value, perror_code);
		if(resp_value_num == 2)
		{			
			presp_header->reg_addr = mb_swap(addr);				
			send_data_len = resp_value_num + MODBUS_CTRL_OFFSET;
		}
	}
	else
	{				
		*perror_code = MODBUS_VALUE_ERR;
	}
	
	return send_data_len;
}

int32 modbus_read_process(uint8 recv_buf[], int32 recv_len, uint8 send_buf[], int32 type, int32 *perror_code)
{
	MODBUS_READ_REQ_HEADER	*preq_header;	
	MODBUS_READ_RESP_HEADER	*presp_header;	
	int32	addr;
	int32	num;
	
	uint8	*presp_value;
	int32	req_value_num;
	int32	resp_value_num;
	int32	send_data_len;
	
	preq_header = (MODBUS_READ_REQ_HEADER *)(recv_buf);
	presp_header = (MODBUS_READ_RESP_HEADER *)(send_buf);
	presp_value = (uint8 *)(send_buf + MODBUS_READ_RESP_OFFSET);
	req_value_num = recv_len - MODBUS_READ_REQ_OFFSET;
	resp_value_num = 0;
	send_data_len = 0;
	
	//调整16进制数据的字节顺序
	addr = mb_swap(preq_header->reg_addr);
	num = mb_swap(preq_header->reg_num);
	
	//总长度等于8，PDU长度为5，数据长度为0
	if(req_value_num == 0)
	{
		if(type == MODBUS_NORMAL)
		{
			resp_value_num = modbus_read_operate(addr, num, presp_value, perror_code);
			if(resp_value_num > 0)
			{			
				presp_header->byte_num = resp_value_num;				
				send_data_len = resp_value_num + MODBUS_READ_RESP_OFFSET;
			}
		}
		else if(type == MODBUS_EXTERN)
		{
			resp_value_num = modbus_extern_read_operate(addr, num, presp_value, perror_code);
			if(resp_value_num > 0)
			{			
				presp_header->byte_num = resp_value_num;				
				send_data_len = resp_value_num + MODBUS_READ_RESP_OFFSET;
			}
		}
	}
	else
	{				
		*perror_code = MODBUS_VALUE_ERR;
	}
	
	return send_data_len;
}

int32 modbus_write_process(uint8 recv_buf[], int32 recv_len, uint8 send_buf[], int32 type, int32 *perror_code)
{
	MODBUS_WRITE_REQ_HEADER		*preq_header;	
	MODBUS_WRITE_RESP_HEADER	*presp_header;	
	int32	is_success;
	int32	addr;
	int32	num;
	int32	nb;
	
	uint8	*preq_value;
	int32	req_value_num;
	int32	resp_value_num;
	int32	send_data_len;
	
	preq_header = (MODBUS_WRITE_REQ_HEADER *)(recv_buf);
	presp_header = (MODBUS_WRITE_RESP_HEADER *)(send_buf);
	preq_value = (uint8 *)(recv_buf + MODBUS_WRITE_REQ_OFFSET);
	req_value_num = recv_len - MODBUS_WRITE_REQ_OFFSET;
	resp_value_num = 0;
	send_data_len = 0;
	
	//调整16进制数据的字节顺序
	addr = mb_swap(preq_header->reg_addr);
	num = mb_swap(preq_header->reg_num);
	nb = preq_header->byte_num;
	
	//总长度等于8，PDU长度为5，数据长度为字节数
	if(req_value_num == nb)
	{
		if(type == MODBUS_NORMAL)
		{
			is_success = modbus_write_operate(addr, num, nb, preq_value, perror_code);
			if(is_success == TRUE)
			{			
				presp_header->reg_addr = mb_swap(addr);	
				presp_header->reg_num = mb_swap(num);
				send_data_len = resp_value_num + MODBUS_WRITE_RESP_OFFSET;
			}
		}
		else if(type == MODBUS_EXTERN)
		{
			is_success = modbus_extern_write_operate(addr, num, nb, preq_value, perror_code);
			if(is_success == TRUE)
			{			
				presp_header->reg_addr = mb_swap(addr);	
				presp_header->reg_num = mb_swap(num);
				send_data_len = resp_value_num + MODBUS_WRITE_RESP_OFFSET;
			}
		}
		else if(type == MODBUS_BROAD_TIME)
		{
			is_success = modbus_broad_time_write_operate(addr, num, nb, preq_value, perror_code);
			if(is_success == TRUE)
			{
				send_data_len = 0;
			}
		}
		else if(type == MODBUS_BROAD_CONF)
		{
			is_success = modbus_broad_conf_write_operate(addr, num, nb, preq_value, perror_code);
			if(is_success == TRUE)
			{
				send_data_len = 0;
			}
		}
	}
	else
	{				
		*perror_code = MODBUS_VALUE_ERR;
	}
	
	return send_data_len;
}

int32 modbus_process(uint8 recv_buf[], int32 recv_len, uint8 send_buf[], int32 is_broadcast)
{
	MODBUS_HEADER	*precv_header;
	MODBUS_HEADER	*psend_header;
	int32	error_code;
	
	uint8	*precv_data;
	uint8	*psend_data;
	int32	recv_data_len;
	int32	send_data_len;
	int32	send_len;
	
	error_code = MODBUS_NO_ERR;
	
	precv_header = (MODBUS_HEADER *)(recv_buf);
	psend_header = (MODBUS_HEADER *)(send_buf);
	precv_data = (uint8 *)(recv_buf + MODBUS_OFFSET);
	psend_data = (uint8 *)(send_buf + MODBUS_OFFSET);
	recv_data_len = recv_len - MODBUS_OFFSET;
	send_data_len = 0;
	send_len = 0;
		
	// 如果接收数据长度为0不返回任何响应
	if(recv_data_len > 0)
	{
		// 解析非广播命令
		if(is_broadcast == FALSE)
		{
			// 按照功能码解析命令
			switch(precv_header->func_code)
			{
				case MODBUS_CTRL_OUTPUT:	
					send_data_len = modbus_ctrl_process(precv_data, recv_data_len, psend_data, &error_code);
					break;
				case MODBUS_READ_REG:	
					send_data_len = modbus_read_process(precv_data, recv_data_len, psend_data, MODBUS_NORMAL, &error_code);
					break;
				case MODBUS_WRITE_REG:		
					send_data_len = modbus_write_process(precv_data, recv_data_len, psend_data, MODBUS_NORMAL, &error_code);
					break;
				case MODBUS_EXT_READ_REG:
					send_data_len = modbus_read_process(precv_data, recv_data_len, psend_data, MODBUS_EXTERN, &error_code);
					break;
				case MODBUS_EXT_WRITE_REG:
					send_data_len = modbus_write_process(precv_data, recv_data_len, psend_data, MODBUS_EXTERN, &error_code);
					break;
				default:
					error_code = MODBUS_FUNC_ERR;
					break;
			}
			
			//如果MODBUS解析失败，返回一个字节的错误代码，功能码最高位设为1	
			if(error_code != MODBUS_NO_ERR)
			{
				send_data_len = modbus_error_process(psend_data, error_code);
			}
			
			//如果需要返回数据，则组装数据并增加功能码
			if(send_data_len > 0)
			{
				psend_header->func_code = precv_header->func_code;
				if(error_code != MODBUS_NO_ERR)
				{
					psend_header->func_code |= 0x80;	
				}
				
				send_len = send_data_len + MODBUS_OFFSET;
			}
		}
		//解析广播命令
		else if(is_broadcast == TRUE)
		{
			// 只有对时支持广播
			switch(precv_header->func_code)
			{
				case MODBUS_WRITE_REG:		
					send_data_len = modbus_write_process(precv_data, recv_data_len, psend_data, MODBUS_BROAD_TIME, &error_code);
					break;
				case MODBUS_EXT_WRITE_REG:		
					send_data_len = modbus_write_process(precv_data, recv_data_len, psend_data, MODBUS_BROAD_CONF, &error_code);
					break;
				default:
					error_code = MODBUS_FUNC_ERR;
					break;
			}
			
			//广播数据不返回数据
			send_len = 0;
		}
	}		
	
	return send_len;
}

/******************************************************************************
* 函 数 名： protocol_modbus_rtu
* 描    述： 通讯处理函数，根据协议类型调用不同的协议
* 输    入： 无
* 输    出： 无
* 返 回 值： 无
* 全局变量： 无
* 调用模块： 无
* 作    者： gooseli@163.com
* 创建日期： 2008.4.23
* 其    他：
* 历    史：
******************************************************************************/
int32 modbus_rtu_process(P_MSG_INFO   pMsg)
{
	MODBUS_RTU_HEADER	*prtu_recv_header;
	MODBUS_RTU_HEADER	*prtu_send_header;
    P_MSG_INFO p_SendMsg = NULL;
	uint16	cacl_checksum;
    
    int32 dev_addr; 
    uint8 *recv_buf;
    int32 recv_len;
	
	uint8	*precv_data;
	uint8	*psend_data;
	int32	recv_data_len;
	int32	send_data_len;
	int32	send_len;

    if(pMsg == NULL)
        return 0;

    if( !(p_SendMsg = (P_MSG_INFO)alloc_send_buffer(MSG_SHORT)) )
    {
        return 0;
    }

    dev_addr = g_sys_conf.dev_addr[0];
    recv_buf = &pMsg->msg_buffer[0];

    //if(pMsg->msg_header.msg_len > 256)
    {
    //    dev_addr = 1;
    }
        
    recv_len = pMsg->msg_header.msg_len;
    
	
	prtu_recv_header = (MODBUS_RTU_HEADER *)(recv_buf);
	prtu_send_header = (MODBUS_RTU_HEADER *)(p_SendMsg->msg_buffer);
	precv_data = (uint8 *)(recv_buf + MODBUS_RTU_OFFSET);
	psend_data = (uint8 *)(p_SendMsg->msg_buffer + MODBUS_RTU_OFFSET);
	recv_data_len = 0;
	send_data_len = 0;
	send_len = 0;

    
		
	///////////////////////////////////////////////////////////////////
	// 解析数据包,首先进行CRC16校验; 数据包正确，返回长度大于0
	////////////////////////////////////////////////////////////////////
	// length must bigger than sizeof(MODBUS_RTU_HEADER)+crcLen=1+2
	if(recv_len > 3)
	{		
	    if(recv_len > UART_RECV_BUF_SIZE)
	    {
	        free_send_buffer(p_SendMsg);
	        return 0;
	    }
        
		if(prtu_recv_header->address == dev_addr || prtu_recv_header->address == 0)
		{
			cacl_checksum = mb_crc16(recv_buf, recv_len);
			if(cacl_checksum == 0)
			{			
				recv_data_len = recv_len - 3;
			}
		}
	}
	
	//////////////////////////////////////////////////////////////////
	// 按照MODBUS协议解析数据报, MODBUS将处理后的响应写入缓冲区
	//////////////////////////////////////////////////////////////////
	if(recv_data_len > 0)
	{
		//protocol_set_cnt();
				
		if(prtu_recv_header->address != 0)
		{
			//非广播命令处理方法
			send_data_len = modbus_process(precv_data, recv_data_len, psend_data, FALSE);
		}
		else
		{
			//广播命令只有部分寄存器支持,且不返回结果,故单独进行处理
			send_data_len = modbus_process(precv_data, recv_data_len, psend_data, TRUE);
		}
	}
	/////////////////////////////////////////////////////////////////////
	// 异常响应特殊处理，并计算CRC
	////////////////////////////////////////////////////////////////////
	if(send_data_len > 0)
	{		
		prtu_send_header->address = dev_addr;
		//计算CRC16，包括首部一个字节；返回长度也要包含两字节CRC值,先低后高
		send_len = send_data_len + 3;
		cacl_checksum = mb_crc16(p_SendMsg->msg_buffer, send_len - 2);
		p_SendMsg->msg_buffer[send_len - 2] = cacl_checksum & 0xFF;
		p_SendMsg->msg_buffer[send_len - 1] = (cacl_checksum >> 8) & 0xFF;

        p_SendMsg->msg_header.msg_len = send_len;/* */

        p_SendMsg->msg_header.end_id = RS485_COM_PORT;

        p_SendMsg->msg_header.need_buffer_free = OK;/* FALSE 标识end 负责buffer 释放*/

        End_send(p_SendMsg);
    
	}
    else
    {
        free_send_buffer(p_SendMsg);
    }
	
	return send_len;
}

/******************************************************************************
* 结束
******************************************************************************/


