#ifndef _MODBUS_H
#define _MODBUS_H

#define		MODBUS_NO_ERR				0
#define		MODBUS_FUNC_ERR				1
#define		MODBUS_ADDR_ERR				2
#define		MODBUS_VALUE_ERR			3
#define		MODBUS_OP_ERR				4


// OP TYPE
#define		MODBUS_NORMAL				0x00		//正常操作
#define		MODBUS_EXTERN				0x01		//特殊寄存器操作
#define		MODBUS_BROAD_TIME			0x02		//广播对时操作
#define		MODBUS_BROAD_CONF			0x03		//广播配置操作

// REG
#define		MODBUS_CTRL_START_ADDR		10000		//遥控寄存器地址
#define		MODBUS_CTRL_END_ADDR		20000		//遥控寄存器地址 

#define		MODBUS_FAST_START_ADDR		30000		//快速寄存器地址
#define		MODBUS_FAST_END_ADDR		40000		//快速寄存器地址

#define		MODBUS_GRP_START_ADDR		40000		//遥测和定值寄存器地址
#define		MODBUS_GRP_END_ADDR			60000		//遥测和定值寄存器地址 
#define		MODBUS_TIME_START_ADDR		60000		//时间寄存器地址
#define		MODBUS_TIME_END_ADDR		60500		//时间寄存器地址

#define		MODBUS_CONF_START_ADDR		61000		//配置寄存器地址
#define		MODBUS_CONF_END_ADDR		65000		//配置寄存器地址

// FUNC
#define		MODBUS_CTRL_OUTPUT			0x05		//设置继电器输出
#define		MODBUS_READ_REG				0x03		//读取单个或多个寄存器
#define		MODBUS_WRITE_REG			0x10		//写入单个或者多个寄存器
#define		MODBUS_EXT_READ_REG			0x46		//读取秘密寄存器
#define		MODBUS_EXT_WRITE_REG		0x47		//写入秘密寄存器

#define		MODBUS_OFFSET				1
#define		MODBUS_ERR_OFFSET			1
#define		MODBUS_CTRL_OFFSET			2
#define		MODBUS_READ_REQ_OFFSET		4
#define		MODBUS_READ_RESP_OFFSET		1
#define		MODBUS_WRITE_REQ_OFFSET		5
#define		MODBUS_WRITE_RESP_OFFSET	4



#define		MODBUS_RTU_OFFSET	1		//数据域起始地址


#pragma pack(1)

typedef		struct _MODBUS_RTU_HEADER
{
	uint8	address;					//设备地址0-247
} MODBUS_RTU_HEADER;

typedef		struct _MODBUS_HEADER
{
	uint8	func_code;					//功能码1-127
} MODBUS_HEADER;

typedef		struct _MODBUS_ERR_HEADER
{
	uint8	error_code;					//异常码1-4
} MODBUS_ERR_HEADER;

typedef		struct _MODBUS_CTRL_HEADER
{
	uint16	reg_addr;					//寄存器地址
} MODBUS_CTRL_HEADER;

typedef		struct _MODBUS_READ_REQ_HEADER
{
	uint16	reg_addr;					//寄存器地址
	uint16	reg_num;					//寄存器数值
} MODBUS_READ_REQ_HEADER;

typedef		struct _MODBUS_READ_RESP_HEADER
{
	uint8	byte_num;					//字节数
} MODBUS_READ_RESP_HEADER;

typedef		struct _MODBUS_WRITE_REQ_HEADER
{
	uint16	reg_addr;					//寄存器地址
	uint16	reg_num;					//寄存器数值
	uint8	byte_num;					//字节数
} MODBUS_WRITE_REQ_HEADER;

typedef		struct _MODBUS_WRITE_RESP_HEADER
{
	uint16	reg_addr;					//寄存器地址
	uint16	reg_num;					//寄存器数值
} MODBUS_WRITE_RESP_HEADER;

#pragma pack()

int32 modbus_rtu_process(P_MSG_INFO   pMsg);

#endif

