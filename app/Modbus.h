#ifndef _MODBUS_H
#define _MODBUS_H

#define		MODBUS_NO_ERR				0
#define		MODBUS_FUNC_ERR				1
#define		MODBUS_ADDR_ERR				2
#define		MODBUS_VALUE_ERR			3
#define		MODBUS_OP_ERR				4


// OP TYPE
#define		MODBUS_NORMAL				0x00		//��������
#define		MODBUS_EXTERN				0x01		//����Ĵ�������
#define		MODBUS_BROAD_TIME			0x02		//�㲥��ʱ����
#define		MODBUS_BROAD_CONF			0x03		//�㲥���ò���

// REG
#define		MODBUS_CTRL_START_ADDR		10000		//ң�ؼĴ�����ַ
#define		MODBUS_CTRL_END_ADDR		20000		//ң�ؼĴ�����ַ 

#define		MODBUS_FAST_START_ADDR		30000		//���ټĴ�����ַ
#define		MODBUS_FAST_END_ADDR		40000		//���ټĴ�����ַ

#define		MODBUS_GRP_START_ADDR		40000		//ң��Ͷ�ֵ�Ĵ�����ַ
#define		MODBUS_GRP_END_ADDR			60000		//ң��Ͷ�ֵ�Ĵ�����ַ 
#define		MODBUS_TIME_START_ADDR		60000		//ʱ��Ĵ�����ַ
#define		MODBUS_TIME_END_ADDR		60500		//ʱ��Ĵ�����ַ

#define		MODBUS_CONF_START_ADDR		61000		//���üĴ�����ַ
#define		MODBUS_CONF_END_ADDR		65000		//���üĴ�����ַ

// FUNC
#define		MODBUS_CTRL_OUTPUT			0x05		//���ü̵������
#define		MODBUS_READ_REG				0x03		//��ȡ���������Ĵ���
#define		MODBUS_WRITE_REG			0x10		//д�뵥�����߶���Ĵ���
#define		MODBUS_EXT_READ_REG			0x46		//��ȡ���ܼĴ���
#define		MODBUS_EXT_WRITE_REG		0x47		//д�����ܼĴ���

#define		MODBUS_OFFSET				1
#define		MODBUS_ERR_OFFSET			1
#define		MODBUS_CTRL_OFFSET			2
#define		MODBUS_READ_REQ_OFFSET		4
#define		MODBUS_READ_RESP_OFFSET		1
#define		MODBUS_WRITE_REQ_OFFSET		5
#define		MODBUS_WRITE_RESP_OFFSET	4



#define		MODBUS_RTU_OFFSET	1		//��������ʼ��ַ


#pragma pack(1)

typedef		struct _MODBUS_RTU_HEADER
{
	uint8	address;					//�豸��ַ0-247
} MODBUS_RTU_HEADER;

typedef		struct _MODBUS_HEADER
{
	uint8	func_code;					//������1-127
} MODBUS_HEADER;

typedef		struct _MODBUS_ERR_HEADER
{
	uint8	error_code;					//�쳣��1-4
} MODBUS_ERR_HEADER;

typedef		struct _MODBUS_CTRL_HEADER
{
	uint16	reg_addr;					//�Ĵ�����ַ
} MODBUS_CTRL_HEADER;

typedef		struct _MODBUS_READ_REQ_HEADER
{
	uint16	reg_addr;					//�Ĵ�����ַ
	uint16	reg_num;					//�Ĵ�����ֵ
} MODBUS_READ_REQ_HEADER;

typedef		struct _MODBUS_READ_RESP_HEADER
{
	uint8	byte_num;					//�ֽ���
} MODBUS_READ_RESP_HEADER;

typedef		struct _MODBUS_WRITE_REQ_HEADER
{
	uint16	reg_addr;					//�Ĵ�����ַ
	uint16	reg_num;					//�Ĵ�����ֵ
	uint8	byte_num;					//�ֽ���
} MODBUS_WRITE_REQ_HEADER;

typedef		struct _MODBUS_WRITE_RESP_HEADER
{
	uint16	reg_addr;					//�Ĵ�����ַ
	uint16	reg_num;					//�Ĵ�����ֵ
} MODBUS_WRITE_RESP_HEADER;

#pragma pack()

int32 modbus_rtu_process(P_MSG_INFO   pMsg);

#endif

