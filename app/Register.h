
#ifndef __REGISTER_H__
#define __REGISTER_H__


// ADDR
#define		MODBUS_GRP0_ADDR		0			//��ѹ����Ƶ�ʹ���
#define		MODBUS_GRP1_ADDR		250			//���ܼĴ�����ַ
#define		MODBUS_GRP2_ADDR		500			//״̬���Ĵ�����ַ
#define		MODBUS_GRP3_ADDR		750			//�������Ĵ�����ַ
#define		MODBUS_GRP4_ADDR		1000		//ģ�����Ĵ�����ַ
#define		MODBUS_GRP5_ADDR		1250		//��Сֵ�Ĵ�����ַ
#define		MODBUS_GRP6_ADDR		1500		//���ֵ�Ĵ�����ַ
#define		MODBUS_GRP7_ADDR		1750		//�¼���Ϣ�Ĵ�����ַ
#define		MODBUS_GRP8_ADDR		2000		//¼����Ϣ�Ĵ�����ַ
#define		MODBUS_GRP10_ADDR		2500		//�����Ĵ�����ַ
#define		MODBUS_GRP11_ADDR		2750		//��ƽ��Ĵ�����ַ
#define		MODBUS_GRP12_ADDR		3000		//г���Ĵ�����ַ
#define		MODBUS_GRP20_ADDR		5000		//ϵͳ�����Ĵ�����ַ
#define		MODBUS_GRP21_ADDR		5250		//Խ�޶�ֵ�Ĵ�����ַ
#define		MODBUS_GRP22_ADDR		5500		//ͨѶ�Ĵ�����ַ
#define		MODBUS_GRP23_ADDR		5750		//���ܲ����Ĵ�����ַ
#define		MODBUS_GRP24_ADDR		6000		//���ڼĴ�����ַ
#define		MODBUS_GRP25_ADDR		6250		//ģ������Ĵ�����ַ 
#define		MODBUS_GRP30_ADDR		7500		//�¼���¼����
#define		MODBUS_GRP40_ADDR		10000		//¼����¼����
#define		MODBUS_GRP_SIZE			20000

#define		MODBUS_EXT_GRP0_ADDR	0			//У׼�Ĵ�����ַ
#define		MODBUS_EXT_GRP_SIZE		2000

#define		MODBUS_TIME0_ADDR		0			//ʱ��Ĵ�����ַ
#define		MODBUS_TIME_SIZE		500

#define		MODBUS_CONF0_ADDR		0			//���üĴ�����ַ
#define		MODBUS_CONF1_ADDR		500			//��ֵ�Ĵ�����ַ
#define		MODBUS_CONF2_ADDR		1000	    //��ֵ�Ĵ�����ַ
#define		MODBUS_CONF3_ADDR		2000	    //��ֵ�Ĵ�����ַ
#define		MODBUS_CONF4_ADDR		3000		//��ֵ�Ĵ�����ַ
#define		MODBUS_CONF_SIZE		4000


//ʱ��
enum	_TIME_CFG
{
	TIME_YEAR,
	TIME_MON,
	TIME_DAY,
	TIME_WEEK,
	TIME_HOUR,
	TIME_MIN,
	TIME_SEC,
	TIME_MSEL,
	TIME_SIZE,
};


//��ֵ�ṹ
typedef struct _RANGE
{
	int32	val;
	int32	min;
	int32	max;	
}RANGE;



// Ctrl Write
int32 register_ctrl_operation(int32 reg_addr, int32 reg_val);

// Fast Read
int32 register_fast_read(int32 reg_addr, int32 reg_num, uint8 resp[]);

// Read + Write
int32 register_group_read(int32 reg_addr, int32 reg_num, uint8 resp[]);
int32 register_group_write(int32 reg_addr, int32 reg_num, const uint8 req[]);


// Time Read + Write
int32 register_time_read(int32 reg_addr, int32 reg_num, uint8 resp[]);
int32 register_time_write(int32 reg_addr, int32 reg_num, const uint8 req[]);

// Extern Read + Write
int32 register_extern_group_read(int32 reg_addr, int32 reg_num, uint8 resp[]);
int32 register_extern_group_write(int32 reg_addr, int32 reg_num, const uint8 req[]);

// Conf Read + Write
int32 register_conf_read(int32 reg_addr, int32 reg_num, uint8 resp[]);
int32 register_conf_write(int32 reg_addr, int32 reg_num, const uint8 req[]);


#endif
