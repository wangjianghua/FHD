
#ifndef __REGISTER_H__
#define __REGISTER_H__


// ADDR
#define		MODBUS_GRP0_ADDR		0			//电压电流频率功率
#define		MODBUS_GRP1_ADDR		250			//电能寄存器地址
#define		MODBUS_GRP2_ADDR		500			//状态量寄存器地址
#define		MODBUS_GRP3_ADDR		750			//开入量寄存器地址
#define		MODBUS_GRP4_ADDR		1000		//模拟量寄存器地址
#define		MODBUS_GRP5_ADDR		1250		//最小值寄存器地址
#define		MODBUS_GRP6_ADDR		1500		//最大值寄存器地址
#define		MODBUS_GRP7_ADDR		1750		//事件信息寄存器地址
#define		MODBUS_GRP8_ADDR		2000		//录波信息寄存器地址
#define		MODBUS_GRP10_ADDR		2500		//基波寄存器地址
#define		MODBUS_GRP11_ADDR		2750		//不平衡寄存器地址
#define		MODBUS_GRP12_ADDR		3000		//谐波寄存器地址
#define		MODBUS_GRP20_ADDR		5000		//系统参数寄存器地址
#define		MODBUS_GRP21_ADDR		5250		//越限定值寄存器地址
#define		MODBUS_GRP22_ADDR		5500		//通讯寄存器地址
#define		MODBUS_GRP23_ADDR		5750		//电能参数寄存器地址
#define		MODBUS_GRP24_ADDR		6000		//出口寄存器地址
#define		MODBUS_GRP25_ADDR		6250		//模拟参数寄存器地址 
#define		MODBUS_GRP30_ADDR		7500		//事件记录数据
#define		MODBUS_GRP40_ADDR		10000		//录波记录数据
#define		MODBUS_GRP_SIZE			20000

#define		MODBUS_EXT_GRP0_ADDR	0			//校准寄存器地址
#define		MODBUS_EXT_GRP_SIZE		2000

#define		MODBUS_TIME0_ADDR		0			//时间寄存器地址
#define		MODBUS_TIME_SIZE		500

#define		MODBUS_CONF0_ADDR		0			//配置寄存器地址
#define		MODBUS_CONF1_ADDR		500			//初值寄存器地址
#define		MODBUS_CONF2_ADDR		1000	    //初值寄存器地址
#define		MODBUS_CONF3_ADDR		2000	    //初值寄存器地址
#define		MODBUS_CONF4_ADDR		3000		//初值寄存器地址
#define		MODBUS_CONF_SIZE		4000


//时间
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


//定值结构
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
