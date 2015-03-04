#ifndef __STORAGE_H__
#define __STORAGE_H__

#define INIT_STATUS_WORD   0x20080207
#define DEFAULT_DROP_KEEP_TIME    800

#define DEFAULT_SELF_POWER_TIME    20

#define DEFAULT_SELF_POWER_DEADLINE_HIGH    20
#define DEFAULT_SELF_POWER_DEADLINE_LOW    0

#define DEFAULT_SELF_POWER_ACTION_TIME_HIGH    20
#define DEFAULT_SELF_POWER_ACTION_TIME_LOW     0

#define SYS_ERROR_EEPROM_RW    0X0001
//#define RECORD_DROP_TIME_FLAG  0x0002   //是否已经保存了最后一次掉电的时间
#define SYS_DROP_ACTION        0x0004   //是否有电容继电器投切动作
#define SYS_POWER_CHECK_FLAG   0x0008   //是否进行再启动检查
#define SYS_RELAY_ON_FLAG      0x0010   //接触器是否在闭合状态
#define SYS_JDQ_ON_FLAG        0x0020   //继电器是否在闭合状态

#define SYS_WARNING_MASK     0x0001 //系统告警，华兄
#define SYS_DROP_KEEP_MASK   0x0002 //晃电保护开关
#define SYS_SELF_POWER_MASK  0x0004 //再上电启动开关

void MEM_Init();
void MEM_SaveDropEvent(unsigned char ok_flag);
void MEM_SaveDropTime();
void MEM_SavePassword(void);
void MEM_RestorePassword(void);
void MEM_SaveSelfEvent(unsigned char ok_flag);


#endif

