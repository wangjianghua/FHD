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
//#define RECORD_DROP_TIME_FLAG  0x0002   //�Ƿ��Ѿ����������һ�ε����ʱ��
#define SYS_DROP_ACTION        0x0004   //�Ƿ��е��ݼ̵���Ͷ�ж���
#define SYS_POWER_CHECK_FLAG   0x0008   //�Ƿ�������������
#define SYS_RELAY_ON_FLAG      0x0010   //�Ӵ����Ƿ��ڱպ�״̬
#define SYS_JDQ_ON_FLAG        0x0020   //�̵����Ƿ��ڱպ�״̬

#define SYS_WARNING_MASK     0x0001 //ϵͳ�澯������
#define SYS_DROP_KEEP_MASK   0x0002 //�ε籣������
#define SYS_SELF_POWER_MASK  0x0004 //���ϵ���������

void MEM_Init();
void MEM_SaveDropEvent(unsigned char ok_flag);
void MEM_SaveDropTime();
void MEM_SavePassword(void);
void MEM_RestorePassword(void);
void MEM_SaveSelfEvent(unsigned char ok_flag);


#endif

