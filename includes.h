#ifndef __INCLUDES_H__
#define __INCLUDES_H__


#define S8    signed char
#define I8    signed char
#define U8  unsigned char     /* unsigned 8  bits. */
#define I16   signed short    /*   signed 16 bits. */
#define U16 unsigned short    /* unsigned 16 bits. */
#define I32   signed long   /*   signed 32 bits. */
#define U32 unsigned long   /* unsigned 32 bits. */
#define I16P I16              /*   signed 16 bits OR MORE ! */
#define U16P U16              /* unsigned 16 bits OR MORE ! */

/* data type defintion */
typedef    unsigned long    ULONG;
typedef    unsigned int    UINT;
typedef    unsigned short    USHORT;
typedef    unsigned char    UCHAR;
typedef    unsigned char    BOOL;
typedef unsigned char      BYTE;
typedef unsigned char*  PBYTE;

//typedef    unsigned char   uchar

typedef unsigned int     BIT_FIELD;
typedef void *pvoid;
typedef void VOID;
typedef void *PVOID;

typedef long long LONGLONG;
typedef LONGLONG* PLONGLONG;

typedef unsigned char		uint8;				//defined for unsigned 8-bits integer variable
typedef signed   char		int8;				//defined for signed 8-bits integer variable
typedef unsigned short		uint16;				//defined for unsigned 16-bits integer variable
typedef signed   short		int16;				//defined for signed 16-bits integer variable
typedef unsigned int		uint32;				//defined for unsigned 32-bits integer variable
typedef signed   int		int32;				//defined for signed 32-bits integer variable
typedef unsigned long long	uint64;				//defined for unsigned 64-bits integer variable
typedef signed   long long	int64;				//defined for signed 64-bits integer variable
typedef float				fp32;				//single precision floating point variable (32bits)
typedef double				fp64;				//double precision floating point variable (64bits)



#define HANDLE_Q         PVOID


#include "stm32f10x.h"
#include "ucos_ii.h"
#include "cpu.h"
#include "bsp.h"
#include "lib_def.h"
#include "app_cfg.h"
#include "GUI_12864.h"
#include "e2prom.h"
#include <string.h>
#include "display_menu.h"
#include "uart_link.h"
#include "queue.h"
#include "storage.h"
#include "ProLibrary.h"
#include "modbus.h"
#include "mod_helper.h"
#include "reg_dvc.h"
#include "register.h"
#include "math.h" //华兄


#define TRUE    1
#define FALSE   0

#define OK 0
#define ERROR   0xff

#define APPLICATION_ADDRESS   0x8003000

#define LCD_DISP_BUF_SIZE   1024

#define SYS_SAVE_TIME_LEN   8

#define SYS_PWD_LEN         7 //华兄

#define REV_OPERATION_EN   0u //华兄

#define SELF_POWER_EN      0u //华兄

#define MULTI_MODE_EN      0u //华兄

#define RTC_CLK_SRC_LSI    0u //华兄
#define RTC_CLK_SRC_LSE    0u
#define RTC_CLK_SRC_HSE    1u

#define MAX_RMS_NUM         8 //华兄
#define MAX_RMS_DISCRETE    4000u

#define LCD_TIMER_PRESCALE  2000u
#define LCD_TIMER_INTERVAL_MS  2000

#if 0
#define AD_SAMPLE_TIMES_PER_PERIOD  100 //100us
#else //华兄
#define AD_SAMPLE_TIMES_PER_PERIOD  400 //25us
#endif

#define SYS_POWER_ON            0
#define SYS_POWER_STEADY        1
#define SYS_DROPING             2
#define SYS_DROPED              3
#define SYS_INIT                4
#define SYS_AUTH_ERROR          5

#define LCD_DISPLAY_TIME_SEC   300

#if 0
#define KEY_VAL_UP      EXTI_Line11
#define KEY_VAL_DOWN    EXTI_Line10
#define KEY_VAL_SET     EXTI_Line13
#define KEY_VAL_OK      EXTI_Line12
#else //华兄

#define KEY_VAL_UP      EXTI_Line12
#define KEY_VAL_DOWN    EXTI_Line13
#define KEY_VAL_SET     EXTI_Line10
#define KEY_VAL_OK      EXTI_Line11

#endif

#define DROP_HISTORY_MAX_COUNT   10

#define SEC_POS   0
#define MIN_POS   1
#define HOUR_POS   2
#define DAY_POS   3
#define DATE_POS   4
#define MONTH_POS   5
#define YEAR_POS   6

/* 华兄 */
#define RELAY_SWITCH_OFF              0
#define RELAY_FORWARD_SWITCH_ON       1
#define RELAY_BACKWARD_SWITCH_ON      2

#define JDO_ON_ACT_TIME  1200

/* Private define ------------------------------------------------------------*/
#define ADC1_DR_Address    ((uint32_t)0x4001244C)

#if 0
#define E2P_SCL_SET()       GPIO_SetBits(GPIOB, GPIO_Pin_6)
#define E2P_SCL_RST()       GPIO_ResetBits(GPIOB, GPIO_Pin_6)

#define E2P_SDA_SET()       GPIO_SetBits(GPIOB, GPIO_Pin_7)
#define E2P_SDA_RST()       GPIO_ResetBits(GPIOB, GPIO_Pin_7)

#define E2P_WP_SET()       GPIO_SetBits(GPIOB, GPIO_Pin_5)
#define E2P_WP_RST()       GPIO_ResetBits(GPIOB, GPIO_Pin_5)
#else //华兄

#define E2P_SCL_SET()       GPIO_SetBits(GPIOB, GPIO_Pin_10)
#define E2P_SCL_RST()       GPIO_ResetBits(GPIOB, GPIO_Pin_10)

#define E2P_SDA_SET()       GPIO_SetBits(GPIOB, GPIO_Pin_11)
#define E2P_SDA_RST()       GPIO_ResetBits(GPIOB, GPIO_Pin_11)

#define E2P_WP_SET()        GPIO_SetBits(GPIOB, GPIO_Pin_12)
#define E2P_WP_RST()        GPIO_ResetBits(GPIOB, GPIO_Pin_12) 

#endif

#if 0
#define RTC_SCL_SET()       GPIO_SetBits(GPIOA, GPIO_Pin_11)
#define RTC_SCL_RST()       GPIO_ResetBits(GPIOA, GPIO_Pin_11)

#define RTC_SDA_SET()       GPIO_SetBits(GPIOA, GPIO_Pin_12)
#define RTC_SDA_RST()       GPIO_ResetBits(GPIOA, GPIO_Pin_12)
#else //华兄

#define RTC_SCL_SET()       GPIO_SetBits(GPIOB, GPIO_Pin_6)
#define RTC_SCL_RST()       GPIO_ResetBits(GPIOB, GPIO_Pin_6)

#define RTC_SDA_SET()       GPIO_SetBits(GPIOB, GPIO_Pin_7)
#define RTC_SDA_RST()       GPIO_ResetBits(GPIOB, GPIO_Pin_7)

#endif

#if 0
#define  LED_RUN_STATUS()   GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_4)
#define  LED_RUN_OFF()       GPIO_ResetBits(GPIOA, GPIO_Pin_4)
#define  LED_RUN_ON()      GPIO_SetBits(GPIOA, GPIO_Pin_4)

#define  LED_ALARM_STATUS()   GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_5)
#define  LED_ALARM_OFF()       GPIO_ResetBits(GPIOA, GPIO_Pin_5)
#define  LED_ALARM_ON()      GPIO_SetBits(GPIOA, GPIO_Pin_5)

#define  LED_2_STATUS()   GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_6)
#define  LED_2_OFF()       GPIO_ResetBits(GPIOA, GPIO_Pin_6)
#define  LED_2_ON()      GPIO_SetBits(GPIOA, GPIO_Pin_6)

#define  LED_3_STATUS()   GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_7)
#define  LED_3_OFF()       GPIO_ResetBits(GPIOA, GPIO_Pin_7)
#define  LED_3_ON()      GPIO_SetBits(GPIOA, GPIO_Pin_7)
#else //华兄

#define LED_RUN_ON()        GPIO_SetBits(GPIOC, GPIO_Pin_0)
#define LED_RUN_OFF()       GPIO_ResetBits(GPIOC, GPIO_Pin_0)
#define LED_RUN_TOGGLE()    GPIO_ToggleBits(GPIOC, GPIO_Pin_0)

#define LED_PWR_ON()        GPIO_SetBits(GPIOC, GPIO_Pin_1)
#define LED_PWR_OFF()       GPIO_ResetBits(GPIOC, GPIO_Pin_1)
#define LED_PWR_TOGGLE()    GPIO_ToggleBits(GPIOC, GPIO_Pin_1)

#define LED_HD_ON()         GPIO_SetBits(GPIOC, GPIO_Pin_2)
#define LED_HD_OFF()        GPIO_ResetBits(GPIOC, GPIO_Pin_2)
#define LED_HD_TOGGLE()     GPIO_ToggleBits(GPIOC, GPIO_Pin_2)

#define LED_UART_ON()       GPIO_SetBits(GPIOC, GPIO_Pin_3)
#define LED_UART_OFF()      GPIO_ResetBits(GPIOC, GPIO_Pin_3)
#define LED_UART_TOGGLE()   GPIO_ToggleBits(GPIOC, GPIO_Pin_3)

/* "\"后面不要有空格 */
#define LED_ON() { \
    LED_RUN_ON();  \
    LED_PWR_ON();  \
    LED_HD_ON();   \
    LED_UART_ON(); \
}

#define LED_OFF() { \
    LED_RUN_OFF();  \
    LED_PWR_OFF();  \
    LED_HD_OFF();   \
    LED_UART_OFF(); \
}

#define MAIN_MOS_BROKEN_ALRAM_ON() { \
    GPIO_SetBits(GPIOC, GPIO_Pin_0); \
    GPIO_SetBits(GPIOC, GPIO_Pin_1); \
    GPIO_SetBits(GPIOC, GPIO_Pin_2); \
    GPIO_SetBits(GPIOC, GPIO_Pin_3); \
}

#define MAIN_MOS_BROKEN_ALRAM_OFF() {  \
    GPIO_ResetBits(GPIOC, GPIO_Pin_0); \
    GPIO_ResetBits(GPIOC, GPIO_Pin_1); \
    GPIO_ResetBits(GPIOC, GPIO_Pin_2); \
    GPIO_ResetBits(GPIOC, GPIO_Pin_3); \
}

/* 华兄 */
//#define DEBUG_LED

#ifdef DEBUG_LED
#define DEBUG_LED_ON()       LED_UART_ON()
#define DEBUG_LED_OFF()      LED_UART_OFF()
#else
#define DEBUG_LED_ON()       
#define DEBUG_LED_OFF()      
#endif

#endif

#if 0
#define   JDQ_ON()    GPIO_SetBits(GPIOC, GPIO_Pin_7)
#define   JDQ_OFF()    GPIO_ResetBits(GPIOC, GPIO_Pin_7)
#define   JDQ_ST()    GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_7)

#define   JDQ_STA()    GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_7)
#else //华兄

#define   JDQ_ON()     GPIO_SetBits(GPIOA, GPIO_Pin_0)
#define   JDQ_OFF()    GPIO_ResetBits(GPIOA, GPIO_Pin_0)
#define   JDQ_ST()     GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_0)

#define   JDQ_STA()    GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_0)

/* 华兄 */
#define MAIN_MOS_CHECK_JDQ_ON()    GPIO_SetBits(GPIOB, GPIO_Pin_5)
#define MAIN_MOS_CHECK_JDQ_OFF()   GPIO_ResetBits(GPIOB, GPIO_Pin_5)
#define MAIN_MOS_CHECK_JDQ_STAT()  GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_5)

#define MAIN_MOS_ON()         TRIC2_ON()
#define MAIN_MOS_OFF()        TRIC2_OFF()

#endif

#if 0
#define LCM_CON_ON()  GPIO_SetBits(GPIOC, GPIO_Pin_2)
#define LCM_CON_OFF()  GPIO_ResetBits(GPIOC, GPIO_Pin_2)
#define BLCD_CON_ON()  GPIO_ResetBits(GPIOC, GPIO_Pin_1)
#define BLCD_CON_OFF()  GPIO_SetBits(GPIOC, GPIO_Pin_1)



#define LCD_RST()  GPIO_ResetBits(GPIOA, GPIO_Pin_2)
#define LCD_SET()  GPIO_SetBits(GPIOA, GPIO_Pin_2)

#define LCD_A0_RST()  GPIO_ResetBits(GPIOA, GPIO_Pin_1)
#define LCD_A0_SET()  GPIO_SetBits(GPIOA, GPIO_Pin_1)

#define LCD_CS_RST()  GPIO_ResetBits(GPIOA, GPIO_Pin_3)
#define LCD_CS_SET()  GPIO_SetBits(GPIOA, GPIO_Pin_3)

#define LCD_SCL_RST()  GPIO_ResetBits(GPIOA, GPIO_Pin_0)
#define LCD_SCL_SET()  GPIO_SetBits(GPIOA, GPIO_Pin_0)

#define LCD_SDA_RST()  GPIO_ResetBits(GPIOC, GPIO_Pin_3)
#define LCD_SDA_SET()  GPIO_SetBits(GPIOC, GPIO_Pin_3)
#else //华兄

#define LCD_BLED_SET()     GPIO_SetBits(GPIOA, GPIO_Pin_12)
#define LCD_BLED_RESET()   GPIO_ResetBits(GPIOA, GPIO_Pin_12)

#define LCD_PWR_SET()      GPIO_SetBits(GPIOA, GPIO_Pin_11)
#define LCD_PWR_RESET()    GPIO_ResetBits(GPIOA, GPIO_Pin_11)

#define LCD_SCLK_SET()     GPIO_SetBits(GPIOB, GPIO_Pin_13)
#define LCD_SCLK_RESET()   GPIO_ResetBits(GPIOB, GPIO_Pin_13)

#define LCD_MOSI_SET()     GPIO_SetBits(GPIOB, GPIO_Pin_15)
#define LCD_MOSI_RESET()   GPIO_ResetBits(GPIOB, GPIO_Pin_15)

#define LCD_CS_SET()       GPIO_SetBits(GPIOC, GPIO_Pin_5)
#define LCD_CS_RESET()     GPIO_ResetBits(GPIOC, GPIO_Pin_5)

#define LCD_RST_SET()      GPIO_SetBits(GPIOC, GPIO_Pin_8)
#define LCD_RST_RESET()    GPIO_ResetBits(GPIOC, GPIO_Pin_8)

#define LCD_RS_SET()       GPIO_SetBits(GPIOC, GPIO_Pin_9)
#define LCD_RS_RESET()     GPIO_ResetBits(GPIOC, GPIO_Pin_9)

#if 0
#define LCM_CON_ON()       LCD_PWR_SET()
#define LCM_CON_OFF()      LCD_PWR_RESET()
#define BLCD_CON_ON()      LCD_BLED_RESET()
#define BLCD_CON_OFF()     LCD_BLED_SET()
#else //华兄
#define LCM_PWR_ON()       LCD_PWR_SET()
#define LCM_PWR_OFF()      LCD_PWR_RESET()
#endif

#endif

#define MAX_ERROR_SN       2

typedef struct 
{
    /* 配置类 */
    unsigned long initStatusWord;
    unsigned char password[SYS_PWD_LEN]; //华兄    
    unsigned long SysSwitch;    
    unsigned long SysRunStatus;
    unsigned short PowerDropKeepTime; //晃电保护时间
    unsigned short SelfPowerTime;

#if (SELF_POWER_EN > 0u)
    unsigned char SelfPowerDeadlineHigh; //华兄
    unsigned char SelfPowerDeadlineLow;

    unsigned char SelfPowerActionTimeHigh; //华兄
    unsigned char SelfPowerActionTimeLow; 
#endif    

#if (MULTI_MODE_EN > 0u)
    unsigned char mode; //华兄
#endif

    unsigned char dev_addr[2];
    unsigned short selfPowerLimit; //再启动电压百分比
    unsigned short SelfPowerValidTime; //再启动最大允许时间
    unsigned int voltageFixCoe; //电压修正系数

    unsigned char break_points; //华兄
    unsigned char adc_diff;

    /* 记录类 */
    unsigned long PowerDropCount;        
    unsigned long SelfPowerOnCount;
    unsigned char PowerDropTime[DROP_HISTORY_MAX_COUNT][SYS_SAVE_TIME_LEN];
    unsigned short PowerDropPeriod[DROP_HISTORY_MAX_COUNT];
    unsigned char SelfPowerOnTime[DROP_HISTORY_MAX_COUNT][SYS_SAVE_TIME_LEN];
    unsigned char lastBreakTimeStamp[SYS_SAVE_TIME_LEN];

    unsigned char main_mos_broken; //华兄

#if 0
    /* 错误类 */     
    unsigned short error_count;
    unsigned short error_sn;
    unsigned char error_time[MAX_ERROR_SN][4];
    unsigned long error_data[MAX_ERROR_SN];
#endif //华兄
}SYS_CONF;



struct RTCCounterValue 
{
	uint8_t Sec;
	uint8_t Min;
	uint8_t Hour;
	uint8_t Week;
	uint8_t Day;	
	uint8_t Month;
	uint8_t Year;
};

#define LEAP_YEAR(year) ((year%4)==0)

#define OffsetOf(type, f) ((USHORT)((char *)&((type *)0)->f - (char *)(type *)0))
#define SizeOf(type, f)   sizeof(((type *)0)->f)


extern uint16_t ADC_RegularConvertedValueTab[2][AD_SAMPLE_TIMES_PER_PERIOD];
extern const unsigned char lcd_disp_buf_welcome[LCD_DISP_BUF_SIZE];
extern INT32U   g_last_uptime;
extern INT32U g_ac_count;
//extern INT32U g_ac_sb1_count;
extern unsigned char g_rtc_time[7];
extern unsigned char lcd_disp_buf[LCD_YSIZE / 8][LCD_XSIZE];
extern INT32U g_droping_timestamp;
extern INT32U g_power_state;
extern INT32U g_scount;

extern const unsigned char g_w_m[8];
extern const unsigned char g_w_cr[256];

extern INT32U mseconds;

extern u32 g_msec_count; //华兄，毫秒计数器
extern u32 g_dropping_msec; //华兄，掉电时毫秒计数值
extern u32 g_power_on_msec; //华兄，来电时毫秒计数值
extern u32 g_power_check_count; //华兄，再启动计数器 
extern u32 g_power_check_flag; //华兄，再启动标志

extern unsigned short g_diff_num ;
extern unsigned short g_diff_array;
extern unsigned char g_diff_pos;
extern unsigned short g_break_pos;
extern INT32U g_droping_power_on_time;
extern INT32U g_main_mos_check_jdq_on_time;
extern INT32U break_off_count;
extern INT32U g_voltage ;
extern INT32U ac_rms;
void BSP_Init(void);
void BSP_ADC_DMA_int_proc();
void Relay_Keep_Mode(uint8_t mode );

extern SYS_CONF g_sys_conf;

extern unsigned char g_relay_switch; //华兄
extern INT32U g_dbg_disp; //华兄

void lcd_init(void);
void display_kuang();
void display_lattice(unsigned char dat1,unsigned char dat2);
void lcd_disp_map(unsigned char *pmap);
void EXTI0_Config(void);
void EXTI15_10_Config(void);
void CLR_WatchDog();
void delay_nms(unsigned int n);
unsigned char E2_ReadByte(unsigned short addr);
void E2_WriteByte(unsigned short addr,unsigned char nContent);
void RTC_ReadTime(unsigned char *time);
void RTC_WriteTime(unsigned char *time);
U32 OS_mktime(struct RTCCounterValue *tmp);
void rtc_adjust_time(void);
u32 rtc_get_time(void);
void GUI_X_Init (void);
void  GUI_X_StoreKey (unsigned int k);
int  GUI_X_WaitKey (void);
void GUI_Key_Proc(unsigned int k);
int LCD_disp_keep_form(unsigned int key_event, unsigned int form_msg);
void aux_res_on(void);
void aux_res_off(void);

#endif
