#ifndef __UART_LINK__
#define __UART_LINK__


#define RS485_COM_PORT            0       //主动通讯端口
#define MAX_COM_PORT              1

#define RS485_UART           USART1

#define END_TX_QUEUE_SIZE         5

#define MAX_MSG_SHORT             4 
#define MAX_MSG_LONG              4
#define MAX_MSG_LARGE             4

#define UART_RECV_BUF_SIZE     256u

#define END_NON_MODULE_USED    0xff

#define GET_MAX_MSG(type)             (MAX_MSG_CNT[MSG_SHORT])

#define END_IDLE    1            /* idle status */
#define END_READ    2            /* read mode */
#define END_WRITE    3            /* write mode */
#define END_BUSY    4            /* busy status */
#define END_FORWORD   (5)
#define END_USERD   0xfe


#define FREE                                  0
#define ALLOC                                 1
#define SENDING                               2
#define SENDED                                3


#define END_STATUS_IDLE         0
#define END_STATUS_SENDING      1
#define END_STATUS_RECEIVING    2

//#define END_STATUS_MUX            2  //双工
//#define END_STATUS_MIRROR       3  //镜像

#define END_STATUS_SPI_R        4  //当前为SPI口读传输状态
#define END_STATUS_SPI_W        5  //当前为SPI口写传输状态

#define END_STATUS_INVALID         6
#define END_STATUS_ERROR         7

//
#define END_STATUS_SEND_DONE         8
#define END_STATUS_SEND_TIMEOUT         9
#define END_STATUS_RECV_DONE         10
#define END_STATUS_RECV_TIMEOUT         11

#define END_DEBUG                 1

#define END_FAST_PROETCT_CYCLE    (unsigned char)1     /* 1 s */
#define END_SLOW_PROETCT_CYCLE    (unsigned char)5     /* 4 s */
#define END_SLOW_PROETCT_CYCLE2    (unsigned char)5     /* 5 s */

//#define RX_INTERNET_BUFFER_SIZE         4800

typedef U32 (* END_RECV_PTR)(UCHAR* txbuf, USHORT txnum);
typedef U32 (* END_SEND_PTR)(UCHAR* txbuf, USHORT txnum);
//typedef MD_STATUS (* END_WRITE_PTR)(UCHAR* wBuf, USHORT wNum,USHORT wAddr);
typedef U32 (* END_UPPER_CALL_BACK)();
typedef U32 (* END_TIMEOUT_CALL_BACK)(pvoid h);
typedef void (* END_START_PTR)(void);
typedef void (* END_STOP_PTR)(void);
typedef void (* END_SWITCH_STATUE_PTR)(unsigned char new_status);

typedef enum
{
    MSG_SHORT = 0,
    MSG_LONG,
    MSG_LARGE,
    MAX_MSG_ITEM
} MSG_TYPE;

//用来保存发送，接受消息的相关信息，内容待完善
typedef struct _msg_header_
{
    unsigned short msg_len;
    //unsigned short msg_wait_time;  //for plc layer
    unsigned short wait_resp_time; //单位0.1s，发送完毕后等待响应的时间，for plc layer
    unsigned long time_stamp;
    OS_EVENT * msg_event;
    unsigned char msg_type;  //
    unsigned char send_pid;
    //unsigned char msg_id;
    unsigned char end_id;
    unsigned char sub_id;
    unsigned char block_state;    /*free, allocated*/
    unsigned char need_buffer_free;    /* TRUE 标识end 发送完后负责释放；FALSE 标识end 不管释放由application  负责释放*/

} MSG_HEADER, *P_MSG_HEADER;


typedef struct _msg_info_
{
    MSG_HEADER msg_header;
    unsigned char  msg_buffer[UART_RECV_BUF_SIZE];
} MSG_INFO, *P_MSG_INFO;


typedef struct _end_object_static_
{
    unsigned long rxPacketCount;
    unsigned long txPacketCount;
} END_STAT, *P_END_STAT;

typedef struct _end_object_
{
    unsigned char end_id;
    unsigned char end_send_status;

    unsigned short receive_len;            //当前收包长度
    unsigned short last_receive_len;    //上次收包长度
    unsigned char  * end_recv_buffer;

    END_START_PTR Start;                //开始工作
    END_STOP_PTR Stop;                //end 复位
    END_SEND_PTR Send;

    END_RECV_PTR RecvData;                //end 复位
    //END_UPPER_CALL_BACK UpperCallBack;
    //END_SWITCH_STATUE_PTR SwitchStatus;  //对于半双工端口，切换端口工作状态
    END_STAT endStatistics;

    P_MSG_INFO pMsgInfo;   //当前正在处理的消息报文

    unsigned char recv_timeout;

} END_OBJ, *P_END_OBJ;

extern UART_CCB g_uart_ccb[MAX_COM_PORT];

P_END_OBJ End_get_end_obj(UCHAR end_id);
void End_Init(void);
unsigned char End_OnTick(void* pTimerHandle, void* HANDLE);
unsigned char End_set_expire(UCHAR end_id );
unsigned char End_get_status(UCHAR end_id);
unsigned char End_set_status(UCHAR end_id, unsigned char state );
unsigned short End_send(P_MSG_INFO pMsgInfo);
unsigned char End_check_recv(P_END_OBJ pEndObj);
unsigned char End_check_send(UCHAR end_id);
unsigned char End_IsIdle(P_END_OBJ pEndObj);
unsigned char End_postProcess(unsigned char end_type,  pvoid h);
unsigned short End_tick_check(void);
U32 UART_ReceiveData(U8 end_id, UCHAR* rxbuf, USHORT rxnum );
P_MSG_INFO alloc_send_buffer(unsigned char type);


#endif
