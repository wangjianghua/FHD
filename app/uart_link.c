#include "includes.h"


END_OBJ g_EndObjectPool[MAX_COM_PORT] =
{
    {RS485_COM_PORT, END_STATUS_IDLE, 0, 0, 0, NULL, NULL, NULL, NULL},
};

queue *g_EndTxQueue[MAX_COM_PORT] = {NULL};
queue *g_EndRxQueue[MAX_COM_PORT] = {NULL};

unsigned char g_TxEndQueueBuf[((END_TX_QUEUE_SIZE+2) * 4)];
unsigned char g_RxEndQueueBuf[((END_TX_QUEUE_SIZE+2) * 4)];

U32 EndTxQueueMem[MAX_COM_PORT][END_TX_QUEUE_SIZE+2];

OS_EVENT *g_sem_end;

const UCHAR MAX_MSG_CNT[MAX_MSG_ITEM] =
{
    MAX_MSG_SHORT,
    MAX_MSG_LONG,
    MAX_MSG_LARGE
};

UCHAR FreeMsgCnt[MAX_MSG_ITEM] =
{
    MAX_MSG_SHORT,
    MAX_MSG_LONG,
    MAX_MSG_LARGE
};

UART_CCB g_uart_ccb[MAX_COM_PORT];

U8 g_UartRxBuf[UART_RECEIVE_BUF_SIZE];

MSG_INFO gShortMsgPool[MAX_MSG_SHORT];

P_MSG_INFO pShortMsgPool[MAX_MSG_SHORT] = {NULL};


/**********************************************************

��������:      ��'from -> to'  ����������memory ��ʼ�����.

���뺯��:    �ڴ���ʼ��ַ-> �ڴ������ַ

**********************************************************/
void mem_zeroinit(pvoid from, pvoid to)
{
    unsigned char* p_start =(unsigned char*)from;
    unsigned char* p_end =(unsigned char*)to;
    unsigned char* tmp;

    unsigned long size = 0;

    if( p_start > p_end )
    {
        tmp = p_end;
        p_end = p_start;
        p_start = tmp;
    }

    size = (unsigned long)p_end - (unsigned long)p_start;
    memset((unsigned char*)p_start, 0, size);

    return;
}


void mem_msg_buffer_init(MSG_INFO * MsgPool, P_MSG_INFO * MsgArray, unsigned short MsgNum, U16 msg_size)
{
    U32 size;
    U32 i;

    size = msg_size * (U32)MsgNum;

    mem_zeroinit((pvoid)(MsgPool),  (pvoid)((unsigned long)(MsgPool) + size) );

    i = 0;
    while (i < MsgNum)
    {
        // ��ʼ���ָ�ײ���������buffer queue (10 memory blocks).
        MsgArray[i] = (P_MSG_INFO)(((U32)MsgPool) + msg_size*i);

        // initial ..
        MsgArray[i]->msg_header.block_state = FREE;    /* ��ʼ��ΪFREE  */

        i ++;
    }

}



/*******************************************************

����˵��:  ��pMsgTxPool[i]  ������һ֡����buffer

*******************************************************/
P_MSG_INFO alloc_send_buffer(unsigned char type)
{
    P_MSG_INFO pmsg = NULL;
    P_MSG_INFO	*pool = NULL;

    unsigned short i;
#if OS_CRITICAL_METHOD == 3                      /* Allocate storage for CPU status register           */
    OS_CPU_SR  cpu_sr = 0;
#endif


#if 0
    if( type == MSG_LARGE)
    {
        pool = (P_MSG_INFO *)pLargeMsgPool;
    }
    else if( type == MSG_LONG)
    {
        pool = (P_MSG_INFO *)pLongMsgPool;
    }
    else if( type == MSG_SHORT)
    {
        pool = (P_MSG_INFO *)pShortMsgPool;
    }
    else
        return NULL;
#endif

    pool = (P_MSG_INFO *)pShortMsgPool;

    /* �������п���buffer , �ҵ�һ�����ÿ���buffer */
    for(i =0; i<GET_MAX_MSG(type); i++)
    {
        pmsg = (P_MSG_INFO)(*(pool + i));

        OS_ENTER_CRITICAL();
        if(pmsg->msg_header.block_state == FREE)
        {
            pmsg->msg_header.block_state = ALLOC;
            OS_EXIT_CRITICAL();
            pmsg->msg_header.msg_len = 0;
            pmsg->msg_header.time_stamp = OSTimeGet();
            break;
        }
        OS_EXIT_CRITICAL();
    }

    if( i >= GET_MAX_MSG(type) )
        return NULL;

   
    memset(pmsg->msg_buffer, 0xff, UART_RECEIVE_BUF_SIZE );

    /* ͳ�Ƽ����ݼ�*/
    //FreeMSGTxCnt --;

    OS_ENTER_CRITICAL();
    FreeMsgCnt[type] --;
    OS_EXIT_CRITICAL();


    //if( FreeMSGTxCnt <= 3 )
    if( FreeMsgCnt[type] == 0 )
    {
        //ALERT(" left msg number �ݼ����ٽ�ֵ ");
        //Alert((type), ALERT_RESET_DEVICE, __FILE__, __LINE__);

    }

    return pmsg;
}

/*******************************************************

����˵��:  ��pMsgTxPool[i]  ���ͷ�һ֡����buffer (   *pmsg
)

*******************************************************/

unsigned char free_send_buffer(pvoid pmsg )
{
    unsigned char i = 0, type;
    P_MSG_INFO	pfree = (P_MSG_INFO)pmsg;
    P_MSG_INFO	*pool = NULL;
#if OS_CRITICAL_METHOD == 3                      /* Allocate storage for CPU status register           */
    OS_CPU_SR  cpu_sr = 0;
#endif

    if(pfree->msg_header.block_state == FREE)
        return ERROR;

    pool =  (P_MSG_INFO *)pShortMsgPool;

    type = MSG_SHORT;


    /*-------------------------------------------*/
    for( i = 0; i < GET_MAX_MSG(type); i++)
    {
        OS_ENTER_CRITICAL();
        if( (  *(pool + i ) == pfree ) )
        {
            /*  �ͷ��ڴ��ΪFREE */
            if(pfree->msg_header.block_state != FREE)
                pfree->msg_header.block_state = FREE;
            else
            {
                OS_EXIT_CRITICAL();
                return ERROR;
            }

            OS_EXIT_CRITICAL();
            break;
        }
        OS_EXIT_CRITICAL();
    }

  


    /* ͳ�Ƽ����ݼ�*/
    OS_ENTER_CRITICAL();
    FreeMsgCnt[type] ++;
    OS_EXIT_CRITICAL();

    //if( FreeMSGTxCnt > GET_MAX_MSG(type) )
    if( FreeMsgCnt[type] > GET_MAX_MSG(type) )
    {
        //ALERT(" mail queue �������� ");
        //Alert(ALERT_NO_MEMORY, ALERT_RESET_DEVICE, __FILE__, __LINE__);
    }

    return OK;
}





U32 UART_ReceiveData(U8 end_id, UCHAR* rxbuf, USHORT rxnum )
{
#if OS_CRITICAL_METHOD == 3                                /* Allocate storage for CPU status register     */
    OS_CPU_SR  cpu_sr = 0;
#endif
    P_UART_CCB p_uc = &g_uart_ccb[end_id];

    if( (rxnum < 1) || (end_id >= MAX_COM_PORT) )
    {
        return ERROR;
    }
    OS_ENTER_CRITICAL();
    //p_uc->gUartRxCnt = 0;
    //p_uc->gUartRxLen = rxnum;
    p_uc->gpUartRxAddress = rxbuf;
    p_uc->gpUartRxStartAddress = rxbuf;
    p_uc->gpUartRxReadAddress = rxbuf;
    p_uc->gpUartRxEndAddress = (UCHAR*)((ULONG)rxbuf + rxnum);
    OS_EXIT_CRITICAL()
    return OK;
}


void End_Init(void)
{
    P_END_OBJ pEndObj = NULL;

    unsigned char i;

    mem_msg_buffer_init((MSG_INFO *)gShortMsgPool, (P_MSG_INFO * )pShortMsgPool, MAX_MSG_SHORT, sizeof(MSG_INFO));

    g_sem_end = OSSemCreate(0);

    //alan test  ��Ҫ��ʱע�͵�, ��֪ΪɶIIC Start һ����, MCU �ͷ���.
    for( i = RS485_COM_PORT; i < MAX_COM_PORT; i++)
    {
        // �ҵ���ǰEnd Object
        pEndObj = g_EndObjectPool + i;

        /* end queue[x] initialize */ /* each end object define '50 block' queue */
        g_EndTxQueue[i] = define_new_queue((queue *)g_TxEndQueueBuf, END_TX_QUEUE_SIZE);
        g_EndRxQueue[i] = define_new_queue((queue *)g_RxEndQueueBuf, END_TX_QUEUE_SIZE);
       
        pEndObj->end_recv_buffer = (unsigned char *)g_UartRxBuf;

        pEndObj->last_receive_len = 0;
        pEndObj->receive_len = 0;

        pEndObj->recv_timeout = 0;

        UART_ReceiveData(i, pEndObj->end_recv_buffer, UART_RECEIVE_BUF_SIZE);

        // ���д���״̬ת��REVC STATUS
        pEndObj->end_send_status = END_STATUS_IDLE;


    }
}


/***********************************************************
Tick������ã����ÿ��END�ӿ��Ƿ����µ�frame����
************************************************************/
unsigned short End_tick_check(void)
{
#if OS_CRITICAL_METHOD == 3                                /* Allocate storage for CPU status register     */
    OS_CPU_SR  cpu_sr = 0;
#endif
    unsigned char i;
    U16 cp_len, msg_len;

    P_END_OBJ     pEndObj = NULL;
    P_MSG_INFO     pnewmsg = NULL;

    P_UART_CCB p_uc;

    for(i = RS485_COM_PORT; i < MAX_COM_PORT; i++ )
    {
        /////////////////////////////
        pEndObj = g_EndObjectPool + i;

        p_uc = &g_uart_ccb[i];

        if(pEndObj->end_send_status == END_STATUS_SENDING)
        {
            /* ��鵱ǰ�����Ƿ������. ��ϵĻ�, ת����,   ����ת����*/
            if( End_check_send(pEndObj->end_id) == OK )
            {

                //Send  �Ѿ������� ת��receive status ��ʼ״̬
                pEndObj->end_send_status = END_STATUS_IDLE;

                pnewmsg = (P_MSG_INFO)pEndObj->pMsgInfo;

                pnewmsg->msg_header.block_state = SENDED;

                // alan dynamic sending buffer.
                // ��ǰend sending ��������������ֱ���ͷ�sending buffer
                if( pnewmsg->msg_header.need_buffer_free == OK)    /* ����Ҫ�������ϲ�application �����ͷ�*/
                {
                    free_send_buffer(pnewmsg);
                }
            }
        }
        else
        {
            if( NULL != (pnewmsg = dequeue( g_EndTxQueue[pEndObj->end_id])) )
            {
                End_send(pnewmsg);
            }
        }

        if( End_check_recv(pEndObj) == OK ) /* û�������ݵ���, ��ʼ����*/
        {
    
            pnewmsg = alloc_send_buffer(MSG_SHORT);

            if(pnewmsg == NULL)
            {               
                continue;
            }

            msg_len = pEndObj->receive_len;

            pnewmsg->msg_header.msg_len = msg_len;
            pnewmsg->msg_header.end_id = i;

            OS_ENTER_CRITICAL();
            cp_len = (U16)(p_uc->gpUartRxEndAddress - p_uc->gpUartRxReadAddress);
            

            if(cp_len >= msg_len)
            {
                //OS_ENTER_CRITICAL();
                memcpy(pnewmsg->msg_buffer, p_uc->gpUartRxReadAddress, msg_len );                
                p_uc->gpUartRxReadAddress += msg_len;
                if(p_uc->gpUartRxReadAddress == p_uc->gpUartRxEndAddress)
                    p_uc->gpUartRxReadAddress = p_uc->gpUartRxStartAddress;
                //OS_EXIT_CRITICAL();
            }
            else
            {
                //OS_ENTER_CRITICAL();
                memcpy(pnewmsg->msg_buffer, p_uc->gpUartRxReadAddress, cp_len);
                memcpy((pnewmsg->msg_buffer + cp_len), p_uc->gpUartRxStartAddress, (msg_len-cp_len));                
                p_uc->gpUartRxReadAddress = p_uc->gpUartRxStartAddress + (msg_len-cp_len);
                //OS_EXIT_CRITICAL();
            }
            OS_EXIT_CRITICAL();

            pEndObj->last_receive_len = 0;
            pEndObj->receive_len = 0;

            pEndObj->endStatistics.rxPacketCount++;

            End_postProcess(pEndObj->end_id, pnewmsg);

            free_send_buffer(pnewmsg);          


        }
    }

    return OK;
}


P_END_OBJ End_get_end_obj(UCHAR end_id)
{
    unsigned char i = 0;
    while( i < MAX_COM_PORT)
    {
        if( g_EndObjectPool[i].end_id == end_id)
            return &g_EndObjectPool[i];
        i++;
    }

    return NULL;
}

U32 End_uart_send(UCHAR end_id,  UCHAR* txbuf, USHORT    txnum )
{
    USART_TypeDef * USARTx;
    P_UART_CCB p_uc;
    UCHAR      send_byte=0;

    if( txnum < 1 )
    {
        return ERROR;
    }

    switch(end_id)
    {    
    case RS485_COM_PORT:      
        USARTx = RS485_UART;
        break;  
    default:
        return ERROR;
    }

    p_uc = &g_uart_ccb[end_id];

    p_uc->gpUartTxAddress = txbuf;
    p_uc->gUartTxCnt = txnum;

    send_byte = *(p_uc->gpUartTxAddress);

    p_uc->gpUartTxAddress++;
    p_uc->gUartTxCnt--;


    USART_SendData(USARTx, send_byte);

    if(p_uc->gUartTxCnt)
        USART_ITConfig(USARTx, USART_IT_TXE, ENABLE);

    return OK;
};

/***********************************************************
pEndObj:  ���ͽӿ�
pMsgInfo: ������Ϣ����ͨ����·�㷢����Ϣ
************************************************************/
unsigned short End_send( P_MSG_INFO pMsgInfo)
{
    P_END_OBJ pEndObj;

    if( (( pEndObj = End_get_end_obj(pMsgInfo->msg_header.end_id)) == NULL) ||
            ( pMsgInfo->msg_header.msg_len == 0) )
    {
        // alan dynamic sending buffer.
        // ��Щ����ʧ��ʱ�����ﲻֱ���ͷ�sending buffer����end send �����ͷš�
        if( pMsgInfo->msg_header.need_buffer_free == OK)
        {
            free_send_buffer(pMsgInfo);
        }

        return ERROR;
    }


    if( END_IDLE != End_IsIdle(pEndObj) )
    {
        /* ����IDLE State, �����ݹ���queue ��*/
        enqueue(g_EndTxQueue[pEndObj->end_id], pMsgInfo);

        pMsgInfo->msg_header.time_stamp = OSTimeGet();

        return OK;
    }


    if( OK == End_uart_send( pEndObj->end_id, pMsgInfo->msg_buffer, pMsgInfo->msg_header.msg_len) )
    {

        // ����buffer ֱ�ӹ���end object �� ��������ֱ���ͷ�!
        pEndObj->pMsgInfo = pMsgInfo;

        // �ײ�״̬Ǩ�Ƶ�SENDING
        pEndObj->end_send_status = END_STATUS_SENDING;

        pMsgInfo->msg_header.block_state = SENDING;

        pEndObj->endStatistics.txPacketCount++;

        pMsgInfo->msg_header.time_stamp = OSTimeGet();

        
        return OK;
    }
    else
    {
        if( pMsgInfo->msg_header.need_buffer_free == OK)
        {
            free_send_buffer(pMsgInfo);
        }

        return ERROR;
    }

}

unsigned char End_check_recv(P_END_OBJ pEndObj)
{
    P_UART_CCB p_uc = &g_uart_ccb[pEndObj->end_id];

    // ʵʱ��¼��ǰUART Rx ���ݳ���
    pEndObj->last_receive_len = pEndObj->receive_len;

    //pEndObj->receive_len = gUart1RxCnt;
    if(p_uc->gpUartRxReadAddress <= p_uc->gpUartRxAddress)
        pEndObj->receive_len = p_uc->gpUartRxAddress - p_uc->gpUartRxReadAddress;//gIic0RxCnt;
    else
        pEndObj->receive_len = (USHORT)((ULONG)p_uc->gpUartRxAddress + UART_RECEIVE_BUF_SIZE - (ULONG)p_uc->gpUartRxReadAddress);

    if(pEndObj->receive_len > (220))//if(pEndObj->receive_len > (UART_RECEIVE_BUF_SIZE/2))
    {
        pEndObj->recv_timeout = 0;
        return OK;
    }


    if((pEndObj->receive_len != 0) && (pEndObj->receive_len == pEndObj->last_receive_len))
    {
        /* û���µĽ������ݵ���,ֱ�ӷ���ERROR �����ϲ���Դ�����*/
        pEndObj->recv_timeout++;
        if(pEndObj->recv_timeout > 1)
        {
            pEndObj->recv_timeout = 0;
            return OK;
        }
        else
        {
            return ERROR;
        }

    }
    else
    {
        /* �����µĽ�������*/
        pEndObj->recv_timeout = 0;
        return ERROR;
    }

}

/**********************************************

�жϵ�ǰEnd Send ���ݷ�����û��. �����귵��OK,  û�����귵��ERROR

**********************************************/
unsigned char End_check_send(UCHAR end_id)
{

    if( g_uart_ccb[end_id].gUartTxCnt == 0)
        /* �����������, ����OK �����ϲ�״̬Ǩ��*/
        return OK;
    else
        /* ��������û�������, ��������*/
        return ERROR;

}


/**************************************************************************************

��������:       �ײ㳭�����ݻ����ص�����. �����ｫ����д����ڴ�, ������һ�����д.

�������:      �س����ݽ��.

**************************************************************************************/
unsigned char End_postProcess(unsigned char end_type,  pvoid h)
{
    P_MSG_INFO pMsg = (P_MSG_INFO)h;
    USHORT iRet = ERROR;
    

    switch(end_type)
    {
    case RS485_COM_PORT:
        modbus_rtu_process(pMsg);
        break;

    default:
        break;
    }

    free_send_buffer(pMsg);

    return iRet;
}


// ��ѯ�ײ�end ��ǰ״̬�Ƿ����??
unsigned char End_IsIdle(P_END_OBJ pEndObj)
{

    if( pEndObj->end_send_status == END_STATUS_IDLE)
        return END_IDLE;
    else
        return END_BUSY;
    

}



