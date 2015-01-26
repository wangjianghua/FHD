/**
  ******************************************************************************
  * @file    usb_core.h
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    06/19/2009
  * @brief   Header of the Core Layer
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2009 STMicroelectronics</center></h2>
  */ 
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USB_CORE_H__
#define __USB_CORE_H__

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "usb_regs.h"

/* Exported types ------------------------------------------------------------*/
typedef enum {
  USB_OTG_OK,
  USB_OTG_FAIL
}USB_OTG_Status;

typedef struct USB_OTG_hc
{
  uint8_t       hc_num;
  uint8_t       dev_addr ;
  uint8_t        ep_num;
  uint8_t       ep_is_in;
  uint8_t       speed;
  uint8_t       ep_type;
  uint16_t       max_packet;
  uint8_t       data_pid;
  uint16_t       multi_count;
  uint8_t        *xfer_buff;
  uint32_t       xfer_len;
}
USB_OTG_HC , *PUSB_OTG_HC;

typedef struct USB_OTG_ep
{
  uint8_t        num;
  uint8_t        is_in;
  uint32_t       tx_fifo_num;
  uint32_t       type;
  uint8_t        data_pid_start;
  uint8_t        even_odd_frame;
  uint32_t       maxpacket;
  uint8_t        *xfer_buff;
  uint32_t       xfer_len;
  uint32_t       xfer_count;
}
USB_OTG_EP , *PUSB_OTG_EP;

/** @defgroup USB_device_structures 
  * @{
  */ 
typedef struct USB_OTG_USBD
{
  USB_OTG_EP ep0;
  uint8_t     ep0state;
  USB_OTG_EP in_ep[ MAX_TX_FIFOS - 1];
  USB_OTG_EP out_ep[ MAX_TX_FIFOS - 1];
  uint8_t Data_Buffer[MAX_PACKET_SIZE];
}
USB_OTG_USBD_DEV , *USB_OTG_USBD_PDEV;
/**
  * @}
  */ 

/** @defgroup otg_core_structure 
  * @{
  */ 
typedef struct USB_OTG_core_cfg
{
  uint32_t       base_address;
  uint32_t       otg_cap;
  uint32_t       dma_enable;
  uint32_t       dma_burst_size;
  uint32_t       speed;
  uint32_t       host_support_fs_ls_low_power;
  uint32_t       host_ls_low_power_phy_clk;
  uint32_t       enable_dynamic_fifo;
  uint32_t       data_fifo_size;
  uint32_t       dev_rx_fifo_size;
  uint32_t       dev_nperio_tx_fifo_size;
  uint32_t       dev_perio_tx_fifo_size[MAX_PERIO_FIFOS];
  uint32_t       host_rx_fifo_size;
  uint32_t       host_nperio_tx_fifo_size;
  uint32_t       host_perio_tx_fifo_size;
  uint32_t       max_transfer_size;
  uint32_t       max_packet_count;
  uint32_t       host_channels;
  uint32_t       dev_endpoints;
  uint32_t       phy_type;
  uint32_t       phy_utmi_width;
  uint32_t       phy_ulpi_ddr;
  uint32_t       phy_ulpi_ext_vbus;
  uint32_t       i2c_enable;
  uint32_t       ulpi_fs_ls;
  uint32_t       ts_dline;
  uint32_t       en_multiple_tx_fifo;
  uint32_t       dev_tx_fifo_size[MAX_TX_FIFOS];
  uint32_t       thr_ctl;
  uint32_t       tx_thr_length;
  uint32_t       rx_thr_length;
}
USB_OTG_CORE_CFGS, *PUSB_OTG_CORE_CFGS;
typedef struct USB_OTG_device
{
  USB_OTG_CORE_CFGS    *cfgs;
  USB_OTG_CORE_REGS    regs;
  USB_OTG_USBD_DEV     dev;
  //USB_OTG_USBH_DEV     host;
  //USB_OTG_USBO_DEV     otg;
  uint32_t                   OTG_State; 
}
USB_OTG_CORE_DEVICE , *PUSB_OTG_CORE_DEVICE;
/**
  * @}
  */ 
/** @defgroup Internal_Macro's 
  * @{
  */ 
#define CLEAR_IN_EP_INTR(epnum,intr) \
  diepint.d32=0; \
  diepint.b.intr = 1; \
  WRITE_REG32(&otgfs_dev1.regs.inep_regs[epnum]->dev_in_ep_int,diepint.d32);
#define CLEAR_OUT_EP_INTR(epnum,intr) \
  doepint.d32=0; \
  doepint.b.intr = 1; \
  WRITE_REG32(&otgfs_dev1.regs.outep_regs[epnum]->dev_out_ep_int,doepint.d32);
#define READ_REG32(reg)  (*(vuint32_t *)reg)
#define WRITE_REG32(reg,value) (*(vuint32_t *)reg = value)
#define MODIFY_REG32(reg,clear_mask,set_mask) \
  WRITE_REG32(reg, (((READ_REG32(reg)) & ~clear_mask) | set_mask ) )
#define uDELAY(usec)  udelay(usec)
#define mDELAY(msec)  uDELAY(msec * 1000)
/**
  * @}
  */ 
/** @defgroup Delays_(this_can_be_changed_for_real_time_base) 
  * @{
  */ 
static void udelay (const uint32_t usec)
{
  uint32_t count = 0;
  const uint32_t utime = 72 * usec;
  do
  {
    if ( ++count > utime )
    {
      return ;
    }
  }
  while (1);
}
/**
  * @}
  */ 

/* Exported functions ------------------------------------------------------- */
USB_OTG_Status  USB_OTG_CoreInit        (USB_OTG_CORE_DEVICE *pdev);
USB_OTG_Status  USB_OTG_SetAddress      (USB_OTG_CORE_DEVICE *pdev, uint32_t BaseAddress);
USB_OTG_Status  USB_OTG_EnableGlobalInt (USB_OTG_CORE_DEVICE *pdev);
USB_OTG_Status  USB_OTG_DisableGlobalInt(USB_OTG_CORE_DEVICE *pdev);
USB_OTG_Status  USB_OTG_CoreInitHost        (USB_OTG_CORE_DEVICE *pdev);
void            USB_OTGMode_CoreInitHost    (USB_OTG_CORE_DEVICE *pdev);
USB_OTG_Status  USB_OTG_EnableHostInt       (USB_OTG_CORE_DEVICE *pdev);
USB_OTG_Status  USB_OTG_EnableHostOnlyInt   (USB_OTG_CORE_DEVICE *pdev);
USB_OTG_Status  USB_OTG_DisableHostOnlyInt  (USB_OTG_CORE_DEVICE *pdev);
void*  USB_OTG_ReadPacket           (USB_OTG_CORE_DEVICE *pdev , uint8_t *dest, uint16_t bytes);
USB_OTG_Status USB_OTG_WritePacket  (USB_OTG_CORE_DEVICE *pdev , uint8_t *src, uint8_t ch_ep_num, uint16_t bytes);
USB_OTG_Status  USB_OTG_HcInit      (USB_OTG_CORE_DEVICE *pdev , USB_OTG_HC *hc);
USB_OTG_Status  USB_OTG_StartXfer   (USB_OTG_CORE_DEVICE *pdev , USB_OTG_HC *hc);
uint32_t USB_OTG_ResetPort( USB_OTG_CORE_DEVICE *pdev);
uint32_t USB_OTG_ReadHPRT0                   (USB_OTG_CORE_DEVICE *pdev);
uint32_t USB_OTG_ReadDevAllInEPItr           (USB_OTG_CORE_DEVICE *pdev);
uint32_t USB_OTG_ReadCoreItr                 (USB_OTG_CORE_DEVICE *pdev);
uint32_t USB_OTG_ReadOtgItr                  (USB_OTG_CORE_DEVICE *pdev);
uint32_t USB_OTG_ReadHostAllChannels_intr    (USB_OTG_CORE_DEVICE *pdev);
uint8_t IsHostMode                           (USB_OTG_CORE_DEVICE *pdev);
uint8_t IsDeviceMode                         (USB_OTG_CORE_DEVICE *pdev);
USB_OTG_Status USB_OTG_HcInit           (USB_OTG_CORE_DEVICE *pdev , USB_OTG_HC *hc);
USB_OTG_Status USB_OTG_HcHalt           (USB_OTG_CORE_DEVICE *pdev , uint8_t hc_num);
USB_OTG_Status  USB_OTG_FlushTxFifo (USB_OTG_CORE_DEVICE *pdev , uint32_t num);
USB_OTG_Status  USB_OTG_FlushRxFifo (USB_OTG_CORE_DEVICE *pdev);
USB_OTG_Status  USB_OTG_SetHostMode (USB_OTG_CORE_DEVICE *pdev);
void            USB_OTG_DriveVbus   (USB_OTG_CORE_DEVICE *pdev, uint8_t state);
USB_OTG_Status  USB_OTG_PhyInit     (USB_OTG_CORE_DEVICE *pdev);
USB_OTG_Status  USB_OTG_HcStartXfer (USB_OTG_CORE_DEVICE *pdev , USB_OTG_HC *hc);
USB_OTG_Status  USB_OTG_CoreInitDev         (USB_OTG_CORE_DEVICE *pdev);
void            USB_OTGMode_CoreInitDev     (USB_OTG_CORE_DEVICE *pdev);
USB_OTG_Status  USB_OTG_EnableDevInt        (USB_OTG_CORE_DEVICE *pdev);
USB_OTG_Status  USB_OTG_EnableDevOnlyInt    (USB_OTG_CORE_DEVICE *pdev);
USB_OTG_Status  USB_OTG_DisableDevOnlyInt   (USB_OTG_CORE_DEVICE *pdev);
USB_OTG_Status  USB_OTG_EP0Activate (USB_OTG_CORE_DEVICE *pdev);
USB_OTG_Status  USB_OTG_EPActivate  (USB_OTG_CORE_DEVICE *pdev , USB_OTG_EP *ep);
USB_OTG_Status  USB_OTG_EPDeactivate(USB_OTG_CORE_DEVICE *pdev , USB_OTG_EP *ep);
USB_OTG_Status  USB_OTG_EPStartXfer (USB_OTG_CORE_DEVICE *pdev , USB_OTG_EP *ep);
USB_OTG_Status  USB_OTG_EP0StartXfer(USB_OTG_CORE_DEVICE *pdev , USB_OTG_EP *ep);
USB_OTG_Status  USB_OTG_EPSetStall          (USB_OTG_CORE_DEVICE *pdev , USB_OTG_EP *ep);
USB_OTG_Status  USB_OTG_EPClearStall        (USB_OTG_CORE_DEVICE *pdev , USB_OTG_EP *ep);
uint32_t             USB_OTG_ReadDevAllOutEp_itr (USB_OTG_CORE_DEVICE *pdev);
uint32_t             USB_OTG_ReadDevOutEP_itr    (USB_OTG_CORE_DEVICE *pdev , USB_OTG_EP *ep);
uint32_t             USB_OTG_ReadDevAllInEPItr   (USB_OTG_CORE_DEVICE *pdev);
void  USB_OTGFS1_GlobalHandler(void);
void USB_OTG_StopDeviceMode (USB_OTG_CORE_DEVICE *pdev);
void USB_OTG_StopHostMode   (USB_OTG_CORE_DEVICE *pdev);
void USB_OTGMode_CoreInitDev (USB_OTG_CORE_DEVICE *pdev);
void USB_OTG_MaskPortInterrupt (USB_OTG_CORE_DEVICE *pdev);
void USB_OTG_UnMaskPortInterrupt (USB_OTG_CORE_DEVICE *pdev); 

#endif

/*********(C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE***************/
