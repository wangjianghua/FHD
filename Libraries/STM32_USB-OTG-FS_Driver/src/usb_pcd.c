/**
  ******************************************************************************
  * @file    usb_pcd.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    06/19/2009
  * @brief   Peripheral Device Interface Layer
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

/* Includes ------------------------------------------------------------------*/
#include "usb_core.h"
#include "usb_pcd.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
extern USB_OTG_CORE_DEVICE   otgfs_dev1;

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initialize the USB device on of the driver.
  * @param  pdev : device instance
  * @retval None
  */
void USB_OTG_USBD_Init(USB_OTG_CORE_DEVICE *pdev)
{
  uint32_t i;
  USB_OTG_EP *ep;
  ep = &pdev->dev.ep0;
  pdev->dev.ep0state = 0;
  /* Init ep structure */
  ep->num = 0;
  ep->tx_fifo_num = 0;
  /* Control until ep is actvated */
  ep->type = EP_TYPE_CTRL;
  ep->maxpacket = MAX_PACKET_SIZE;
  ep->xfer_buff = 0;
  ep->xfer_len = 0;
  for (i = 1; i < MAX_TX_FIFOS ; i++)
  {
    ep = &pdev->dev.in_ep[i-1];
    /* Init ep structure */
    ep->is_in = 1;
    ep->num = i;
    ep->tx_fifo_num = i;
    /* Control until ep is actvated */
    ep->type = EP_TYPE_CTRL;
    ep->maxpacket = MAX_PACKET_SIZE;
    ep->xfer_buff = 0;
    ep->xfer_len = 0;
  }
  for (i = 1; i < MAX_TX_FIFOS; i++)
  {
    ep = &pdev->dev.out_ep[i-1];
    /* Init ep structure */
    ep->is_in = 0;
    ep->num = i;
    ep->tx_fifo_num = i;
    /* Control until ep is activated */
    ep->type = EP_TYPE_CTRL;
    ep->maxpacket = MAX_PACKET_SIZE;
    ep->xfer_buff = 0;
    ep->xfer_len = 0;
  }
  pdev->dev.ep0.maxpacket = MAX_EP0_SIZE;
  pdev->dev.ep0.type = EP_TYPE_CTRL;
}


/**
  * @brief  Configure an EP
  * @param  pdev : Device instance
  * @param  epdesc : Endpoint Descriptor  
  * @retval status
  */
uint32_t USB_OTG_USBD_EP_Open(USB_OTG_CORE_DEVICE *pdev , EP_DESCRIPTOR *epdesc)
{
  USB_OTG_EP *ep;
  if ((0x80 & epdesc->bEndpointAddress) != 0)
  {
    ep = USB_OTG_USBD_GetInEP(pdev, epdesc->bEndpointAddress & 0x7F);
  }
  else
  {
    ep = USB_OTG_USBD_GetOutEP(pdev, epdesc->bEndpointAddress & 0x7F);
  }
  ep->num   = epdesc->bEndpointAddress & 0x7F;
  ep->is_in = (0x80 & epdesc->bEndpointAddress) != 0;
  ep->maxpacket = epdesc->wMaxPacketSize;
  ep->type = epdesc->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK;
  if (ep->is_in)
  {
    /* Assign a Tx FIFO */
    ep->tx_fifo_num = ep->num;
  }
  /* Set initial data PID. */
  if ((epdesc->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) == USB_ENDPOINT_XFER_BULK )
  {
    ep->data_pid_start = 0;
  }
  USB_OTG_EPActivate(pdev , ep );
  return 0;
}


/**
  * @brief  called when an EP is disabled
  * @param  pdev : device instance
  * @param  ep_addr : endpoint address
  * @retval status
  */
uint32_t USB_OTG_USBD_EP_Close(USB_OTG_CORE_DEVICE *pdev , uint8_t  ep_addr)
{
  USB_OTG_EP *ep;
  if ((0x80 & ep_addr) != 0)
  {
    ep = USB_OTG_USBD_GetInEP(pdev, ep_addr & 0x7F);
  }
  else
  {
    ep = USB_OTG_USBD_GetOutEP(pdev, ep_addr & 0x7F);
  }
  ep->num   = ep_addr & 0x7F;
  ep->is_in = (0x80 & ep_addr) != 0;
  USB_OTG_EPDeactivate(pdev , ep );
  return 0;
}


/**
  * @brief  Read data from Fifo
  * @param  pdev : device instance
  * @param  ep_addr : endpoint address
  * @param  pbuf : buffer adress
  * @param  buf_len : data length
  * @retval status
  */
uint32_t USB_OTG_USBD_EP_Read (   USB_OTG_CORE_DEVICE *pdev,
                             uint8_t   ep_addr,
                             uint8_t  *pbuf,
                             uint32_t  buf_len)
{
  USB_OTG_EP *ep;
  uint32_t i;
  ep = USB_OTG_USBD_GetOutEP(pdev, ep_addr & 0x7F);
  /* copy received data into application buffer */
  for (i = 0 ; i < buf_len ; i++)
  {
    pbuf[i] = ep->xfer_buff[i];
  }
  /*setup and start the Xfer */
  ep->xfer_buff = pbuf;
  ep->xfer_len = buf_len;
  ep->xfer_count = 0;
  ep->is_in = 0;
  ep->num = ep_addr & 0x7F;
  if ( ep->num == 0 )
  {
    USB_OTG_EP0StartXfer(pdev , ep);
  }
  else if (pdev->dev.ep0state == 0)
  {
    USB_OTG_EPStartXfer(pdev, ep );
  }
  return 0;
}


/**
  * @brief  Read data from Fifo
  * @param  pdev : device instance
  * @param  ep_addr : endpoint address
  * @param  pbuf : buffer adress
  * @param  buf_len : data length
  * @retval status
  */
uint32_t  USB_OTG_USBD_EP_Write ( USB_OTG_CORE_DEVICE *pdev,
                             uint8_t   ep_addr,
                             uint8_t  *pbuf,
                             uint32_t   buf_len)
{
  USB_OTG_EP *ep;
  ep = USB_OTG_USBD_GetInEP(pdev, ep_addr & 0x7f);
  /* Setup and start the Transfer */
  ep->xfer_buff = pbuf;
  ep->xfer_count = 0;
  ep->xfer_len = buf_len;
  ep->is_in = 1;
  ep->num = ep_addr & 0x7F;
  if ( ep->num == 0 )
  {
    USB_OTG_EP0StartXfer(pdev , ep);
  }
  else if (pdev->dev.ep0state == 0)
  {
    USB_OTG_EPStartXfer(pdev, ep );
  }
  return 0;
}


/**
  * @brief  Stall an endpoint.
  * @param  pdev : device instance
  * @param  epnum : endpoint index
  * @retval status
  */
uint32_t  USB_OTG_USBD_EP_Stall (USB_OTG_CORE_DEVICE *pdev, uint8_t   epnum)
{
  USB_OTG_EP *ep;
  if ((0x80 & epnum) != 0)
  {
    ep = USB_OTG_USBD_GetInEP(pdev, epnum & 0x7F);
  }
  else
  {
    ep = USB_OTG_USBD_GetOutEP(pdev, epnum & 0x7F);
  }
  ep->num   = epnum & 0x7F;
  ep->is_in = ((epnum & 0x80) == 0x80) ? 1 : 0;
  USB_OTG_EPSetStall(pdev , ep);
  return (0);
}


/**
  * @brief  Clear stall condition on endpoints.
  * @param  pdev : device instance
  * @param  epnum : endpoint index
  * @retval status
  */
uint32_t  USB_OTG_USBD_EP_ClrStall (USB_OTG_CORE_DEVICE *pdev, uint8_t epnum)
{
  USB_OTG_EP *ep;
  if ((0x80 & epnum) != 0)
  {
    ep = USB_OTG_USBD_GetInEP(pdev, epnum & 0x7F);
  }
  else
  {
    ep = USB_OTG_USBD_GetOutEP(pdev, epnum & 0x7F);
  }
  ep->num   = epnum & 0x7F;
  ep->is_in = ((epnum & 0x80) == 0x80) ? 1 : 0;
  USB_OTG_EPClearStall(pdev , ep);
  return (0);
}


/**
  * @brief  This Function flushes the buffer.
  * @param  pdev : device instance
  * @param  epnum : endpoint index
  * @retval status
  */
uint32_t  USB_OTG_USBD_EP_Flush (USB_OTG_CORE_DEVICE *pdev , uint8_t epnum)
{
  uint8_t  is_out;
  uint8_t  ep_nbr;
  ep_nbr   = epnum & 0x7F;
  is_out = ((epnum & 0x80) == 0x80) ? 0 : 1;
  if (is_out == 0)
  {
    USB_OTG_FlushTxFifo(pdev, ep_nbr);
  }
  else
  {
    USB_OTG_FlushRxFifo(pdev);
  }
  USB_OTG_USBD_EP_ClrStall(pdev, epnum);
  return (0);
}


/**
  * @brief  This Function set USB device address
  * @param  pdev : device instance
  * @param  address : USB Address
  * @retval none
  */
void  USB_OTG_USBD_EP_SetAddress (USB_OTG_CORE_DEVICE *pdev, uint8_t address)
{
  USB_OTG_dev_cfg_data dcfg;
  dcfg.d32 = 0;
  dcfg.b.devaddr = address;
  MODIFY_REG32( &pdev->regs.dev_regs->dev_cfg, 0, dcfg.d32);
}


/**
  * @brief  This function returns pointer to in ep struct with number ep_num
  * @param  pdev : device instance
  * @param  ep_num : endpoint index
  * @retval endpoint structure
  */
USB_OTG_EP* USB_OTG_USBD_GetInEP(USB_OTG_CORE_DEVICE *pdev , uint32_t ep_num)
{
  uint32_t i;
  if (ep_num == 0)
  {
    return &pdev->dev.ep0;
  }
  else
  {
    for (i = 0; i < MAX_TX_FIFOS; ++i)
    {
      if (pdev->dev.in_ep[i].num == ep_num)
        return &pdev->dev.in_ep[i];
    }
    return 0;
  }
}


/**
  * @brief  returns pointer to out ep struct with number ep_num
  * @param  pdev : device instance
  * @param  ep_num : endpoint index
  * @retval endpoint structure
  */
USB_OTG_EP* USB_OTG_USBD_GetOutEP(USB_OTG_CORE_DEVICE *pdev, uint32_t ep_num)
{
  uint32_t i;
  if (ep_num == 0)
  {
    return &pdev->dev.ep0;
  }
  else
  {
    for (i = 0; i < MAX_TX_FIFOS; ++i)
    {
      if (pdev->dev.out_ep[i].num == ep_num)
        return &pdev->dev.out_ep[i];
    }
    return 0;
  }
}


/**
  * @param  pdev : device instance
  * @retval None
  */
void  USB_OTG_USBD_DevConnect (USB_OTG_CORE_DEVICE *pdev)
{
#ifndef DUAL_ROLE_MODE_ENABLED
  USB_OTG_dev_ctl_data dctl;
  dctl.d32 = READ_REG32(&pdev->regs.dev_regs->dev_ctl);
  /* Connect device */
  dctl.b.sftdiscon  = 0;
  WRITE_REG32(&pdev->regs.dev_regs->dev_ctl, dctl.d32);
  mDELAY(3);
#endif  
}


/**
  * @brief  Disconnect device
  * @param  pdev : device instance
  * @retval status
  */
void  USB_OTG_USBD_DevDisconnect (USB_OTG_CORE_DEVICE *pdev)
{
#ifndef DUAL_ROLE_MODE_ENABLED
  USB_OTG_dev_ctl_data dctl;
  dctl.d32 = READ_REG32(&pdev->regs.dev_regs->dev_ctl);
  /* Disconnect device for 20ms */
  dctl.b.sftdiscon  = 1;
  WRITE_REG32(&pdev->regs.dev_regs->dev_ctl, dctl.d32);
  mDELAY(3);
#endif
}


/**
  * @brief  configures EPO to receive SETUP packets
  * @param  pdev : device instance
  * @retval None
  */
void USB_OTG_USBD_EP0_OutStart(USB_OTG_CORE_DEVICE *pdev)
{
  USB_OTG_dev_ep_txfer_size0_data doeptsize0;
  doeptsize0.d32 = 0;
  doeptsize0.b.supcnt = 3;
  doeptsize0.b.pktcnt = 1;
  doeptsize0.b.xfersize = 8 * 3;
  WRITE_REG32( &pdev->regs.outep_regs[0]->dev_out_ep_txfer_siz, doeptsize0.d32 );
}


/**
  * @brief  handles all USB Interrupts
  * @param  pdev : device instance
  * @retval None
  */

uint32_t STM32_USBF_OTG_ISR_Handler (USB_OTG_CORE_DEVICE *pdev)
{
  USB_OTG_int_sts_data gintr_status;
  uint32_t retval = 0;

  if (IsDeviceMode(pdev)) /* ensure that we are in device mode */
  {
    gintr_status.d32 = USB_OTG_ReadCoreItr(pdev);
    if (!gintr_status.d32)
    {
      return 0;
    }

    if (gintr_status.b.modemismatch)
    {
      USB_OTG_int_sts_data gintsts;

      /* Clear interrupt */
      gintsts.d32 = 0;
      gintsts.b.modemismatch = 1;
      WRITE_REG32(&pdev->regs.common_regs->int_sts, gintsts.d32);
    }

    if (gintr_status.b.ptxfempty)
    {
      retval |= USB_OTG_Handle_PTXFEmpty_ISR(pdev);
    }

    if (gintr_status.b.erlysuspend)
    {
      retval |= USB_OTG_Handle_ErleySuspend_ISR(pdev);
    }

    if (gintr_status.b.eopframe)
    {
      retval |= USB_OTG_Handle_EOPF_ISR(pdev);
    }

    if (gintr_status.b.nptxfempty)
    {
      retval |= USB_OTG_Handle_NPTxFE_ISR(pdev);
    }

    if (gintr_status.b.wkupintr)
    {
      retval |= USB_OTG_USBD_HandleWakeup_ISR(pdev);
    }
    if (gintr_status.b.usbsuspend)
    {
      retval |= USB_OTG_USBD_HandleUSBSuspend_ISR(pdev);
    }
    if (gintr_status.b.sofintr)
    {
      retval |= USB_OTG_USBD_HandleSof_ISR(pdev);

    }
    if (gintr_status.b.rxstsqlvl)
    {
      retval |= USB_OTG_USBD_HandleRxStatusQueueLevel_ISR(pdev);

    }
    if (gintr_status.b.usbreset)
    {
      retval |= USB_OTG_USBD_HandleUsbReset_ISR(pdev);

    }
    if (gintr_status.b.enumdone)
    {
      retval |= USB_OTG_USBD_HandleEnumDone_ISR(pdev);
    }
    if (gintr_status.b.inepint)
    {
      retval |= USB_OTG_USBD_HandleInEP_ISR(pdev);
    }

    if (gintr_status.b.outepintr)
    {
      retval |= USB_OTG_USBD_HandleOutEP_ISR(pdev);
    }
  }
  return retval;
}
/**
  * @brief  Indicates that the USB_OTG controller has detected an EOF
  * @param  pdev : device instance
  * @retval None
  */

static uint32_t USB_OTG_Handle_EOPF_ISR(USB_OTG_CORE_DEVICE *pdev )
{
  USB_OTG_int_sts_data gintsts;
  USB_OTG_int_msk_data gintmsk;
  gintmsk.d32 = 0;

  gintmsk.b.eopframe = 1;
  MODIFY_REG32(&pdev->regs.common_regs->int_msk, gintmsk.d32, 0 );

  /* Clear interrupt */
  gintsts.d32 = 0;
  gintsts.b.eopframe = 1;
  WRITE_REG32(&pdev->regs.common_regs->int_sts, gintsts.d32);
  return 1;
}

/**
  * @brief  Indicates that the USB_OTG controller has detected an early suspend
  * @param  pdev : device instance
  * @retval None
  */

static uint32_t USB_OTG_Handle_ErleySuspend_ISR(USB_OTG_CORE_DEVICE *pdev )
{
  USB_OTG_int_sts_data gintsts;
  USB_OTG_int_msk_data gintmsk;
  gintmsk.d32 = 0;

  gintmsk.b.erlysuspend = 1;
  MODIFY_REG32(&pdev->regs.common_regs->int_msk, gintmsk.d32, 0 );

  /* Clear interrupt */
  gintsts.d32 = 0;
  gintsts.b.erlysuspend = 1;
  WRITE_REG32(&pdev->regs.common_regs->int_sts, gintsts.d32);
  return 1;
}

/**
  * @brief  Handle non periodic TX FIFO empty interrupt
  * @param  pdev : device instance
  * @retval None
  */
  
static uint32_t USB_OTG_Handle_NPTxFE_ISR(USB_OTG_CORE_DEVICE *pdev )
{
  USB_OTG_int_sts_data gintsts;
  USB_OTG_int_msk_data gintmsk;
  gintmsk.d32 = 0;

  gintmsk.b.nptxfempty = 1;
  MODIFY_REG32(&pdev->regs.common_regs->int_msk, gintmsk.d32, 0 );

  /* Clear interrupt */
  gintsts.d32 = 0;
  gintsts.b.nptxfempty = 1;
  WRITE_REG32(&pdev->regs.common_regs->int_sts, gintsts.d32);
  return 1;
}

/**
  * @brief  Handle periodic TX FIFO empty interrupt
  * @param  pdev : device instance
  * @retval None
  */

static uint32_t USB_OTG_Handle_PTXFEmpty_ISR(USB_OTG_CORE_DEVICE *pdev )
{
  USB_OTG_int_sts_data gintsts;
  USB_OTG_int_msk_data gintmsk;
  gintmsk.d32 = 0;

  gintmsk.b.ptxfempty = 1;
  MODIFY_REG32(&pdev->regs.common_regs->int_msk, gintmsk.d32, 0 );

  /* Clear interrupt */
  gintsts.d32 = 0;
  gintsts.b.ptxfempty = 1;
  WRITE_REG32(&pdev->regs.common_regs->int_sts, gintsts.d32);
  return 1;
}

/**
  * @brief  Handle USB Wakeup interrupt
  * @param  pdev : device instance
  * @retval None
  */
  
static uint32_t USB_OTG_USBD_HandleWakeup_ISR(USB_OTG_CORE_DEVICE *pdev)
{
  USB_OTG_int_sts_data gintsts;
  USB_OTG_host_pcgcctl_data power;
  USB_OTG_dev_ctl_data dctl;
  USBF_DevResumeEvent(USBF_USB_OTG_DevPtr);

  /* restore full power */

  power.d32 = READ_REG32(&pdev->regs.pcgcctl);
  power.b.gatehclk = 0;
  power.b.stoppclk = 0;
  WRITE_REG32(&pdev->regs.pcgcctl, power.d32);

  dctl.d32 = 0 ;
  dctl.b.rmtwkupsig = 1;
  MODIFY_REG32(&pdev->regs.dev_regs->dev_ctl, dctl.d32, 0 );

  /* Clear interrupt */
  gintsts.d32 = 0;
  gintsts.b.wkupintr = 1;
  WRITE_REG32 (&pdev->regs.common_regs->int_sts, gintsts.d32);

  return 1;
}
/**
  * @brief  Handle Suspend interrupt
  * @param  pdev : device instance
  * @retval None
  */

static uint32_t USB_OTG_USBD_HandleUSBSuspend_ISR(USB_OTG_CORE_DEVICE *pdev)
{
  USB_OTG_int_sts_data gintsts;

  USBF_DevSuspendEvent(USBF_USB_OTG_DevPtr);

#ifndef DUAL_ROLE_MODE_ENABLED
  USB_OTG_host_pcgcctl_data power;
  USB_OTG_dev_sts_data dctl;

  dctl.d32 = READ_REG32 (&pdev->regs.dev_regs->dev_ctl);

  if(dctl.b.suspsts)
  {
     power.d32=0;
     power.b.stoppclk = 1;
     MODIFY_REG32(&pdev->regs.pcgcctl, 0, power.d32);
  }

  power.b.gatehclk = 1;
  MODIFY_REG32(&pdev->regs.pcgcctl, 0, power.d32);

#endif
  /* Clear interrupt */
  gintsts.d32 = 0;
  gintsts.b.usbsuspend = 1;
  WRITE_REG32(&pdev->regs.common_regs->int_sts, gintsts.d32);

  return 1;
}

/**
  * @brief  Handle IN EP interrupt
  * @param  pdev : device instance
  * @retval None
  */
  
static uint32_t USB_OTG_USBD_HandleInEP_ISR(USB_OTG_CORE_DEVICE *pdev)
{
  USB_OTG_dev_in_ep_int_data diepint;

  uint32_t ep_intr;
  uint32_t epnum = 0;
  USB_OTG_EP *ep;
  uint32_t fifoemptymsk;
  diepint.d32 = 0;
  ep_intr = USB_OTG_ReadDevAllInEPItr(pdev);
  while ( ep_intr )
  {
    if (ep_intr&0x1) /* In ITR */
    {
      ep = USB_OTG_USBD_GetInEP(pdev, epnum);
      diepint.d32 = USB_OTG_USBD_ReadDevInEP(pdev , ep); /* Get In ITR status */
      if ( diepint.b.xfercompl )
      {
        fifoemptymsk = 0x1 << ep->num;
        MODIFY_REG32(&pdev->regs.dev_regs->dev_fifo_empty_msk, fifoemptymsk, 0);
        CLEAR_IN_EP_INTR(epnum, xfercompl);
        USBF_EP_TxPktSent(USBF_USB_OTG_DevPtr, 0x80 | epnum);

      }
      if ( diepint.b.ahberr )
      {
        CLEAR_IN_EP_INTR(epnum, ahberr);
      }
      if ( diepint.b.timeout )
      {
        CLEAR_IN_EP_INTR(epnum, timeout);
      }
      if (diepint.b.intktxfemp)
      {
        CLEAR_IN_EP_INTR(epnum, intktxfemp);
      }
      if (diepint.b.intknepmis)
      {
        CLEAR_IN_EP_INTR(epnum, intknepmis);
      }
      if (diepint.b.inepnakeff)
      {
        CLEAR_IN_EP_INTR(epnum, inepnakeff);
      }
      if (diepint.b.emptyintr)
      {
        USB_OTG_USBD_WriteEmptyTxFifo(pdev , epnum);
        CLEAR_IN_EP_INTR(epnum, emptyintr);
      }
    }
    epnum++;
    ep_intr >>= 1;
  }

  return 1;
}

/**
  * @brief  Handle OUT EP interrupt
  * @param  pdev : device instance
  * @retval None
  */

static uint32_t USB_OTG_USBD_HandleOutEP_ISR(USB_OTG_CORE_DEVICE *pdev)
{
  uint32_t ep_intr;
  USB_OTG_dev_out_ep_int_data doepint;
  uint32_t epnum = 0;
  USB_OTG_EP *ep;

  doepint.d32 = 0;

  /* Read in the device interrupt bits */
  ep_intr = USB_OTG_ReadDevAllOutEp_itr(pdev);

  while ( ep_intr )
  {
    if (ep_intr&0x1)
    {
      /* Get EP pointer */
      ep = USB_OTG_USBD_GetOutEP(pdev , epnum);
      doepint.d32 = USB_OTG_ReadDevOutEP_itr(pdev, ep);

      /* Transfer complete */
      if ( doepint.b.xfercompl )
      {
        /* Clear the bit in DOEPINTn for this interrupt */
        CLEAR_OUT_EP_INTR(epnum, xfercompl);

        /* Inform upper layer: data ready */
        USBF_EP_RxPktRdy(USBF_USB_OTG_DevPtr, epnum);
      }
      /* Endpoint disable  */
      if ( doepint.b.epdisabled )
      {
        /* Clear the bit in DOEPINTn for this interrupt */
        CLEAR_OUT_EP_INTR(epnum, epdisabled);
      }
      /* AHB Error */
      if ( doepint.b.ahberr )
      {
        CLEAR_OUT_EP_INTR(epnum, ahberr);
      }
      /* Setup Phase Done (control EPs) */
      if ( doepint.b.setup )
      {

        /* inform the upper layer that a setup packet is available */
        USBF_EP_DoSetupPkt(USBF_USB_OTG_DevPtr, (void *)&pdev->dev.Data_Buffer[0]);
        CLEAR_OUT_EP_INTR(epnum, setup);
      }
    }
    epnum++;
    ep_intr >>= 1;
  }
  return 1;
}

/**
  * @brief  Handle SOF interrupt
  * @param  pdev : device instance
  * @retval None
  */

static uint32_t USB_OTG_USBD_HandleSof_ISR(USB_OTG_CORE_DEVICE *pdev)
{
  USB_OTG_int_sts_data int_sts;

  /* Clear interrupt */
  int_sts.d32 = 0;
  int_sts.b.sofintr = 1;
  WRITE_REG32 (&pdev->regs.common_regs->int_sts, int_sts.d32);

  return 1;
}


/**
  * @brief  Handle Rx FIFO empty interrupt
  * @param  pdev : device instance
  * @retval None
  */
  
static uint32_t USB_OTG_USBD_HandleRxStatusQueueLevel_ISR(USB_OTG_CORE_DEVICE *pdev)
{
  USB_OTG_int_msk_data int_mask;
  USB_OTG_dev_rx_sts_data status;
  USB_OTG_int_sts_data int_sts;
  USB_OTG_EP *ep;

  /* Disable the Rx Status Queue Level interrupt */
  int_mask.b.rxstsqlvl = 1;
  MODIFY_REG32( &pdev->regs.common_regs->int_msk, int_mask.d32, 0);

  /* Get the Status from the top of the FIFO */
  status.d32 = READ_REG32( &pdev->regs.common_regs->rx_stsp );

  ep = USB_OTG_USBD_GetOutEP(pdev , status.b.epnum);

  switch (status.b.pktsts)
  {
    case STS_GOUT_NAK:
      break;
    case STS_DATA_UPDT:
      if (status.b.bcnt)
      {
        USB_OTG_ReadPacket(pdev, pdev->dev.Data_Buffer, status.b.bcnt);
        ep->xfer_buff = pdev->dev.Data_Buffer;
        ep->xfer_len  = status.b.bcnt;
        ep->xfer_count += status.b.bcnt;
      }
      break;
    case STS_XFER_COMP:
      break;
    case STS_SETUP_COMP:
      break;
    case STS_SETUP_UPDT:
      /* Copy the setup packet received in Fifo into the setup buffer in RAM */
      USB_OTG_ReadPacket(pdev , pdev->dev.Data_Buffer, 8);
      ep->xfer_count += status.b.bcnt;
      ep->xfer_len  = status.b.bcnt;
      break;
    default:
      break;
  }

  /* Enable the Rx Status Queue Level interrupt */
  MODIFY_REG32( &pdev->regs.common_regs->int_msk, 0, int_mask.d32);
  /* Clear interrupt */
  int_sts.d32 = 0;
  int_sts.b.rxstsqlvl = 1;
  WRITE_REG32 (&pdev->regs.common_regs->int_sts, int_sts.d32);

  return 1;
}

/**
  * @brief  Handle Reset interrupt
  * @param  pdev : device instance
  * @retval None
  */
static uint32_t USB_OTG_USBD_HandleUsbReset_ISR(USB_OTG_CORE_DEVICE *pdev)
{
  USB_OTG_dev_all_int_data daintmsk;
  USB_OTG_dev_out_ep_msk_data doepmsk;
  USB_OTG_dev_in_ep_msk_data diepmsk;
  USB_OTG_dev_cfg_data dcfg;
  USB_OTG_dev_ctl_data dctl;
  USB_OTG_int_sts_data gintsts;
  uint32_t i;

  daintmsk.d32 = 0;
  doepmsk.d32 = 0;
  diepmsk.d32 = 0;
  dcfg.d32 = 0;
  dctl.d32 = 0;

  /* Clear the Remote Wakeup Signalling */
  dctl.b.rmtwkupsig = 1;
  MODIFY_REG32(&pdev->regs.dev_regs->dev_ctl, dctl.d32, 0 );

  /* Flush the NP Tx FIFO */
  USB_OTG_FlushTxFifo(pdev ,  0 );

  for (i = 0; i <= 8; i++)  /*synopsys Spec recommends clearing any before unmasking to avoid servicing old IT*/
  {
    WRITE_REG32( &pdev->regs.inep_regs[i]->dev_in_ep_int, 0xFF);
    WRITE_REG32( &pdev->regs.outep_regs[i]->dev_out_ep_int, 0xFF);
  }
  WRITE_REG32( &pdev->regs.dev_regs->dev_all_int, 0xFFFFFFFF );

  daintmsk.b.inep0 = 1;
  daintmsk.b.outep0 = 1;
  WRITE_REG32( &pdev->regs.dev_regs->dev_all_int_msk, daintmsk.d32 );

  doepmsk.b.setup = 1;
  doepmsk.b.xfercompl = 1;
  doepmsk.b.ahberr = 1;
  doepmsk.b.epdisabled = 1;
  WRITE_REG32( &pdev->regs.dev_regs->dev_out_ep_msk, doepmsk.d32 );

  diepmsk.b.xfercompl = 1;
  diepmsk.b.timeout = 1;
  diepmsk.b.epdisabled = 1;
  diepmsk.b.ahberr = 1;
  diepmsk.b.intknepmis = 1;
  WRITE_REG32( &pdev->regs.dev_regs->dev_in_ep_msk, diepmsk.d32 );

  /* Reset Device Address */
  dcfg.d32 = READ_REG32( &pdev->regs.dev_regs->dev_cfg);
  dcfg.b.devaddr = 0;
  WRITE_REG32( &pdev->regs.dev_regs->dev_cfg, dcfg.d32);


  /* setup EP0 to receive SETUP packets */
  USB_OTG_USBD_EP0_OutStart(pdev);

  /* Clear interrupt */
  gintsts.d32 = 0;
  gintsts.b.usbreset = 1;
  WRITE_REG32 (&pdev->regs.common_regs->int_sts, gintsts.d32);

  /* call stack to reset internal state machine */
  USBF_DevResetEvent(USBF_USB_OTG_DevPtr);
  return 1;
}

/**
  * @brief  Handle Enumeration done interrupt
  * @param  pdev : device instance
  * @retval None
  */
  
static uint32_t USB_OTG_USBD_HandleEnumDone_ISR(USB_OTG_CORE_DEVICE *pdev)
{
  USB_OTG_int_sts_data gintsts;
  USB_OTG_usb_cfg_data gusbcfg;

  USB_OTG_EP0Activate(pdev);

  /* Set USB turnaround time based on device speed and PHY interface. */
  gusbcfg.d32 = READ_REG32(&pdev->regs.common_regs->usb_cfg);

  /* Full or low speed */
  if ( USB_OTG_USBD_GetDeviceSpeed(pdev) == USB_SPEED_FULL)
  {
    gusbcfg.b.usbtrdtim = 9;
  }
  WRITE_REG32(&pdev->regs.common_regs->usb_cfg, gusbcfg.d32);

  /* Clear interrupt */
  gintsts.d32 = 0;
  gintsts.b.enumdone = 1;
  WRITE_REG32( &pdev->regs.common_regs->int_sts, gintsts.d32 );
  return 1;
}

/**
  * @brief  Get device speed
  * @param  pdev : device instance
  * @retval None
  */

static enum usb_device_speed USB_OTG_USBD_GetDeviceSpeed(USB_OTG_CORE_DEVICE *pdev)
{
  USB_OTG_dev_sts_data dsts;
  enum usb_device_speed speed = USB_SPEED_UNKNOWN;
  dsts.d32 = READ_REG32(&pdev->regs.dev_regs->dev_sts);

  switch (dsts.b.enumspd)
  {
    case DSTS_ENUMSPD_HS_PHY_30MHZ_OR_60MHZ:
      speed = USB_SPEED_HIGH;
      break;
    case DSTS_ENUMSPD_FS_PHY_30MHZ_OR_60MHZ:
    case DSTS_ENUMSPD_FS_PHY_48MHZ:
      speed = USB_SPEED_FULL;
      break;

    case DSTS_ENUMSPD_LS_PHY_6MHZ:
      speed = USB_SPEED_LOW;
      break;
  }

  return speed;
}
/**
  * @brief  Read device IN interrupt
  * @param  pdev : device instance
  * @retval None
  */
  
static uint32_t USB_OTG_USBD_ReadDevInEP(USB_OTG_CORE_DEVICE *pdev, USB_OTG_EP *ep)
{
  uint32_t v, msk, emp;
  msk = READ_REG32(&pdev->regs.dev_regs->dev_in_ep_msk);
  emp = READ_REG32(&pdev->regs.dev_regs->dev_fifo_empty_msk);
  msk |= ((emp >> ep->num) & 0x1) << 7;
  v = READ_REG32(&pdev->regs.inep_regs[ep->num]->dev_in_ep_int) & msk;
  return v;
}

/**
  * @brief  check Fifo for the next packet to be loaded
  * @param  pdev : device instance
  * @retval None
  */
  
static uint32_t USB_OTG_USBD_WriteEmptyTxFifo(USB_OTG_CORE_DEVICE *pdev, uint32_t epnum)
{
  USB_OTG_dev_tx_fifo_sts_data txstatus;
  USB_OTG_EP *ep;
  uint32_t len = 0;
  uint32_t dwords;
  txstatus.d32 = 0;

  ep = USB_OTG_USBD_GetInEP(pdev , epnum);

  len = ep->xfer_len - ep->xfer_count;

  if (len > ep->maxpacket)
  {
    len = ep->maxpacket;
  }

  dwords = (len + 3) / 4;
  txstatus.d32 = READ_REG32( &pdev->regs.inep_regs[epnum]->dev_tx_fifo_sts);



  while  (txstatus.b.txfspcavail > dwords &&
          ep->xfer_count < ep->xfer_len &&
          ep->xfer_len != 0)
  {
    /* Write the FIFO */
    len = ep->xfer_len - ep->xfer_count;

    if (len > ep->maxpacket)
    {
      len = ep->maxpacket;
    }
    dwords = (len + 3) / 4;

    USB_OTG_WritePacket (pdev , ep->xfer_buff, epnum, len);

    ep->xfer_count += len;

    txstatus.d32 = READ_REG32(&pdev->regs.inep_regs[epnum]->dev_tx_fifo_sts);
  }

  USBF_EP_PktXferlenUpdate(USBF_USB_OTG_DevPtr, epnum | 0x80, ep->xfer_len, USBF_ERR_NONE);
  return 1;
}

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
