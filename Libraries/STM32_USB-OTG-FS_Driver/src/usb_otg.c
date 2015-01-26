/**
  ******************************************************************************
  * @file    usb_otg.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    06/19/2009
  * @brief   OTG Core Layer
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
#include "usb_defines.h"
#include "usb_regs.h"
#include "usb_core.h"
#include <stdio.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static uint32_t USB_OTG_HandleOTG_ISR(void);
static uint32_t USB_OTG_HandleConnectorIDStatusChange_ISR(void);
static uint32_t USB_OTG_HandleSessionRequest_ISR(void);
static uint32_t USBO_OTG_Read_itr(void);
static void USB_OTG_Stop (USB_OTG_CORE_DEVICE *pdev);
extern USB_OTG_CORE_DEVICE   otgfs_dev1;

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  OTG ISR
  * @param  None
  * @retval Status
  */
uint32_t STM32_USBO_OTG_ISR_Handler(void)
{
  uint32_t retval = 0;
  USB_OTG_int_sts_data gintsts;
  gintsts.d32 = USBO_OTG_Read_itr();
  if (gintsts.d32 == 0)
  {
    return 0;
  }
  if (gintsts.b.otgintr)
  {
    retval |= USB_OTG_HandleOTG_ISR();
  }
  if (gintsts.b.conidstschng)
  {
    retval |= USB_OTG_HandleConnectorIDStatusChange_ISR();
  }
  if (gintsts.b.sessreqintr)
  {
    retval |= USB_OTG_HandleSessionRequest_ISR();
  }
  return retval;
}


/**
  * @brief  returns the OTG Interrupt register
  * @param  None
  * @retval status
  */
static uint32_t USBO_OTG_Read_itr(void)
{
  USB_OTG_int_sts_data gintsts;
  USB_OTG_int_msk_data gintmsk;
  USB_OTG_int_msk_data gintmsk_common ;
  gintmsk_common.d32 = 0;
  /* ORG interrupts */
  gintmsk_common.b.sessreqintr = 1;
  gintmsk_common.b.conidstschng = 1;
  gintmsk_common.b.otgintr = 1;
  /* Power Management Interrupt */
  gintsts.d32 = READ_REG32(&otgfs_dev1.regs.common_regs->int_sts);
  gintmsk.d32 = READ_REG32(&otgfs_dev1.regs.common_regs->int_msk);
  return ((gintsts.d32 & gintmsk.d32 ) & gintmsk_common.d32);
}


/**
  * @brief  handles the OTG Interrupts
  * @param  None
  * @retval status
  */
static uint32_t USB_OTG_HandleOTG_ISR(void)
{
  USB_OTG_OTG_int_data gotgint;
  USB_OTG_OTG_ctl_data gotgctl;
  gotgint.d32 = READ_REG32(&otgfs_dev1.regs.common_regs->otg_int);
  gotgctl.d32 = READ_REG32(&otgfs_dev1.regs.common_regs->otg_ctl);
  if (gotgint.b.sesenddet)
  {
    gotgctl.d32 = READ_REG32(&otgfs_dev1.regs.common_regs->otg_ctl);
    /* B peripheral */
    if (IsDeviceMode(&otgfs_dev1))
    {
      /* inform upper layer here */
      
      /***************************/
      USB_OTG_Stop(&otgfs_dev1);
    }
    else if (IsHostMode(&otgfs_dev1))
    {
      /* inform upper layer here */
      /***************************/
    }
  }
  
  /* ----> SRP SUCCESS or FAILURE INTERRUPT <---- */
  if (gotgint.b.sesreqsucstschng)
  {
    gotgctl.d32 = READ_REG32(&otgfs_dev1.regs.common_regs->otg_ctl);
    if (gotgctl.b.sesreqscs)                                    /* Session request success                                          */
    {
      if (IsDeviceMode(&otgfs_dev1))
      {
      /* inform upper layer here */
      /***************************/
        printf("INFO:B-Device SRP Done.\n");
      }
      /* Clear Session Request */
      gotgctl.d32 = 0;
      gotgctl.b.sesreq = 1;
      MODIFY_REG32(&otgfs_dev1.regs.common_regs->otg_ctl, gotgctl.d32, 0);
    }
    else                                                        /* Session request failure                                          */
    {
      if (IsDeviceMode(&otgfs_dev1))
      {
      /* inform upper layer here */
      /***************************/
      }
    }
  }
  /* ----> HNP SUCCESS or FAILURE INTERRUPT <---- */
  if (gotgint.b.hstnegsucstschng)
  {
    gotgctl.d32 = READ_REG32(&otgfs_dev1.regs.common_regs->otg_ctl);
    if (gotgctl.b.hstnegscs)                                    /* Host negotiation success                                         */
    {
      if (IsHostMode(&otgfs_dev1))                              /* The core AUTOMATICALLY sets the Host mode                        */
      {                                                         
      /* inform upper layer here */
      /***************************/
        USB_OTG_DisableGlobalInt(&otgfs_dev1);
        USB_OTG_CoreInitHost(&otgfs_dev1);                      /* Init the core to work as a Host                                  */
        USB_OTG_EnableDevOnlyInt(&otgfs_dev1);                 /* Enable Device Only interrupts. Needed when DWC_Core B-Device switches back to Device mode after B-Device Host role */
        USB_OTG_DriveVbus(&otgfs_dev1, 1);                      /* DWC_Core must generate IT dev connected                          */
        USB_OTG_EnableGlobalInt(&otgfs_dev1);      
        otgfs_dev1.OTG_State = B_HOST;
      }
    }
    else                                                        /* Host negotiation failure */
    {
      printf("INFO: B-Dev Host Neg. init FAILED!\n");
      /* inform upper layer here */
      /***************************/
      gotgctl.d32 = 0;
      gotgctl.b.hnpreq = 1;                                   /* Clear the HNP request. The core clears GOTGCTL[HstNegScs] when GOTGCTL[HNPReq] is set   */
      MODIFY_REG32(&otgfs_dev1.regs.common_regs->otg_ctl, gotgctl.d32, 0);
    }
    gotgint.b.hstnegsucstschng = 1;                             /* Ack "Host Negotiation Success Status Change" interrupt.          */
  }
  /* ----> HOST NEGOTIATION DETECTED INTERRUPT <---- */
  if (gotgint.b.hstnegdet)
  {
    if (IsDeviceMode(&otgfs_dev1))                              /* The core AUTOMATICALLY sets the Host mode                        */
    {
      /* inform upper layer here */
      /***************************/
        USB_OTG_DisableGlobalInt(&otgfs_dev1);
        USB_OTG_Stop(&otgfs_dev1);                              /* Flush Tx/Rx Device FIFOs                                         */
        USB_OTGMode_CoreInitDev(&otgfs_dev1);
        USB_OTG_EnableHostOnlyInt(&otgfs_dev1);                 /* Enable Host Only interrupts. Needed when DWC_Core A-Device switches back to Host mode after A-Device Device role */
        USB_OTG_EnableGlobalInt(&otgfs_dev1);
        otgfs_dev1.OTG_State = A_PERIPHERAL;
    }
    else
    {
      printf("INFO: A-Dev Host Neg. A-dev always in Host mode!\n");
      /* inform upper layer here */
      /***************************/
    }
  }
  if (gotgint.b.adevtoutchng)
  {}
  if (gotgint.b.debdone)
  {
    USB_OTG_ResetPort(&otgfs_dev1);  
  }
  /* Clear OTG INT */
  WRITE_REG32(&otgfs_dev1.regs.common_regs->otg_int, gotgint.d32);
  return 1;
}


/**
  * @brief  handles the Connector ID Status Change Interrupt
  * @param  None
  * @retval status
  */
static uint32_t USB_OTG_HandleConnectorIDStatusChange_ISR(void)
{
  USB_OTG_int_sts_data gintsts ;
  USB_OTG_int_msk_data gintmsk ;
  USB_OTG_OTG_ctl_data gotgctl ;
  gintsts.d32 = 0 ;
  gintmsk.d32 = 0 ;
  gotgctl.d32 = 0 ;
  gintmsk.b.sofintr = 1;
  MODIFY_REG32(&otgfs_dev1.regs.common_regs->int_msk, gintmsk.d32, 0);
  gotgctl.d32 = READ_REG32(&otgfs_dev1.regs.common_regs->otg_ctl);
  /* B-Device connector (Device Mode) */
  if (gotgctl.b.conidsts)
  {
    USB_OTG_DisableGlobalInt(&otgfs_dev1);
    USB_OTG_CoreInitDev(&otgfs_dev1);
    USB_OTG_EnableGlobalInt(&otgfs_dev1);
      /* inform upper layer here */
      /***************************/
    otgfs_dev1.OTG_State = B_PERIPHERAL;
  }
  else
  {
    USB_OTG_DisableGlobalInt(&otgfs_dev1);
    USB_OTG_CoreInitHost(&otgfs_dev1);
    USB_OTG_EnableGlobalInt(&otgfs_dev1);
      /* inform upper layer here */
      /***************************/
    otgfs_dev1.OTG_State = A_HOST;
  }
  /* Set flag and clear interrupt */
  gintsts.b.conidstschng = 1;
  WRITE_REG32 (&otgfs_dev1.regs.common_regs->int_sts, gintsts.d32);
  return 1;
}


/**
  * @brief  initiating the Session Request Protocol
  * @param  None
  * @retval status
  */
static uint32_t USB_OTG_HandleSessionRequest_ISR(void)
{
  USB_OTG_int_sts_data gintsts;
  USB_OTG_OTG_ctl_data otg_ctl;
  otg_ctl.d32 = READ_REG32( &otgfs_dev1.regs.common_regs->otg_ctl );
  if (IsDeviceMode(&otgfs_dev1) && (otg_ctl.b.bsesvld))
  {
      /* inform upper layer here */
      /***************************/
  }
  else if (otg_ctl.b.asesvld)
  {
      /* inform upper layer here */
      /***************************/
  }
  /* Clear interrupt */
  gintsts.d32 = 0;
  gintsts.b.sessreqintr = 1;
  WRITE_REG32 (&otgfs_dev1.regs.common_regs->int_sts, gintsts.d32);
  return 1;
}


/**
  * @brief  Initiate an srp session
  * @param  None
  * @retval None
  */
void USB_OTG_InitiateSRP(void)
{
  USB_OTG_OTG_ctl_data otgctl;
  otgctl.d32 = READ_REG32( &otgfs_dev1.regs.common_regs->otg_ctl );
  if (otgctl.b.sesreq)
  {
    return; /* SRP in progress */
  }
  otgctl.b.sesreq = 1;
  WRITE_REG32(&otgfs_dev1.regs.common_regs->otg_ctl, otgctl.d32);
}


/**
  * @brief  Initiate HNP
  * @param  state : 0 - inactive / 1 : active
  * @param  mode : 1 : device mode / 1 : host mode
  * @retval None
  */
void USB_OTG_InitiateHNP(uint8_t state, uint8_t mode)
{
  USB_OTG_OTG_ctl_data  otgctl;
  USB_OTG_hprt0_data    hprt0;
  otgctl.d32 = READ_REG32( &otgfs_dev1.regs.common_regs->otg_ctl );
  if (mode)
  { /* Device mode */
    if (state)
    {
      USB_OTG_StopDeviceMode(&otgfs_dev1);      
      
      otgctl.b.devhnpen = 1; /* B-Dev has been enabled to perform HNP         */
      otgctl.b.hnpreq   = 1; /* Initiate an HNP req. to the connected USB host*/
      WRITE_REG32(&otgfs_dev1.regs.common_regs->otg_ctl, otgctl.d32);
    }
  }
  else
  { /* Host mode */
    if (state)
    {
      otgctl.b.hstsethnpen = 1; /* A-Dev has enabled B-device for HNP       */
      WRITE_REG32(&otgfs_dev1.regs.common_regs->otg_ctl, otgctl.d32);
      USB_OTG_DisableHostOnlyInt(&otgfs_dev1);      
      /* Suspend the bus so that B-dev will disconnect indicating the initial condition for HNP to DWC_Core */
      hprt0.d32  = USB_OTG_ReadHPRT0(&otgfs_dev1);
      hprt0.b.prtsusp = 1; /* The core clear this bit when disconnect interrupt generated (GINTSTS.DisconnInt = '1') */
      WRITE_REG32(otgfs_dev1.regs.hprt0, hprt0.d32);
    }
  }
}


/**
  * @brief  Stop Device
  * @param  pdev : device instance
  * @retval None
  */
static void USB_OTG_Stop (USB_OTG_CORE_DEVICE *pdev)
{
  USB_OTG_int_msk_data gintmsk = {.d32 = 0};
  /* Disable the NP Tx Fifo Empty Interrupt. */
  gintmsk.b.nptxfempty = 1;
  MODIFY_REG32(&pdev->regs.common_regs->int_msk, gintmsk.d32, 0 );
  /* Flush the Rx and Tx FIFOs */
  USB_OTG_FlushTxFifo(pdev ,  0x10 );
  USB_OTG_FlushRxFifo(pdev);
}


/**
  * @brief  Stop Device mode
  * @param  pdev : device instance
  * @retval None
  */
void USB_OTG_StopDeviceMode (USB_OTG_CORE_DEVICE *pdev)  // !!!! Added 
{
    int i;
  USB_OTG_int_msk_data gintmsk = {.d32 = 0};
  /* Disable the NP Tx Fifo Empty Interrupt. */
  gintmsk.b.nptxfempty = 1;
  MODIFY_REG32(&pdev->regs.common_regs->int_msk, gintmsk.d32, 0 );
  /* Mask all EP IN/OUT interrupt and clear all pending Device interrupts */  
  WRITE_REG32( &pdev->regs.dev_regs->dev_in_ep_msk, 0 );
  WRITE_REG32( &pdev->regs.dev_regs->dev_out_ep_msk, 0 );
  WRITE_REG32( &pdev->regs.dev_regs->dev_all_int_msk, 0 );
  WRITE_REG32( &pdev->regs.dev_regs->dev_all_int, 0xFFFFFFFF );
  for (i = 0; i <= 8; i++)  // ???? to check 
  {
    WRITE_REG32( &pdev->regs.inep_regs[i]->dev_in_ep_int, 0xFF);
    WRITE_REG32( &pdev->regs.outep_regs[i]->dev_out_ep_int, 0xFF);
  }
  /* Flush the Rx and Tx FIFOs */
  USB_OTG_FlushTxFifo(pdev ,  0x10 );
  USB_OTG_FlushRxFifo(pdev);
}


/**
  * @brief  Stop Host mode
  * @param  pdev : device instance
  * @retval None
  */
void USB_OTG_StopHostMode(USB_OTG_CORE_DEVICE *pdev)     
{
  USB_OTG_hc_char_data  hcchar;
  uint32_t               num_channels, i;
  /* Mask all Channel IN/OUT interrupt and clear all pending Host interrupts */ 
  WRITE_REG32(&pdev->regs.host_regs->host_all_int_msk , 0);
  WRITE_REG32(&pdev->regs.host_regs->host_all_int,      0xFFFFFFFF);
  /* Flush out any leftover queued requests. */
  num_channels = 8;
  for (i = 0; i < num_channels; i++)
  {
    hcchar.d32 = READ_REG32(&pdev->regs.hc_regs[i]->hc_char);
    hcchar.b.chen = 0;
    hcchar.b.chdis = 1;
    hcchar.b.epdir = 0;
    WRITE_REG32(&pdev->regs.hc_regs[i]->hc_char, hcchar.d32);
  }
}


/**
  * @brief  return to A host state
  * @param  pdev : device instance
  * @retval None
  */
void USB_OTG_Switchback (USB_OTG_CORE_DEVICE *pdev)
{
      /* inform upper layer here */
      /***************************/
     USB_OTG_DisableGlobalInt(&otgfs_dev1);					
     USB_OTG_CoreInitHost(&otgfs_dev1);                      /* Init the core to work as a Host                                  */
     USB_OTG_DriveVbus(&otgfs_dev1, 1);                      /* DWC_Core must generate IT dev connected                          */
     USB_OTG_EnableGlobalInt(&otgfs_dev1);   
     otgfs_dev1.OTG_State = A_HOST;
}


/**
  * @brief  return current OTG State
  * @param  pdev : device instance
  * @retval None
  */
uint32_t USB_OTG_GetCurrentState (USB_OTG_CORE_DEVICE *pdev)
{
 return pdev->OTG_State;
}

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
