/**
  ******************************************************************************
  * @file    usb_core.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    06/19/2009
  * @brief   Host Core Layer
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
#include "usb_core.h"
#include "usb_hcd.h"
#include "usb_conf.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
USB_OTG_CORE_DEVICE   otgfs_dev1;
void BSP_Drive_VBUS  (uint8_t state);
static uint32_t GetMode(USB_OTG_CORE_DEVICE *pdev);
static void EnableCommonInt(USB_OTG_CORE_DEVICE *pdev);
static USB_OTG_Status SetID(USB_OTG_CORE_DEVICE *pdev);
static USB_OTG_Status USB_OTG_CoreReset(USB_OTG_CORE_DEVICE *pdev);
static uint32_t USB_OTG_ReadCommon_itr(USB_OTG_CORE_DEVICE *pdev);
extern uint32_t STM32_USBH_OTG_ISR_Handler  (USB_OTG_CORE_DEVICE *pdev);
extern uint32_t STM32_USBF_OTG_ISR_Handler (USB_OTG_CORE_DEVICE *pdev);
extern uint32_t STM32_USBO_OTG_ISR_Handler (USB_OTG_CORE_DEVICE *pdev);

#ifdef HOST_MODE_ENABLED
static void InitFSLSPClkSel(USB_OTG_CORE_DEVICE *pdev);
#endif

/* core configuration for this driver */
USB_OTG_CORE_CFGS otgfs1_cfg =
  {
    USB_OTG_FS1_BASE_ADDR,      /*  base_address   */
    MODE_HNP_SRP_CAPABLE,       /*  otg_cap        */
    0,                          /*  dma_enable     */
    0,                          /*  dma_burst_size */
    USB_OTG_SPEED_PARAM_FULL,   /*  speed          */
    0,                          /*  host_support_fs_ls_low_power */
    USB_OTG_HST_LS_LOW_POWER_PHY_CLK_48MHZ, /*    host_ls_low_power_phy_clk */
    1,                          /*  enable_dynamic_fifo */
    1280,                       /*  data_fifo_size   */
    RX_FIFO_SIZE,               /*  dev_rx_fifo_size */
    TX_FIFO_SIZE,               /*  dev_nperio_tx_fifo_size */
    {
      256,
      256,
      256,
      256,
      256,
      256,
      256,
      256
    },
    256,                        /*  host_rx_fifo_size */
    256,                        /*  host_nperio_tx_fifo_size */
    256,                        /*  host_perio_tx_fifo_size */
    64,                         /*  max_transfer_size */
    64,                         /*  max_packet_count */
    8,                          /*  host_channels */
    4,                          /*  dev_endpoints */
    USB_OTG_PHY_TYPE_PARAM_UTMI,/*  phy_type      */
    16,                          /*  phy_utmi_width */
    0,                           /*  phy_ulpi_ddr   */
    0,                           /*  phy_ulpi_ext_vbus */
    1,                           /*  i2c_enable      */
    0,                           /*  ulpi_fs_ls      */
    0,                           /*  ts_dline        */
    1,                           /* en_multiple_tx_fifo */
    {
      256,
      256,
      256,
      256
    },
    0,                           /* thr_ctl          */
    64,                          /* tx_thr_length    */
    64,                          /*  rx_thr_length   */
  };


/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Initialize the phy
  * @param  pdev : device instance
  * @retval Status
  */
USB_OTG_Status USB_OTG_PhyInit(USB_OTG_CORE_DEVICE *pdev)
{
  USB_OTG_gpio_data    gpioctl;
  USB_OTG_usb_cfg_data usbcfg;
  USB_OTG_Status status = USB_OTG_OK;
  /* Enable the I2C interface and deactivate the power down*/
  gpioctl.d32 = 0;
  gpioctl.b.pwdn = 1;
  gpioctl.b.i2cifen = 0;
  gpioctl.b.vbussensingA = 1 ;
  gpioctl.b.vbussensingB = 1 ;
  WRITE_REG32 (&pdev->regs.common_regs->gpio, gpioctl.d32);
  mDELAY(200);
  /* Program GUSBCFG.OtgUtmifsSel to I2C*/
  usbcfg.d32 = READ_REG32(&pdev->regs.common_regs->usb_cfg);
  usbcfg.b.otgutmifssel = 0; /* normally this should be 0 */
  WRITE_REG32 (&pdev->regs.common_regs->usb_cfg, usbcfg.d32);
  return status;
}


/**
  * @brief  Writes a packet into the Tx FIFO associated with the EP
  * @param  pdev : device instance
  * @param  src : source buffer
  * @param  ch_ep_num : channel index or EP index
  * @param  bytes : number of bytes to write.
  * @retval Status
  */
USB_OTG_Status USB_OTG_WritePacket(USB_OTG_CORE_DEVICE *pdev, uint8_t *src, uint8_t ch_ep_num, uint16_t bytes)
{
  USB_OTG_Status status = USB_OTG_OK;
  uint32_t dword_count , i;
  vuint32_t *fifo;
  uint32_t *data_buff = (uint32_t *)src;
  /* Find the DWORD length, padded by extra bytes as neccessary if MPS
   * is not a multiple of DWORD */
  dword_count =  (bytes + 3) / 4;
  fifo = pdev->regs.data_fifo[ch_ep_num];
  for (i = 0; i < dword_count; i++, data_buff++)
  {
    WRITE_REG32( fifo, *data_buff );
  }
  return status;
}


/**
  * @brief  Reads a packet from the Rx FIFO
  * @param  pdev : device instance
  * @param  dest : destination buffer
  * @param  bytes : number of bytes to read.
  * @retval status
  */
void* USB_OTG_ReadPacket(USB_OTG_CORE_DEVICE *pdev, uint8_t *dest, uint16_t bytes)
{
  uint32_t i;
  uint32_t word_count = (bytes + 3) / 4;
  vuint32_t *fifo = pdev->regs.data_fifo[0];
  uint32_t *data_buff = (uint32_t *)dest;
  for (i = 0; i < word_count; i++, data_buff++)
  {
    *data_buff = READ_REG32(fifo);
  }
  return ((void *)data_buff);
}


/**
  * @brief  initializes the commmon interrupts, used in both device and
  *   host modes
  * @param  pdev : device instance
  * @retval None
  */
static void EnableCommonInt(USB_OTG_CORE_DEVICE *pdev)
{
  USB_OTG_int_msk_data int_mask;
  int_mask.d32 = 0;
  /* Clear any pending USB_OTG Interrupts */
#ifndef DUAL_ROLE_MODE_ENABLED
  WRITE_REG32( &pdev->regs.common_regs->otg_int, 0xFFFFFFFF);
#endif
  /* Clear any pending interrupts */
  WRITE_REG32( &pdev->regs.common_regs->int_sts, 0xFFFFFFFF);
  /* Enable the interrupts in the INTMSK */
  int_mask.b.wkupintr = 1;
#ifdef DUAL_ROLE_MODE_ENABLED
  int_mask.b.otgintr = 1;
  int_mask.b.sessreqintr = 1;
  int_mask.b.conidstschng = 1;
  int_mask.b.portintr = 1;        /* Must be unmasked for HNP reason */
#endif
  WRITE_REG32( &pdev->regs.common_regs->int_msk, int_mask.d32);
}


/**
  * @brief  Initialize core registers address.
  * @param  pdev : device instance
  * @param  BaseAddress : Base Address for OTG core
  * @retval status
  */
USB_OTG_Status USB_OTG_SetAddress(USB_OTG_CORE_DEVICE *pdev, uint32_t BaseAddress)
{
  uint32_t i = 0;
  USB_OTG_Status status = USB_OTG_OK;
  pdev->regs.common_regs = (USB_OTG_common_regs *)(BaseAddress + USB_OTG_CORE_GLOBAL_REGS_OFFSET);
  pdev->regs.dev_regs =  (USB_OTG_dev_regs  *)  (BaseAddress + USB_OTG_DEV_GLOBAL_REG_OFFSET);
  for (i = 0; i < MAX_EPS_CHANNELS; i++)
  {
    pdev->regs.inep_regs[i]  = (USB_OTG_dev_in_ep_regs *)  (BaseAddress + USB_OTG_DEV_IN_EP_REG_OFFSET + (i * USB_OTG_EP_REG_OFFSET));
    pdev->regs.outep_regs[i] = (USB_OTG_dev_out_ep_regs *) (BaseAddress + USB_OTG_DEV_OUT_EP_REG_OFFSET + (i * USB_OTG_EP_REG_OFFSET));
  }
  pdev->regs.host_regs = (USB_OTG_host_regs *)(BaseAddress + USB_OTG_HOST_GLOBAL_REG_OFFSET);
  pdev->regs.hprt0 = (uint32_t *)(BaseAddress + USB_OTG_HOST_PORT_REGS_OFFSET);
  for (i = 0; i < MAX_EPS_CHANNELS; i++)
  {
    pdev->regs.hc_regs[i] = (USB_OTG_hc_regs *)(BaseAddress + USB_OTG_HOST_CHAN_REGS_OFFSET + (i * USB_OTG_CHAN_REGS_OFFSET));
  }
  for (i = 0; i < MAX_EPS_CHANNELS; i++)
  {
    pdev->regs.data_fifo[i] = (uint32_t *)(BaseAddress + USB_OTG_DATA_FIFO_OFFSET + (i * USB_OTG_DATA_FIFO_SIZE));
  }
  pdev->regs.pcgcctl = (uint32_t *)(BaseAddress + USB_OTG_PCGCCTL_OFFSET);
  return status;
}


/**
  * @brief  Initialize the USB_OTG controller registers and prepares the core
  *   device mode or host mode operation.
  * @param  pdev : device instance
  * @retval Status
  */
USB_OTG_Status USB_OTG_CoreInit(USB_OTG_CORE_DEVICE *pdev)
{
  USB_OTG_Status status = USB_OTG_OK;
  USB_OTG_usb_cfg_data usbcfg;
  usbcfg.d32 = 0;
  usbcfg.d32 = READ_REG32(&pdev->regs.common_regs->usb_cfg);
  usbcfg.b.physel = pdev->cfgs->phy_type; /* phy_type = 1*/
  WRITE_REG32 (&pdev->regs.common_regs->usb_cfg, usbcfg.d32);
  /* Reset after a PHY select and set Host mode */
  USB_OTG_CoreReset(pdev);
  /* init and configure the phy */
  USB_OTG_PhyInit(pdev);
  /* initialize OTG features */
#ifdef  DUAL_ROLE_MODE_ENABLED
  usbcfg.d32 = READ_REG32(&pdev->regs.common_regs->usb_cfg);
  usbcfg.b.hnpcap = 1;
  usbcfg.b.srpcap = 1;
  WRITE_REG32(&pdev->regs.common_regs->usb_cfg, usbcfg.d32);
  EnableCommonInt(pdev);
#endif
  /* Set Host or Device Mode following ID Line*/
  SetID(pdev);

  return status;
}


/**
  * @brief  Soft reset of the core
  * @param  pdev : device instance
  * @retval Status
  */
static USB_OTG_Status USB_OTG_CoreReset(USB_OTG_CORE_DEVICE *pdev)
{
  USB_OTG_Status status = USB_OTG_OK;
  volatile USB_OTG_rst_ctl_data greset;
  uint32_t count = 0;
  greset.d32 = 0;
  /* Wait for AHB master IDLE state. */
  do
  {
    uDELAY(3);
    greset.d32 = READ_REG32(&pdev->regs.common_regs->rst_ctl);
    if (++count > 200000)
    {
      return USB_OTG_OK;
    }
  }
  while (greset.b.ahbidle == 0);
  /* Core Soft Reset */
  count = 0;
  greset.b.csftrst = 1;
  WRITE_REG32(&pdev->regs.common_regs->rst_ctl, greset.d32 );
  do
  {
    greset.d32 = READ_REG32(&pdev->regs.common_regs->rst_ctl);
    if (++count > 200000)
    {
      break;
    }
  }
  while (greset.b.csftrst == 1);
  /* Wait for 3 PHY Clocks*/
  uDELAY(10);
  return status;
}


/**
  * @brief  Enables the controller's Global Int in the AHB Config reg
  * @param  pdev : device instance
  * @retval Status
  */
USB_OTG_Status USB_OTG_EnableGlobalInt(USB_OTG_CORE_DEVICE *pdev)
{
  USB_OTG_Status status = USB_OTG_OK;
  USB_OTG_ahb_cfg_data ahb_cfg;
  ahb_cfg.d32 = 0;
  ahb_cfg.b.glblintrmsk = 1; /* Enable interrupts */
  MODIFY_REG32(&pdev->regs.common_regs->ahb_cfg, 0, ahb_cfg.d32);
  return status;
}


/**
  * @brief  Enables the controller's Global Int in the AHB Config reg
  * @param  pdev : device instance
  * @retval Status
  */
USB_OTG_Status USB_OTG_DisableGlobalInt(USB_OTG_CORE_DEVICE *pdev)
{
  USB_OTG_Status status = USB_OTG_OK;
  USB_OTG_ahb_cfg_data ahbcfg;
  ahbcfg.d32 = 0;
  ahbcfg.b.glblintrmsk = 1; /* Enable interrupts */
  MODIFY_REG32(&pdev->regs.common_regs->ahb_cfg, ahbcfg.d32, 0);
  return status;
}


/**
  * @brief  Flush a Tx FIFO
  * @param  pdev : device instance
  * @param  num : Tx FIFO number
  * @retval status
  */
USB_OTG_Status USB_OTG_FlushTxFifo (USB_OTG_CORE_DEVICE *pdev , uint32_t num )
{
  USB_OTG_Status status = USB_OTG_OK;
  volatile USB_OTG_rst_ctl_data greset;
  int count = 0;
  greset.d32 = 0;
  greset.b.txfflsh = 1;
  greset.b.txfnum  = num;
  WRITE_REG32( &pdev->regs.common_regs->rst_ctl, greset.d32 );
  do
  {
    greset.d32 = READ_REG32( &pdev->regs.common_regs->rst_ctl);
    if (++count > 200000)
    {
      break;
    }
  }
  while (greset.b.txfflsh == 1);
  /* Wait for 3 PHY Clocks*/
  uDELAY(3);
  return status;
}


/**
  * @brief  Flush the Rx FIFO
  * @param  pdev : device instance
  * @retval status
  */
USB_OTG_Status USB_OTG_FlushRxFifo( USB_OTG_CORE_DEVICE *pdev )
{
  USB_OTG_Status status = USB_OTG_OK;
  volatile USB_OTG_rst_ctl_data greset;
  int count = 0;
  greset.d32 = 0;
  greset.b.rxfflsh = 1;
  WRITE_REG32( &pdev->regs.common_regs->rst_ctl, greset.d32 );
  do
  {
    greset.d32 = READ_REG32( &pdev->regs.common_regs->rst_ctl);
    if (++count > 200000)
    {
      break;
    }
  }
  while (greset.b.rxfflsh == 1);
  /* Wait for 3 PHY Clocks*/
  uDELAY(3);
  return status;
}


/**
  * @brief  Set Core Mode (force the role)
  * @param  pdev : device instance
  * @retval status
  */
USB_OTG_Status SetID(USB_OTG_CORE_DEVICE *pdev)
{
  USB_OTG_Status status = USB_OTG_OK;
  USB_OTG_usb_cfg_data usbcfg;
  usbcfg.d32 = READ_REG32(&pdev->regs.common_regs->usb_cfg);
#ifdef HOST_MODE_ENABLED
  usbcfg.b.force_host = 1;
#endif
#ifdef DEVICE_MODE_ENABLED
  usbcfg.b.force_dev = 1;
#endif
#ifdef DUAL_ROLE_MODE_ENABLED
  usbcfg.b.force_host = 0;
  usbcfg.b.force_dev = 0;
#endif
  WRITE_REG32(&pdev->regs.common_regs->usb_cfg, usbcfg.d32);
  mDELAY(50);
  return status;
}


/**
  * @brief  Get current mode
  * @param  pdev : device instance
  * @retval current mode
  */
static uint32_t GetMode(USB_OTG_CORE_DEVICE *pdev)
{
  return (READ_REG32(&pdev->regs.common_regs->int_sts ) & 0x1);
}


/**
  * @brief  Check if it is device mode
  * @param  pdev : device instance
  *   return         : 1 - device / 0 - Host
  */
uint8_t IsDeviceMode(USB_OTG_CORE_DEVICE *pdev)
{
  return (GetMode(pdev) != HOST_MODE);
}


/**
  * @brief  Check if it is host mode
  * @param  pdev : device instance
  * @retval num_in_ep
  */
uint8_t IsHostMode(USB_OTG_CORE_DEVICE *pdev)
{
  return (GetMode(pdev) == HOST_MODE);
}


/**
  * @brief  returns the Core Interrupt register
  * @param  pdev : device instance
  * @retval None
  */
uint32_t USB_OTG_ReadCoreItr(USB_OTG_CORE_DEVICE *pdev)
{
  uint32_t v;
  v = READ_REG32(&pdev->regs.common_regs->int_sts);
  v &= READ_REG32(&pdev->regs.common_regs->int_msk);
  return v;
}


/**
  * @brief  returns the USB_OTG Interrupt register
  * @param  pdev : device instance
  * @retval None
  */
uint32_t USB_OTG_ReadOtgItr (USB_OTG_CORE_DEVICE *pdev)
{
  return (READ_REG32 (&pdev->regs.common_regs->otg_int));
}
#ifdef HOST_MODE_ENABLED


/**
  * @brief  Initializes the USB_OTG controller for host mode
  * @param  pdev : device instance
  * @retval status
  */
USB_OTG_Status USB_OTG_CoreInitHost(USB_OTG_CORE_DEVICE *pdev)
{
  USB_OTG_Status              status = USB_OTG_OK;
  USB_OTG_fifo_size_data    nptxfifosize;
  USB_OTG_fifo_size_data  ptxfifosize;
  USB_OTG_OTG_ctl_data         gotgctl;
  USB_OTG_hc_char_data  hcchar;
  USB_OTG_host_cfg_data       hcfg;
  uint32_t               num_channels, i;
  gotgctl.d32 = 0;
  /* Restart the Phy Clock */
  WRITE_REG32(pdev->regs.pcgcctl, 0);
  /* Initialize Host Configuration Register */
  InitFSLSPClkSel(pdev);
  hcfg.d32 = READ_REG32(&pdev->regs.host_regs->host_cfg);
  hcfg.b.fslssupp = 1;
  WRITE_REG32(&pdev->regs.host_regs->host_cfg, hcfg.d32);
  /* Configure data FIFO sizes */
  /* Rx FIFO */
  WRITE_REG32(&pdev->regs.common_regs->rx_fifo_siz, 160);
  /* Non-periodic Tx FIFO */
  nptxfifosize.b.depth  = 160;
  nptxfifosize.b.startaddr = 160;
  WRITE_REG32(&pdev->regs.common_regs->np_tx_fifo_siz, nptxfifosize.d32);
  /* Periodic Tx FIFO */
  ptxfifosize.b.depth  = 128;
  ptxfifosize.b.startaddr = nptxfifosize.b.startaddr + nptxfifosize.b.depth;
  WRITE_REG32(&pdev->regs.common_regs->host_p_tx_fifo_siz, ptxfifosize.d32);
#ifdef DUAL_ROLE_MODE_ENABLED
  /* Clear Host Set HNP Enable in the USB_OTG Control Register */
  gotgctl.b.hstsethnpen = 1;
  MODIFY_REG32( &pdev->regs.common_regs->otg_ctl, gotgctl.d32, 0);
#endif
  /* Make sure the FIFOs are flushed. */
  USB_OTG_FlushTxFifo(pdev, 0x10 );         /* all Tx FIFOs */
  USB_OTG_FlushRxFifo(pdev);
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
  /* Halt all channels to put them into a known state. */
  for (i = 0; i < num_channels; i++)
  {
    hcchar.d32 = READ_REG32(&pdev->regs.hc_regs[i]->hc_char);
    hcchar.b.chen = 1;
    hcchar.b.chdis = 1;
    hcchar.b.epdir = 0;
    WRITE_REG32(&pdev->regs.hc_regs[i]->hc_char, hcchar.d32);
    do
    {
      hcchar.d32 = READ_REG32(&pdev->regs.hc_regs[i]->hc_char);
      uDELAY (20);
    }
    while (hcchar.b.chen);
  }
  /* Disable HALT interrupt Masks */
  for (i = 0; i < num_channels; i++)
  {
    USB_OTG_hc_int_msk_data hcintmsk;
    hcintmsk.d32 = READ_REG32(&pdev->regs.hc_regs[i]->hc_int_msk);
    hcintmsk.b.chhltd = 0;
    WRITE_REG32(&pdev->regs.hc_regs[i]->hc_int_msk , hcintmsk.d32);
  }
#ifndef DUAL_ROLE_MODE_ENABLED
  USB_OTG_DriveVbus(pdev, 1);       
#endif
  USB_OTG_EnableHostInt(pdev);
  return status;
}


/**
  * @brief  Enables the Host mode interrupts
  * @param  pdev : device instance
  * @retval status
  */
USB_OTG_Status USB_OTG_EnableHostInt(USB_OTG_CORE_DEVICE *pdev)
{
  USB_OTG_Status       status = USB_OTG_OK;
  USB_OTG_int_msk_data intr_mask;
  intr_mask.d32 = 0;
  /* Disable all interrupts. */
  WRITE_REG32(&pdev->regs.common_regs->int_msk, 0);
  /* Clear any pending interrupts. */
  WRITE_REG32(&pdev->regs.common_regs->int_sts, 0xFFFFFFFF);
  /* Enable the common interrupts */
  EnableCommonInt(pdev);
  intr_mask.b.rxstsqlvl = 1;
  MODIFY_REG32(&pdev->regs.common_regs->int_msk, intr_mask.d32, intr_mask.d32);
  return status;
}


/**
  * @brief  Enables the interrupts corresponding to the Host mode ONLY
  * @param  pdev : device instance
  * @retval status
  */
USB_OTG_Status  USB_OTG_EnableHostOnlyInt(USB_OTG_CORE_DEVICE *pdev)
{
  USB_OTG_Status       status = USB_OTG_OK;
  USB_OTG_int_msk_data intr_mask;
  intr_mask.d32 = 0;
  intr_mask.b.portintr = 1;
  intr_mask.b.hcintr = 1;
  intr_mask.b.ptxfempty = 1;
  MODIFY_REG32(&pdev->regs.common_regs->int_msk, intr_mask.d32, intr_mask.d32);
  return status;
}


/**
  * @brief  Disables the interrupts corresponding to the Host mode ONLY
  * @param  pdev : device instance
  * @retval status
  */
USB_OTG_Status  USB_OTG_DisableHostOnlyInt(USB_OTG_CORE_DEVICE *pdev)
{
  USB_OTG_Status       status = USB_OTG_OK;
  USB_OTG_int_msk_data intr_mask;
  intr_mask.d32 = 0;
  /* Disable host mode interrupts without disturbing common interrupts */
  intr_mask.b.sofintr    = 1;   
  intr_mask.b.portintr   = 1;
  intr_mask.b.hcintr     = 1;
  intr_mask.b.ptxfempty  = 1;
  intr_mask.b.disconnect = 1;    
  MODIFY_REG32(&pdev->regs.common_regs->int_msk, intr_mask.d32, 0);
  return status;
}


/**
  * @brief  Initializes the FSLSPClkSel field of the HCFG register
  *   depending on the PHY type
  * @param  pdev : device instance
  * @retval None
  */
static void InitFSLSPClkSel(USB_OTG_CORE_DEVICE *pdev)
{
  USB_OTG_host_cfg_data  hcfg;
  hcfg.d32 = READ_REG32(&pdev->regs.host_regs->host_cfg);
  hcfg.b.fslspclksel = HCFG_48_MHZ;
  WRITE_REG32(&pdev->regs.host_regs->host_cfg, hcfg.d32);
}


/**
  * @brief  Reads HPRT0 to modify later
  * @param  pdev : device instance
  * @retval hprt0 value
  */
uint32_t USB_OTG_ReadHPRT0(USB_OTG_CORE_DEVICE *pdev)
{
  USB_OTG_hprt0_data hprt0;
  hprt0.d32 = READ_REG32(pdev->regs.hprt0);
  hprt0.b.prtena = 0;
  hprt0.b.prtconndet = 0;
  hprt0.b.prtenchng = 0;
  hprt0.b.prtovrcurrchng = 0;
  return hprt0.d32;
}


/**
  * @brief  return HC interrupt status
  * @param  pdev : device instance
  * @retval HC interrupt status
  */
uint32_t USB_OTG_ReadHostAllChannels_intr (USB_OTG_CORE_DEVICE *pdev)
{
  return (READ_REG32 (&pdev->regs.host_regs->host_all_int));
}


/**
  * @brief  Reset Host Port
  * @param  pdev : device instance
  * @retval status
  */
uint32_t USB_OTG_ResetPort(USB_OTG_CORE_DEVICE *pdev)
{
  USB_OTG_hprt0_data hprt0;
  hprt0.d32 = USB_OTG_ReadHPRT0(pdev);
  hprt0.b.prtrst = 1;
  WRITE_REG32(pdev->regs.hprt0, hprt0.d32);
  mDELAY (3);                                /* See Note #1 */
  hprt0.b.prtrst = 0;
  WRITE_REG32(pdev->regs.hprt0, hprt0.d32);
  return 1;
}


/**
  * @brief  Prepares a host channel for transferring packets
  * @param  pdev : device instance
  * @param  hc : Host channel
  * @retval status
  */
USB_OTG_Status USB_OTG_HcInit(USB_OTG_CORE_DEVICE *pdev , USB_OTG_HC *hc)
{
  USB_OTG_Status status = USB_OTG_OK;
  uint32_t intr_enable;
  USB_OTG_hc_int_msk_data hc_intr_mask;
  USB_OTG_int_msk_data gintmsk;
  USB_OTG_hc_char_data hcchar;
  uint8_t hc_num;
  hc_num = hc->hc_num;
  gintmsk.d32 = 0;
  /* Clear old interrupt conditions for this host channel. */
  hc_intr_mask.d32 = 0xFFFFFFFF;
  hc_intr_mask.b.reserved = 0;
  WRITE_REG32(&pdev->regs.hc_regs[hc_num]->hc_int, hc_intr_mask.d32);
  /* Enable channel interrupts required for this transfer. */
  hc_intr_mask.d32 = 0;
  hc_intr_mask.b.xfercompl = 1;
  hc_intr_mask.b.ack = 1;
  hc_intr_mask.b.stall = 1;
  hc_intr_mask.b.nak = 1;
  hc_intr_mask.b.xacterr = 1;
  hc_intr_mask.b.datatglerr = 1;
  if (hc->ep_is_in)
  {
    hc_intr_mask.b.bblerr = 1;
  }
  WRITE_REG32(&pdev->regs.hc_regs[hc_num]->hc_int_msk, hc_intr_mask.d32);
  /* Enable the top level host channel interrupt. */
  intr_enable = (1 << hc_num);
  MODIFY_REG32(&pdev->regs.host_regs->host_all_int_msk, 0, intr_enable);
  /* Make sure host channel interrupts are enabled. */
  gintmsk.b.hcintr = 1;
  MODIFY_REG32(&pdev->regs.common_regs->int_msk, 0, gintmsk.d32);
  /*
   * Program the HCCHARn register with the endpoint characteristics for
   * the current transfer.
   */
  hcchar.d32 = 0;
  hcchar.b.devaddr = hc->dev_addr;
  hcchar.b.epnum   = hc->ep_num;
  hcchar.b.epdir   = hc->ep_is_in;
  hcchar.b.lspddev = (hc->speed == EP_SPEED_LOW);
  hcchar.b.eptype  = hc->ep_type;
  hcchar.b.mps     = hc->max_packet;
  if (hc->ep_type == HCCHAR_INTR)
  {
    hcchar.b.oddfrm  = 1;
  }
  WRITE_REG32(&pdev->regs.hc_regs[hc_num]->hc_char, hcchar.d32);
  return status;
}


/**
  * @brief  Start transfer
  * @param  pdev : device instance
  * @param  hc : host channel
  * @retval status
  */
USB_OTG_Status USB_OTG_StartXfer(USB_OTG_CORE_DEVICE *pdev , USB_OTG_HC *hc)
{
  USB_OTG_Status status = USB_OTG_OK;
  USB_OTG_hc_char_data hcchar;
  USB_OTG_hc_txfer_siz_data hctsiz;
  uint16_t num_packets;
  uint16_t max_hc_pkt_count;
  max_hc_pkt_count = 256;
  hctsiz.d32 = 0;
  /* Compute the expected number of packets associated to the transfer */
  if (hc->xfer_len > 0)
  {
    num_packets = (hc->xfer_len + hc->max_packet - 1) / hc->max_packet;
    if (num_packets > max_hc_pkt_count)
    {
      num_packets = max_hc_pkt_count;
      hc->xfer_len = num_packets * hc->max_packet;
    }
  }
  else
  {
    num_packets = 1;
  }
  if (hc->ep_is_in)
  {
    hc->xfer_len = num_packets * hc->max_packet;
  }
  /* Initialize the HCTSIZn register */
  hctsiz.b.xfersize = hc->xfer_len;
  hctsiz.b.pktcnt = num_packets;
  hctsiz.b.pid = hc->data_pid;
  WRITE_REG32(&pdev->regs.hc_regs[hc->hc_num]->hc_txfer_siz, hctsiz.d32);
  hcchar.d32 = READ_REG32(&pdev->regs.hc_regs[hc->hc_num]->hc_char);
  hcchar.b.multicnt = hc->multi_count;
  /* Set host channel enable */
  hcchar.b.chen = 1;
  hcchar.b.chdis = 0;
  WRITE_REG32(&pdev->regs.hc_regs[hc->hc_num]->hc_char, hcchar.d32);
  if (!hc->ep_is_in && hc->xfer_len > 0)
  {
    /* Load OUT packet into the appropriate Tx FIFO. */
    USB_OTG_WritePacket(pdev, hc->xfer_buff , hc->hc_num, hc->xfer_len);
  }
  return status;
}


/**
  * @brief  Halt channel
  * @param  pdev : device instance
  * @param  Host channel index
  * @retval status
  */
USB_OTG_Status USB_OTG_HcHalt(USB_OTG_CORE_DEVICE *pdev , uint8_t hc_num)
{
  USB_OTG_Status status = USB_OTG_OK;
  USB_OTG_np_tx_sts_data           nptxsts;
  USB_OTG_host_perio_tx_sts_data   hptxsts;
  USB_OTG_hc_char_data             hcchar;
  hcchar.d32 = READ_REG32(&pdev->regs.hc_regs[hc_num]->hc_char);
  hcchar.b.chen = 1;
  hcchar.b.chdis = 1;
  /* Check for space in the request queue to issue the halt. */
  if (hcchar.b.eptype == HCCHAR_CTRL || hcchar.b.eptype == HCCHAR_BULK)
  {
    nptxsts.d32 = READ_REG32(&pdev->regs.common_regs->np_tx_sts);
    if (nptxsts.b.nptxqspcavail == 0)
    {
      hcchar.b.chen = 0;
    }
  }
  else
  {
    hptxsts.d32 = READ_REG32(&pdev->regs.host_regs->host_p_tx_sts);
    if (hptxsts.b.ptxqspcavail == 0)
    {
      hcchar.b.chen = 0;
    }
  }
  WRITE_REG32(&pdev->regs.hc_regs[hc_num]->hc_char, hcchar.d32);
  return status;
}
#endif

#ifdef DEVICE_MODE_ENABLED
/**
  * @brief  Initializes the DevSpd field of the DCFG register depending
  *   on the PHY type and the enumeration speed of the device.
  * @param  pdev : device instance
  * @retval None
  */
static void InitDevSpeed(USB_OTG_CORE_DEVICE *pdev)
{
  USB_OTG_dev_cfg_data  dcfg;
  dcfg.d32 = READ_REG32(&pdev->regs.dev_regs->dev_cfg);
  dcfg.b.devspd = 0x3;  /* Full speed PHY */
  WRITE_REG32(&pdev->regs.dev_regs->dev_cfg, dcfg.d32);
}


/**
  * @brief  Initialize the USB_OTG controller registers for device mode
  * @param  pdev : device instance
  * @retval Status
  */
USB_OTG_Status USB_OTG_CoreInitDev (USB_OTG_CORE_DEVICE *pdev)
{
  USB_OTG_Status status = USB_OTG_OK;
  USB_OTG_dev_ep_ctl_data   depctl;
  uint32_t i;
  USB_OTG_dev_cfg_data   dcfg;
  USB_OTG_fifo_size_data nptxfifosize;
  USB_OTG_fifo_size_data txfifosize;
  USB_OTG_dev_in_ep_msk_data msk;
  dcfg.d32 = 0;
  /* Set device speed */
  InitDevSpeed (pdev);
  /* Restart the Phy Clock */
  WRITE_REG32(pdev->regs.pcgcctl, 0);
  /* Device configuration register */
  dcfg.d32 = READ_REG32( &pdev->regs.dev_regs->dev_cfg);
  dcfg.b.perfrint = DCFG_FRAME_INTERVAL_80;
  WRITE_REG32( &pdev->regs.dev_regs->dev_cfg, dcfg.d32 );
  /* set Rx FIFO size */
  WRITE_REG32( &pdev->regs.common_regs->rx_fifo_siz, 160/*pdev->cfgs->host_rx_fifo_size*/);
  /* Non-periodic Tx FIFO */
  nptxfifosize.b.depth     = DEV_NP_TX_FIFO_SIZE;
  nptxfifosize.b.startaddr = RX_FIFO_SIZE;
  WRITE_REG32( &pdev->regs.common_regs->np_tx_fifo_siz, nptxfifosize.d32 );
  txfifosize.b.depth = DEV_NP_TX_FIFO_SIZE;
  WRITE_REG32( &pdev->regs.common_regs->dev_p_tx_fsiz_dieptxf[0], txfifosize.d32 );
  txfifosize.b.startaddr += txfifosize.b.depth;
  txfifosize.b.startaddr = nptxfifosize.b.startaddr + nptxfifosize.b.depth;
  /* Flush the FIFOs */
  USB_OTG_FlushTxFifo(pdev , 0x10); /* all Tx FIFOs */
  USB_OTG_FlushRxFifo(pdev);
  /* Clear all pending Device Interrupts */
  WRITE_REG32( &pdev->regs.dev_regs->dev_in_ep_msk, 0 );
  WRITE_REG32( &pdev->regs.dev_regs->dev_out_ep_msk, 0 );
  WRITE_REG32( &pdev->regs.dev_regs->dev_all_int, 0xFFFFFFFF );
  WRITE_REG32( &pdev->regs.dev_regs->dev_all_int_msk, 0 );
  for (i = 0; i <= MAX_TX_FIFOS; i++)
  {
    depctl.d32 = READ_REG32(&pdev->regs.inep_regs[i]->dev_in_ep_ctl);
    if (depctl.b.epena)
    {
      depctl.d32 = 0;
      depctl.b.epdis = 1;
      depctl.b.snak = 1;
    }
    else
    {
      depctl.d32 = 0;
    }
    WRITE_REG32( &pdev->regs.inep_regs[i]->dev_in_ep_ctl, depctl.d32);
    WRITE_REG32( &pdev->regs.inep_regs[i]->dev_in_ep_txfer_siz, 0);
    WRITE_REG32( &pdev->regs.inep_regs[i]->dev_in_ep_int, 0xFF);
  }
  for (i = 0; i < 1/* NUM_OUT_EPS*/; i++)
  {
    USB_OTG_dev_ep_ctl_data depctl;
    depctl.d32 = READ_REG32(&pdev->regs.outep_regs[i]->dev_out_ep_ctl);
    if (depctl.b.epena)
    {
      depctl.d32 = 0;
      depctl.b.epdis = 1;
      depctl.b.snak = 1;
    }
    else
    {
      depctl.d32 = 0;
    }
    WRITE_REG32( &pdev->regs.outep_regs[i]->dev_out_ep_ctl, depctl.d32);
    WRITE_REG32( &pdev->regs.outep_regs[i]->dev_out_ep_txfer_siz, 0);
    WRITE_REG32( &pdev->regs.outep_regs[i]->dev_out_ep_int, 0xFF);
  }
  msk.d32 = 0;
  msk.b.txfifoundrn = 1;
  MODIFY_REG32(&pdev->regs.dev_regs->dev_in_ep_msk, msk.d32, msk.d32);
  USB_OTG_EnableDevInt(pdev);
  return status;
}


/**
  * @brief  Enables the Device mode interrupts
  * @param  pdev : device instance
  * @retval status
  */
USB_OTG_Status USB_OTG_EnableDevInt(USB_OTG_CORE_DEVICE *pdev)
{
  USB_OTG_Status status = USB_OTG_OK;
  USB_OTG_int_msk_data intr_mask;
  intr_mask.d32 = 0;
  /* Disable all interrupts. */
  WRITE_REG32( &pdev->regs.common_regs->int_msk, 0);
  /* Clear any pending interrupts */
  WRITE_REG32( &pdev->regs.common_regs->int_sts, 0xFFFFFFFF);
  /* Enable the common interrupts */
  EnableCommonInt(pdev);
  intr_mask.b.rxstsqlvl = 1;
  /* Enable interrupts matching to the Device mode ONLY */
  intr_mask.b.usbsuspend = 1;      
  intr_mask.b.usbreset   = 1;
  intr_mask.b.enumdone   = 1;
  intr_mask.b.inepintr   = 1;
  intr_mask.b.outepintr  = 1;
  MODIFY_REG32( &pdev->regs.common_regs->int_msk, intr_mask.d32, intr_mask.d32);
  return status;
}


/**
  * @brief  Enable the Device mode ONLY interrupts
  * @param  pdev : device instance
  * @retval status
  */
USB_OTG_Status  USB_OTG_EnableDevOnlyInt(USB_OTG_CORE_DEVICE *pdev)
{
  USB_OTG_Status status = USB_OTG_OK;
  USB_OTG_int_msk_data intr_mask;
  
  /* Enable interrupts matching to the Device mode ONLY */
  intr_mask.d32         = 0;
  intr_mask.b.usbreset  = 1;
  intr_mask.b.enumdone  = 1;
  intr_mask.b.inepintr  = 1;        
  intr_mask.b.outepintr = 1;         
  MODIFY_REG32( &pdev->regs.common_regs->int_msk, intr_mask.d32, intr_mask.d32);
  
  return status;   
}


/**
  * @brief  Disable the Device mode ONLY interrupts
  * @param  pdev : device instance
  * @retval status
  */
USB_OTG_Status  USB_OTG_DisableDevOnlyInt(USB_OTG_CORE_DEVICE *pdev)
{
  USB_OTG_Status status = USB_OTG_OK;
  USB_OTG_int_msk_data intr_mask;
  
  /* Disable interrupts matching to the Device mode ONLY */
  intr_mask.d32             = 0;
  intr_mask.b.erlysuspend   = 1;
  intr_mask.b.usbsuspend    = 1;  
  intr_mask.b.eopframe      = 1; 
  intr_mask.b.epmismatch    = 1;
  intr_mask.b.usbreset      = 1;
  intr_mask.b.enumdone      = 1;
  intr_mask.b.inepintr      = 1;
  intr_mask.b.outepintr     = 1;
  MODIFY_REG32( &pdev->regs.common_regs->int_msk, intr_mask.d32, 0);
  
  return status;   
    
}


/**
  * @brief  enables EP0 OUT to receive SETUP packets and configures EP0
  *   for transmitting packets
  * @param  pdev : device instance
  * @retval status
  */
USB_OTG_Status  USB_OTG_EP0Activate(USB_OTG_CORE_DEVICE *pdev)
{
  USB_OTG_Status          status = USB_OTG_OK;
  USB_OTG_dev_sts_data    dsts;
  USB_OTG_dev_ep_ctl_data diepctl;
  USB_OTG_dev_ctl_data    dctl;
  dctl.d32 = 0;
  /* Read the Device Status and Endpoint 0 Control registers */
  dsts.d32 = READ_REG32(&pdev->regs.dev_regs->dev_sts);
  diepctl.d32 = READ_REG32(&pdev->regs.inep_regs[0]->dev_in_ep_ctl);
  /* Set the MPS of the IN EP based on the enumeration speed */
  switch (dsts.b.enumspd)
  {
    case DSTS_ENUMSPD_HS_PHY_30MHZ_OR_60MHZ:
    case DSTS_ENUMSPD_FS_PHY_30MHZ_OR_60MHZ:
    case DSTS_ENUMSPD_FS_PHY_48MHZ:
      diepctl.b.mps = DEP0CTL_MPS_64;
      break;
    case DSTS_ENUMSPD_LS_PHY_6MHZ:
      diepctl.b.mps = DEP0CTL_MPS_8;
      break;
  }
  WRITE_REG32(&pdev->regs.inep_regs[0]->dev_in_ep_ctl, diepctl.d32);
  dctl.b.cgnpinnak = 1;
  MODIFY_REG32(&pdev->regs.dev_regs->dev_ctl, dctl.d32, dctl.d32);
  return status;
}


/**
  * @brief  Activates an EP
  * @param  pdev : device instance
  * @param  ep : endpoint
  * @retval num_in_ep
  */
USB_OTG_Status USB_OTG_EPActivate(USB_OTG_CORE_DEVICE *pdev , USB_OTG_EP *ep)
{
  USB_OTG_Status status = USB_OTG_OK;
  USB_OTG_dev_ep_ctl_data depctl;
  volatile uint32_t *addr;
  USB_OTG_dev_all_int_data daintmsk;
  daintmsk.d32 = 0;
  /* Read DEPCTLn register */
  if (ep->is_in == 1)
  {
    addr = &pdev->regs.inep_regs[ep->num]->dev_in_ep_ctl;
    daintmsk.ep.in = 1 << ep->num;
  }
  else
  {
    addr = &pdev->regs.outep_regs[ep->num]->dev_out_ep_ctl;
    daintmsk.ep.out = 1 << ep->num;
  }
  /* If the EP is already active don't change the EP Control
   * register. */
  depctl.d32 = READ_REG32(addr);
  if (!depctl.b.usbactep)
  {
    depctl.b.mps    = ep->maxpacket;
    depctl.b.eptype = ep->type;
    depctl.b.txfnum = ep->tx_fifo_num;
    depctl.b.setd0pid = 1;
    depctl.b.usbactep = 1;
    WRITE_REG32(addr, depctl.d32);
  }
  /* Enable the Interrupt for this EP */
  MODIFY_REG32(&pdev->regs.dev_regs->dev_all_int_msk, 0, daintmsk.d32);
  return status;
}


/**
  * @brief  Deactivates an EP
  * @param  pdev : device instance
  * @param  ep : endpoint
  * @retval num_in_ep
  */
USB_OTG_Status USB_OTG_EPDeactivate(USB_OTG_CORE_DEVICE *pdev , USB_OTG_EP *ep)
{
  USB_OTG_Status status = USB_OTG_OK;
  USB_OTG_dev_ep_ctl_data depctl;
  volatile uint32_t *addr;
  USB_OTG_dev_all_int_data daintmsk;
  depctl.d32 = 0;
  daintmsk.d32 = 0;
  /* Read DEPCTLn register */
  if (ep->is_in == 1)
  {
    addr = &pdev->regs.inep_regs[ep->num]->dev_in_ep_ctl;
    daintmsk.ep.in = 1 << ep->num;
  }
  else
  {
    addr = &pdev->regs.outep_regs[ep->num]->dev_out_ep_ctl;
    daintmsk.ep.out = 1 << ep->num;
  }
  depctl.b.usbactep = 0;
  WRITE_REG32(addr, depctl.d32);
  /* Disable the Interrupt for this EP */
  MODIFY_REG32(&pdev->regs.dev_regs->dev_all_int_msk, daintmsk.d32, 0);
  return status;
}


/**
  * @brief  Handle the setup for data xfer for an EP and starts the xfer
  * @param  pdev : device instance
  * @param  ep : endpoint
  * @retval status
  */
USB_OTG_Status USB_OTG_EPStartXfer(USB_OTG_CORE_DEVICE *pdev , USB_OTG_EP *ep)
{
  USB_OTG_Status status = USB_OTG_OK;
  USB_OTG_dev_ep_ctl_data depctl;
  USB_OTG_dev_ep_txfer_siz_data deptsiz;
  /* IN endpoint */
  if (ep->is_in == 1)
  {
    depctl.d32  = READ_REG32(&(pdev->regs.inep_regs[ep->num]->dev_in_ep_ctl));
    deptsiz.d32 = READ_REG32(&(pdev->regs.inep_regs[ep->num]->dev_in_ep_txfer_siz));
    /* Zero Length Packet? */
    if (ep->xfer_len == 0)
    {
      deptsiz.b.xfersize = 0;
      deptsiz.b.pktcnt = 1;
    }
    else
    {
      /* Program the transfer size and packet count
       * as follows: xfersize = N * maxpacket +
       * short_packet pktcnt = N + (short_packet
       * exist ? 1 : 0)
       */
      deptsiz.b.xfersize = ep->xfer_len;
      deptsiz.b.pktcnt = (ep->xfer_len - 1 + ep->maxpacket) / ep->maxpacket;
    }
    WRITE_REG32(&pdev->regs.inep_regs[ep->num]->dev_in_ep_txfer_siz, deptsiz.d32);
    if (ep->type != EP_TYPE_ISOC)
    {
      /* Enable the Tx FIFO Empty Interrupt for this EP */
      if (ep->xfer_len > 0)
      {
        uint32_t fifoemptymsk = 0;
        fifoemptymsk = 1 << ep->num;
        MODIFY_REG32(&pdev->regs.dev_regs->dev_fifo_empty_msk, 0, fifoemptymsk);
      }
    }
    /* EP enable, IN data in FIFO */
    depctl.b.cnak = 1;
    depctl.b.epena = 1;
    WRITE_REG32(&pdev->regs.inep_regs[ep->num]->dev_in_ep_ctl, depctl.d32);
    depctl.d32 = READ_REG32 (&pdev->regs.inep_regs[0]->dev_in_ep_ctl);
    depctl.b.nextep = ep->num;
    WRITE_REG32 (&pdev->regs.inep_regs[0]->dev_in_ep_ctl, depctl.d32);
  }
  else
  {
    /* OUT endpoint */
    depctl.d32  = READ_REG32(&(pdev->regs.outep_regs[ep->num]->dev_out_ep_ctl));
    deptsiz.d32 = READ_REG32(&(pdev->regs.outep_regs[ep->num]->dev_out_ep_txfer_siz));
    /* Program the transfer size and packet count as follows:
     * pktcnt = N
     * xfersize = N * maxpacket
     */
    if (ep->xfer_len == 0)
    {
      deptsiz.b.xfersize = ep->maxpacket;
      deptsiz.b.pktcnt = 1;
    }
    else
    {
      deptsiz.b.pktcnt = (ep->xfer_len + (ep->maxpacket - 1)) / ep->maxpacket;
      deptsiz.b.xfersize = deptsiz.b.pktcnt * ep->maxpacket;
    }
    WRITE_REG32(&pdev->regs.outep_regs[ep->num]->dev_out_ep_txfer_siz, deptsiz.d32);
    if (ep->type == EP_TYPE_ISOC)
    {
      if (ep->even_odd_frame)
      {
        depctl.b.setd1pid = 1;
      }
      else
      {
        depctl.b.setd0pid = 1;
      }
    }
    /* EP enable */
    depctl.b.cnak = 1;
    depctl.b.epena = 1;
    WRITE_REG32(&pdev->regs.outep_regs[ep->num]->dev_out_ep_ctl, depctl.d32);
  }
  return status;
}


/**
  * @brief  Handle the setup for a data xfer for EP0 and starts the xfer
  * @param  pdev : device instance
  * @param  ep : endpoint
  * @retval status
  */
USB_OTG_Status USB_OTG_EP0StartXfer(USB_OTG_CORE_DEVICE *pdev , USB_OTG_EP *ep)
{
  USB_OTG_Status                  status = USB_OTG_OK;
  uint32_t                         fifoemptymsk = 0;
  USB_OTG_dev_ep_ctl_data         depctl;
  USB_OTG_dev_ep_txfer_size0_data deptsiz;
  USB_OTG_dev_in_ep_regs          *in_regs ;
  /* IN endpoint */
  if (ep->is_in == 1)
  {
    in_regs = pdev->regs.inep_regs[0];
    depctl.d32  = READ_REG32(&in_regs->dev_in_ep_ctl);
    deptsiz.d32 = READ_REG32(&in_regs->dev_in_ep_txfer_siz);
    /* Zero Length Packet? */
    if (ep->xfer_len == 0)
    {
      deptsiz.b.xfersize = 0;
      deptsiz.b.pktcnt = 1;
    }
    else
    {
      if (ep->xfer_len > ep->maxpacket)
      {
        ep->xfer_len = ep->maxpacket;
        deptsiz.b.xfersize = ep->maxpacket;
      }
      else
      {
        deptsiz.b.xfersize = ep->xfer_len;
      }
      deptsiz.b.pktcnt = 1;
    }
    WRITE_REG32(&in_regs->dev_in_ep_txfer_siz, deptsiz.d32);
    /* EP enable, IN data in FIFO */
    depctl.b.cnak = 1;
    depctl.b.epena = 1;
    WRITE_REG32(&in_regs->dev_in_ep_ctl, depctl.d32);
    /* Enable the Tx FIFO Empty Interrupt for this EP */
    if (ep->xfer_len > 0)
    {
      fifoemptymsk |= 1 << ep->num;
      MODIFY_REG32(&pdev->regs.dev_regs->dev_fifo_empty_msk, 0, fifoemptymsk);
    }
  }
  else
  {
    /* OUT endpoint */
    depctl.d32  = READ_REG32(&pdev->regs.outep_regs[ep->num]->dev_out_ep_ctl);
    deptsiz.d32 = READ_REG32(&pdev->regs.outep_regs[ep->num]->dev_out_ep_txfer_siz);
    /* Program the transfer size and packet count as follows:
     * xfersize = N * (maxpacket + 4 - (maxpacket % 4))
     * pktcnt = N           */
    if (ep->xfer_len == 0)
    {
      deptsiz.b.xfersize = ep->maxpacket;
      deptsiz.b.pktcnt = 1;
    }
    else
    {
      deptsiz.b.pktcnt = (ep->xfer_len + (ep->maxpacket - 1)) / ep->maxpacket;
      deptsiz.b.xfersize = deptsiz.b.pktcnt * ep->maxpacket;
    }
    WRITE_REG32(&pdev->regs.outep_regs[ep->num]->dev_out_ep_txfer_siz, deptsiz.d32);
    /* EP enable */
    depctl.b.cnak = 1;
    depctl.b.epena = 1;
    WRITE_REG32 (&(pdev->regs.outep_regs[ep->num]->dev_out_ep_ctl), depctl.d32);
  }
  return status;
}


/**
  * @brief  Set the EP STALL
  * @param  pdev : device instance
  * @param  ep : endpoint
  * @retval Status
  */
USB_OTG_Status USB_OTG_EPSetStall(USB_OTG_CORE_DEVICE *pdev , USB_OTG_EP *ep)
{
  USB_OTG_Status status = USB_OTG_OK;
  USB_OTG_dev_ep_ctl_data depctl;
  vuint32_t *depctl_addr;
  if (ep->is_in == 1)
  {
    depctl_addr = &(pdev->regs.inep_regs[ep->num]->dev_in_ep_ctl);
    depctl.d32 = READ_REG32(depctl_addr);
    /* set the disable and stall bits */
    if (depctl.b.epena)
    {
      depctl.b.epdis = 1;
    }
    depctl.b.stall = 1;
    WRITE_REG32(depctl_addr, depctl.d32);
  }
  else
  {
    depctl_addr = &(pdev->regs.outep_regs[ep->num]->dev_out_ep_ctl);
    depctl.d32 = READ_REG32(depctl_addr);
    /* set the stall bit */
    depctl.b.stall = 1;
    WRITE_REG32(depctl_addr, depctl.d32);
  }
  return status;
}


/**
  * @brief  Clear the EP STALL
  * @param  pdev : device instance
  * @param  ep : endpoint
  * @retval Status
  */
USB_OTG_Status USB_OTG_EPClearStall(USB_OTG_CORE_DEVICE *pdev , USB_OTG_EP *ep)
{
  USB_OTG_Status status = USB_OTG_OK;
  USB_OTG_dev_ep_ctl_data depctl;
  vuint32_t *depctl_addr;
  if (ep->is_in == 1)
  {
    depctl_addr = &(pdev->regs.inep_regs[ep->num]->dev_in_ep_ctl);
  }
  else
  {
    depctl_addr = &(pdev->regs.outep_regs[ep->num]->dev_out_ep_ctl);
  }
  depctl.d32 = READ_REG32(depctl_addr);
  /* clear the stall bits */
  depctl.b.stall = 0;
  if (ep->type == EP_TYPE_INTR || ep->type == EP_TYPE_BULK)
  {
    depctl.b.setd0pid = 1; /* DATA0 */
  }
  WRITE_REG32(depctl_addr, depctl.d32);
  return status;
}


/**
  * @brief  returns the OUT endpoint interrupt bits
  * @param  pdev : device instance
  * @retval None
  */
uint32_t USB_OTG_ReadDevAllOutEp_itr(USB_OTG_CORE_DEVICE *pdev)
{
  uint32_t v;
  v  = READ_REG32(&pdev->regs.dev_regs->dev_all_int);
  v &= READ_REG32(&pdev->regs.dev_regs->dev_all_int_msk);
  return ((v & 0xffff0000) >> 16);
}


/**
  * @brief  returns the Device OUT EP Interrupt register
  * @param  pdev : device instance
  * @param  ep : endpoint
  * @retval None
  */
uint32_t USB_OTG_ReadDevOutEP_itr(USB_OTG_CORE_DEVICE *pdev , USB_OTG_EP *ep)
{
  uint32_t v;
  v  = READ_REG32(&pdev->regs.outep_regs[ep->num]->dev_out_ep_int);
  v &= READ_REG32(&pdev->regs.dev_regs->dev_out_ep_msk);
  return v;
}


/**
  * @brief  Get int status register
  * @param  pdev : device instance
  * @retval None
  */
uint32_t USB_OTG_ReadDevAllInEPItr(USB_OTG_CORE_DEVICE *pdev)
{
  uint32_t v;
  v = READ_REG32(&pdev->regs.dev_regs->dev_all_int);
  v &= READ_REG32(&pdev->regs.dev_regs->dev_all_int_msk);
  return (v & 0xffff);
}
#endif


/**
  * @brief  Global ISR
  * @param  None
  * @retval None
  */
void USB_OTGFS1_GlobalHandler(void)
{
  uint32_t retval = 0;
  USB_OTG_int_sts_data gintsts;
  USB_OTG_CORE_DEVICE *pdev = &otgfs_dev1;
  USB_OTG_hprt0_data   hprt0;
  gintsts.d32 = USB_OTG_ReadCommon_itr(pdev);
#ifdef DUAL_ROLE_MODE_ENABLED
  if (gintsts.b.portintr && IsDeviceMode(pdev))  
  {
    hprt0.d32 = USB_OTG_ReadHPRT0(pdev);
    hprt0.b.prtenchng = 1;
    //hprt0.b.prtconndet = 1;
    WRITE_REG32(pdev->regs.hprt0, hprt0.d32);
  }
  retval |= STM32_USBO_OTG_ISR_Handler (pdev);
  
  if (gintsts.b.disconnect && IsDeviceMode(pdev))  
  {
    USBO_ClrInput(USBO_INPUT_CONN);
    gintsts.d32 = 0;
    gintsts.b.disconnect = 1;  
    WRITE_REG32(&pdev->regs.common_regs->int_sts, gintsts.d32);
  }
  
  if (gintsts.b.usbsuspend && IsHostMode(pdev))  
  {
    USBO_SetInput(USBO_INPUT_BUS_SUSPEND);
    
    otgfs_dev1.OTG_State = A_SUSPEND;    
    gintsts.d32 = 0;
    gintsts.b.usbsuspend = 1;  
    WRITE_REG32(&pdev->regs.common_regs->int_sts, gintsts.d32);
  } 
#endif
  
#ifdef DEVICE_MODE_ENABLED
  retval |= STM32_USBF_OTG_ISR_Handler (pdev);
#endif
#ifdef HOST_MODE_ENABLED
  retval |= STM32_USBH_OTG_ISR_Handler (pdev);
#endif
}


/**
  * @brief  returns the Core Interrupt register
  * @param  pdev : device instance
  * @retval status
  */
static uint32_t USB_OTG_ReadCommon_itr (USB_OTG_CORE_DEVICE *pdev)
{
  USB_OTG_int_sts_data gintsts;
  USB_OTG_int_msk_data gintmsk;
  USB_OTG_int_msk_data gintmsk_common ;
  gintmsk_common.d32 = 0;
  /* ORG interrupts */
  gintmsk_common.b.sessreqintr = 1;
  gintmsk_common.b.conidstschng = 1;
  gintmsk_common.b.otgintr = 1;
  gintmsk_common.b.disconnect = 1;
  gintmsk_common.b.usbsuspend = 1;
  gintmsk_common.b.portintr = 1;
  gintmsk_common.b.wkupintr = 1;
  /* Power Management Interrupt */
  gintsts.d32 = READ_REG32(&pdev->regs.common_regs->int_sts);
  gintmsk.d32 = READ_REG32(&pdev->regs.common_regs->int_msk);
  
  gintmsk.b.disconnect = 1;
  gintmsk.b.usbsuspend = 1;
  gintmsk.b.portintr = 1;  
  
  return ((gintsts.d32 & gintmsk.d32 ) & gintmsk_common.d32);
}


/**
  * @brief  set/reset vbus
  * @param  pdev : device instance
  * @param  state : VBus state active or not
  * @retval status
  */
void USB_OTG_DriveVbus (USB_OTG_CORE_DEVICE *pdev, uint8_t state)
{
  USB_OTG_hprt0_data    hprt0;
  hprt0.d32 = 0;
  /* active the charge pump */
  BSP_Drive_VBUS(state);
  /* Turn on the Host port power. */
  hprt0.d32 = USB_OTG_ReadHPRT0(pdev);
  if ((hprt0.b.prtpwr == 0 ) && (state == 1 ))
  {
    hprt0.b.prtpwr = 1;
    WRITE_REG32(pdev->regs.hprt0, hprt0.d32);
  }
  if ((hprt0.b.prtpwr == 1 ) && (state == 0 ))
  {
    hprt0.b.prtpwr = 0;
    WRITE_REG32(pdev->regs.hprt0, hprt0.d32);
  }
  mDELAY(200);
}


/**
  * @brief  Initialize the USB_OTG controller registers for device mode
  *   The core is used as an OTG device which can act as a Host 
  *   or as a Device
  * @param  pdev : device instance
  * @retval Status
  */
void USB_OTGMode_CoreInitDev (USB_OTG_CORE_DEVICE *pdev)        
{
  USB_OTG_dev_ep_ctl_data       depctl;
  USB_OTG_dev_cfg_data          dcfg;
  USB_OTG_fifo_size_data        nptxfifosize;
  USB_OTG_fifo_size_data        txfifosize;
  USB_OTG_dev_in_ep_msk_data    msk;
  uint32_t i;
  /* Set device speed */
  InitDevSpeed (pdev);
  /* Restart the Phy Clock */
  WRITE_REG32(pdev->regs.pcgcctl, 0);
  /* Device configuration register */
  dcfg.d32 = 0;
  dcfg.d32 = READ_REG32( &pdev->regs.dev_regs->dev_cfg);
  dcfg.b.perfrint = DCFG_FRAME_INTERVAL_80;
  WRITE_REG32( &pdev->regs.dev_regs->dev_cfg, dcfg.d32 );
  /* set Rx FIFO size */
  WRITE_REG32( &pdev->regs.common_regs->rx_fifo_siz, 160);
  /* Non-periodic Tx FIFO size */
  nptxfifosize.b.depth     = 160;
  nptxfifosize.b.startaddr = 160;
  WRITE_REG32( &pdev->regs.common_regs->np_tx_fifo_siz, nptxfifosize.d32 );
  /* Periodic Tx FIFO size */
  txfifosize.b.depth = 160;
  WRITE_REG32( &pdev->regs.common_regs->dev_p_tx_fsiz_dieptxf[0], txfifosize.d32 );
  /* Clear all pending Device Interrupts */
  WRITE_REG32( &pdev->regs.dev_regs->dev_in_ep_msk, 0 );
  WRITE_REG32( &pdev->regs.dev_regs->dev_out_ep_msk, 0 );
  WRITE_REG32( &pdev->regs.dev_regs->dev_all_int, 0xFFFFFFFF );
  WRITE_REG32( &pdev->regs.dev_regs->dev_all_int_msk, 0 );
  /* Clear interrupts and configure registers related to Device IN Endpoints */
  for (i = 0; i <= MAX_TX_FIFOS; i++)   
  {
    depctl.d32 = READ_REG32(&pdev->regs.inep_regs[i]->dev_in_ep_ctl);
    if (depctl.b.epena)
    {
      depctl.d32 = 0;
      depctl.b.epdis = 1;
      depctl.b.snak = 1;
    }
    else
    {
      depctl.d32 = 0;
    }
    WRITE_REG32( &pdev->regs.inep_regs[i]->dev_in_ep_ctl, depctl.d32);
    WRITE_REG32( &pdev->regs.inep_regs[i]->dev_in_ep_txfer_siz, 0);
    WRITE_REG32( &pdev->regs.inep_regs[i]->dev_in_ep_int, 0xFF);
  }
  /* Clear interrupts and configure registers related to Device OUT Endpoints */
  for (i = 0; i < 1/* NUM_OUT_EPS*/; i++)
  {
    USB_OTG_dev_ep_ctl_data depctl;
    depctl.d32 = READ_REG32(&pdev->regs.outep_regs[i]->dev_out_ep_ctl);
    if (depctl.b.epena)
    {
      depctl.d32 = 0;
      depctl.b.epdis = 1;
      depctl.b.snak = 1;
    }
    else
    {
      depctl.d32 = 0;
    }
    WRITE_REG32( &pdev->regs.outep_regs[i]->dev_out_ep_ctl, depctl.d32);
    WRITE_REG32( &pdev->regs.outep_regs[i]->dev_out_ep_txfer_siz, 0);
    WRITE_REG32( &pdev->regs.outep_regs[i]->dev_out_ep_int, 0xFF);
  }
  /* Unmask Fifo Underrun interrupt for all Device IN Endpoints */
  msk.d32 = 0;
  msk.b.txfifoundrn = 1;
  MODIFY_REG32(&pdev->regs.dev_regs->dev_in_ep_msk, msk.d32, msk.d32);
  /* Enable Device mode only interrupts */
  USB_OTG_EnableDevOnlyInt(pdev);
}

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
