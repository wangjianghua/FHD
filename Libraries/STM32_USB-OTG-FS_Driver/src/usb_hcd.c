/**
  ******************************************************************************
  * @file    usb_hcd.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    06/19/2009
  * @brief   Host Interface Layer
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
#include "stm32f10x.h"
#include "usb_core.h"
#include "usb_hcd.h"
#include "usb_conf.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Initialize the HOST portion of the driver.
  * @param  pdev : device instance
  * @retval Status
  */
uint32_t HOST_Init(USB_OTG_CORE_DEVICE *pdev)
{
#ifndef DUAL_ROLE_MODE_ENABLED
  USB_OTG_SetAddress(pdev, USB_OTG_FS1_BASE_ADDR); /* Initilialize the base address of the memory-mapped registers */
  USB_OTG_DisableGlobalInt(pdev);                  /* Disable the global interrupt in AHB Configuration register */
  USB_OTG_CoreInit(pdev);                          /* Initialize all the required registers for the Core */
  USB_OTG_CoreInitHost(pdev);                      /* Initialize the core as a Host */
  USB_OTG_EnableGlobalInt(pdev);                   /* Enable the global interrupt in AHB Configuration register */
#endif
  return 0;
}


/**
  * @brief  Initialize the HOST Channel for a specific transfer type
  * @param  pdev : device instance
  * @param  pHostChannel: pointer to USB_OTG_HC structure
  * @retval Status
  */
uint32_t HOST_ChannelInit(USB_OTG_CORE_DEVICE *pdev, USB_OTG_HC *pHostChannel)
{
  USB_OTG_HcInit(pdev, pHostChannel);
  return 0;
}


/**
  * @brief  Submit the USB request block (URB) to the right Host channel
  * @param  pHostChannel: pointer to USB_OTG_HC structure
  * @retval Status
  */
uint32_t HOST_StartXfer(USB_OTG_CORE_DEVICE *pdev, USB_OTG_HC *pHostChannel)
{
  USB_OTG_StartXfer(pdev , pHostChannel);
  return 0;
}

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
