/**
  ******************************************************************************
  * @file    usb_hcd.h
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    06/19/2009
  * @brief   Host layer Header file
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
#ifndef __USB_HCD_H__
#define __USB_HCD_H__

/* Includes ------------------------------------------------------------------*/

#include "usb_regs.h"
#include "usb_core.h"

/* Exported functions ------------------------------------------------------- */
uint32_t HOST_Init       (USB_OTG_CORE_DEVICE *pdev);
uint32_t HOST_ChannelInit(USB_OTG_CORE_DEVICE *pdev, USB_OTG_HC *pHostChannel);
uint32_t HOST_StartXfer  (USB_OTG_CORE_DEVICE *pdev, USB_OTG_HC *pHostChannel);

#endif /* __USB_HCD_H__ */

/*********(C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE***************/
