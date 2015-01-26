/**
  ******************************************************************************
  * @file    usb_pcd.h
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    06/19/2009
  * @brief   Peripheral Device Interface Layer Header file
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
#ifndef __USB_OTG_USBD_H__
#define __USB_OTG_USBD_H__

/* Includes ------------------------------------------------------------------*/
#include "usb_defines.h"
#include "usb_regs.h"

/* Exported constants --------------------------------------------------------*/
#define USB_ENDPOINT_XFER_CONTROL       0
#define USB_ENDPOINT_XFER_ISOC          1
#define USB_ENDPOINT_XFER_BULK          2
#define USB_ENDPOINT_XFER_INT           3
#define USB_ENDPOINT_XFERTYPE_MASK      3
/********************************************************************************
                              ENUMERATION TYPE
********************************************************************************/
enum usb_device_speed {
  USB_SPEED_UNKNOWN = 0,
  USB_SPEED_LOW, USB_SPEED_FULL,
  USB_SPEED_HIGH
};

/* Exported types ------------------------------------------------------------*/
/********************************************************************************
                              Data structure type
********************************************************************************/
typedef struct usb_ep_descriptor
{
  uint8_t  bLength;
  uint8_t  bDescriptorType;
  uint8_t  bEndpointAddress;
  uint8_t  bmAttributes;
  uint16_t wMaxPacketSize;
  uint8_t  bInterval;
}
EP_DESCRIPTOR , *PEP_DESCRIPTOR;

/* Exported functions ------------------------------------------------------- */
/********************************************************************************
                     EXPORTED FUNCTION FROM THE USB_OTG LAYER
********************************************************************************/
void  USB_OTG_USBD_Init(USB_OTG_CORE_DEVICE *pdev);
void  USB_OTG_USBD_DevConnect (USB_OTG_CORE_DEVICE *pdev);
void  USB_OTG_USBD_DevDisconnect (USB_OTG_CORE_DEVICE *pdev);
void  USB_OTG_USBD_EP_SetAddress (USB_OTG_CORE_DEVICE *pdev, uint8_t address);
uint32_t   USB_OTG_USBD_EP_Open(USB_OTG_CORE_DEVICE *pdev, EP_DESCRIPTOR *epdesc);
uint32_t   USB_OTG_USBD_EP_Close  (USB_OTG_CORE_DEVICE *pdev, uint8_t  ep_addr);
uint32_t   USB_OTG_USBD_EP_Read  ( USB_OTG_CORE_DEVICE *pdev, uint8_t  ep_addr, uint8_t  *pbuf, uint32_t   buf_len);
uint32_t   USB_OTG_USBD_EP_Write (USB_OTG_CORE_DEVICE *pdev,  uint8_t  ep_addr, uint8_t  *pbuf, uint32_t   buf_len);
uint32_t   USB_OTG_USBD_EP_Stall (USB_OTG_CORE_DEVICE *pdev, uint8_t   epnum);
uint32_t   USB_OTG_USBD_EP_ClrStall (USB_OTG_CORE_DEVICE *pdev, uint8_t epnum);
uint32_t   USB_OTG_USBD_EP_Flush (USB_OTG_CORE_DEVICE *pdev, uint8_t epnum);
uint32_t   USB_OTG_USBD_Handle_ISR(USB_OTG_CORE_DEVICE *pdev);
USB_OTG_EP* USB_OTG_USBD_GetOutEP(USB_OTG_CORE_DEVICE *pdev, uint32_t ep_num) ;
USB_OTG_EP* USB_OTG_USBD_GetInEP(USB_OTG_CORE_DEVICE *pdev, uint32_t ep_num);
void USB_OTG_USBD_EP0_OutStart(USB_OTG_CORE_DEVICE *pdev);

#endif /* __USB_OTG_USBD_H__ */

/*********(C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE***************/
