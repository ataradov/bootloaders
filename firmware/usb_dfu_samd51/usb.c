// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2023, Alex Taradov <alex@taradov.com>. All rights reserved.

/*- Includes ----------------------------------------------------------------*/
#include <string.h>
#include <stdbool.h>
#include <stdalign.h>
#include "samd51.h"
#include "utils.h"
#include "nvm_data.h"
#include "usb.h"
#include "usb_std.h"
#include "usb_dfu.h"
#include "usb_descriptors.h"

/*- Definitions -------------------------------------------------------------*/
enum
{
  USB_DEVICE_EPCFG_EPTYPE_DISABLED    = 0,
  USB_DEVICE_EPCFG_EPTYPE_CONTROL     = 1,
  USB_DEVICE_EPCFG_EPTYPE_ISOCHRONOUS = 2,
  USB_DEVICE_EPCFG_EPTYPE_BULK        = 3,
  USB_DEVICE_EPCFG_EPTYPE_INTERRUPT   = 4,
  USB_DEVICE_EPCFG_EPTYPE_DUAL_BANK   = 5,
};

enum
{
  USB_DEVICE_PCKSIZE_SIZE_8    = 0,
  USB_DEVICE_PCKSIZE_SIZE_16   = 1,
  USB_DEVICE_PCKSIZE_SIZE_32   = 2,
  USB_DEVICE_PCKSIZE_SIZE_64   = 3,
  USB_DEVICE_PCKSIZE_SIZE_128  = 4,
  USB_DEVICE_PCKSIZE_SIZE_256  = 5,
  USB_DEVICE_PCKSIZE_SIZE_512  = 6,
  USB_DEVICE_PCKSIZE_SIZE_1023 = 7,
};

/*- Types -------------------------------------------------------------------*/
typedef struct
{
  UsbDeviceDescBank  out;
  UsbDeviceDescBank  in;
} usb_ep_t;

/*- Variables ---------------------------------------------------------------*/
static alignas(4) usb_ep_t usb_ep;
static alignas(4) uint8_t usb_ctrl_out_buf[USB_CONTROL_EP_SIZE];
static int usb_setup_length = 0;

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
void usb_init(void)
{
  PORT->Group[0].PINCFG[24].reg |= PORT_PINCFG_PMUXEN;
  PORT->Group[0].PINCFG[25].reg |= PORT_PINCFG_PMUXEN;
  PORT->Group[0].PMUX[24>>1].reg = PORT_PMUX_PMUXO(MUX_PA25H_USB_DP) |
      PORT_PMUX_PMUXE(MUX_PA24H_USB_DM);

  MCLK->APBBMASK.reg |= MCLK_APBBMASK_USB;

  GCLK->PCHCTRL[USB_GCLK_ID].reg = GCLK_PCHCTRL_GEN(0) | GCLK_PCHCTRL_CHEN;
  while (0 == (GCLK->PCHCTRL[USB_GCLK_ID].reg & GCLK_PCHCTRL_CHEN));

  OSCCTRL->DFLLMUL.reg = OSCCTRL_DFLLMUL_MUL(48000);

  OSCCTRL->DFLLCTRLB.reg = OSCCTRL_DFLLCTRLB_MODE | OSCCTRL_DFLLCTRLB_USBCRM |
      OSCCTRL_DFLLCTRLB_CCDIS;
  OSCCTRL->DFLLCTRLA.reg = OSCCTRL_DFLLCTRLA_ENABLE | OSCCTRL_DFLLCTRLA_RUNSTDBY;

  int tn = NVM_READ_CAL(NVM_USB_TRANSN);
  int tp = NVM_READ_CAL(NVM_USB_TRANSP);
  int tr = NVM_READ_CAL(NVM_USB_TRIM);

  USB->DEVICE.PADCAL.reg  = USB_PADCAL_TRANSP(tp) | USB_PADCAL_TRANSN(tn) | USB_PADCAL_TRIM(tr);
  USB->DEVICE.DESCADD.reg = (uint32_t)&usb_ep;
  USB->DEVICE.CTRLB.reg   = USB_DEVICE_CTRLB_SPDCONF_FS;
  USB->DEVICE.CTRLA.reg   = USB_CTRLA_ENABLE | USB_CTRLA_MODE_DEVICE;
}

//-----------------------------------------------------------------------------
void usb_set_address(int address)
{
  USB->DEVICE.DADD.reg = USB_DEVICE_DADD_ADDEN | USB_DEVICE_DADD_DADD(address);
}

//-----------------------------------------------------------------------------
void usb_control_stall(void)
{
  USB->DEVICE.DeviceEndpoint[0].EPSTATUSSET.bit.STALLRQ1 = 1;
}

//-----------------------------------------------------------------------------
void usb_control_send(uint8_t *data, int size)
{
  if (size > usb_setup_length)
    size = usb_setup_length;

  usb_ep.in.ADDR.reg = (uint32_t)data;
  usb_ep.in.PCKSIZE.bit.BYTE_COUNT = size;

  USB->DEVICE.DeviceEndpoint[0].EPINTFLAG.reg = USB_DEVICE_EPINTFLAG_TRCPT1;
  USB->DEVICE.DeviceEndpoint[0].EPSTATUSSET.bit.BK1RDY = 1;

  while (0 == USB->DEVICE.DeviceEndpoint[0].EPINTFLAG.bit.TRCPT1);
}

//-----------------------------------------------------------------------------
void usb_control_send_zlp(void)
{
  usb_control_send(NULL, 0);
}

//-----------------------------------------------------------------------------
void usb_task(void)
{
  if (USB->DEVICE.INTFLAG.bit.EORST)
  {
    usb_dfu_handle_bus_reset();

    USB->DEVICE.INTFLAG.reg = USB_DEVICE_INTFLAG_EORST;
    USB->DEVICE.DADD.reg = USB_DEVICE_DADD_ADDEN;

    usb_ep.in.PCKSIZE.bit.SIZE = USB_DEVICE_PCKSIZE_SIZE_64;
    usb_ep.in.PCKSIZE.bit.MULTI_PACKET_SIZE = 0;

    usb_ep.out.ADDR.reg = (uint32_t)usb_ctrl_out_buf;
    usb_ep.out.PCKSIZE.bit.SIZE = USB_DEVICE_PCKSIZE_SIZE_64;
    usb_ep.out.PCKSIZE.bit.MULTI_PACKET_SIZE = USB_CONTROL_EP_SIZE;
    usb_ep.out.PCKSIZE.bit.BYTE_COUNT = 0;

    USB->DEVICE.DeviceEndpoint[0].EPCFG.reg =
        USB_DEVICE_EPCFG_EPTYPE0(USB_DEVICE_EPCFG_EPTYPE_CONTROL) |
        USB_DEVICE_EPCFG_EPTYPE1(USB_DEVICE_EPCFG_EPTYPE_CONTROL);
    USB->DEVICE.DeviceEndpoint[0].EPSTATUSSET.bit.BK0RDY = 1;
    USB->DEVICE.DeviceEndpoint[0].EPSTATUSCLR.bit.BK1RDY = 1;
  }

  if (USB->DEVICE.DeviceEndpoint[0].EPINTFLAG.bit.RXSTP)
  {
    usb_request_t *request = (usb_request_t *)usb_ctrl_out_buf;

    usb_setup_length = request->wLength;

    if (sizeof(usb_request_t) == usb_ep.out.PCKSIZE.bit.BYTE_COUNT)
    {
      if (usb_handle_standard_request(request))
      {
        usb_ep.out.PCKSIZE.bit.BYTE_COUNT = 0;
        USB->DEVICE.DeviceEndpoint[0].EPSTATUSCLR.bit.BK0RDY = 1;
      }
      else
      {
        usb_control_stall();
      }
    }
    else
    {
      usb_control_stall();
    }

    USB->DEVICE.DeviceEndpoint[0].EPINTFLAG.reg = USB_DEVICE_EPINTFLAG_RXSTP;
  }
  else if (USB->DEVICE.DeviceEndpoint[0].EPINTFLAG.bit.TRCPT0)
  {
    if (usb_dfu_handle_data(usb_ctrl_out_buf, usb_ep.out.PCKSIZE.bit.BYTE_COUNT))
      usb_control_send_zlp();

    USB->DEVICE.DeviceEndpoint[0].EPINTFLAG.reg = USB_DEVICE_EPINTFLAG_TRCPT0;
  }
}


