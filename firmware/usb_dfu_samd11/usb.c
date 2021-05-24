/*
 * Copyright (c) 2016-2021, Alex Taradov <alex@taradov.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*- Includes ----------------------------------------------------------------*/
#include <string.h>
#include <stdbool.h>
#include <stdalign.h>
#include "samd11.h"
#include "utils.h"
#include "nvm_data.h"
#include "usb.h"
#include "usb_std.h"
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
  PORT_IOBUS->Group[0].PINCFG[24].reg |= PORT_PINCFG_PMUXEN;
  PORT_IOBUS->Group[0].PINCFG[25].reg |= PORT_PINCFG_PMUXEN;
  PORT_IOBUS->Group[0].PMUX[24>>1].reg = PORT_PMUX_PMUXO_G | PORT_PMUX_PMUXE_G;

  PM->APBBMASK.reg |= PM_APBBMASK_USB;

  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_ID(USB_GCLK_ID) |
      GCLK_CLKCTRL_GEN(0);

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


