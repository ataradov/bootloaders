/*
 * Copyright (c) 2021, Alex Taradov <alex@taradov.com>
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
#include <stdalign.h>
#include "usb.h"
#include "usb_std.h"
#include "usb_dfu.h"

/*- Variables ---------------------------------------------------------------*/
static int usb_dfu_size = 0;
static int usb_dfu_block = 0;

static alignas(4) usb_dfu_status_t usb_dfu_status =
{
  .bStatus        = USB_DFU_STATUS_OK,
  .bwPollTimeout0 = 0,
  .bwPollTimeout1 = 0,
  .bState         = USB_DFU_STATE_IDLE,
  .iString        = 0,
};

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
static void usb_dfu_error(int status)
{
  usb_dfu_size = 0;
  usb_dfu_status.bStatus = status;
  usb_dfu_status.bState  = USB_DFU_STATE_ERROR;
}

//-----------------------------------------------------------------------------
bool usb_dfu_handle_standard_request(usb_request_t *request)
{
  switch ((request->bRequest << 8) | request->bmRequestType)
  {
    case USB_CMD(OUT, INTERFACE, CLASS, DFU_DNLOAD):
    {
      if (USB_DFU_STATE_IDLE == usb_dfu_status.bState && 0 == request->wLength)
        return false;

      if (USB_DFU_STATE_IDLE != usb_dfu_status.bState &&
          USB_DFU_STATE_DNLOAD_IDLE != usb_dfu_status.bState)
        return false;

      if (request->wLength)
      {
        usb_dfu_size  = request->wLength;
        usb_dfu_block = request->wValue;
        usb_dfu_status.bState = USB_DFU_STATE_DNLOAD_SYNC;
      }
      else
      {
        usb_dfu_size  = 0;
        usb_dfu_status.bState = USB_DFU_STATE_MANIFEST_SYNC;
        usb_control_send_zlp();
      }
    } break;

    case USB_CMD(IN, INTERFACE, CLASS, DFU_GETSTATUS):
    {
      if (usb_dfu_status.bState == USB_DFU_STATE_DNLOAD_SYNC)
        usb_dfu_status.bState = USB_DFU_STATE_DNLOAD_IDLE;
      else if (usb_dfu_status.bState == USB_DFU_STATE_MANIFEST_SYNC)
        usb_dfu_status.bState = USB_DFU_STATE_IDLE;

      usb_control_send((uint8_t *)&usb_dfu_status, sizeof(usb_dfu_status_t));
    } break;

    case USB_CMD(IN, INTERFACE, CLASS, DFU_GETSTATE):
    {
      alignas(4) uint8_t state = usb_dfu_status.bState;
      usb_control_send((uint8_t *)&state, sizeof(uint8_t));
    } break;

    case USB_CMD(IN, INTERFACE, CLASS, DFU_UPLOAD):
    case USB_CMD(OUT, INTERFACE, CLASS, DFU_DETACH):
    {
      usb_control_send_zlp();
    } break;

    case USB_CMD(OUT, INTERFACE, CLASS, DFU_ABORT):
    case USB_CMD(OUT, INTERFACE, CLASS, DFU_CLRSTATUS):
    {
      usb_dfu_status.bStatus = USB_DFU_STATUS_OK;
      usb_dfu_status.bState  = USB_DFU_STATE_IDLE;
      usb_control_send_zlp();
    } break;

    default:
    {
      return false;
    } break;
  }

  return true;
}

//-----------------------------------------------------------------------------
bool usb_dfu_handle_data(uint8_t *data, int size)
{
  if (0 == usb_dfu_size)
    return false;

  if (size != usb_dfu_size)
  {
    usb_dfu_error(USB_DFU_STATUS_UNKNOWN);
    return true;
  }

  for (int i = size; i < USB_DFU_TRANSFER_SIZE; i++)
    data[i] = 0xff;

  int status = usb_dfu_data_callback(usb_dfu_block, (uint32_t *)data);

  if (USB_DFU_STATUS_OK != status)
  {
    usb_dfu_error(status);
    return true;
  }

  usb_dfu_size = 0;

  return true;
}


