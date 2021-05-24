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
#include <stdbool.h>
#include <stdalign.h>
#include <string.h>
#include "utils.h"
#include "usb.h"
#include "usb_std.h"
#include "usb_descriptors.h"

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
static void usb_control_send_str(const char *str)
{
  alignas(4) uint8_t buf[USB_CONTROL_EP_SIZE];
  int size = 2;

  while (*str)
  {
    buf[size+0] = *str++;
    buf[size+1] = 0;
    size += 2;
  }

  buf[0] = size;
  buf[1] = USB_STRING_DESCRIPTOR;

  usb_control_send(buf, size);
}

//-----------------------------------------------------------------------------
bool usb_handle_standard_request(usb_request_t *request)
{
  static alignas(4) uint8_t usb_config = 0;

  switch ((request->bRequest << 8) | request->bmRequestType)
  {
    case USB_CMD(IN, DEVICE, STANDARD, GET_DESCRIPTOR):
    {
      int type  = request->wValue >> 8;
      int index = request->wValue & 0xff;

      if (USB_DEVICE_DESCRIPTOR == type)
      {
        usb_control_send((uint8_t *)&usb_device_descriptor, usb_device_descriptor.bLength);
      }
      else if (USB_CONFIGURATION_DESCRIPTOR == type)
      {
        usb_control_send((uint8_t *)&usb_configuration_hierarchy,
            usb_configuration_hierarchy.configuration.wTotalLength);
      }
      else if (USB_STRING_DESCRIPTOR == type)
      {
        if (0 == index)
        {
          usb_control_send((uint8_t *)&usb_string_descriptor_zero,
              usb_string_descriptor_zero.bLength);
        }
        else if (USB_MSFT_VENDOR_STR_INDEX == index)
        {
          usb_control_send_str(USB_MSFT_VENDOR_STR);
        }
        else if (index < USB_STR_COUNT)
        {
          usb_control_send_str(usb_strings[index]);
        }
        else
        {
          return false;
        }
      }
      else
      {
        return false;
      }
    } break;

    case USB_CMD(OUT, DEVICE, STANDARD, SET_ADDRESS):
    {
      usb_control_send_zlp();
      usb_set_address(request->wValue);
    } break;

    case USB_CMD(OUT, DEVICE, STANDARD, SET_CONFIGURATION):
    {
      usb_config = request->wValue;
      usb_control_send_zlp();
    } break;

    case USB_CMD(IN, DEVICE, STANDARD, GET_CONFIGURATION):
    {
      usb_control_send(&usb_config, sizeof(uint8_t));
    } break;

    case USB_CMD(IN, DEVICE, STANDARD, GET_STATUS):
    case USB_CMD(IN, INTERFACE, STANDARD, GET_STATUS):
    case USB_CMD(IN, ENDPOINT, STANDARD, GET_STATUS):
    {
      alignas(4) uint16_t status = 0;
      usb_control_send((uint8_t *)&status, sizeof(uint16_t));
    } break;

    case USB_CMD(OUT, INTERFACE, STANDARD, SET_FEATURE):
    case USB_CMD(OUT, INTERFACE, STANDARD, CLEAR_FEATURE):
    {
      usb_control_send_zlp();
    } break;

    case USB_CMD(OUT, DEVICE, STANDARD, SET_FEATURE):
    case USB_CMD(OUT, DEVICE, STANDARD, CLEAR_FEATURE):
    case USB_CMD(OUT, ENDPOINT, STANDARD, SET_FEATURE):
    case USB_CMD(OUT, ENDPOINT, STANDARD, CLEAR_FEATURE):
    {
      return false;
    } break;

    case USB_CMD(IN, DEVICE, VENDOR, MSFT_VENDOR_CODE):
    {
      if (USB_MSFT_VENDOR_INDEX == request->wIndex)
      {
        usb_control_send((uint8_t *)&usb_msft_compat_descriptor,
            sizeof(usb_msft_compat_descriptor_t));
      }
      else
      {
        return false;
      }
    } break;

    default:
    {
      return usb_dfu_handle_standard_request(request);
    } break;
  }

  return true;
}


