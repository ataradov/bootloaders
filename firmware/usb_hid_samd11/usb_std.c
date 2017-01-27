/*
 * Copyright (c) 2016-2017, Alex Taradov <alex@taradov.com>
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
#include <string.h>
#include "utils.h"
#include "usb.h"
#include "usb_std.h"
#include "usb_descriptors.h"

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
void usb_handle_standard_request(usb_request_t *request)
{
  static int usb_config = 0;

  switch ((request->bRequest << 8) | request->bmRequestType)
  {
    case USB_CMD(IN, DEVICE, STANDARD, GET_DESCRIPTOR):
    {
      uint8_t type = request->wValue >> 8;
      uint8_t index = request->wValue & 0xff;
      uint16_t length = request->wLength;

      if (USB_DEVICE_DESCRIPTOR == type)
      {
        length = LIMIT(length, usb_device_descriptor.bLength);

        usb_control_send((uint8_t *)&usb_device_descriptor, length);
      }
      else if (USB_CONFIGURATION_DESCRIPTOR == type)
      {
        length = LIMIT(length, usb_configuration_hierarchy.configuration.wTotalLength);

        usb_control_send((uint8_t *)&usb_configuration_hierarchy, length);
      }
      else if (USB_STRING_DESCRIPTOR == type)
      {
        if (0 == index)
        {
          length = LIMIT(length, usb_string_descriptor_zero.bLength);

          usb_control_send((uint8_t *)&usb_string_descriptor_zero, length);
        }
        else if (index < USB_STR_COUNT)
        {
          const char *str = usb_strings[index];
          int len;

          for (len = 0; *str; len++, str++)
          {
            usb_string_descriptor_buffer[2 + len*2] = *str;
            usb_string_descriptor_buffer[3 + len*2] = 0;
          }

          usb_string_descriptor_buffer[0] = len*2 + 2;
          usb_string_descriptor_buffer[1] = USB_STRING_DESCRIPTOR;

          length = LIMIT(length, usb_string_descriptor_buffer[0]);

          usb_control_send(usb_string_descriptor_buffer, length);
        }
        else
        {
          usb_control_stall();
        }
      }
      else
        usb_control_stall();
    }  break;

    case USB_CMD(OUT, DEVICE, STANDARD, SET_ADDRESS):
    {
      usb_control_send_zlp();
      usb_set_address(request->wValue);
    } break;

    case USB_CMD(OUT, DEVICE, STANDARD, SET_CONFIGURATION):
    {
      usb_config = request->wValue;

      usb_control_send_zlp();

      if (usb_config)
      {
        int size = usb_configuration_hierarchy.configuration.wTotalLength;
        usb_descriptor_header_t *desc = (usb_descriptor_header_t *)&usb_configuration_hierarchy;

        while (size)
        {
          if (USB_ENDPOINT_DESCRIPTOR == desc->bDescriptorType)
            usb_configure_endpoint((usb_endpoint_descriptor_t *)desc);

          size -= desc->bLength;
          desc = (usb_descriptor_header_t *)((uint8_t *)desc + desc->bLength);
        }

        usb_configuration_callback(usb_config);
      }
    } break;

    case USB_CMD(IN, DEVICE, STANDARD, GET_CONFIGURATION):
    {
      uint8_t config = usb_config;
      usb_control_send(&config, sizeof(config));
    } break;

    case USB_CMD(IN, DEVICE, STANDARD, GET_STATUS):
    case USB_CMD(IN, INTERFACE, STANDARD, GET_STATUS):
    {
      uint16_t status = 0;
      usb_control_send((uint8_t *)&status, sizeof(status));
    } break;

    case USB_CMD(IN, ENDPOINT, STANDARD, GET_STATUS):
    {
      int ep = request->wIndex & USB_INDEX_MASK;
      int dir = request->wIndex & USB_DIRECTION_MASK;
      uint16_t status = 0;

      if (usb_endpoint_configured(ep, dir))
      {
        status = usb_endpoint_get_status(ep, dir);
        usb_control_send((uint8_t *)&status, sizeof(status));
      }
      else
      {
        usb_control_stall();
      }
    } break;

    case USB_CMD(OUT, DEVICE, STANDARD, SET_FEATURE):
    {
      usb_control_stall();
    } break;

    case USB_CMD(OUT, INTERFACE, STANDARD, SET_FEATURE):
    {
      usb_control_send_zlp();
    } break;

    case USB_CMD(OUT, ENDPOINT, STANDARD, SET_FEATURE):
    {
      int ep = request->wIndex & USB_INDEX_MASK;
      int dir = request->wIndex & USB_DIRECTION_MASK;

      if (0 == request->wValue && ep && usb_endpoint_configured(ep, dir))
      {
        usb_endpoint_set_feature(ep, dir);
        usb_control_send_zlp();
      }
      else
      {
        usb_control_stall();
      }
    } break;

    case USB_CMD(OUT, DEVICE, STANDARD, CLEAR_FEATURE):
    {
      usb_control_stall();
    } break;

    case USB_CMD(OUT, INTERFACE, STANDARD, CLEAR_FEATURE):
    {
      usb_control_send_zlp();
    } break;

    case USB_CMD(OUT, ENDPOINT, STANDARD, CLEAR_FEATURE):
    {
      int ep = request->wIndex & USB_INDEX_MASK;
      int dir = request->wIndex & USB_DIRECTION_MASK;

      if (0 == request->wValue && ep && usb_endpoint_configured(ep, dir))
      {
        usb_endpoint_clear_feature(ep, dir);
        usb_control_send_zlp();
      }
      else
      {
        usb_control_stall();
      }
    } break;

    // TODO: This actually belongs to HID implementation
    case USB_CMD(IN, INTERFACE, STANDARD, GET_DESCRIPTOR):
    {
      uint16_t length = request->wLength;

      length = LIMIT(length, sizeof(usb_hid_report_descriptor));

      usb_control_send((uint8_t *)usb_hid_report_descriptor, length);
    } break;

    default:
    {
      usb_control_stall();
    } break;
  }
}
