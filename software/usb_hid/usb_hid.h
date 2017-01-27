/*
 * Copyright (c) 2013-2017, Alex Taradov <alex@taradov.com>
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

#ifndef _USB_HID_H_
#define _USB_HID_H_

/*- Includes ----------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

/*- Definitions -------------------------------------------------------------*/

/*- Types -------------------------------------------------------------------*/
typedef enum
{
  USB_HID_SUCCESS = 0, 
  USB_HID_IO_ERROR = -1,
  USB_HID_OUT_OF_HANDLES = -2,
  USB_HID_MALFORMED_REPORT_DESCRIPTOR = -3,
  USB_HID_INVALID_REPORT_SIZE = -4,
} usb_hid_status_t;

typedef struct
{
  int      handle;
  char     *path;
  char     *serial;
  char     *manufacturer;
  char     *product;
  int      report_size;
  int      vid;
  int      pid;
} usb_hid_device_t;

/*- Prototypes --------------------------------------------------------------*/
usb_hid_status_t usb_hid_init(void);
int usb_hid_enumerate(usb_hid_device_t *devices, int size);
void usb_hid_cleanup(usb_hid_device_t *devices, int size);
usb_hid_status_t usb_hid_open(usb_hid_device_t *device);
usb_hid_status_t usb_hid_close(usb_hid_device_t *device);
usb_hid_status_t usb_hid_read(usb_hid_device_t *device, uint8_t *data, int size);
usb_hid_status_t usb_hid_write(usb_hid_device_t *device, uint8_t *data, int size);

#endif // _USB_HID_H_

