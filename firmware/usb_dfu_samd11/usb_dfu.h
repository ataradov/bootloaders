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

#ifndef _USB_DFU_H_
#define _USB_DFU_H_

/*- Includes ----------------------------------------------------------------*/
#include <stdint.h>

/*- Definitions -------------------------------------------------------------*/
#define USB_DFU_INTERFACE_CLASS        0xfe
#define USB_DFU_INTERFACE_SUBCLASS     0x01
#define USB_DFU_INTERFACE_PROTOCOL_APP 0x01
#define USB_DFU_INTERFACE_PROTOCOL_DFU 0x02

#define USB_DFU_FUNCTIONAL_DESCRIPTOR  0x21
#define USB_DFU_VERSION                0x0101

#define USB_DFU_TRANSFER_SIZE          64

enum
{
  USB_DFU_CAN_DOWNLOAD           = (1 << 0),
  USB_DFU_CAN_UPLOAD             = (1 << 1),
  USB_DFU_MANIFESTATION_TOLERANT = (1 << 2),
  USB_DFU_WILL_DETACH            = (1 << 3),
};

enum
{
  USB_DFU_DETACH    = 0,
  USB_DFU_DNLOAD    = 1,
  USB_DFU_UPLOAD    = 2,
  USB_DFU_GETSTATUS = 3,
  USB_DFU_CLRSTATUS = 4,
  USB_DFU_GETSTATE  = 5,
  USB_DFU_ABORT     = 6,
};

enum
{
  USB_DFU_STATUS_OK            = 0x00,
  USB_DFU_STATUS_TARGET        = 0x01,
  USB_DFU_STATUS_FILE          = 0x02,
  USB_DFU_STATUS_WRITE         = 0x03,
  USB_DFU_STATUS_ERASE         = 0x04,
  USB_DFU_STATUS_CHECK_ERASED  = 0x05,
  USB_DFU_STATUS_PROG          = 0x06,
  USB_DFU_STATUS_VERIFY        = 0x07,
  USB_DFU_STATUS_ADDRESS       = 0x08,
  USB_DFU_STATUS_NOTDONE       = 0x09,
  USB_DFU_STATUS_FIRMWARE      = 0x0a,
  USB_DFU_STATUS_VENDOR        = 0x0b,
  USB_DFU_STATUS_USBR          = 0x0c,
  USB_DFU_STATUS_POR           = 0x0d,
  USB_DFU_STATUS_UNKNOWN       = 0x0e,
  USB_DFU_STATUS_STALLEDPKT    = 0x0f,
};

enum
{
  USB_DFU_STATE_APP_IDLE             = 0,
  USB_DFU_STATE_APP_DETACH           = 1,
  USB_DFU_STATE_IDLE                 = 2,
  USB_DFU_STATE_DNLOAD_SYNC          = 3,
  USB_DFU_STATE_DNBUSY               = 4,
  USB_DFU_STATE_DNLOAD_IDLE          = 5,
  USB_DFU_STATE_MANIFEST_SYNC        = 6,
  USB_DFU_STATE_MANIFEST             = 7,
  USB_DFU_STATE_MANIFEST_WAIT_RESET  = 8,
  USB_DFU_STATE_UPLOAD_IDLE          = 9,
  USB_DFU_STATE_ERROR                = 10,
};

/*- Types -------------------------------------------------------------------*/
typedef struct PACK
{
  uint8_t   bLength;
  uint8_t   bDescriptorType;
  uint8_t   bmAttributes;
  uint16_t  wDetachTimeOut;
  uint16_t  wTransferSize;
  uint16_t  bcdDFUVersion;
} usb_dfu_descriptor_t;

typedef struct PACK
{
  uint8_t   bStatus;
  uint16_t  bwPollTimeout0;
  uint8_t   bwPollTimeout1;
  uint8_t   bState;
  uint8_t   iString;
} usb_dfu_status_t;

/*- Prototypes --------------------------------------------------------------*/
bool usb_dfu_handle_standard_request(usb_request_t *request);
bool usb_dfu_handle_data(uint8_t *data, int size);
int usb_dfu_data_callback(int index, int block, uint32_t *data);
void usb_dfu_detach_callback(void);

#endif // _USB_DFU_H_


