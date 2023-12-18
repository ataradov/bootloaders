// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2023, Alex Taradov <alex@taradov.com>. All rights reserved.

#ifndef _USB_H_
#define _USB_H_

/*- Includes ----------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "usb_descriptors.h"

/*- Prototypes --------------------------------------------------------------*/
void usb_init(void);
void usb_set_address(int address);
void usb_control_stall(void);
void usb_control_send(uint8_t *data, int size);
void usb_control_send_zlp(void);
void usb_task(void);

#endif // _USB_H_

