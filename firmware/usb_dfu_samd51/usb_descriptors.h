// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2023, Alex Taradov <alex@taradov.com>. All rights reserved.

#ifndef _USB_DESCRIPTORS_H_
#define _USB_DESCRIPTORS_H_

/*- Includes ----------------------------------------------------------------*/
#include "usb.h"
#include "usb_std.h"
#include "usb_dfu.h"

/*- Definitions -------------------------------------------------------------*/
enum
{
  USB_STR_ZERO,
  USB_STR_MANUFACTURER,
  USB_STR_PRODUCT,
  USB_STR_NAME_BOOT,
  USB_STR_NAME_APP,
  USB_STR_COUNT,
};

enum
{
  DFU_INDEX_BOOT,
  DFU_INDEX_APP,
};

/*- Types -------------------------------------------------------------------*/
typedef struct PACK
{
  usb_configuration_descriptor_t     configuration;
  usb_interface_descriptor_t         interface_dfu_boot;
  usb_interface_descriptor_t         interface_dfu_app;
  usb_dfu_descriptor_t               dfu;
} usb_configuration_hierarchy_t;

typedef struct PACK
{
  usb_msft_compat_descriptor_t       compat;
  usb_msft_interface_descriptor_t    interface;
} usb_msft_compat_hierarchy_t;

//-----------------------------------------------------------------------------
extern const usb_device_descriptor_t usb_device_descriptor;
extern const usb_configuration_hierarchy_t usb_configuration_hierarchy;
extern const usb_msft_compat_hierarchy_t usb_msft_compat_hierarchy;
extern const usb_string_descriptor_zero_t usb_string_descriptor_zero;
extern const usb_msft_compat_descriptor_t usb_msft_compat_descriptor;
extern const char *usb_strings[];

#endif // _USB_DESCRIPTORS_H_


