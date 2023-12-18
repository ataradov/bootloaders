// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2023, Alex Taradov <alex@taradov.com>. All rights reserved.

/*- Includes ----------------------------------------------------------------*/
#include <stdalign.h>
#include "usb_std.h"
#include "usb_descriptors.h"

/*- Variables ---------------------------------------------------------------*/
const alignas(4) usb_device_descriptor_t usb_device_descriptor =
{
  .bLength            = sizeof(usb_device_descriptor_t),
  .bDescriptorType    = USB_DEVICE_DESCRIPTOR,
  .bcdUSB             = 0x0200,
  .bDeviceClass       = 0x00,
  .bDeviceSubClass    = 0x00,
  .bDeviceProtocol    = 0x00,
  .bMaxPacketSize0    = USB_CONTROL_EP_SIZE,
  .idVendor           = 0x6666,
  .idProduct          = 0x1010,
  .bcdDevice          = 0x0100,
  .iManufacturer      = USB_STR_MANUFACTURER,
  .iProduct           = USB_STR_PRODUCT,
  .iSerialNumber      = 0,
  .bNumConfigurations = 1,
};

const alignas(4) usb_configuration_hierarchy_t usb_configuration_hierarchy =
{
  .configuration =
  {
    .bLength             = sizeof(usb_configuration_descriptor_t),
    .bDescriptorType     = USB_CONFIGURATION_DESCRIPTOR,
    .wTotalLength        = sizeof(usb_configuration_hierarchy_t),
    .bNumInterfaces      = 1,
    .bConfigurationValue = 1,
    .iConfiguration      = 0,
    .bmAttributes        = 0x80,
    .bMaxPower           = 250, // 500 mA
  },

  .interface_dfu_boot =
  {
    .bLength             = sizeof(usb_interface_descriptor_t),
    .bDescriptorType     = USB_INTERFACE_DESCRIPTOR,
    .bInterfaceNumber    = 0,
    .bAlternateSetting   = DFU_INDEX_BOOT,
    .bNumEndpoints       = 0,
    .bInterfaceClass     = USB_DFU_INTERFACE_CLASS,
    .bInterfaceSubClass  = USB_DFU_INTERFACE_SUBCLASS,
    .bInterfaceProtocol  = USB_DFU_INTERFACE_PROTOCOL_DFU,
    .iInterface          = USB_STR_NAME_BOOT,
  },

  .interface_dfu_app =
  {
    .bLength             = sizeof(usb_interface_descriptor_t),
    .bDescriptorType     = USB_INTERFACE_DESCRIPTOR,
    .bInterfaceNumber    = 0,
    .bAlternateSetting   = DFU_INDEX_APP,
    .bNumEndpoints       = 0,
    .bInterfaceClass     = USB_DFU_INTERFACE_CLASS,
    .bInterfaceSubClass  = USB_DFU_INTERFACE_SUBCLASS,
    .bInterfaceProtocol  = USB_DFU_INTERFACE_PROTOCOL_DFU,
    .iInterface          = USB_STR_NAME_APP,
  },

  .dfu =
  {
    .bLength             = sizeof(usb_dfu_descriptor_t),
    .bDescriptorType     = USB_DFU_FUNCTIONAL_DESCRIPTOR,
    .bmAttributes        = USB_DFU_CAN_DOWNLOAD | USB_DFU_MANIFESTATION_TOLERANT,
    .wDetachTimeOut      = 0,
    .wTransferSize       = USB_DFU_TRANSFER_SIZE,
    .bcdDFUVersion       = USB_DFU_VERSION,
  },
};

const alignas(4) usb_string_descriptor_zero_t usb_string_descriptor_zero =
{
  .bLength               = sizeof(usb_string_descriptor_zero_t),
  .bDescriptorType       = USB_STRING_DESCRIPTOR,
  .wLANGID               = 0x0409, // English (United States)
};

const alignas(4) usb_msft_compat_hierarchy_t usb_msft_compat_hierarchy =
{
  .compat =
  {
    .dwLength              = sizeof(usb_msft_compat_hierarchy_t),
    .bcdVersion            = 0x0100,
    .wIndex                = USB_MSFT_VENDOR_INDEX,
    .bCount                = 1,
    .reserved              = { 0 },
  },

  .interface =
  {
    .bFirstInterfaceNumber = 0,
    .reserved1             = 1,
    .compatibleID          = "WINUSB\0\0",
    .subCompatibleID       = { 0 },
    .reserved2             = { 0 },
  },
};

const char *usb_strings[] =
{
  [USB_STR_MANUFACTURER]  = "Alex Taradov",
  [USB_STR_PRODUCT]       = "DFU Bootloader",
  [USB_STR_NAME_BOOT]     = "boot",
  [USB_STR_NAME_APP]      = "app",
};


