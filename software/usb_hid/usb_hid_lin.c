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
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <linux/types.h>
#include <linux/input.h>
#include <linux/hidraw.h>
#include <libudev.h>
#include "usb_hid.h"

/*- Definitions -------------------------------------------------------------*/
#define MAX_REPORT_SIZE   1024

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
usb_hid_status_t usb_hid_init(void)
{
  return USB_HID_SUCCESS;
}

//-----------------------------------------------------------------------------
int usb_hid_enumerate(usb_hid_device_t *devices, int size)
{
  struct udev *udev;
  struct udev_enumerate *enumerate;
  struct udev_list_entry *devs, *dev_list_entry;
  struct udev_device *dev, *parent;
  int rsize = 0;

  udev = udev_new();
  enumerate = udev_enumerate_new(udev);
  udev_enumerate_add_match_subsystem(enumerate, "hidraw");
  udev_enumerate_scan_devices(enumerate);
  devs = udev_enumerate_get_list_entry(enumerate);

  udev_list_entry_foreach(dev_list_entry, devs)
  {
    const char *path;

    path = udev_list_entry_get_name(dev_list_entry);
    dev = udev_device_new_from_syspath(udev, path);

    parent = udev_device_get_parent_with_subsystem_devtype(dev, "usb", "usb_device");

    if (NULL == parent)
      continue;

    if (rsize < size)
    {
      const char *serial = udev_device_get_sysattr_value(parent, "serial");
      const char *manufacturer = udev_device_get_sysattr_value(parent, "manufacturer");
      const char *product = udev_device_get_sysattr_value(parent, "product");

      devices[rsize].path = strdup(udev_device_get_devnode(dev));
      devices[rsize].serial = strdup(serial ? serial : "");
      devices[rsize].manufacturer = strdup(manufacturer ? manufacturer : "");
      devices[rsize].product = strdup(product ? product : "");
      devices[rsize].vid = strtol(udev_device_get_sysattr_value(parent, "idVendor"), NULL, 16);
      devices[rsize].pid = strtol(udev_device_get_sysattr_value(parent, "idProduct"), NULL, 16);
      devices[rsize].handle = -1;
      devices[rsize].report_size = -1;

      rsize++;
    }

    udev_device_unref(parent);
  }

  udev_enumerate_unref(enumerate);
  udev_unref(udev);

  return rsize;
}

//-----------------------------------------------------------------------------
void usb_hid_cleanup(usb_hid_device_t *devices, int size)
{
  for (int i = 0; i < size; i++)
  {
    usb_hid_close(&devices[i]);

    free(devices[i].path);
    free(devices[i].serial);
    free(devices[i].manufacturer);
    free(devices[i].product);
  }
}

//-----------------------------------------------------------------------------
static int parse_hid_report_desc(uint8_t *data, int size)
{
  uint32_t count = 0;
  uint32_t input = 0;
  uint32_t output = 0;

  // This is a very primitive parser, but descriptors
  // we are interested in are pretty uniform
  for (int i = 0; i < size; )
  {
    int prefix = data[i++];
    int bTag = (prefix >> 4) & 0x0f;
    int bType = (prefix >> 2) & 0x03;
    int bSize = prefix & 0x03;

    bSize = (3 == bSize) ? 4 : bSize;

    if (1 == bType && 9 == bTag)
    {
      count = 0;

      for (int j = 0; j < bSize; j++)
        count |= (data[i + j] << (j * 8));
    }
    else if (0 == bType && 8 == bTag)
      input = count;
    else if (0 == bType && 9 == bTag)
      output = count;

    i += bSize;
  }

  if (input != output)
    return USB_HID_MALFORMED_REPORT_DESCRIPTOR;

  if (64 != input && 512 != input && 1024 != input)
    return USB_HID_INVALID_REPORT_SIZE;

  return input;
}

//-----------------------------------------------------------------------------
usb_hid_status_t usb_hid_open(usb_hid_device_t *device)
{
  struct hidraw_report_descriptor rpt_desc;
  struct hidraw_devinfo info;
  int desc_size, res, fd;

  if (-1 != device->handle)
    usb_hid_close(device);

  fd = open(device->path, O_RDWR);

  if (fd < 0)
    return USB_HID_IO_ERROR;

  memset(&rpt_desc, 0, sizeof(rpt_desc));
  memset(&info, 0, sizeof(info));

  res = ioctl(fd, HIDIOCGRDESCSIZE, &desc_size);
  if (res < 0)
    return USB_HID_IO_ERROR;

  rpt_desc.size = desc_size;
  res = ioctl(fd, HIDIOCGRDESC, &rpt_desc);
  if (res < 0)
    return USB_HID_IO_ERROR;

  res = parse_hid_report_desc(rpt_desc.value, rpt_desc.size);

  if (res < 0)
    return res;

  device->handle = fd;
  device->report_size = res;

  return USB_HID_SUCCESS;
}

//-----------------------------------------------------------------------------
usb_hid_status_t usb_hid_close(usb_hid_device_t *device)
{
  if (-1 != device->handle)
  {
    close(device->handle);
    device->handle = -1;
  }

  return USB_HID_SUCCESS;
}

//-----------------------------------------------------------------------------
usb_hid_status_t usb_hid_read(usb_hid_device_t *device, uint8_t *data, int size)
{
  uint8_t buf[MAX_REPORT_SIZE + 1];
  int res;

  res = read(device->handle, buf, device->report_size + 1);
  if (res < 0)
    return USB_HID_IO_ERROR;

  memcpy(data, buf, (size < res) ? size : res);

  return USB_HID_SUCCESS;
}

//-----------------------------------------------------------------------------
usb_hid_status_t usb_hid_write(usb_hid_device_t *device, uint8_t *data, int size)
{
  uint8_t buf[MAX_REPORT_SIZE + 1];
  int res;

  memset(buf, 0, sizeof(buf));
  memcpy(&buf[1], data, size);

  res = write(device->handle, buf, device->report_size + 1);
  if (res < 0)
    return USB_HID_IO_ERROR;

  return USB_HID_SUCCESS;
}


