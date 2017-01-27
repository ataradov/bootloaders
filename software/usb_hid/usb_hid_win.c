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
#include <stdint.h>
#include <stdbool.h>
#include <windows.h>
#include <setupapi.h>          
#include <ddk/hidsdi.h>
#include <ddk/hidpi.h>
#include "usb_hid.h"

/*- Definitions -------------------------------------------------------------*/
#define MAX_STRING_SIZE   256
#define MAX_REPORT_SIZE   1024
#define MAX_OPEN_DEVICES  16

/*- Variables ---------------------------------------------------------------*/
static HANDLE usb_hid_handles[MAX_OPEN_DEVICES];

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
usb_hid_status_t usb_hid_init(void)
{
  for (int i = 0; i < MAX_OPEN_DEVICES; i++)
    usb_hid_handles[i] = INVALID_HANDLE_VALUE;

  return USB_HID_SUCCESS;
}

//-----------------------------------------------------------------------------
int usb_hid_enumerate(usb_hid_device_t *devices, int size)
{
  GUID hid_guid;
  HDEVINFO hid_dev_info;
  HIDD_ATTRIBUTES hid_attr;
  SP_DEVICE_INTERFACE_DATA dev_info_data;
  PSP_DEVICE_INTERFACE_DETAIL_DATA detail_data;
  DWORD detail_size;
  HANDLE handle;
  int rsize = 0;

  HidD_GetHidGuid(&hid_guid);

  hid_dev_info = SetupDiGetClassDevs(&hid_guid, NULL, NULL, DIGCF_PRESENT | DIGCF_INTERFACEDEVICE);

  dev_info_data.cbSize = sizeof(dev_info_data);

  for (int i = 0; i < size; i++)
  {
    if (FALSE == SetupDiEnumDeviceInterfaces(hid_dev_info, 0, &hid_guid, i, &dev_info_data))
      break;

    SetupDiGetDeviceInterfaceDetail(hid_dev_info, &dev_info_data, NULL, 0,
        &detail_size, NULL);

    detail_data = (PSP_DEVICE_INTERFACE_DETAIL_DATA)malloc(detail_size);
    detail_data->cbSize = sizeof(SP_DEVICE_INTERFACE_DETAIL_DATA);

    SetupDiGetDeviceInterfaceDetail(hid_dev_info, &dev_info_data, detail_data,
        detail_size, NULL, NULL);

    handle = CreateFile(detail_data->DevicePath, GENERIC_READ | GENERIC_WRITE,
        FILE_SHARE_READ | FILE_SHARE_WRITE, NULL, OPEN_EXISTING, 0, NULL);

    if (INVALID_HANDLE_VALUE != handle)
    {
      wchar_t wstr[MAX_STRING_SIZE];
      char str[MAX_STRING_SIZE];

      hid_attr.Size = sizeof(hid_attr);
      HidD_GetAttributes(handle, &hid_attr);

      devices[rsize].path = strdup(detail_data->DevicePath);

      HidD_GetSerialNumberString(handle, (PVOID)wstr, sizeof(wstr));
      memset(str, 0, sizeof(str));
      wcstombs(str, wstr, MAX_STRING_SIZE);
      devices[rsize].serial = strdup(str);

      HidD_GetManufacturerString(handle, (PVOID)wstr, sizeof(wstr));
      memset(str, 0, sizeof(str));
      wcstombs(str, wstr, MAX_STRING_SIZE);
      devices[rsize].manufacturer = strdup(str);

      HidD_GetProductString(handle, (PVOID)wstr, sizeof(wstr));
      memset(str, 0, sizeof(str));
      wcstombs(str, wstr, MAX_STRING_SIZE);
      devices[rsize].product = strdup(str);

      devices[rsize].handle = -1;
      devices[rsize].report_size = -1;
      devices[rsize].vid = hid_attr.VendorID;
      devices[rsize].pid = hid_attr.ProductID;

      rsize++;

      CloseHandle(handle);
    }

    free(detail_data);
  }

  SetupDiDestroyDeviceInfoList(hid_dev_info);

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
usb_hid_status_t usb_hid_open(usb_hid_device_t *device)
{
  HIDP_CAPS caps;
  PHIDP_PREPARSED_DATA prep;
  int input, output;
  HANDLE handle;
  int dev_handle = -1;

  if (-1 != device->handle)
    usb_hid_close(device);

  for (int i = 0; i < MAX_OPEN_DEVICES; i++)
  {
    if (INVALID_HANDLE_VALUE == usb_hid_handles[i])
    {
      dev_handle = i;
      break;
    }
  }

  if (-1 == dev_handle)
    return USB_HID_OUT_OF_HANDLES;

  handle = CreateFile(device->path, GENERIC_READ | GENERIC_WRITE,
      FILE_SHARE_READ | FILE_SHARE_WRITE, NULL, OPEN_EXISTING, 0, NULL);

  if (INVALID_HANDLE_VALUE == handle)
    return USB_HID_IO_ERROR;

  HidD_GetPreparsedData(handle, &prep);
  HidP_GetCaps(prep, &caps);

  input = caps.InputReportByteLength - 1;
  output = caps.OutputReportByteLength - 1;

  if (input != output)
    return USB_HID_MALFORMED_REPORT_DESCRIPTOR;

  if (64 != input && 512 != input && 1024 != input)
    return USB_HID_INVALID_REPORT_SIZE;

  device->report_size = input;
  device->handle = dev_handle;
  usb_hid_handles[dev_handle] = handle;

  return USB_HID_SUCCESS;
}

//-----------------------------------------------------------------------------
usb_hid_status_t usb_hid_close(usb_hid_device_t *device)
{
  if (-1 != device->handle)
  {
    CloseHandle(usb_hid_handles[device->handle]);
    usb_hid_handles[device->handle] = INVALID_HANDLE_VALUE;
    device->handle = -1;
  }

  return USB_HID_SUCCESS;
}

//-----------------------------------------------------------------------------
usb_hid_status_t usb_hid_read(usb_hid_device_t *device, uint8_t *data, int size)
{
  HANDLE handle = usb_hid_handles[device->handle];
  uint8_t buf[MAX_REPORT_SIZE + 1];
  unsigned long res;

  if (FALSE == ReadFile(handle, (LPVOID)buf, device->report_size + 1, &res, NULL))
    return USB_HID_IO_ERROR;

  res--;
  memcpy(data, &buf[1], (size < (int)res) ? size : (int)res);

  return USB_HID_SUCCESS;
}

//-----------------------------------------------------------------------------
usb_hid_status_t usb_hid_write(usb_hid_device_t *device, uint8_t *data, int size)
{
  HANDLE handle = usb_hid_handles[device->handle];
  uint8_t buf[MAX_REPORT_SIZE + 1];
  unsigned long res;

  memset(buf, 0, sizeof(buf));
  memcpy(&buf[1], data, size);

  if (FALSE == WriteFile(handle, (LPCVOID)buf, device->report_size + 1, &res, NULL))
    return USB_HID_IO_ERROR;

  return USB_HID_SUCCESS;
}

