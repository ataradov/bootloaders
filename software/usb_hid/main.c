/*
 * Copyright (c) 2017, Alex Taradov <alex@taradov.com>
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
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdarg.h>
#include <unistd.h>
#include <getopt.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include "usb_hid.h"

/*- Definitions -------------------------------------------------------------*/
#define VERSION               "v0.2"

#define PAGE_SIZE              64
#define ERASE_SIZE             256
#define BL_REQUEST             0x78656c41

#define CONFIG_MARKER_BEGIN    0xc70d8e1a
#define CONFIG_MARKER_END      0xceab9d56

#ifndef O_BINARY
#define O_BINARY 0
#endif

enum
{
  BL_CMD_WRITE  = 0xa0,
  BL_CMD_FLUSH  = 0xa1,
  BL_CMD_READ   = 0xa2,
  BL_CMD_VERIFY = 0xa3,
  BL_CMD_REBOOT = 0xa4,
};

enum
{
  BL_STATUS_OK        = 0x50,
  BL_STATUS_ERROR     = 0x51,
  BL_STATUS_MALFORMED = 0x52,
  BL_STATUS_INVALID   = 0x53,
  BL_STATUS_CRC_OK    = 0x54,
  BL_STATUS_CRC_FAIL  = 0x55,
};

/*- Types -------------------------------------------------------------------*/
typedef struct
{
  int          size;
  int          aligned_size;
  uint8_t      *data;
} file_info_t;

/*- Variables ---------------------------------------------------------------*/
static const struct option long_options[] =
{
  { "help",      no_argument,        0, 'h' },
  { "quiet",     no_argument,        0, 'q' },
  { "file",      required_argument,  0, 'f' },
  { "offset",    required_argument,  0, 'o' },
  { "word",      required_argument,  0, 'w' },
  { "byte",      required_argument,  0, 'b' },
  { "reset",     no_argument,        0, 'r' },
  { 0, 0, 0, 0 }
};

static const char *short_options = "hqf:o:w:b:r";

static bool g_verbose = true;
static char *g_file = NULL;
static int g_offset = -1;
static bool g_byte = false;
static bool g_word = false;
static bool g_reset = false;
static uint32_t g_data;

static uint32_t crc32_tab[256];

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
static void verbose(char *fmt, ...)
{
  va_list args;

  if (g_verbose)
  {
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);

    fflush(stdout);
  }
}

//-----------------------------------------------------------------------------
static void check(bool cond, char *fmt, ...)
{
  if (!cond)
  {
    va_list args;

    va_start(args, fmt);
    fprintf(stderr, "Error: ");
    vfprintf(stderr, fmt, args);
    fprintf(stderr, "\n");
    va_end(args);

    exit(1);
  }
}

//-----------------------------------------------------------------------------
static void error_exit(char *fmt, ...)
{
  va_list args;

  va_start(args, fmt);
  fprintf(stderr, "Error: ");
  vfprintf(stderr, fmt, args);
  fprintf(stderr, "\n");
  va_end(args);

  exit(1);
}

//-----------------------------------------------------------------------------
static void crc32_tab_gen(void)
{
  for (int i = 0; i < 256; i++)
  {
    uint32_t value = i;

    for (int j = 0; j < 8; j++)
    {
      if (value & 1)
        value = (value >> 1) ^ 0xedb88320ul;
      else
        value = value >> 1;
    }

    crc32_tab[i] = value;
  }
}

//-----------------------------------------------------------------------------
static uint32_t crc32(uint8_t *data, int size)
{
  uint32_t crc = 0xffffffff;

  for (int i = 0; i < size; i++)
    crc = crc32_tab[(crc ^ data[i]) & 0xff] ^ (crc >> 8);

  return crc;
}

//-----------------------------------------------------------------------------
static file_info_t load_file(char *name)
{
  struct stat stat;
  int fd, rsize, fill_size;
  file_info_t info;

  fd = open(name, O_RDONLY | O_BINARY);
  check(fd >= 0, "open()");

  fstat(fd, &stat);
  info.size = stat.st_size;
  check(info.size < 8*1024*1024, "file is too big to be a firmware");

  fill_size = (PAGE_SIZE - (info.size % PAGE_SIZE)) % PAGE_SIZE;
  info.aligned_size = info.size + fill_size;

  info.data = malloc(info.aligned_size);
  check(NULL != info.data, "malloc()");

  rsize = read(fd, info.data, info.size);
  check(rsize == info.size, "read()");

  close(fd);

  memset(info.data + info.size, 0xff, fill_size);

  return info;
}

//-----------------------------------------------------------------------------
static int bl_command(usb_hid_device_t *device, uint8_t *req, int size)
{
  uint8_t resp[2];

  if (USB_HID_SUCCESS != usb_hid_write(device, req, size))
    error_exit("hid_write()");

  if (USB_HID_SUCCESS != usb_hid_read(device, resp, sizeof(resp)))
    error_exit("hid_read()");

  if (req[0] != resp[0])
    error_exit("invalid command code in response (0x%02x)", resp[0]);

  return resp[1];
}

//-----------------------------------------------------------------------------
static void bl_cmd_write(usb_hid_device_t *device, uint32_t offset, uint8_t *data, int size)
{
  uint8_t req[1024];
  uint32_t *req_w = (uint32_t *)req;
  int max_size = device->report_size - 12;
  uint8_t *req_ptr = data;
  int status;

  verbose("Programming...");

  while (size)
  {
    int req_size = (size > max_size) ? max_size : size;

    req[0] = BL_CMD_WRITE;
    req[1] = 0;
    req[2] = req_size % 0xff;
    req[3] = req_size >> 8;
    req_w[1] = BL_REQUEST;
    req_w[2] = offset;

    memcpy(&req[12], req_ptr, req_size);

    status = bl_command(device, req, 12 + req_size);
    check(BL_STATUS_OK == status, "data write (0x%02x)", status);

    size -= req_size;
    offset += req_size;
    req_ptr += req_size;

    verbose(".");
  }

  req[0] = BL_CMD_FLUSH;
  req_w[1] = BL_REQUEST;

  status = bl_command(device, req, 8);
  check(BL_STATUS_OK == status, "flush (0x%02x)", status);

  verbose(" done.\n");
}

//-----------------------------------------------------------------------------
static void bl_cmd_verify(usb_hid_device_t *device, int offset, int size, uint32_t crc)
{
  uint8_t req[20];
  uint32_t *req_w = (uint32_t *)req;
  int status;

  verbose("Verification...");

  req[0] = BL_CMD_VERIFY;
  req_w[1] = BL_REQUEST;
  req_w[2] = offset;
  req_w[3] = size;
  req_w[4] = crc;

  status = bl_command(device, req, 20);

  if (BL_STATUS_CRC_OK == status)
  {
    verbose(" success.\n");
  }
  else if (BL_STATUS_CRC_FAIL == status)
  {
    verbose(" fail.\n");
    error_exit("verification failed");
  }
  else
  {
    error_exit("verify (0x%02x)", status);
  }
}

//-----------------------------------------------------------------------------
static void bl_cmd_reset(usb_hid_device_t *device, uint32_t w0, uint32_t w1, uint32_t w2, uint32_t w3)
{
  uint8_t req[24];
  uint32_t *req_w = (uint32_t *)req;
  int status;

  verbose("Resetting...");

  req[0] = BL_CMD_REBOOT;
  req_w[1] = BL_REQUEST;
  req_w[2] = w0;
  req_w[3] = w1;
  req_w[4] = w2;
  req_w[5] = w3;

  status = bl_command(device, req, 24);

  if (BL_STATUS_CRC_OK == status)
    verbose(" success.\n");
  else
    error_exit("reset (0x%02x)", status);
}

//-----------------------------------------------------------------------------
static void print_help(char *name)
{
  printf("USB HID bootloader " VERSION ", built " __DATE__ " " __TIME__ "\n");
  printf("Usage: %s [options]\n", name);
  printf("Options:\n");
  printf("  -h, --help                 print this help message and exit\n");
  printf("  -q, --quiet                suppress informational messages\n");
  printf("  -f, --file <file>          binary file to be programmed\n");
  printf("  -o, --offset <value>       location offset of the image or data\n");
  printf("  -b, --byte <value>         write a byte at the specified offset\n");
  printf("  -w, --word <value>         write a word (32-bit) at the specified offset\n");
  printf("  -r, --reset                reset after the programming\n");
  exit(0);
}

//-----------------------------------------------------------------------------
static void parse_command_line(int argc, char **argv)
{
  int option_index = 0;
  int c;

  while ((c = getopt_long(argc, argv, short_options, long_options, &option_index)) != -1)
  {
    switch (c)
    {
      case 'h': print_help(argv[0]); break;
      case 'q': g_verbose = false; break;
      case 'f': g_file = optarg; break;
      case 'o': g_offset = (int)strtoul(optarg, NULL, 0); break;
      case 'b': g_data = (uint32_t)strtoul(optarg, NULL, 0); g_byte = true; break;
      case 'w': g_data = (uint32_t)strtoul(optarg, NULL, 0); g_word = true; break;
      case 'r': g_reset = true; break;
      default: exit(1); break;
    }
  }

  check(optind >= argc, "malformed command line, use '-h' for more information");
}

//-----------------------------------------------------------------------------
int main(int argc, char **argv)
{
  usb_hid_device_t devices[256];
  int n_devices;
  usb_hid_device_t *device = NULL;

  parse_command_line(argc, argv);

  usb_hid_init();

  n_devices = usb_hid_enumerate(devices, 256);

  for (int i = 0; i < n_devices; i++)
  {
    if (strstr(devices[i].product, "HID BL"))
    {
      check(NULL == device, "more than one bootloadable device detected");
      device = &devices[i];
    }
  }

  check(NULL != device, "no bootloadable devices detected");
  check(-1 != g_offset, "offset is not specified");

  usb_hid_open(device);

  if (g_byte)
  {
    bl_cmd_write(device, g_offset, (uint8_t *)&g_data, 1);
  }
  else if (g_word)
  {
    bl_cmd_write(device, g_offset, (uint8_t *)&g_data, 4);
  }
  else
  {
    uint32_t crc;
    file_info_t info;

    check(NULL != g_file, "input file name is not specified");

    crc32_tab_gen();

    info = load_file(g_file);
    crc = crc32(info.data, info.aligned_size);

    bl_cmd_write(device, g_offset, info.data, info.aligned_size);
    bl_cmd_verify(device, g_offset, info.aligned_size, crc);
  }

  if (g_reset)
  {
    bl_cmd_reset(device, 0, 0, 0, 0);
  }

  usb_hid_close(device);

  usb_hid_cleanup(devices, n_devices);

  return 0;
}

