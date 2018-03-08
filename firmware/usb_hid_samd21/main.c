/*
 * Copyright (c) 2017-2018, Alex Taradov <alex@taradov.com>
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
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdalign.h>
#include <string.h>
#include "samd21.h"
#include "hal_gpio.h"
#include "nvm_data.h"
#include "usb.h"

/*- Definitions -------------------------------------------------------------*/
//#define BOARD_SWD_USB_MINI
#define BOARD_SWD_USB_STD

#if defined(BOARD_SWD_USB_MINI)
  HAL_GPIO_PIN(LED, A, 14);
#elif defined(BOARD_SWD_USB_STD)
  HAL_GPIO_PIN(LED, A, 4);
#else
  #error Undefined board
#endif

HAL_GPIO_PIN(BOOT_ENTER, A, 31);

#define USB_EP_SEND           1
#define USB_EP_RECV           2
#define USB_BUFFER_SIZE       64

#define APPLICATION_START     4096
#define PAGES_IN_ERASE_BLOCK  NVMCTRL_ROW_PAGES
#define ERASE_BLOCK_SIZE      NVMCTRL_ROW_SIZE
#define FLASH_PAGE_SIZE_WORDS (int)(FLASH_PAGE_SIZE / sizeof(uint32_t))

#define BL_REQUEST            0x78656c41

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

/*- Variables ---------------------------------------------------------------*/
static uint32_t *ram = (uint32_t *)HMCRAMC0_ADDR;
static alignas(4) uint8_t app_request_buffer[USB_BUFFER_SIZE];
static alignas(4) uint8_t app_response_buffer[USB_BUFFER_SIZE];
static alignas(4) uint8_t app_flash_buf[ERASE_BLOCK_SIZE];
static int app_block_index = -1;
static bool app_reset_request = false;

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
static void sys_init(void)
{
  uint32_t coarse, fine;

  PAC1->WPCLR.reg = PAC1->WPCLR.reg;
  PM->AHBMASK.reg |= PM_AHBMASK_NVMCTRL | PM_AHBMASK_DSU;
  PM->APBBMASK.reg |= PM_APBBMASK_NVMCTRL | PM_APBBMASK_DSU;
  NVMCTRL->CTRLB.reg = NVMCTRL_CTRLB_CACHEDIS | NVMCTRL_CTRLB_RWS(2);

  SYSCTRL->INTFLAG.reg = SYSCTRL_INTFLAG_BOD33RDY | SYSCTRL_INTFLAG_BOD33DET |
      SYSCTRL_INTFLAG_DFLLRDY;

  coarse = NVM_READ_CAL(NVM_DFLL48M_COARSE_CAL);
  fine = NVM_READ_CAL(NVM_DFLL48M_FINE_CAL);

  SYSCTRL->DFLLCTRL.reg = 0; // See Errata 9905
  while (0 == (SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_DFLLRDY));

  SYSCTRL->DFLLMUL.reg = SYSCTRL_DFLLMUL_MUL(48000);
  SYSCTRL->DFLLVAL.reg = SYSCTRL_DFLLVAL_COARSE(coarse) | SYSCTRL_DFLLVAL_FINE(fine);

  SYSCTRL->DFLLCTRL.reg = SYSCTRL_DFLLCTRL_ENABLE | SYSCTRL_DFLLCTRL_USBCRM |
      SYSCTRL_DFLLCTRL_MODE | SYSCTRL_DFLLCTRL_BPLCKC | SYSCTRL_DFLLCTRL_CCDIS |
      SYSCTRL_DFLLCTRL_STABLE;

  while (0 == (SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_DFLLRDY));

  GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(0) | GCLK_GENCTRL_SRC(GCLK_SOURCE_DFLL48M) |
      GCLK_GENCTRL_RUNSTDBY | GCLK_GENCTRL_GENEN;
  while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);
}

//-----------------------------------------------------------------------------
static void run_application(void)
{
  uint32_t msp = *(uint32_t *)(APPLICATION_START);
  uint32_t reset_vector = *(uint32_t *)(APPLICATION_START + 4);

  if (0xffffffff == msp)
    return;

  __set_MSP(msp);

  asm("bx %0"::"r" (reset_vector));
}

//-----------------------------------------------------------------------------
static bool bl_request(void)
{
  HAL_GPIO_BOOT_ENTER_in();
  HAL_GPIO_BOOT_ENTER_pullup();

  if (0 == HAL_GPIO_BOOT_ENTER_read())
    return true;

  if (BL_REQUEST == ram[0] && BL_REQUEST == ram[1] &&
      BL_REQUEST == ram[2] && BL_REQUEST == ram[3])
  {
    ram[0] = 0;
    return true;
  }

  return false;
}

//-----------------------------------------------------------------------------
static void flash_flush(void)
{
  uint32_t addr = FLASH_ADDR + app_block_index * ERASE_BLOCK_SIZE;
  uint32_t *flash_offset = (uint32_t *)addr;
  uint32_t *flash_data = (uint32_t *)app_flash_buf;

  if (-1 == app_block_index)
    return;

  NVMCTRL->ADDR.reg = addr >> 1;

  NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMDEX_KEY | NVMCTRL_CTRLA_CMD_UR;
  while (0 == NVMCTRL->INTFLAG.bit.READY);

  NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMDEX_KEY | NVMCTRL_CTRLA_CMD_ER;
  while (0 == NVMCTRL->INTFLAG.bit.READY);


  for (int page = 0; page < PAGES_IN_ERASE_BLOCK; page++)
  {
    for (int i = 0; i < FLASH_PAGE_SIZE_WORDS; i++)
      *flash_offset++ = *flash_data++;

    while (0 == NVMCTRL->INTFLAG.bit.READY);
  }

  app_block_index = -1;
}

//-----------------------------------------------------------------------------
static void flash_write(uint32_t addr, uint8_t data)
{
  int block_index = (addr % FLASH_SIZE) / ERASE_BLOCK_SIZE;
  
  if (block_index != app_block_index)
  {
    uint32_t block_addr = FLASH_ADDR + block_index * ERASE_BLOCK_SIZE;

    flash_flush();

    for (int i = 0; i < ERASE_BLOCK_SIZE; i++)
      app_flash_buf[i] = ((uint8_t *)block_addr)[i];

    app_block_index = block_index;
  }

  app_flash_buf[addr % ERASE_BLOCK_SIZE] = data;
}

//-----------------------------------------------------------------------------
static void boot_process_request(uint8_t *req, uint8_t *resp)
{
  uint32_t *req_w = (uint32_t *)req;
  int cmd = req[0];

  resp[0] = cmd;
  resp[1] = BL_STATUS_OK;

  if (BL_REQUEST != req_w[1])
  {
    resp[1] = BL_STATUS_MALFORMED;
  }

  else if (BL_CMD_WRITE == cmd)
  {
    uint32_t addr = req_w[2];
    int size = *(uint16_t *)&req[2];

    if (size <= (int)(USB_BUFFER_SIZE - sizeof(uint32_t) * 3))
    {
      for (int i = 0; i < size; i++, addr++)
        flash_write(addr, req[sizeof(uint32_t) * 3 + i]);
    }
    else
    {
      resp[1] = BL_STATUS_ERROR;
    }
  }

  else if (BL_CMD_FLUSH == cmd)
  {
    flash_flush();
  }

  else if (BL_CMD_VERIFY == cmd)
  {
    uint32_t addr = FLASH_ADDR + req_w[2] % FLASH_SIZE;
    uint32_t size = req_w[3] % FLASH_SIZE;
    uint32_t crc = req_w[4];

    DSU->ADDR.reg = addr;
    DSU->LENGTH.reg = size;
    DSU->DATA.reg = 0xffffffff;
    DSU->STATUSA.reg = DSU->STATUSA.reg;
    DSU->CTRL.reg = DSU_CTRL_CRC;

    while (0 == DSU->STATUSA.bit.DONE);

    if (0 == DSU->STATUSA.bit.BERR && crc == DSU->DATA.reg)
      resp[1] = BL_STATUS_CRC_OK;
    else
      resp[1] = BL_STATUS_CRC_FAIL;
  }

  else if (BL_CMD_REBOOT == cmd)
  {
    ram[0] = req_w[2];
    ram[1] = req_w[3];
    ram[2] = req_w[4];
    ram[3] = req_w[5];

    app_reset_request = true;
  }

  else
  {
    resp[1] = BL_STATUS_INVALID;
  }
}

//-----------------------------------------------------------------------------
void usb_send_callback(int ep)
{
  if (app_reset_request)
    NVIC_SystemReset();

  (void)ep;
}

//-----------------------------------------------------------------------------
void usb_recv_callback(int ep)
{
  if (USB_EP_RECV  == ep)
  {
    boot_process_request(app_request_buffer, app_response_buffer);
    usb_send(USB_EP_SEND, app_response_buffer, sizeof(app_response_buffer));
    usb_recv(USB_EP_RECV, app_request_buffer, sizeof(app_request_buffer));
  }
}

//-----------------------------------------------------------------------------
void usb_configuration_callback(int config)
{
  usb_recv(USB_EP_RECV, app_request_buffer, sizeof(app_request_buffer));
  (void)config;
}

//-----------------------------------------------------------------------------
static void led_task(void)
{
  static int cnt = 0;

  if (100000 == cnt++)
  {
    HAL_GPIO_LED_toggle();
    cnt = 0;
  }
}

//-----------------------------------------------------------------------------
__attribute__ ((noinline))
int main(void)
{
  if (!bl_request())
    run_application();

  HAL_GPIO_LED_out();

  sys_init();
  usb_init();

  while (1)
  {
    usb_task();
    led_task();
  }

  return 0;
}

