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

/*- Includes ----------------------------------------------------------------*/
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdalign.h>
#include <string.h>
#include "samd11.h"
#include "nvm_data.h"
#include "usb.h"
#include "usb_dfu.h"
#include "hal_gpio.h"

/*- Definitions -------------------------------------------------------------*/
HAL_GPIO_PIN(BOOT_ENTER, A, 31);

#define BOOT_START     0
#define BOOT_SIZE      2048
#define APP_START      BOOT_SIZE
#define APP_SIZE       (FLASH_SIZE - BOOT_SIZE)

#define BL_REQUEST     0x78656c41

/*- Variables ---------------------------------------------------------------*/
static uint32_t *ram = (uint32_t *)HMCRAMC0_ADDR;
static int app_section_index = 0;

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
static void sys_init(void)
{
  NVMCTRL->CTRLB.reg = NVMCTRL_CTRLB_RWS(1) | NVMCTRL_CTRLB_CACHEDIS;

  PM->AHBMASK.reg  |= PM_AHBMASK_NVMCTRL;
  PM->APBBMASK.reg |= PM_APBBMASK_NVMCTRL;

  SYSCTRL->DFLLCTRL.reg = 0; // See Errata 9905
  SYSCTRL->DFLLMUL.reg = SYSCTRL_DFLLMUL_CSTEP(1) | SYSCTRL_DFLLMUL_FSTEP(1) |
      SYSCTRL_DFLLMUL_MUL(48000);
  SYSCTRL->DFLLVAL.reg = SYSCTRL_DFLLVAL_COARSE(32) | SYSCTRL_DFLLVAL_FINE(512);
  SYSCTRL->DFLLCTRL.reg = SYSCTRL_DFLLCTRL_ENABLE | SYSCTRL_DFLLCTRL_USBCRM |
      SYSCTRL_DFLLCTRL_MODE | SYSCTRL_DFLLCTRL_CCDIS;

  while (0 == (SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_DFLLRDY));

  GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(0) | GCLK_GENCTRL_SRC(GCLK_SOURCE_DFLL48M) |
      GCLK_GENCTRL_RUNSTDBY | GCLK_GENCTRL_GENEN;
  while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);
}


//-----------------------------------------------------------------------------
static void run_application(void)
{
  uint32_t msp = *(uint32_t *)(APP_START);
  uint32_t reset_vector = *(uint32_t *)(APP_START + 4);

  if (msp < HMCRAMC0_ADDR || msp > (HMCRAMC0_ADDR + HMCRAMC0_SIZE))
    return;

  __set_MSP(msp);

  asm("bx %0"::"r" (reset_vector));
}

//-----------------------------------------------------------------------------
static bool bl_request(void)
{
  HAL_GPIO_BOOT_ENTER_in();
  HAL_GPIO_BOOT_ENTER_pullup();

  if (BL_REQUEST == ram[0] && BL_REQUEST == ram[1] &&
      BL_REQUEST == ram[2] && BL_REQUEST == ram[3])
  {
    ram[0] = 0;
    return true;
  }

  if (0 == HAL_GPIO_BOOT_ENTER_read())
    return true;

  return false;
}

//-----------------------------------------------------------------------------
void usb_set_interface_callback(int index, int alt_setting)
{
  app_section_index = alt_setting;
  (void)index;
}

//-----------------------------------------------------------------------------
int usb_dfu_data_callback(int block, uint32_t *data)
{
  uint32_t addr = block * FLASH_PAGE_SIZE;
  uint32_t *flash;

  if (DFU_INDEX_APP == app_section_index)
  {
    if (addr > APP_SIZE)
      return USB_DFU_STATUS_ADDRESS;

    addr += BOOT_SIZE;
  }
  else if (DFU_INDEX_BOOT == app_section_index)
  {
    if (addr > BOOT_SIZE)
      return USB_DFU_STATUS_ADDRESS;
  }
  else
    return USB_DFU_STATUS_ADDRESS;

  NVMCTRL->ADDR.reg = addr >> 1;

  NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMDEX_KEY | NVMCTRL_CTRLA_CMD_UR;
  while (0 == NVMCTRL->INTFLAG.bit.READY);

  if ((addr % NVMCTRL_ROW_SIZE) == 0)
  {
    NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMDEX_KEY | NVMCTRL_CTRLA_CMD_ER;
    while (0 == NVMCTRL->INTFLAG.bit.READY);
  }

  flash = (uint32_t *)addr;

  for (int i = 0; i < (int)(FLASH_PAGE_SIZE / sizeof(uint32_t)); i++)
    flash[i] = data[i];

  while (0 == NVMCTRL->INTFLAG.bit.READY);

  for (int i = 0; i < (int)(FLASH_PAGE_SIZE / sizeof(uint32_t)); i++)
  {
    if (flash[i] != data[i])
      return USB_DFU_STATUS_VERIFY;
  }

  return USB_DFU_STATUS_OK;
}

//-----------------------------------------------------------------------------
__attribute__ ((noinline))
void main(void)
{
  if (!bl_request())
    run_application();

  sys_init();
  usb_init();

  while (1)
  {
    usb_task();
  }
}


