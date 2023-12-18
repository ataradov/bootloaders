// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2023, Alex Taradov <alex@taradov.com>. All rights reserved.

/*- Includes ----------------------------------------------------------------*/
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdalign.h>
#include <string.h>
#include "samd51.h"
#include "nvm_data.h"
#include "usb.h"
#include "usb_dfu.h"
#include "hal_gpio.h"
#include "nvm_data.h"

/*- Definitions -------------------------------------------------------------*/
HAL_GPIO_PIN(BOOT_ENTER, A, 31);

#define BOOT_START     0
#define BOOT_SIZE      8192
#define APP_START      BOOT_SIZE
#define APP_SIZE       (FLASH_SIZE - BOOT_SIZE)

#define PAGE_SIZE      512
#define BLOCK_SIZE     8192

#define BL_REQUEST     0x78656c41

#define INVALID_INDEX  -1

/*- Variables ---------------------------------------------------------------*/
static uint32_t *ram = (uint32_t *)HSRAM_ADDR;
static int app_section_index = INVALID_INDEX;

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
static void sys_init(void)
{
  MCLK->AHBMASK.reg  |= MCLK_AHBMASK_NVMCTRL;
  MCLK->APBBMASK.reg |= MCLK_APBBMASK_NVMCTRL;

  NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_AUTOWS | NVMCTRL_CTRLA_WMODE_MAN |
      NVMCTRL_CTRLA_PRM_MANUAL | NVMCTRL_CTRLA_CACHEDIS0 | NVMCTRL_CTRLA_CACHEDIS1;
}

//-----------------------------------------------------------------------------
static void run_application(void)
{
  uint32_t msp = *(uint32_t *)(APP_START);
  uint32_t reset_vector = *(uint32_t *)(APP_START + 4);

  if (msp < HSRAM_ADDR || msp > (HSRAM_ADDR + HSRAM_SIZE))
    return;

  asm("msr msp, %0 \n bx %1" :: "r" (msp), "r" (reset_vector));
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

  // Small delay to let the pull-up work
  for (int i = 0; i < 10000; i++)
    asm("nop");

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
  uint32_t addr = block * USB_DFU_TRANSFER_SIZE;
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

  NVMCTRL->ADDR.reg = addr & ~(PAGE_SIZE-1);

  NVMCTRL->CTRLB.reg = NVMCTRL_CTRLB_CMDEX_KEY | NVMCTRL_CTRLB_CMD_UR;
  while (0 == NVMCTRL->STATUS.bit.READY);

  if (0 == (addr % BLOCK_SIZE))
  {
    NVMCTRL->CTRLB.reg = NVMCTRL_CTRLB_CMDEX_KEY | NVMCTRL_CTRLB_CMD_EB;
    while (0 == NVMCTRL->STATUS.bit.READY);
  }

  flash = (uint32_t *)addr;

  NVMCTRL->CTRLB.reg = NVMCTRL_CTRLB_CMDEX_KEY | NVMCTRL_CTRLB_CMD_PBC;
  while (0 == NVMCTRL->STATUS.bit.READY);

  for (int i = 0; i < (int)(USB_DFU_TRANSFER_SIZE / sizeof(uint32_t)); i++)
    flash[i] = data[i];

  NVMCTRL->CTRLB.reg = NVMCTRL_CTRLB_CMDEX_KEY | NVMCTRL_CTRLB_CMD_WP;
  while (0 == NVMCTRL->STATUS.bit.READY);

  for (int i = 0; i < (int)(USB_DFU_TRANSFER_SIZE / sizeof(uint32_t)); i++)
  {
    if (flash[i] != data[i])
      return USB_DFU_STATUS_VERIFY;
  }

  return USB_DFU_STATUS_OK;
}

//-----------------------------------------------------------------------------
void usb_dfu_reset_callback(void)
{
  if (INVALID_INDEX != app_section_index)
    NVIC_SystemReset();
}

//-----------------------------------------------------------------------------
__attribute__ ((noinline))
int main(void)
{
  if (!bl_request())
    run_application();

  sys_init();
  usb_init();

  while (1)
  {
    usb_task();
  }

  return 0;
}


