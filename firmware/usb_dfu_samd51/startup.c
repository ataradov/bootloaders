// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2023, Alex Taradov <alex@taradov.com>. All rights reserved.

/*- Prototypes --------------------------------------------------------------*/
void irq_handler_reset(void);

/*- Variables ---------------------------------------------------------------*/
extern int main(void);
extern void _stack_top(void);
extern unsigned int _etext;
extern unsigned int _data;
extern unsigned int _edata;
extern unsigned int _bss;
extern unsigned int _ebss;

//-----------------------------------------------------------------------------
__attribute__ ((used, section(".vectors")))
void (* const vectors[])(void) =
{
  &_stack_top,
  irq_handler_reset,
};

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
__attribute__ ((noinline, section(".romfunc")))
void irq_handler_reset(void)
{
  unsigned int *src, *dst;

  src = &_etext;
  dst = &_data;
  while (dst < &_edata)
    *dst++ = *src++;

  dst = &_bss;
  while (dst < &_ebss)
    *dst++ = 0;

  main();
}

