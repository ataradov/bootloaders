// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2023, Alex Taradov <alex@taradov.com>. All rights reserved.

#ifndef _NVM_DATA_H_
#define _NVM_DATA_H_

// SAM D5x/E5x NVM Software Calibration Area Mapping

/*- Definitions -------------------------------------------------------------*/
#define NVM_AC_BIAS_POS              0
#define NVM_AC_BIAS_SIZE             2

#define NVM_ADC0_BIASCOMP_POS        2
#define NVM_ADC0_BIASCOMP_SIZE       3

#define NVM_ADC0_BIASREFBUF_POS      5
#define NVM_ADC0_BIASREFBUF_SIZE     3

#define NVM_ADC0_BIASR2R_POS         8
#define NVM_ADC0_BIASR2R_SIZE        3

#define NVM_ADC1_BIASCOMP_POS        16
#define NVM_ADC1_BIASCOMP_SIZE       3

#define NVM_ADC1_BIASREFBUF_POS      19
#define NVM_ADC1_BIASREFBUF_SIZE     3

#define NVM_ADC1_BIASR2R_POS         22
#define NVM_ADC1_BIASR2R_SIZE        3

#define NVM_USB_TRANSN_POS           32
#define NVM_USB_TRANSN_SIZE          5

#define NVM_USB_TRANSP_POS           37
#define NVM_USB_TRANSP_SIZE          5

#define NVM_USB_TRIM_POS             42
#define NVM_USB_TRIM_SIZE            3

#define NVM_READ_CAL(cal) \
    ((*((uint32_t *)NVMCTRL_SW0 + cal##_POS / 32)) >> (cal##_POS % 32)) & ((1 << cal##_SIZE) - 1)

#endif // _NVM_DATA_H_

