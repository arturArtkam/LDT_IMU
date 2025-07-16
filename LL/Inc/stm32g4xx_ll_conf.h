

#ifndef __STM32G4xx_LL_CONF_H
#define __STM32G4xx_LL_CONF_H


/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx.h"
#include "stm32g4xx_ll_adc.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_comp.h"
#include "stm32g4xx_ll_cordic.h"
#include "stm32g4xx_ll_cortex.h"
#include "stm32g4xx_ll_crc.h"
#include "stm32g4xx_ll_crs.h"
#include "stm32g4xx_ll_dac.h"
#include "stm32g4xx_ll_dma.h"
#include "stm32g4xx_ll_dmamux.h"
#include "stm32g4xx_ll_exti.h"
#include "stm32g4xx_ll_fmac.h"
//#include "stm32g4xx_ll_fmc.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_hrtim.h"
#include "stm32g4xx_ll_i2c.h"
#include "stm32g4xx_ll_iwdg.h"
#include "stm32g4xx_ll_lptim.h"
#include "stm32g4xx_ll_lpuart.h"
#include "stm32g4xx_ll_opamp.h"
#include "stm32g4xx_ll_pwr.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_rng.h"
#include "stm32g4xx_ll_rtc.h"
#include "stm32g4xx_ll_spi.h"
#include "stm32g4xx_ll_system.h"
#include "stm32g4xx_ll_tim.h"
#include "stm32g4xx_ll_ucpd.h"
#include "stm32g4xx_ll_usart.h"
//#include "stm32g4xx_ll_usb.h"
#include "stm32g4xx_ll_utils.h"
#include "stm32g4xx_ll_wwdg.h"


/* Private define ------------------------------------------------------------*/
#ifndef NVIC_PRIORITYGROUP_0
#define NVIC_PRIORITYGROUP_0         ((uint32_t)0x00000007) /*!< 0 bit  For pre-emption priority,
                                                                 4 bits For subpriority */
#define NVIC_PRIORITYGROUP_1         ((uint32_t)0x00000006) /*!< 1 bit  For pre-emption priority,
                                                                 3 bits For subpriority */
#define NVIC_PRIORITYGROUP_2         ((uint32_t)0x00000005) /*!< 2 bits For pre-emption priority,
                                                                 2 bits For subpriority */
#define NVIC_PRIORITYGROUP_3         ((uint32_t)0x00000004) /*!< 3 bits For pre-emption priority,
                                                                 1 bit  For subpriority */
#define NVIC_PRIORITYGROUP_4         ((uint32_t)0x00000003) /*!< 4 bits For pre-emption priority,
                                                                 0 bit  For subpriority */
#endif

#endif /* __STM32G4xx_LL_CONF_H */

