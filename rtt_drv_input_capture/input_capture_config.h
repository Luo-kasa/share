/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2026-01-08     28784       the first version
 */
#ifndef DRIVERS_INCLUDE_CONFIG_INPUT_CAPTURE_CONFIG_C_
#define DRIVERS_INCLUDE_CONFIG_INPUT_CAPTURE_CONFIG_C_

#include "board.h"
#ifdef __cplusplus
extern "C" {
#endif

#ifdef RT_USING_INPUT_CAPTURE

#if defined(BSP_USING_TIMER1_CAPTURE) && defined(TIMER1_CAPTURE_CHANNEL1)
#ifndef TIMER1_CAPTURE_CH1_CONFIG
#define TIMER1_CAPTURE_CH1_CONFIG               \
        {                                       \
    .timer.Instance          = TIM1,            \
    .name                    = "tim1_ic1",           \
    .u32PluseCnt             = 0,               \
    .irq                     = TIM1_CC_IRQn,       \
    .ch                      = TIM_CHANNEL_1,   \
        }
#endif /* TIMER1_CAPTURE_CH1_CONFIG */
#endif /* defined(BSP_USING_TIMER1_CAPTURE) && defined(TIMER1_CAPTURE_CHANNEL1) */

#if defined(BSP_USING_TIMER2_CAPTURE) && defined(TIMER2_CAPTURE_CHANNEL2)
#ifndef TIMER2_CAPTURE_CH2_CONFIG
#define TIMER2_CAPTURE_CH2_CONFIG               \
        {                                       \
    .timer.Instance          = TIM2,            \
    .name                    = "tim2_ic2",           \
    .u32PluseCnt             = 0,               \
    .irq                     = TIM2_IRQn,       \
    .ch                      = TIM_CHANNEL_2,   \
        }
#endif /* TIMER2_CAPTURE_CH2_CONFIG */
#endif /* defined(BSP_USING_TIMER2_CAPTURE) && defined(TIMER2_CAPTURE_CHANNEL2) */

#if defined(BSP_USING_TIMER3_CAPTURE) && defined(TIMER3_CAPTURE_CHANNEL2)
#ifndef TIMER3_CAPTURE_CH2_CONFIG
#define TIMER3_CAPTURE_CH2_CONFIG               \
        {                                       \
    .timer.Instance          = TIM3,            \
    .name                    = "tim3_ic2",           \
    .u32PluseCnt             = 0,               \
    .irq                     = TIM3_IRQn,       \
    .ch                      = TIM_CHANNEL_2,   \
        }
#endif /* TIMER3_CAPTURE_CH2_CONFIG */
#endif /* defined(BSP_USING_TIMER3_CAPTURE) && defined(TIMER3_CAPTURE_CHANNEL2) */

#if defined(BSP_USING_TIMER4_CAPTURE) && defined(TIMER4_CAPTURE_CHANNEL1)
#ifndef TIMER4_CAPTURE_CH1_CONFIG
#define TIMER4_CAPTURE_CH1_CONFIG               \
        {                                       \
    .timer.Instance          = TIM4,            \
    .name                    = "tim4_ic1",           \
    .u32PluseCnt             = 0,               \
    .irq                     = TIM4_IRQn,       \
    .ch                      = TIM_CHANNEL_1,   \
        }
#endif /* TIMER4_CAPTURE_CH1_CONFIG */
#endif /* defined(BSP_USING_TIMER4_CAPTURE) && defined(TIMER4_CAPTURE_CHANNEL1) */

#if defined(BSP_USING_TIMER4_CAPTURE) && defined(TIMER4_CAPTURE_CHANNEL2)
#ifndef TIMER4_CAPTURE_CH2_CONFIG
#define TIMER4_CAPTURE_CH2_CONFIG               \
        {                                       \
    .timer.Instance          = TIM4,            \
    .name                    = "tim4_ic2",           \
    .u32PluseCnt             = 0,               \
    .irq                     = TIM4_IRQn,       \
    .ch                      = TIM_CHANNEL_2,   \
        }
#endif /* TIMER4_CAPTURE_CH1_CONFIG */
#endif /* BSP_USING_TIMER4_CAPTURE */

#if defined(BSP_USING_TIMER4_CAPTURE) && defined(TIMER4_CAPTURE_CHANNEL3)
#ifndef TIMER4_CAPTURE_CH3_CONFIG
#define TIMER4_CAPTURE_CH3_CONFIG               \
        {                                       \
    .timer.Instance          = TIM4,            \
    .name                    = "tim4_ic3",           \
    .u32PluseCnt             = 0,               \
    .irq                     = TIM4_IRQn,       \
    .ch                      = TIM_CHANNEL_3,   \
        }
#endif /* TIMER4_CAPTURE_CH1_CONFIG */
#endif /* BSP_USING_TIMER4_CAPTURE */

#if defined(BSP_USING_TIMER4_CAPTURE) && defined(TIMER4_CAPTURE_CHANNEL4)
#ifndef TIMER4_CAPTURE_CH4_CONFIG
#define TIMER4_CAPTURE_CH4_CONFIG               \
        {                                       \
    .timer.Instance          = TIM4,            \
    .name                    = "tim4_ic4",           \
    .u32PluseCnt             = 0,               \
    .irq                     = TIM4_IRQn,       \
    .ch                      = TIM_CHANNEL_4,   \
        }
#endif /* TIMER4_CAPTURE_CH1_CONFIG */
#endif /* BSP_USING_TIMER4_CAPTURE */

#endif /* RT_USING_INPUT_CAPTURE */

#endif /* DRIVERS_INCLUDE_CONFIG_INPUT_CAPTURE_CONFIG_H_ */

