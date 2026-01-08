/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2026-01-08     28784       the first version
 */

/*
 * ==>>一般注意事项：
 * 中断NVIC使能需在cubemx中设置，生成的msp函数会有
 * 在RT-Thread Settings设置的环形缓冲区是以8字节为单位的，因此不应太大，避免使用过多堆空间
 * 使用TIM1（高级定时器）需要注意其中断处理函数的名字
 * 其他定时器的中断处理函数需要自己加
 * 其他定时器的枚举索引需要自己加（按顺序）
 * 其他定时器的config需要自己加（在drv_config.h文件中写定义，在drv_inputcapture.c中写声明）
 * 其他定时器的中断处理函数中调用的isr函数中的条件编译需要自己加
 * 其他定时器的初始化init需要自己加（在drv_inputcapture.c中的stm32_timer_capture_init函数中的if else判断语句）
 * 触发回调后，一定要清空环形缓冲区数据，否则满时将警告缓冲区空间不足
 * ==>>IC与pwm同定时器：
 * 与pwm同定时器的话pwm设置的周期会影响输入捕获的周期
 * 周期>=10000000ns时影响捕获准确度较小，但实际上不会这么长时间的周期
 * 周期<=10000000ns时影响捕获准确度很大，周期越小影响越大
 * 周期过小时会导致中断频繁而占用较多的cpu从而使线程无法清除缓冲区而报错
 * 因此不建议一个定时器同时使用IC与pwm

 * 原文链接：https://club.rt-thread.org/ask/article/798724ca63ab008c.html
 * */

#include <board.h>
#define DRV_DEBUG
#define LOG_TAG             "drv.tcap"
#include <drv_log.h>
#include <rtconfig.h>

#ifdef RT_USING_INPUT_CAPTURE
#include <rtdevice.h>
#include "drv_config.h"

/* Private typedef --------------------------------------------------------------*/
typedef struct stm32_capture_device{
    struct rt_inputcapture_device parent;   // 上层句柄
    TIM_HandleTypeDef   timer;              // 定时器句柄
    IRQn_Type   irq;                        // 中断类型
    char*       name;                       // 应用层rt_device_find时用这个名字
    rt_uint32_t ch;                         // 不是十进制1/2/3/4，是TIM_CHANNEL_1/TIM_CHANNEL_2...
    rt_uint32_t u32LastCnt;                 // 每次捕获边沿时的计数值
    rt_uint32_t u32PluseCnt;                // 高/低电平持续时间
    rt_uint32_t over_under_flowcount;       // 定时器计数溢出次数
    rt_uint8_t  input_data_level;           // 高/低电平
    rt_uint8_t  not_first_edge;             // 不是第一边沿（首次检测下降沿，1：不是第一边沿，0：是第一边沿，初始化为0）
}stm32_capture_device;
/* Private functions ------------------------------------------------------------*/
static  rt_err_t stm32_capture_init(struct rt_inputcapture_device *inputcapture);
static  rt_err_t stm32_capture_open(struct rt_inputcapture_device *inputcapture);
static  rt_err_t stm32_capture_close(struct rt_inputcapture_device *inputcapture);
static  rt_err_t stm32_capture_get_pulsewidth(struct rt_inputcapture_device *inputcapture, rt_uint32_t *pulsewidth_us);
/* Private define ---------------------------------------------------------------*/
/* Public functions -------------------------------------------------------------*/
/* Private variables ------------------------------------------------------------*/
enum
{
#ifdef BSP_USING_TIMER1_CAPTURE
#ifdef TIMER1_CAPTURE_CHANNEL1
    TIMER1_CAPTURE_CH1_INDEX,
#endif
#ifdef TIMER1_CAPTURE_CHANNEL2
    TIMER1_CAPTURE_CH2_INDEX,
#endif
#ifdef TIMER1_CAPTURE_CHANNEL3
    TIMER1_CAPTURE_CH3_INDEX,
#endif
#ifdef TIMER1_CAPTURE_CHANNEL4
    TIMER1_CAPTURE_CH4_INDEX,
#endif
#endif

#ifdef BSP_USING_TIMER2_CAPTURE
#ifdef TIMER2_CAPTURE_CHANNEL1
    TIMER2_CAPTURE_CH1_INDEX,
#endif
#ifdef TIMER2_CAPTURE_CHANNEL2
    TIMER2_CAPTURE_CH2_INDEX,
#endif
#ifdef TIMER2_CAPTURE_CHANNEL3
    TIMER2_CAPTURE_CH3_INDEX,
#endif
#ifdef TIMER2_CAPTURE_CHANNEL4
    TIMER2_CAPTURE_CH4_INDEX,
#endif
#endif

#ifdef BSP_USING_TIMER3_CAPTURE
#ifdef TIMER3_CAPTURE_CHANNEL1
    TIMER3_CAPTURE_CH1_INDEX,
#endif
#ifdef TIMER3_CAPTURE_CHANNEL2
    TIMER3_CAPTURE_CH2_INDEX,
#endif
#ifdef TIMER3_CAPTURE_CHANNEL3
    TIMER3_CAPTURE_CH3_INDEX,
#endif
#ifdef TIMER3_CAPTURE_CHANNEL4
    TIMER3_CAPTURE_CH4_INDEX,
#endif
#endif

#ifdef BSP_USING_TIMER4_CAPTURE
#ifdef TIMER4_CAPTURE_CHANNEL1
    TIMER4_CAPTURE_CH1_INDEX,
#endif
#ifdef TIMER4_CAPTURE_CHANNEL2
    TIMER4_CAPTURE_CH2_INDEX,
#endif
#ifdef TIMER4_CAPTURE_CHANNEL3
    TIMER4_CAPTURE_CH3_INDEX,
#endif
#ifdef TIMER4_CAPTURE_CHANNEL4
    TIMER4_CAPTURE_CH4_INDEX,
#endif
#endif
    TIMER_CAPTURE_INDEX_MAX,
};

static struct stm32_capture_device stm32_capture_obj[] =
{
#ifdef BSP_USING_TIMER1_CAPTURE
#ifdef TIMER1_CAPTURE_CHANNEL1
        TIMER1_CAPTURE_CH1_CONFIG,
#endif
#ifdef TIMER1_CAPTURE_CHANNEL2
        TIMER1_CAPTURE_CH2_CONFIG,
#endif
#ifdef TIMER1_CAPTURE_CHANNEL3
        TIMER1_CAPTURE_CH3_CONFIG,
#endif
#ifdef TIMER1_CAPTURE_CHANNEL4
        TIMER1_CAPTURE_CH4_CONFIG,
#endif
#endif /* BSP_USING_TIMER1_CAPTURE */

#ifdef BSP_USING_TIMER2_CAPTURE
#ifdef TIMER2_CAPTURE_CHANNEL1
        TIMER2_CAPTURE_CH1_CONFIG,
#endif
#ifdef TIMER2_CAPTURE_CHANNEL2
        TIMER2_CAPTURE_CH2_CONFIG,
#endif
#ifdef TIMER2_CAPTURE_CHANNEL3
        TIMER2_CAPTURE_CH3_CONFIG,
#endif
#ifdef TIMER2_CAPTURE_CHANNEL4
        TIMER2_CAPTURE_CH4_CONFIG,
#endif
#endif /* BSP_USING_TIMER2_CAPTURE */

#ifdef BSP_USING_TIMER3_CAPTURE
#ifdef TIMER3_CAPTURE_CHANNEL1
        TIMER3_CAPTURE_CH1_CONFIG,
#endif
#ifdef TIMER3_CAPTURE_CHANNEL2
        TIMER3_CAPTURE_CH2_CONFIG,
#endif
#ifdef TIMER3_CAPTURE_CHANNEL3
        TIMER3_CAPTURE_CH3_CONFIG,
#endif
#ifdef TIMER3_CAPTURE_CHANNEL4
        TIMER3_CAPTURE_CH4_CONFIG,
#endif
#endif /* BSP_USING_TIMER3_CAPTURE */

#ifdef BSP_USING_TIMER4_CAPTURE
#ifdef TIMER4_CAPTURE_CHANNEL1
        TIMER4_CAPTURE_CH1_CONFIG,
#endif
#ifdef TIMER4_CAPTURE_CHANNEL2
        TIMER4_CAPTURE_CH2_CONFIG,
#endif
#ifdef TIMER4_CAPTURE_CHANNEL3
        TIMER4_CAPTURE_CH3_CONFIG,
#endif
#ifdef TIMER4_CAPTURE_CHANNEL4
        TIMER4_CAPTURE_CH4_CONFIG,
#endif
#endif /* BSP_USING_TIMER4_CAPTURE */
};
static struct rt_inputcapture_ops stm32_capture_ops = {
        .init   =   stm32_capture_init,
        .open   =   stm32_capture_open,
        .close  =   stm32_capture_close,
        .get_pulsewidth =   stm32_capture_get_pulsewidth,
};
/* Functions define ------------------------------------------------------------*/
void input_capture_cc1_isr(struct stm32_capture_device* device)
{
    /* Capture compare 1 event */
    if (__HAL_TIM_GET_FLAG(&device->timer, TIM_FLAG_CC1) != RESET)
    {
        if (__HAL_TIM_GET_IT_SOURCE(&device->timer, TIM_IT_CC1) != RESET)
        {
            __HAL_TIM_CLEAR_IT(&device->timer, TIM_IT_CC1);
#if defined(TIMER1_CAPTURE_CHANNEL1) || defined(TIMER2_CAPTURE_CHANNEL1) || defined(TIMER3_CAPTURE_CHANNEL1) || \
        defined(TIMER4_CAPTURE_CHANNEL1)
            device->timer.Channel = HAL_TIM_ACTIVE_CHANNEL_1;
            if ((device->timer.Instance->CCMR1 & TIM_CCMR1_CC1S) != 0x00U)// input capture
            {
                rt_uint32_t cnt = HAL_TIM_ReadCapturedValue(&device->timer, device->ch);//获取当前的捕获值.
                if(!device->not_first_edge){    //首次检测下降沿
                    device->not_first_edge = 1;
                    device->input_data_level = 0;
                    device->u32LastCnt = cnt;
                }else{
                    device->u32PluseCnt = cnt + 0x10000 * device->over_under_flowcount - device->u32LastCnt;
                    rt_hw_inputcapture_isr(&device->parent, device->input_data_level);
                    device->input_data_level = !device->input_data_level;
                }
                if(device->input_data_level)
                    __HAL_TIM_SET_CAPTUREPOLARITY(&device->timer, device->ch, TIM_INPUTCHANNELPOLARITY_FALLING);     //切换捕获极性
                else
                    __HAL_TIM_SET_CAPTUREPOLARITY(&device->timer, device->ch, TIM_INPUTCHANNELPOLARITY_RISING);    //切换捕获极性
                device->over_under_flowcount = 0;
                device->u32LastCnt = cnt;

            }
            device->timer.Channel = HAL_TIM_ACTIVE_CHANNEL_CLEARED;
#endif /* TIMERx_CAPTURE_CHANNEL1 */
        }
    }
}
void input_capture_cc2_isr(struct stm32_capture_device* device)
{
    /* Capture compare 2 event */
    if (__HAL_TIM_GET_FLAG(&device->timer, TIM_FLAG_CC2) != RESET)
    {
        if (__HAL_TIM_GET_IT_SOURCE(&device->timer, TIM_IT_CC2) != RESET)
        {
            __HAL_TIM_CLEAR_IT(&device->timer, TIM_IT_CC2);
#if defined(TIMER1_CAPTURE_CHANNEL2) || defined(TIMER2_CAPTURE_CHANNEL2) || defined(TIMER3_CAPTURE_CHANNEL2) || \
        defined(TIMER4_CAPTURE_CHANNEL2)
            device->timer.Channel = HAL_TIM_ACTIVE_CHANNEL_2;
            if ((device->timer.Instance->CCMR1 & TIM_CCMR1_CC2S) != 0x00U)// input capture
            {
                rt_uint32_t cnt = HAL_TIM_ReadCapturedValue(&device->timer, device->ch);//获取当前的捕获值.
                if(!device->not_first_edge){    //首次检测下降沿
                    device->not_first_edge = 1;
                    device->input_data_level = 0;
                    device->u32LastCnt = cnt;
                }else{
                    device->u32PluseCnt = cnt + 0x10000 * device->over_under_flowcount - device->u32LastCnt;
                    rt_hw_inputcapture_isr(&device->parent, device->input_data_level);
                    device->input_data_level = !device->input_data_level;
                }
                if(device->input_data_level)
                    __HAL_TIM_SET_CAPTUREPOLARITY(&device->timer, device->ch, TIM_INPUTCHANNELPOLARITY_FALLING);     //切换捕获极性
                else
                    __HAL_TIM_SET_CAPTUREPOLARITY(&device->timer, device->ch, TIM_INPUTCHANNELPOLARITY_RISING);    //切换捕获极性
                device->over_under_flowcount = 0;
                device->u32LastCnt = cnt;
            }
            device->timer.Channel = HAL_TIM_ACTIVE_CHANNEL_CLEARED;
#endif /* TIMERx_CAPTURE_CHANNEL2 */
        }
    }
}
void input_capture_cc3_isr(struct stm32_capture_device* device)
{
    /* Capture compare 3 event */
    if (__HAL_TIM_GET_FLAG(&device->timer, TIM_FLAG_CC3) != RESET)
    {
        if (__HAL_TIM_GET_IT_SOURCE(&device->timer, TIM_IT_CC3) != RESET)
        {
            __HAL_TIM_CLEAR_IT(&device->timer, TIM_IT_CC3);
#if defined(TIMER1_CAPTURE_CHANNEL3) || defined(TIMER2_CAPTURE_CHANNEL3) || defined(TIMER3_CAPTURE_CHANNEL3) || \
        defined(TIMER4_CAPTURE_CHANNEL3)
            device->timer.Channel = HAL_TIM_ACTIVE_CHANNEL_3;
            if ((device->timer.Instance->CCMR2 & TIM_CCMR2_CC3S) != 0x00U)// input capture
            {
                rt_uint32_t cnt = HAL_TIM_ReadCapturedValue(&device->timer, device->ch);//获取当前的捕获值.
                if(!device->not_first_edge){    //首次检测下降沿
                    device->not_first_edge = 1;
                    device->input_data_level = 0;
                    device->u32LastCnt = cnt;
                }else{
                    device->u32PluseCnt = cnt + 0x10000 * device->over_under_flowcount - device->u32LastCnt;
                    rt_hw_inputcapture_isr(&device->parent, device->input_data_level);
                    device->input_data_level = !device->input_data_level;
                }
                if(device->input_data_level)
                    __HAL_TIM_SET_CAPTUREPOLARITY(&device->timer, device->ch, TIM_INPUTCHANNELPOLARITY_FALLING);     //切换捕获极性
                else
                    __HAL_TIM_SET_CAPTUREPOLARITY(&device->timer, device->ch, TIM_INPUTCHANNELPOLARITY_RISING);    //切换捕获极性
                device->over_under_flowcount = 0;
                device->u32LastCnt = cnt;

            }
            device->timer.Channel = HAL_TIM_ACTIVE_CHANNEL_CLEARED;
#endif /* TIMERx_CAPTURE_CHANNEL3 */
        }
    }
}
void input_capture_cc4_isr(struct stm32_capture_device* device)
{
    /* Capture compare 4 event */
    if (__HAL_TIM_GET_FLAG(&device->timer, TIM_FLAG_CC4) != RESET)
    {
        if (__HAL_TIM_GET_IT_SOURCE(&device->timer, TIM_IT_CC4) != RESET)
        {
            __HAL_TIM_CLEAR_IT(&device->timer, TIM_IT_CC4);
#if defined(TIMER1_CAPTURE_CHANNEL4) || defined(TIMER2_CAPTURE_CHANNEL4) || defined(TIMER3_CAPTURE_CHANNEL4) || \
        defined(TIMER4_CAPTURE_CHANNEL4)
            device->timer.Channel = HAL_TIM_ACTIVE_CHANNEL_4;
            if ((device->timer.Instance->CCMR2 & TIM_CCMR2_CC4S) != 0x00U)// input capture
            {
                rt_uint32_t cnt = HAL_TIM_ReadCapturedValue(&device->timer, device->ch);//获取当前的捕获值.
                if(!device->not_first_edge){    //首次检测下降沿
                    device->not_first_edge = 1;
                    device->input_data_level = 0;
                    device->u32LastCnt = cnt;
                }else{
                    device->u32PluseCnt = cnt + 0x10000 * device->over_under_flowcount - device->u32LastCnt;
                    rt_hw_inputcapture_isr(&device->parent, device->input_data_level);
                    device->input_data_level = !device->input_data_level;
                }
                if(device->input_data_level)
                    __HAL_TIM_SET_CAPTUREPOLARITY(&device->timer, device->ch, TIM_INPUTCHANNELPOLARITY_FALLING);     //切换捕获极性
                else
                    __HAL_TIM_SET_CAPTUREPOLARITY(&device->timer, device->ch, TIM_INPUTCHANNELPOLARITY_RISING);    //切换捕获极性
                device->over_under_flowcount = 0;
                device->u32LastCnt = cnt;

            }
            device->timer.Channel = HAL_TIM_ACTIVE_CHANNEL_CLEARED;
#endif /* TIMERx_CAPTURE_CHANNEL4 */
        }
    }
}

#ifdef BSP_USING_TIMER1_CAPTURE
void TIM1_UP_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();
    TIM_HandleTypeDef timer;
#if defined(TIMER1_CAPTURE_CHANNEL1)
    timer = stm32_capture_obj[TIMER1_CAPTURE_CH1_INDEX].timer;
#endif
#if defined(TIMER1_CAPTURE_CHANNEL2)
    timer = stm32_capture_obj[TIMER1_CAPTURE_CH2_INDEX].timer;
#endif
#if defined(TIMER1_CAPTURE_CHANNEL3)
    timer = stm32_capture_obj[TIMER1_CAPTURE_CH3_INDEX].timer;
#endif
#if defined(TIMER1_CAPTURE_CHANNEL4)
    timer = stm32_capture_obj[TIMER1_CAPTURE_CH4_INDEX].timer;
#endif
    /* TIM Update event */
    if (__HAL_TIM_GET_FLAG(&timer, TIM_FLAG_UPDATE) != RESET)
    {
        if (__HAL_TIM_GET_IT_SOURCE(&timer, TIM_IT_UPDATE) != RESET)
        {
            __HAL_TIM_CLEAR_IT(&timer, TIM_IT_UPDATE);
            for(rt_uint8_t i = 0; i < TIMER_CAPTURE_INDEX_MAX; i++)
            {
                if (stm32_capture_obj[i].timer.Instance == TIM1) {// 避免其他的TIM参数被修改
                    stm32_capture_obj[i].over_under_flowcount++;
                }
            }
        }
    }
    /* leave interrupt */
    rt_interrupt_leave();
}
void TIM1_CC_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();
#if defined(TIMER1_CAPTURE_CHANNEL1)
    input_capture_cc1_isr(&stm32_capture_obj[TIMER1_CAPTURE_CH1_INDEX]);
#endif
#if defined(TIMER1_CAPTURE_CHANNEL2)
    input_capture_cc2_isr(&stm32_capture_obj[TIMER1_CAPTURE_CH2_INDEX]);
#endif
#if defined(TIMER1_CAPTURE_CHANNEL3)
    input_capture_cc3_isr(&stm32_capture_obj[TIMER1_CAPTURE_CH3_INDEX]);
#endif
#if defined(TIMER1_CAPTURE_CHANNEL4)
    input_capture_cc4_isr(&stm32_capture_obj[TIMER1_CAPTURE_CH4_INDEX]);
#endif
    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* BSP_USING_TIMER1_CAPTURE */

#ifdef BSP_USING_TIMER3_CAPTURE
void TIM3_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();
    TIM_HandleTypeDef timer;// 4个通道的timer句柄都一样，用哪个都行，条件编译是为了避免不知道使用了哪个通道
#if defined(TIMER3_CAPTURE_CHANNEL1)
    input_capture_cc1_isr(&stm32_capture_obj[TIMER3_CAPTURE_CH1_INDEX]);
    timer = stm32_capture_obj[TIMER3_CAPTURE_CH1_INDEX].timer;
#endif
#if defined(TIMER3_CAPTURE_CHANNEL2)
    input_capture_cc2_isr(&stm32_capture_obj[TIMER3_CAPTURE_CH2_INDEX]);
    timer = stm32_capture_obj[TIMER3_CAPTURE_CH2_INDEX].timer;
#endif
#if defined(TIMER3_CAPTURE_CHANNEL3)
    input_capture_cc3_isr(&stm32_capture_obj[TIMER3_CAPTURE_CH3_INDEX]);
    timer = stm32_capture_obj[TIMER3_CAPTURE_CH3_INDEX].timer;
#endif
#if defined(TIMER3_CAPTURE_CHANNEL4)
    input_capture_cc4_isr(&stm32_capture_obj[TIMER3_CAPTURE_CH4_INDEX]);
    timer = stm32_capture_obj[TIMER3_CAPTURE_CH4_INDEX].timer;
#endif
    /* TIM Update event */
    if (__HAL_TIM_GET_FLAG(&timer, TIM_FLAG_UPDATE) != RESET)
    {
        if (__HAL_TIM_GET_IT_SOURCE(&timer, TIM_IT_UPDATE) != RESET)
        {
            __HAL_TIM_CLEAR_IT(&timer, TIM_IT_UPDATE);
            for(rt_uint8_t i = 0; i < TIMER_CAPTURE_INDEX_MAX; i++)
            {
                if (stm32_capture_obj[i].timer.Instance == TIM3) {// 避免其他的TIM参数被修改
                    stm32_capture_obj[i].over_under_flowcount++;
                }
            }
        }
    }
    /* TIM Break input event */
    if (__HAL_TIM_GET_FLAG(&timer, TIM_FLAG_BREAK) != RESET)
    {
        __HAL_TIM_CLEAR_IT(&timer, TIM_IT_BREAK);
    }
    /* TIM Trigger detection event */
    if (__HAL_TIM_GET_FLAG(&timer, TIM_FLAG_TRIGGER) != RESET)
    {
        __HAL_TIM_CLEAR_IT(&timer, TIM_IT_TRIGGER);
    }
    /* TIM commutation event */
    if (__HAL_TIM_GET_FLAG(&timer, TIM_FLAG_COM) != RESET)
    {
        __HAL_TIM_CLEAR_IT(&timer, TIM_FLAG_COM);
    }
    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* BSP_USING_TIMER3_CAPTURE */

#ifdef BSP_USING_TIMER2_CAPTURE
void TIM2_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();
    TIM_HandleTypeDef timer;
#if defined(TIMER2_CAPTURE_CHANNEL1)
    input_capture_cc1_isr(&stm32_capture_obj[TIMER2_CAPTURE_CH1_INDEX]);
    timer = stm32_capture_obj[TIMER2_CAPTURE_CH1_INDEX].timer;
#endif
#if defined(TIMER2_CAPTURE_CHANNEL2)
    input_capture_cc2_isr(&stm32_capture_obj[TIMER2_CAPTURE_CH2_INDEX]);
    timer = stm32_capture_obj[TIMER2_CAPTURE_CH2_INDEX].timer;
#endif
#if defined(TIMER2_CAPTURE_CHANNEL3)
    input_capture_cc3_isr(&stm32_capture_obj[TIMER2_CAPTURE_CH3_INDEX]);
    timer = stm32_capture_obj[TIMER3_CAPTURE_CH3_INDEX].timer;
#endif
#if defined(TIMER2_CAPTURE_CHANNEL4)
    input_capture_cc2_isr(&stm32_capture_obj[TIMER2_CAPTURE_CH4_INDEX]);
    timer = stm32_capture_obj[TIMER2_CAPTURE_CH4_INDEX].timer;
#endif
    /* TIM Update event */
    if (__HAL_TIM_GET_FLAG(&timer, TIM_FLAG_UPDATE) != RESET)
    {
        if (__HAL_TIM_GET_IT_SOURCE(&timer, TIM_IT_UPDATE) != RESET)
        {
            __HAL_TIM_CLEAR_IT(&timer, TIM_IT_UPDATE);
            for(rt_uint8_t i = 0; i < TIMER_CAPTURE_INDEX_MAX; i++)
            {
                if (stm32_capture_obj[i].timer.Instance == TIM2) {// 避免其他的TIM参数被修改
                    stm32_capture_obj[i].over_under_flowcount++;
                }
            }
        }
    }
    /* TIM Break input event */
    if (__HAL_TIM_GET_FLAG(&timer, TIM_FLAG_BREAK) != RESET)
    {
        __HAL_TIM_CLEAR_IT(&timer, TIM_IT_BREAK);
    }
    /* TIM Trigger detection event */
    if (__HAL_TIM_GET_FLAG(&timer, TIM_FLAG_TRIGGER) != RESET)
    {
        __HAL_TIM_CLEAR_IT(&timer, TIM_IT_TRIGGER);
    }
    /* TIM commutation event */
    if (__HAL_TIM_GET_FLAG(&timer, TIM_FLAG_COM) != RESET)
    {
        __HAL_TIM_CLEAR_IT(&timer, TIM_FLAG_COM);
    }
    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* BSP_USING_TIMER2_CAPTURE */

#ifdef BSP_USING_TIMER4_CAPTURE
void TIM4_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();
    TIM_HandleTypeDef timer;
#if defined(TIMER4_CAPTURE_CHANNEL1)
    input_capture_cc1_isr(&stm32_capture_obj[TIMER4_CAPTURE_CH1_INDEX]);
    timer = stm32_capture_obj[TIMER4_CAPTURE_CH1_INDEX].timer;
#endif
#if defined(TIMER4_CAPTURE_CHANNEL2)
    input_capture_cc2_isr(&stm32_capture_obj[TIMER4_CAPTURE_CH2_INDEX]);
    timer = stm32_capture_obj[TIMER4_CAPTURE_CH2_INDEX].timer;
#endif
#if defined(TIMER4_CAPTURE_CHANNEL3)
    input_capture_cc3_isr(&stm32_capture_obj[TIMER4_CAPTURE_CH3_INDEX]);
    timer = stm32_capture_obj[TIMER3_CAPTURE_CH3_INDEX].timer;
#endif
#if defined(TIMER4_CAPTURE_CHANNEL4)
    input_capture_cc4_isr(&stm32_capture_obj[TIMER4_CAPTURE_CH4_INDEX]);
    timer = stm32_capture_obj[TIMER4_CAPTURE_CH4_INDEX].timer;
#endif
    /* TIM Update event */
    if (__HAL_TIM_GET_FLAG(&timer, TIM_FLAG_UPDATE) != RESET)
    {
        if (__HAL_TIM_GET_IT_SOURCE(&timer, TIM_IT_UPDATE) != RESET)
        {
            __HAL_TIM_CLEAR_IT(&timer, TIM_IT_UPDATE);
            for(rt_uint8_t i = 0; i < TIMER_CAPTURE_INDEX_MAX; i++)
            {
                if (stm32_capture_obj[i].timer.Instance == TIM4) {// 避免其他的TIM参数被修改
                    stm32_capture_obj[i].over_under_flowcount++;
                }
            }
        }
    }
    /* TIM Break input event */
    if (__HAL_TIM_GET_FLAG(&timer, TIM_FLAG_BREAK) != RESET)
    {
        __HAL_TIM_CLEAR_IT(&timer, TIM_IT_BREAK);
    }
    /* TIM Trigger detection event */
    if (__HAL_TIM_GET_FLAG(&timer, TIM_FLAG_TRIGGER) != RESET)
    {
        __HAL_TIM_CLEAR_IT(&timer, TIM_IT_TRIGGER);
    }
    /* TIM commutation event */
    if (__HAL_TIM_GET_FLAG(&timer, TIM_FLAG_COM) != RESET)
    {
        __HAL_TIM_CLEAR_IT(&timer, TIM_FLAG_COM);
    }
    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* BSP_USING_TIMER4_CAPTURE */

static rt_err_t stm32_capture_get_pulsewidth(struct rt_inputcapture_device *inputcapture, rt_uint32_t *pulsewidth_us)
{
    rt_err_t ret = RT_EOK;
    struct stm32_capture_device *stm32_capture;
    stm32_capture = (stm32_capture_device *)inputcapture;
    *pulsewidth_us = stm32_capture->u32PluseCnt;
    return -(ret);
}
int zzz = 0;
static rt_err_t stm32_timer_capture_init(struct stm32_capture_device* device)
{
    rt_err_t ret = RT_EOK;
    static rt_uint8_t tim3_init = 0;
    static rt_uint8_t tim4_init = 0;
    TIM_HandleTypeDef *tim = RT_NULL;
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_IC_InitTypeDef sConfigIC = {0};

    tim = (TIM_HandleTypeDef *)&device->timer;

    //    rt_uint8_t psc = (HAL_RCC_GetPCLK1Freq() * 2) / 1000000UL - 1;

    /* 同一个定时器避免重复初始化, 但不同通道要配置*/
    /* 其他的定时器需自行定义 */
    if (tim->Instance == TIM3) {
        if (tim3_init == 0) { //未初始化
            tim3_init = 1;
            tim->Init.Prescaler = 72-1;
            tim->Init.Period    = 0xffff;
            goto tim_init;
        }
        else
            goto channel_config;
    }
    else if(tim->Instance == TIM4) {
        if (tim4_init == 0) { //未初始化
            tim4_init = 1;
            tim->Init.Prescaler = 72-1;
            tim->Init.Period    = 0xffff;
            goto tim_init;
        }
        else
            goto channel_config;
    }
    else {
        tim->Init.Prescaler = 72-1;
        tim->Init.Period    = 0xffff;
        LOG_I("need to add code by yourself");
    }

    tim_init:
    tim->Init.CounterMode = TIM_COUNTERMODE_UP;
    tim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    tim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(tim) != HAL_OK){

        Error_Handler();
    }
    if (HAL_TIM_IC_Init(tim) != HAL_OK){
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(tim, &sClockSourceConfig) != HAL_OK){
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(tim, &sMasterConfig) != HAL_OK){
        Error_Handler();
    }
    if(HAL_OK != HAL_TIM_Base_Start_IT(tim)){
        Error_Handler();
    }
    __HAL_TIM_CLEAR_IT(tim, TIM_IT_UPDATE);//

    channel_config:
    sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;// 首次检测下降沿
    sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
    sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
    sConfigIC.ICFilter = 0;
    if (HAL_TIM_IC_ConfigChannel(tim, &sConfigIC, device->ch) != HAL_OK){
        Error_Handler();
    }

    return -(ret);
}
static rt_err_t stm32_capture_init(struct rt_inputcapture_device *inputcapture)
{
    rt_err_t ret = RT_EOK;
    RT_ASSERT(inputcapture != RT_NULL);
    if (stm32_timer_capture_init((struct stm32_capture_device *) inputcapture) != RT_EOK){
        rt_kprintf("Failed to initialize TIMER.\n");
        ret = RT_ERROR;
    }
    return -(ret);
}
static rt_err_t stm32_capture_open(struct rt_inputcapture_device *inputcapture)
{
    rt_err_t ret = RT_EOK;
    rt_uint32_t CCx = 0xffff;
    RT_ASSERT(inputcapture != RT_NULL);
    struct stm32_capture_device* device = (struct stm32_capture_device*)inputcapture;
    device->not_first_edge = 0;
    device->input_data_level = 0;
    device->over_under_flowcount = 0;
    device->u32LastCnt = 0;
    __HAL_TIM_SET_CAPTUREPOLARITY(&device->timer, device->ch, TIM_INPUTCHANNELPOLARITY_FALLING);
    if(HAL_OK != HAL_TIM_IC_Start_IT(&device->timer, device->ch)){
        ret = RT_ERROR;
    }
    switch (device->ch) {
    case TIM_CHANNEL_1:
        CCx = TIM_IT_CC1;
        break;
    case TIM_CHANNEL_2:
        CCx = TIM_IT_CC2;
        break;
    case TIM_CHANNEL_3:
        CCx = TIM_IT_CC3;
        break;
    case TIM_CHANNEL_4:
        CCx = TIM_IT_CC4;
        break;
    default:
        LOG_E("tim channel error");
        ret = RT_ERROR;
        break;
    }
    __HAL_TIM_CLEAR_IT(&device->timer, CCx);
    return -(ret);
}
static rt_err_t stm32_capture_close(struct rt_inputcapture_device *inputcapture)
{
    rt_err_t ret = RT_EOK;
    RT_ASSERT(inputcapture != RT_NULL);
    struct stm32_capture_device* device = (struct stm32_capture_device*)inputcapture;
    HAL_TIM_IC_Stop_IT(&device->timer, device->ch);
    return ret;
}
/* Init and register timer capture */
static int stm32_timer_capture_device_init(void)
{
    struct stm32_capture_device *device;
    for (rt_uint8_t i = 0; i < sizeof(stm32_capture_obj) / sizeof(stm32_capture_obj[0]); i++){
        device = &stm32_capture_obj[i];
        device->parent.ops = &stm32_capture_ops;
        if (rt_device_inputcapture_register(&device->parent, stm32_capture_obj[i].name, device)!= RT_EOK){
            LOG_E("%s register failed", stm32_capture_obj[i].name);
            return -RT_ERROR;
        }
    }
    return 0;
}
INIT_DEVICE_EXPORT(stm32_timer_capture_device_init);
#endif //#ifdef RT_USING_INPUT_CAPTURE

