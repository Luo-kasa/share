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
 * @FILE 输入捕获的驱动测试及用法
 * @NOTE 开启输入捕获的驱动时需要将其环形缓冲区调大，或者使用rt_device_control将watermark值调大
 * @NOTE 添加文件drivers/drv_inputcapture.c、drivers/include/config/input_capture_config.h
 * @NOTE 修改文件drivers/include/drv_config.h、drivers/board.h
 * @see  drivers/drv_inputcapture.c
 */


#include "drv_common.h"

#if defined(RT_USING_INPUT_CAPTURE) && defined(BSP_USING_TIMER4_CAPTURE) && defined(TIMER4_CAPTURE_CHANNEL1)

#define LOG_LVL LOG_LVL_DBG
#define LOG_TAG "i_c_test.c"
#include "ulog.h"

#define IC_DEV_NAME "tim4_ic1"
#define IC_THREAD_STACK_SIZE 1024

static rt_device_t ic_dev = RT_NULL;
static struct rt_inputcapture_data ic_buffer[16] = {0};
static rt_uint8_t ic_buffer_size = 0;
static struct rt_semaphore ic_sem = {0};
static struct rt_thread ic_thd = {0};
static rt_uint8_t ic_thread_stack[IC_THREAD_STACK_SIZE] = {0};

/* 参数size是环形缓冲区中已存放的struct rt_inputcapture_device结构体个数，每个8BYTE
 * 环形缓冲区的值是以此为单位的，因此不要设置太大
 * 当size>=watermark时才会触发该中断回调，watermark默认为环形缓冲区的1/2 */
rt_err_t ic_rx_ind(rt_device_t dev, rt_size_t size)
{
    rt_sem_release(&ic_sem);
    return 0;
}

void ic_thd_entry(void *p)
{
    rt_device_set_rx_indicate(ic_dev, ic_rx_ind);

    while(1)
    {
        if(RT_EOK != rt_sem_take(&ic_sem, RT_WAITING_FOREVER)){
            LOG_E("tim4_ic1 rt_sem_take failed");
        }
        else {
            /* @清空信号量，原因：size（ic_rx_ind函数的参数）>=watermark后将释放信号量，
             * @但可能因线程延时使得不能及时获取信号量而清空环形缓冲区，
             * @因此可能会多次释放信号量，导致信号量积攒*/
            while(!rt_sem_trytake(&ic_sem));
            /* =>读到的结构体值是交替的高低电平持续时间及其电平值
             * =>例如{200,0}，{800,1}，{200,0}，{800,1}..... ({200,0}：200是200us，0表示低电平)
             * =>因此通过相邻的两个值即可算出占空比*/
            ic_buffer_size = rt_device_read(ic_dev, 0, ic_buffer, 10);
            /* =>执行清空，否则缓冲区满了会警告（DBG级别日志）*/
            if(RT_EOK != rt_device_control(ic_dev, INPUTCAPTURE_CMD_CLEAR_BUF, (void*)0)){
                LOG_E("tim4_ic1 rt_device_control INPUTCAPTURE_CMD_CLEAR_BUF Failed");
            }
            if(10 != ic_buffer_size){
                LOG_W("warning, tim4_ic1 ic_buffer_size = %d", ic_buffer_size);
            }
            else {
                for (int var = 0; var < 10; ++var) {
                    rt_kprintf("%s:[%d].pw_us=%d, is_high=%d\n", IC_DEV_NAME, var, ic_buffer[var].pulsewidth_us, ic_buffer[var].is_high);
                }
                rt_kprintf("\n");
            }
            rt_memset(ic_buffer, 0, sizeof(ic_buffer));
        }
        /* @延时时间可以根据pwm周期、watermark及环形缓冲区大小进行调整，避免环形缓冲区满了都还没做处理
         * @但由于rt_inputcapture.c中已经修改日志为INFO级别，因此即使延时过长而使环形缓冲区满，都不会打印日志导致程序卡住
         * @所以可以任意延时 */
        rt_thread_mdelay(200);
    }
}

int ic_init(void)
{
    ic_dev = rt_device_find(IC_DEV_NAME);
    if (ic_dev == RT_NULL) {
        LOG_E("%s rt_device_find failed", IC_DEV_NAME);
        return -1;
    }
    else {
        LOG_I("%s rt_device_find success", IC_DEV_NAME);
    }

    // 配置中断回调阈值（水位阈值）
    rt_uint8_t watermark = 10;
    if(RT_EOK != rt_device_control(ic_dev, INPUTCAPTURE_CMD_SET_WATERMARK, (void*)&watermark)) {
        LOG_E("%s rt_device_control failed", IC_DEV_NAME);
        return -1;
    }
    else {
        LOG_I("%s rt_device_control success", IC_DEV_NAME);
    }

    // 打开设备（驱动中将init和open）
    if(RT_EOK != rt_device_open(ic_dev, 0)){
        LOG_E("%s rt_device_open failed", IC_DEV_NAME);
        return -1;
    }
    else {
        LOG_I("%s rt_device_open success", IC_DEV_NAME);
    }

    //
    if(RT_EOK != rt_sem_init(&ic_sem, "ic_sem", 0, RT_IPC_FLAG_FIFO)) {
        LOG_E("%s rt_sem_init failed", IC_DEV_NAME);
    }
    else {
        LOG_I("%s rt_sem_init success", IC_DEV_NAME);
    }

    //
    if(RT_EOK != rt_thread_init(&ic_thd, "ic_thd", &ic_thd_entry,
            RT_NULL, &ic_thread_stack[0], IC_THREAD_STACK_SIZE, 15, 10)) {
        LOG_E("%s rt_thread_init failed", IC_DEV_NAME);
        return -1;
    }
    else {
        rt_thread_startup(&ic_thd);
    }

    LOG_D("%s init success", IC_DEV_NAME);

    return 0;
}
INIT_APP_EXPORT(ic_init);

#endif //defined(RT_USING_INPUT_CAPTURE) && defined(BSP_USING_TIMER1_CAPTURE) && defined(TIMER1_CAPTURE_CHANNEL1)

