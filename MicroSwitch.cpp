/*
 * Copyright (c) 2021.zhongkeruixin Technologies.
 * MicroSwitch.cpp
 *
 *  Created on: 2022年4月15日
 *      Author: wulifu
 */

#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/time.h>
#include <signal.h>
#include <errno.h>
#include <string.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <netdb.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <linux/ioctl.h>
#include <sys/stat.h>
#include <linux/types.h>
#include <sys/mman.h>

#define REG_BASE 0x20E0000
#define MX6UL_PAD_LCD_DATA02__GPIO3_IO07_PAD 0x03ac
#define MX6UL_PAD_LCD_DATA07__GPIO3_IO12_PAD 0x03c0
#define MX6UL_PAD_LCD_DATA07__GPIO3_IO27_PAD 0x03fc

#include "MicroSwitch.h"
#include "comLog.h"
#include "errorCode.h"

MicroSwitch::MicroSwitch()
{
    // TODO Auto-generated constructor stub
    m_status = 0;
    m_gpin[0] = 71; // 0 - 屏幕按键 71
                    //    m_gpin[1] = 76; // 1 - 微动开关 76      (北京环境)
    m_gpin[1] = 91; // 1 - 微动开关 91      (济南环境)
    m_buttonFunc = NULL;
    m_closeTimes = 0;
}

MicroSwitch::~MicroSwitch()
{
    // TODO Auto-generated destructor stub
    m_status = 0;
}

/*
 * @description  : 初始化微动开关模块(启动屏幕开关按键监视线程)
 * @param        : 无
 * @return:      0->正常结束    other->处理异常
 */
int MicroSwitch::init()
{
    LOGF_START();
    std::lock_guard<std::mutex> locker(m_lock);
    if (m_status == 0)
    {
        LOGF_INF("MicroSwitch::initing  status:%d\n", m_status);
        if (gpio_init() >= 0)
        {
            m_status = 1;
            m_thread = std::shared_ptr<std::thread>(new std::thread(&MicroSwitch::threadLoop, this));
            m_thread->detach();
            LOGF_INF("MicroSwitch::initing success status:%d\n", m_status);
        }
        else
        {
            LOGF_INF("MicroSwitch::initing gpio init fail status:%d\n", m_status);
            LOGF_END();
            return ERR_CODE_E::SCREEN_ERR_NO_DEV;
        }
    }
    else
    {
        LOGF_INF("warn MicroSwitch::inited again  status:%d\n", m_status);
    }
    LOGF_END();
    return 0;
}

/*
 * @description  : 读取箱子的状态(开/关)
 * @param(in)    : gpin ->   GPIO 地址
 * @param(out)   : status -> 开关状态  true:开  false:关
 * @return:      0->正常结束    other->处理异常
 */
int MicroSwitch::readBoxStatus(int gpin, bool &status)
{
    LOGF_START();

    /********************* 待开发  *********************/
    LOGF_DBG("gpin:%d m_gpin[1]%d", gpin, m_gpin[1]);
    gpin = m_gpin[1]; // 外部输入的管脚号不使用,内部写死
    int gfd = opengpio(gpin, "in", "falling");
    if (gfd < 0)
    {
        LOGF_ERR("opengpio[%d] NG", gpin);
        return ERR_CODE_E::MICROSWITCH_ERR_NO_DEV;
    }

    char output[4] = {0};
    int state = 0, code = 0;
    lseek(gfd, 0, SEEK_SET);
    code = read(gfd, output, 4);
    LOGF_DBG("read gpio:%d code = %d, value = %s", gpin, code, output);
    state = atoi(output);
    close(gfd);
    status = false;
    if (state == 1)
    {
        status = true;
    }
    LOGF_END();
    return 0;
}

/*
 * @description  : 读取两次读取间，触发关箱子的次数，读取完后内部计数清空
 * @param(out)   : closeTimes -> 距上次读取，检测到的关箱子次数
 * @return:      0->正常结束    other->处理异常
 */
int MicroSwitch::getCloseBoxTimes(int &closeTimes)
{
    LOGF_START();
    {
        std::lock_guard<std::mutex> locker(m_lock);
        closeTimes = m_closeTimes;
        m_closeTimes = 0;
    }
    LOGF_END();
    return 0;
}

int MicroSwitch::gpio_n(const char *devname, const char *pin, int sz)
{
    int fd = open(devname, O_WRONLY);
    if (fd < 0)
    {
        LOGF_ERR("gpio_n:%s %s %d %d \n", devname, pin, sz, fd);
        return -1;
    }

    write(fd, pin, sz);
    close(fd);
    return 0;
}

/*
 * @description  : 注册屏幕按键按下后的回调函数
 * @param(in)   : buttonNum ->  按下屏幕开关的次数
 * @return:      0->正常结束    other->处理异常
 */
int MicroSwitch::registerScreenButtonFunc(buttonFunc func)
{
    LOGF_START();
    {
        std::lock_guard<std::mutex> locker(m_lock);
        m_buttonFunc = func;
    }
    LOGF_END();
    return 0;
}

int MicroSwitch::opengpio(int gpin, const char *direction, const char *edge)
{
    int gfd = -1;
    char gpiname[256] = {0};
    int sz = 0, code = 0;
    char buffer[16] = "in";
    sprintf(gpiname, "/sys/class/gpio/gpio%d", gpin);
    if (access(gpiname, F_OK) < 0)
    {
        sz = sprintf(gpiname, "/sys/class/gpio/export");
        sz = sprintf(buffer, "%d", gpin);
        code = gpio_n(gpiname, buffer, sz);
        if (code < 0)
        {
            LOGF_ERR("can not write %s to %s :%d\n", buffer, gpiname, code);
            return code;
        }

        sz = sprintf(gpiname, "/sys/class/gpio/gpio%d/direction", gpin);
        sz = sprintf(buffer, "%s", direction);
        code = gpio_n(gpiname, buffer, sz);
        if (code < 0)
        {
            LOGF_ERR("can not write %s to %s :%d\n", buffer, gpiname, code);
            return code;
        }
        sz = sprintf(gpiname, "/sys/class/gpio/gpio%d/edge", gpin);
        sz = sprintf(buffer, "%s", edge);
        code = gpio_n(gpiname, buffer, sz);
        if (code < 0)
        {
            LOGF_ERR("can not write %s to %s :%d\n", buffer, gpiname, code);
            return code;
        }
    }

    sprintf(gpiname, "/sys/class/gpio/gpio%d/value", gpin);
    gfd = open(gpiname, O_RDONLY);
    if (gfd < 0)
    {
        LOGF_ERR("open %s:%d wrong\n", gpiname, gfd);
        return -2;
    }
    return gfd;
}

int MicroSwitch::threadLoop()
{
    int gfd[2] = {-1, -1}, i = 0, fd = 0;
    fd_set efds;
    struct timeval timeout = {2, 0}; //"none", "rising", "falling"，"both"
    for (i = 0; i < 2; ++i)
    {
        gfd[i] = opengpio(m_gpin[i], "in", "falling");
        if (gfd[i] < 0)
        {
            LOGF_ERR("opengpio[%d] NG", m_gpin[i]);
            return -2;
        }
    }

    LOGF_INF("threadLoop start\n");
    char buffer[4] = {0};
    while (1)
    {
        FD_ZERO(&efds); //每次循环都要清空集合，否则不能检测描述符变化

        {
            std::lock_guard<std::mutex> locker(m_lock);
            if (m_status == 0)
                break;
        }
        LOGF_DBG("threadLoop runnig\n");

        int maxFd = gfd[0];
        maxFd = (gfd[1] > maxFd ? gfd[1] : maxFd);
        FD_SET(gfd[0], &efds); //添加描述符
        FD_SET(gfd[1], &efds); //添加描述符

        timeout.tv_sec = 2;
        timeout.tv_usec = 0;
        int code = select(maxFd + 1, NULL, NULL, &efds, &timeout);
        if (code <= 0)
        {
            LOGF_DBG("threadLoop select continue\n");
            continue;
        }

        fd = gfd[0];
        if (FD_ISSET(fd, &efds))
        {
            code = lseek(fd, 0, SEEK_SET);
            code = read(fd, buffer, 2);
            // buffer[1] = '\0';
            LOGF_INF("read gpio:%d code = %d, value = %s\n", m_gpin[0], code, buffer);
            std::lock_guard<std::mutex> locker(m_lock);
            if (m_buttonFunc == NULL)
            {
                LOGF_DBG("m_buttonFunc == NULL\n");
                continue;
            }
            m_buttonFunc();
        }

        fd = gfd[1];
        if (FD_ISSET(fd, &efds))
        {
            code = lseek(fd, 0, SEEK_SET);
            code = read(fd, buffer, 2);
            // buffer[1] = '\0';
            LOGF_INF("read gpio:%d code = %d, value = %s\n", m_gpin[1], code, buffer);
            std::lock_guard<std::mutex> locker(m_lock);
            m_closeTimes = m_closeTimes + 1;
        }
    }

    if (gfd[0] >= 0)
    {
        close(gfd[0]);
    }

    if (gfd[1] >= 0)
    {
        close(gfd[1]);
    }

    LOGF_INF("exit\n");
    std::lock_guard<std::mutex> locker(m_lock);
    m_status = 2;
    return 0;
}

int MicroSwitch::exit()
{
    int cnt = 0;
    {
        std::lock_guard<std::mutex> locker(m_lock);
        if (m_status == 0)
        {
            return 1;
        }
        m_status = 0;
    }

    do
    {
        {
            std::lock_guard<std::mutex> locker(m_lock);
            if (m_status == 2)
            {
                break;
            }
        }
        usleep(500000);
    } while (cnt++ < 6);
    LOGF_INF("exit %d\n", m_status);
    m_status = 0;
    return 0;
}

int MicroSwitch::gpio_init()
{
    int fd;
    unsigned char *buf;
    uint32_t reg = 0;
    fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd == -1)
    {
        LOGF_ERR("Error to open /dev/mem!");
        return -1;
    }

    buf = (unsigned char *)mmap(0, 64 * 1024 * 1024, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0x0);
    close(fd);
    reg = REG_BASE + MX6UL_PAD_LCD_DATA02__GPIO3_IO07_PAD;
    *(uint32_t *)(buf + reg) = 0x1F0b0; // 0x130b0
    // reg = REG_BASE + MX6UL_PAD_LCD_DATA07__GPIO3_IO12_PAD;
    reg = REG_BASE + MX6UL_PAD_LCD_DATA07__GPIO3_IO27_PAD;
    *(uint32_t *)(buf + reg) = 0x1F0b0;
    munmap(buf, 64 * 1024 * 1024);

    return 0;
}

/*
 * @description  : 检测屏幕按键是否存在
 * @param        : 无
 * @return:      0->正常结束    other->参照 errorCode.h 内的 ERR_CODE_E
 */
int MicroSwitch::checkScreenButton()
{
    LOGF_START();

    /********************* 待开发  *********************/
    int gpin = m_gpin[0];
    int gfd = opengpio(gpin, "in", "falling");
    if (gfd < 0)
    {
        LOGF_ERR("opengpio[%d] NG", gpin);
        return ERR_CODE_E::SCREEN_ERR_NO_DEV;
    }
    char output[4] = {0};
    int state = 0, code = 0;
    lseek(gfd, 0, SEEK_SET);
    code = read(gfd, output, 4);
    state = atoi(output);
    LOGF_DBG("read gpio:%d code=%d, value=%s state=%d\n", gpin, code, output, state);
    close(gfd);
    LOGF_END();
    return 0;
}