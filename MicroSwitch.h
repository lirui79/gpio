/*
 * Copyright (c) 2021.zhongkeruixin Technologies.
 * MicroSwitch.h
 *
 *  Created on: 2022年4月15日
 *      Author: wulifu
 */

#ifndef SMART_BOX_V2_SUBMODULE_MICROSWITCH_MICROSWITCH_H_
#define SMART_BOX_V2_SUBMODULE_MICROSWITCH_MICROSWITCH_H_

#include <memory>
#include <thread>
#include <mutex>

// 屏幕按钮回调函数类型
typedef void (*buttonFunc)();

class MicroSwitch
{
protected:
    int gpio_n(const char *devname, const char *pin, int sz);
    int opengpio(int gpin, const char *direction, const char *edge);
    int threadLoop();
    int gpio_init();

public:
    MicroSwitch();
    virtual ~MicroSwitch();

    /*
     * @description  : 初始化微动开关模块(启动屏幕开关按键监视线程)
                       包含箱子开关的检测，和屏幕开关的检测
     * @param        : 无
     * @return:      0->正常结束    other->处理异常
     */
    int init();

    /*
     * @description  : 读取箱子的状态(开/关)
     * @param(in)    : gpin ->   GPIO 地址
     * @param(out)   : status -> 开关状态  true:开  false:关
     * @return:      0->正常结束    other->处理异常
     */
    int readBoxStatus(int gpin, bool &status);

    /*
     * @description  : 从上一次调用该接口到本次调用该接口之间，获取用户触发关箱子的次数，读取完后内部计数清空
     * @param(out)   : closeTimes -> 距上次读取，检测到的关箱子次数
     * @return:      0->正常结束    other->处理异常
     */
    int getCloseBoxTimes(int &closeTimes);

    /*
     * @description  : 注册屏幕按键按下后的回调函数
     * @param(out)   : buttonNum ->  按下屏幕开关的次数
     * @return:      0->正常结束    other->处理异常
     */
    int registerScreenButtonFunc(buttonFunc func);

    /*
     * @description  : 退出函数(退出屏幕开关按键监视线程)
     * @param        : 无
     * @return:      0->正常结束    other->处理异常
     */
    int exit();

    /*
     * @description  : 检测屏幕按键是否存在
     * @param        : 无
     * @return:      0->正常结束    other->参照 errorCode.h 内的 ERR_CODE_E
     */
    int checkScreenButton();

private:
    // 屏幕按钮回调函数
    buttonFunc m_buttonFunc;
    int m_gpin[2]; // 0 - 屏幕按键 71  1 - 微动开关 76
    int m_status;
    int m_closeTimes;
    std::mutex m_lock;
    std::shared_ptr<std::thread> m_thread;
};

#endif /* SMART_BOX_V2_SUBMODULE_MICROSWITCH_MICROSWITCH_H_ */
