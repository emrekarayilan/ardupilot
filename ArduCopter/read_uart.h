#pragma once
#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_HAL/Semaphores.h>
#include <AP_Param/AP_Param.h>
#include <AP_MSP/msp.h>


class ReadUart {

public:

    static void setup_uart(AP_HAL::UARTDriver *uart, const char *name);
    static void setup(void);
    static void test_uart(AP_HAL::UARTDriver *uart, const char *name);
    static void run(void);







};