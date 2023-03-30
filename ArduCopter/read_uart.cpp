/*
  simple test of UART interfaces
 */

#include <AP_HAL/AP_HAL.h>
#include "read_uart.h"
#if HAL_OS_POSIX_IO
#include <stdio.h>
#endif


//const AP_HAL::HAL& hal = AP_HAL::get_HAL();
extern const AP_HAL::HAL &hal;

/*
  setup one UART at 57600
 */
void ReadUart::setup_uart(AP_HAL::UARTDriver *uart, const char *name)
{
    if (uart == nullptr) {
        // that UART doesn't exist on this platform
        return;
    }
    uart->begin(9600);
}


void ReadUart::setup(void)
{
    /*
      start all UARTs at 57600 with default buffer sizes
    */

    hal.scheduler->delay(1000); //Ensure that the uartA can be initialized

    setup_uart(hal.serial(0), "SERIAL0");  // console
    setup_uart(hal.serial(1), "SERIAL1");  // telemetry 1
    setup_uart(hal.serial(2), "SERIAL2");  // telemetry 2
    setup_uart(hal.serial(3), "SERIAL3");  // 1st GPS
    setup_uart(hal.serial(4), "SERIAL4");  // 2nd GPS

    
}

void ReadUart::test_uart(AP_HAL::UARTDriver *uart, const char *name)
{   const char* str = nullptr;
    if (uart == nullptr) {
        // that UART doesn't exist on this platform
        return;
    }
//    uart->printf("Hello on UART %s at %.3f seconds\n",
//                 name, (double)(AP_HAL::millis() * 0.001f));

    while (hal.serial(4)->available() > 0) {
        char c = hal.serial(4)->read();
        hal.console->write(c);
        
        str = str + c ;
    }
    //uart->printf("String from serial 4 --->  %s at %.3f seconds\n",
                // str, (double)(AP_HAL::millis() * 0.001f));
    
}

void ReadUart::run(void)
{   
    //hal.serial(0)->printf("rundayim");
    test_uart(hal.serial(0), "SERIAL0");
    test_uart(hal.serial(1), "SERIAL1");
    test_uart(hal.serial(2), "SERIAL2");
    test_uart(hal.serial(3), "SERIAL3");
    test_uart(hal.serial(4), "SERIAL4");

    // also do a raw printf() on some platforms, which prints to the
    // debug console
#if HAL_OS_POSIX_IO
    ::printf("Hello on debug console at %.3f seconds\n", (double)(AP_HAL::millis() * 0.001f));
#endif

   // hal.scheduler->delay(1000);
}




