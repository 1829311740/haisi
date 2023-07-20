#include <stdio.h>
#include <stdlib.h>
#include <memory.h>

#include "ohos_init.h"
#include "cmsis_os2.h"
#include "iot_gpio.h"
#include "hi_io.h"
#include "hi_time.h"

#define GPIO2 2
void set_angle( unsigned int duty) {
    IoTGpioInit(GPIO2);
    IoTGpioSetDir(GPIO2, IOT_GPIO_DIR_OUT);
    IoTGpioSetOutputVal(GPIO2, IOT_GPIO_VALUE1);
    hi_udelay(duty);
    IoTGpioSetOutputVal(GPIO2, IOT_GPIO_VALUE0);
    hi_udelay(20000 - duty);
}

/*Steering gear turn left*/
void engine_turn_left(int a)
{
    printf("engine_turn_left\n");
    if (a>50){
        a=50;
    }
        set_angle(1500-10*a);

}

/*Steering gear turn right*/
void engine_turn_right(int a)
{
    printf("engine_turn_right\n");
    if (a>50){
        a=50;
    }
    set_angle(1500+10*a);

}

/*Steering gear return to middle*/
void regress_middle(void)
{
    printf("regress_middle\n");
    for (int i = 0; i <10; i++) {
        set_angle(1500);
    }
}
