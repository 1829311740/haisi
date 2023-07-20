#include <stdio.h>
#include <stdlib.h>
#include <memory.h>

#include "ohos_init.h"
#include "cmsis_os2.h"
#include "iot_gpio.h"
#include "hi_io.h"
#include "hi_time.h"
#include "iot_pwm.h"
#include "hi_pwm.h"

#define GPIO0 0
#define GPIO1 1
#define GPIO9 9
#define GPIO10 10
#define GPIOFUNC 0
#define PWM_FREQ_FREQUENCY  (60000)

void gpio_control (unsigned int  gpio, IotGpioValue value) {
    hi_io_set_func(gpio, GPIOFUNC);
    IoTGpioSetDir(gpio, IOT_GPIO_DIR_OUT);
    IoTGpioSetOutputVal(gpio, value);
}

void car_backward(void) {
    printf("back\n");
    gpio_control(GPIO0, IOT_GPIO_VALUE0);
    gpio_control(GPIO1, IOT_GPIO_VALUE1);
    gpio_control(GPIO9, IOT_GPIO_VALUE0);
    gpio_control(GPIO10, IOT_GPIO_VALUE1);
}


void car_left(void) {
    printf("left\n");
    gpio_control(GPIO0, IOT_GPIO_VALUE1);
    gpio_control(GPIO1, IOT_GPIO_VALUE0);
    gpio_control(GPIO9, IOT_GPIO_VALUE1);
    gpio_control(GPIO10, IOT_GPIO_VALUE1);
}

void car_right(void) {
    printf("right\n");
    gpio_control(GPIO0, IOT_GPIO_VALUE1);
    gpio_control(GPIO1, IOT_GPIO_VALUE1);
    gpio_control(GPIO9, IOT_GPIO_VALUE1);
    gpio_control(GPIO10, IOT_GPIO_VALUE0);
}

void car_stop(void) {
    printf("stop\n");
    gpio_control(GPIO0, IOT_GPIO_VALUE0);
    gpio_control(GPIO1, IOT_GPIO_VALUE0);
    gpio_control(GPIO9, IOT_GPIO_VALUE0);
    gpio_control(GPIO10, IOT_GPIO_VALUE0);
}


void car_forward(int a) {
    printf("forward\n");
    for(int i=0;i<a;i++){
        car_left();
        hi_sleep(10);
        car_right();
        hi_sleep(10);
    }
}