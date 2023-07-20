#include <stdio.h>
#include <stdlib.h>
#include <memory.h>

#include "ohos_init.h"
#include "cmsis_os2.h"
#include "iot_gpio.h"
#include "hi_io.h"
#include "hi_time.h"


// #define GPIO_8 8
// #define GPIO_7 7
// #define GPIO_FUNC 0

// float GetDistance  (void) {
//     static unsigned long start_time = 0, time = 0;
//     float distance = 0.0;
//     //printf("GetDistance\n");
//     IotGpioValue value = IOT_GPIO_VALUE0;
//     unsigned int flag = 0;
//     IoTWatchDogDisable();
//     //printf("IoTWatchDogDisable\n");
//     hi_io_set_func(GPIO_8, GPIO_FUNC);
//     IoTGpioSetDir(GPIO_8, IOT_GPIO_DIR_IN);

//     IoTGpioSetDir(GPIO_7, IOT_GPIO_DIR_OUT);
//     IoTGpioSetOutputVal(GPIO_7, IOT_GPIO_VALUE1);
//     hi_udelay(20);
//     IoTGpioSetOutputVal(GPIO_7, IOT_GPIO_VALUE0);

//     while (1) {
//         IoTGpioGetInputVal(GPIO_8, &value);
//         printf("%dvalue\n",value);
//         printf("%dIOT_GPIO_VALUE1\n",IOT_GPIO_VALUE1);
//         if ( value == IOT_GPIO_VALUE1 && flag == 0) {
//             start_time = hi_get_us();
//             flag = 1;
//         }
//         if (value == IOT_GPIO_VALUE0 && flag == 1) {
//             time = hi_get_us() - start_time;
//             start_time = 0;
//             break;
//         }
//     }
//     distance = time * 0.034 / 2;
//     printf("distance is %f\r\n",distance);
//     return distance;
// }
