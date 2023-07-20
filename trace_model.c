#include "ohos_init.h"
#include "cmsis_os2.h"
#include "iot_gpio.h"
#include "hi_io.h"
#include "hi_time.h"
#include "iot_watchdog.h"
#include "robot_control.h"
#include "iot_errno.h"
#include "hi_pwm.h"
#include "hi_timer.h"
#include "iot_pwm.h"
#include "hi_adc.h"
#include "iot_errno.h"
#include "iot_gpio_ex.h"
#include "hi_task.h"
#include "hi_types_base.h"
#include <stdio.h>
#include <stdlib.h>
#include <memory.h>

#define GPIO0 0
#define GPIO1 1
#define GPIO9 9
#define GPIO10 10
#define GPIO11 11
#define GPIO12 12
#define GPIO_FUNC 0
#define car_speed_left 0
#define car_speed_right 0
#define GPIO5 5
#define FUNC_GPIO 0

#define DELAY_US20    20
#define DELAY_MS10    10
#define     ADC_TEST_LENGTH             (20)
#define     VLT_MIN                     (100)
#define     OLED_FALG_ON                ((unsigned char)0x01)
#define     OLED_FALG_OFF               ((unsigned char)0x00)

// unsigned char   g_car_status = CAR_STOP_STATUS;
// extern float GetDistance(void);
extern void trace_module(void);
extern void car_backward(void);
extern void car_forward(int a);
extern void car_left(void);
extern void car_right(void);
extern void car_stop(void);
extern void engine_turn_left(int a);
extern void engine_turn_right(int a);
extern void regress_middle(void);
extern unsigned char g_car_status;
unsigned int g_car_speed_left = car_speed_left;
unsigned int g_car_speed_right = car_speed_right;
IotGpioValue io_status_left;
IotGpioValue io_status_right;
void timer1_callback(unsigned int arg)
{
    //printf("timer1_callback\n");
    IotGpioValue io_status;
    if(g_car_speed_left != car_speed_left)   
    {
        IoTGpioGetInputVal(GPIO11,&io_status);
        if(io_status != IOT_GPIO_VALUE0){
            g_car_speed_left = car_speed_left;
            printf("left speed change \r\n");
        }
    }

    if(g_car_speed_right != car_speed_right)   
    {
        IoTGpioGetInputVal(GPIO12,&io_status);
        if(io_status != IOT_GPIO_VALUE0){
            g_car_speed_right = car_speed_right;
            printf("right speed change \r\n");
        }
    }
    if(g_car_speed_left != car_speed_left && g_car_speed_right != car_speed_right)
    {
        g_car_speed_left = car_speed_left;
        g_car_speed_right = car_speed_right;
    }
    IoTGpioGetInputVal(GPIO11,&io_status_left);
    // printf("IoTGpioGetInputVal\r\n");
    IoTGpioGetInputVal(GPIO12,&io_status_right);
    if(io_status_right == IOT_GPIO_VALUE0 && io_status_left != IOT_GPIO_VALUE0)//run right
    {
        g_car_speed_left = car_speed_left;
        g_car_speed_right = 1;
        // printf("g_car_speed_right\r\n");
    } 
    if(io_status_right != IOT_GPIO_VALUE0 && io_status_left == IOT_GPIO_VALUE0)//run left
    {
        g_car_speed_left = 1;
        g_car_speed_right = car_speed_right;
        // printf("g_car_speed_left\r\n");
    }
}

void Hcsr04Init(void)
{
    /*
     * 设置超声波Echo为输入模式
     * 设置GPIO8功能（设置为GPIO功能）
     * Set ultrasonic echo as input mode
     * Set GPIO8 function (set as GPIO function)
     */
    IoSetFunc(IOT_IO_NAME_GPIO_8, IOT_IO_FUNC_GPIO_8_GPIO);
    /*
     * 设置GPIO8为输入方向
     * Set GPIO8 as the input direction
     */
    IoTGpioSetDir(IOT_IO_NAME_GPIO_8, IOT_GPIO_DIR_IN);
    /*
     * 设置GPIO7功能（设置为GPIO功能）
     * Set GPIO7 function (set as GPIO function)
     */
    IoSetFunc(IOT_IO_NAME_GPIO_7, IOT_IO_FUNC_GPIO_7_GPIO);
    /*
     * 设置GPIO7为输出方向
     * Set GPIO7 as the output direction
     */
    IoTGpioSetDir(IOT_IO_NAME_GPIO_7, IOT_GPIO_DIR_OUT);
}


float GetDistance(void)
{
    static unsigned long start_time = 0, time = 0;
    float distance = 0.0;
    IotGpioValue value = IOT_GPIO_VALUE0;
    unsigned int flag = 0;
    /*
     * 设置GPIO7输出低电平
     * 给trig发送至少10us的高电平脉冲，以触发传感器测距
     * Set GPIO7 to output direction
     * Send a high level pulse of at least 10us to the trig to trigger the range measurement of the sensor
     */
    printf("distance begin\n");
    IoTGpioSetOutputVal(IOT_IO_NAME_GPIO_7, IOT_GPIO_VALUE1);
    hi_udelay(DELAY_US20);
    IoTGpioSetOutputVal(IOT_IO_NAME_GPIO_7, IOT_GPIO_VALUE0);
    /*
     * 计算与障碍物之间的距离
     * Calculate the distance from the obstacle
     */
    while (1) {
        /*
         * 获取GPIO8的输入电平状态
         * Get the input level status of GPIO8
         */
        IoTGpioGetInputVal(IOT_IO_NAME_GPIO_8, &value);
        /*
         * 判断GPIO8的输入电平是否为高电平并且flag为0
         * Judge whether the input level of GPIO8 is high and the flag is 0
         */
        if (value == IOT_GPIO_VALUE1 && flag == 0) {
            /*
             * 获取系统时间
             * get SysTime
             */
            start_time = hi_get_us();
            flag = 1;
        }
        /*
         * 判断GPIO8的输入电平是否为低电平并且flag为1
         * Judge whether the input level of GPIO8 is low and the flag is 1
         */
        if (value == IOT_GPIO_VALUE0 && flag == 1) {
            /*
             * 获取高电平持续时间
             * Get high level duration
             */
            time = hi_get_us() - start_time;
            break;
        }
    }
    /* 计算距离障碍物距离（340米/秒 转换为 0.034厘米/微秒, 2代表去来，两倍距离） */
    /* Calculate the distance from the obstacle */
    /* (340 m/s is converted to 0.034 cm/microsecond 2 represents going and coming, twice the distance) */
    distance = time * 0.034 / 2;
    printf("distance is %0.2f cm\r\n", distance);
    return distance;
}




/*Judge steering gear*/
static unsigned int engine_go_where(void)
{
    //printf("engine_go_where\n");
    float left_distance = 100;
    float right_distance = 100;
    float temp=99;
    /*舵机往左转动测量左边障碍物的距离*/

    for(int i=0;i<50;i++){
            engine_turn_left(i);
            temp = GetDistance();
            hi_sleep(50);
            if(temp <left_distance){
                left_distance=temp;

            }
            printf("left_distance=%f\r\n",left_distance);
    }

    /*归中*/
    regress_middle();
    hi_sleep(50);
    temp=0;

    /*舵机往右转动测量右边障碍物的距离*/

    for(int i=0;i<50;i++){
            engine_turn_right(i);
            temp = GetDistance();
            hi_sleep(50);
            if(temp <right_distance){
                right_distance=temp;

            }
            printf("right_distance=%f\r\n",right_distance);
    }
    
    /*归中*/
    regress_middle();
    if (left_distance > right_distance) {
        return CAR_TURN_LEFT;
    } else {
        return CAR_TURN_RIGHT;
    }
}
/*根据障碍物的距离来判断小车的行走方向
1、距离大于等于20cm继续前进
2、距离小于20cm，先停止再后退0.5s,再继续进行测距,再进行判断
*/
/*Judge the direction of the car*/
static void car_where_to_go(float distance)
{
        
        printf("block go back\n");
        // car_backward();
        hi_sleep(1000);
        car_stop();
        unsigned int ret = engine_go_where();
        // unsigned int retb;
        float left_distance = 0;
        float right_distance = 0;
        printf("ret is %d\r\n", ret);
        if (ret == CAR_TURN_LEFT) {
            car_left();
            hi_sleep(1500);
            // retb=CAR_TURN_RIGHT;
        }
        if (ret == CAR_TURN_RIGHT) {
            car_right();
            hi_sleep(1500);
            // retb=CAR_TURN_LEFT;
        }
        // car_stop();
        regress_middle();
        if(ret == CAR_TURN_LEFT) {
            car_forward(10);
        }else{
            car_forward(1);
        }
        // hi_sleep(1000);
        car_stop();
//# if 0
        while(io_status_right != IOT_GPIO_VALUE1 && io_status_left != IOT_GPIO_VALUE1){
            IoTGpioGetInputVal(GPIO11,&io_status_left);
            printf("find block go around\r\n");
            IoTGpioGetInputVal(GPIO12,&io_status_right);
            if(ret == CAR_TURN_LEFT){
                car_right();
                hi_sleep(12);
                car_forward(10);
                
                // engine_turn_left();
                // hi_sleep(100);
                // right_distance = GetDistance();
                // hi_sleep(100);
                // if(right_distance<DISTANCE_BETWEEN_CAR_AND_OBSTACLE){
                //     car_forward();
                //     hi_sleep(500);
                //     car_stop();
                // } else {
                //     car_right();
                //     hi_sleep(500);
                //     car_stop();
                // }
            }
            if(ret == CAR_TURN_RIGHT){
                car_left();
                hi_sleep(10);
                car_forward(5);
                
                // engine_turn_right();
                // hi_sleep(100);
                // left_distance = GetDistance();
                // hi_sleep(100);

                // if(right_distance<DISTANCE_BETWEEN_CAR_AND_OBSTACLE){
                //     car_forward();
                //     hi_sleep(500);
                //     car_stop();
                // }else {
                //     car_left();
                //     hi_sleep(500);
                //     car_stop();
                // }
            }
            regress_middle();
        }
        //#endif
    // } else {
    //     car_forward();
    //     } 
}


void trace_module(void)
{  
    printf("trace_module\n");
    unsigned int timer_id1;
    float m_distance = 0.0;
    hi_timer_create(&timer_id1);
    printf("hi_timer_create\n");
    hi_timer_start(timer_id1, HI_TIMER_TYPE_PERIOD, 1, timer1_callback, 0);
    printf("hi_timer_start\n");
    //regress_middle();
     /*获取前方物体的距离*/
    m_distance = GetDistance();
    //printf("m_distance = %f\n",m_distance);
    if (m_distance > DISTANCE_BETWEEN_CAR_AND_OBSTACLE) {
        //trace
        printf("traceing\n");
        gpio_control(GPIO0, IOT_GPIO_VALUE1);
        gpio_control(GPIO1, g_car_speed_left);
        gpio_control(GPIO9, IOT_GPIO_VALUE1);
        gpio_control(GPIO10, g_car_speed_right);
        
        // gpio_control(GPIO1, g_car_speed_left);
        // gpio_control(GPIO10, g_car_speed_right);
        hi_sleep(50);
        car_stop();
    } else {
        printf("detect block\n");
        car_where_to_go(m_distance);
        regress_middle();
        // }

            // if (g_car_status != CAR_TRACE_STATUS) {
            //     break;
            // }
        // }
        hi_timer_delete(timer_id1);
    }

       
}


void RobotCarTestTask(void* param)
{
	// printf("switch\r\n");
    // switch_init();
    // interrupt_monitor();
    // while (1) {
    //     printf("CAR_TRACE_STATUS\n");
    //     gpio_control(GPIO0, IOT_GPIO_VALUE1);
    //     gpio_control(GPIO1, IOT_GPIO_VALUE1);
    //     gpio_control(GPIO9, IOT_GPIO_VALUE1);
    //     gpio_control(GPIO10, IOT_GPIO_VALUE1);
    //     printf("CAR_TRACE_STATUS1\n");
    // }
    
    
    // while(1);
    while (1) {
        
        switch (g_car_status) {
            case CAR_STOP_STATUS:
                car_stop();
                break;
            case CAR_OBSTACLE_AVOIDANCE_STATUS:
                //car_mode_control_func();
                break;
            case CAR_TRACE_STATUS:
                printf("CAR_TRACE_STATUS\n");
                trace_module();
                break;
            default:
                break;
        }
        IoTWatchDogDisable();        
    }
}

void RobotCarDemo(void)
{
    osThreadAttr_t attr;
    Hcsr04Init();
    IoTWatchDogDisable();
    regress_middle();
    attr.name = "RobotCarTestTask";
    attr.attr_bits = 0U;
    attr.cb_mem = NULL;
    attr.cb_size = 0U;
    attr.stack_mem = NULL;
    attr.stack_size = 10240;
    attr.priority = 25;

    if (osThreadNew(RobotCarTestTask, NULL, &attr) == NULL) {
        printf("[Ssd1306TestDemo] Falied to create RobotCarTestTask!\n");
    }
}

APP_FEATURE_INIT(RobotCarDemo);