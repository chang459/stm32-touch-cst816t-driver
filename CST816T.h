/**
*@file CST816T.h
*@brief CST816T接触芯片头文件
*@time 2026_1_15
*@author changyongzhi
*@version 1.0
*硬件iic，轮询模式，实现读取屏幕x@y驱动
*可移植keil5
*/

#ifndef __CST816T_H__
#define __CST816T_H__

#include "stm32f4xx_hal.h"
#include "main.h"

/* CST816T I2C地址 */
#define CST816D_I2C_ADDRESS 0x15 // 7位地址：0x15（左移1位为0x2A）

/* CST816D寄存器地址 */
#define CST816D_REG_GESTURE_ID 0x01 // 手势ID地址
#define CST816D_REG_FINGER_NUM 0x02 // 手指数量
#define CST816D_REG_XPOS_H 0x03 // X坐标高8位
#define CST816D_REG_XPOS_L 0x04 // X坐标低8位
#define CST816D_REG_YPOS_H 0x05 // Y坐标高8位
#define CST816D_REG_YPOS_L 0x06 // Y坐标低8位
#define CST816D_REG_CHIP_ID 0xA7 // 芯片ID
#define CST816D_REG_PROJ_ID 0xA8 // 项目ID
#define CST816D_REG_FW_VERSION 0xA9 // 固件版本
#define CST816D_REG_MOTION_FLAG 0x01 // 动作标志
#define CST816D_REG_IRQ_CTL 0xFA // 中断控制
#define CST816D_REG_AUTO_RST 0xFB // 自动复位时间
#define CST816D_REG_BTL_DIS 0xFC // 自举加载器禁用
#define CST816D_REG_SLEEP 0xFD // 睡眠模式
#define CST816D_REG_LONG_PRESS 0xFE // 长按时间
#define CST816D_REG_IO_CTL 0xFF // I/O控制

/* 手势定义 */
typedef enum {
    GESTURE_NONE = 0x00, // 无手势
    GESTURE_SLIDE_UP = 0x01, // 向上滑动
    GESTURE_SLIDE_DOWN = 0x02, // 向下滑动
    GESTURE_SLIDE_LEFT = 0x03, // 向左滑动
    GESTURE_SLIDE_RIGHT = 0x04, // 向右滑动
    GESTURE_CLICK = 0x05, // 单击
    GESTURE_DOUBLE_CLICK = 0x06, // 双击
    GESTURE_LONG_PRESS = 0x07, // 长按
} cst816d_gesture_t;

/* 触摸点结构体 */
typedef struct {
    uint16_t x; // X坐标
    uint16_t y; // Y坐标
    uint8_t event; // 事件事件(按下/抬起/接触)
    uint8_t weight; // 触摸权重
    uint8_t area; // 触摸面积
} cst816d_point_t;

/* CST816D状态结构体 */
typedef struct {
    uint8_t gesture_id; // 手势ID
    uint8_t finger_num; // 手指数量
    cst816d_point_t point; // 触摸点信息
    uint8_t chip_id; // 芯片ID
    uint8_t proj_id; // 项目ID
    uint8_t fw_version; // 固件版本
} cst816d_info_t;

/* CST816D句柄结构体 */
typedef struct {
    I2C_HandleTypeDef *hi2c; // I2C句柄
    GPIO_TypeDef *reset_gpio; // 复位引脚
    uint16_t reset_pin; // 复位引脚号
    GPIO_TypeDef *int_gpio; // 中断引脚
    uint16_t int_pin; // 中断引脚号
    uint8_t initialized; // 初始化标志
    cst816d_info_t info; // 芯片信息
} cst816d_handle_t;


uint8_t cst816t_init(cst816d_handle_t*handle,I2C_HandleTypeDef* hi2c,GPIO_TypeDef* reset_gpio,uint16_t reset_pin,GPIO_TypeDef* int_gpio,uint16_t int_pin);//初始化
uint8_t cst816t_touch_value(cst816d_handle_t*handle,cst816d_info_t*info);//接触数据获取

uint8_t cst816t_reset(cst816d_handle_t*handle);//硬件复位（初始化内部）
uint8_t cst816t_wakeup(cst816d_handle_t*handle);//唤醒芯片
uint8_t cst816t_sleep(cst816d_handle_t*handle);//睡眠模式
uint8_t cst816t_int_station(cst816d_handle_t*handle,uint8_t reg,uint8_t mode);//设置中断状态
uint8_t cst816t_get_version(cst816d_handle_t*handle,uint8_t* version);//得到版本号
uint8_t cst816t_get_chip_id(cst816d_handle_t*handle,uint8_t* chipid);//得到芯片地址
uint8_t reg_iic_read(cst816d_handle_t*handle,uint8_t* date,uint8_t reg);//读一个地址
uint8_t reg_iic_write(cst816d_handle_t*handle,uint8_t date,uint8_t reg);//写一个地址

#endif

/**
使用方法：
注意：文件没有开启中断模式，使用的是轮询模式。如果要需要，使用cst816t_int_station（...），根据手册调整
1.c文件中的内置函数，找到标语（替换成对应的iic读写函数，使用hal库的不用关）的函数，按照标语修改
2.h文件中的宏定义，根据自己芯片的数据手册，修改对应的宏定义，如地址等细小变化（有的使用cst816d的）
3.在cst816t_init函数中，填入对应的硬件接口（下面会有事例）
4.屏幕范围，自行根据串口测试调整（下面会有事例）

实例工程（轮询读取x与y的坐标）：

#include "CST816T.h"
#include "stdio.h"

main:
//全局变量初始化
  cst816d_handle_t handle;
  cst816d_info_t info;
//触摸屏初始化程序
  uint8_t ret = cst816t_init(&handle,&hi2c1,GPIOB,GPIO_PIN_8,GPIOB,GPIO_PIN_9);
  char message[30] = "";

  if(ret == 0){
	  sprintf(message, "success init!\r\n");
  }else{
	  sprintf(message, "sorry due to %d\r\n",ret);		
  }
  HAL_UART_Transmit(&huart1,(uint8_t*)message,sizeof(message),HAL_MAX_DELAY );
while:
  	  if(cst816t_touch_value(&handle,&info) == 0){
		// 有触摸坐标事件
         if(info.point.event != 0) {
            // 转换为屏幕坐标（假设240x280屏幕）（根据自己的屏幕调节）
            uint16_t screen_x = info.point.x * 240 / 222;
            uint16_t screen_y = info.point.y ;
			 
            char message[20] = "";
			sprintf(message, "x:%d y:%d\r\n",(int)screen_x,(int)screen_y);
			HAL_UART_Transmit(&huart1,(uint8_t*)message,sizeof(message),HAL_MAX_DELAY );

		 } 
	  }
	  HAL_Delay(50);

*/














