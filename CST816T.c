
#include "stm32f4xx_hal.h"
#include "CST816T.h"
#include <stdio.h>
#include "string.h"

/*内置函数，方便封装替换*/
static uint8_t cst816t_iic_write(cst816d_handle_t*handle,uint8_t reg,uint8_t* date,uint16_t len);
static uint8_t cst816t_iic_read(cst816d_handle_t*handle,uint8_t reg,uint8_t* date,uint16_t len);
static void cst816t_delay(uint32_t ms);


/**
*@brief 初始化代码
*@param handle:芯片句柄
*@param hi2c:你的硬件i2c句柄
*@param reset_gpio：复位GPIO端口（如GPIOB）
*@param reset_pin：具体复位的PIN口
*@param int_gpio：中断GPIO端口（如GPIOB）
*@param int_pin：具体复位的PIN口
*@return 0正确码  1，2，3参数错误
*/
uint8_t cst816t_init(cst816d_handle_t*handle,I2C_HandleTypeDef* hi2c,
GPIO_TypeDef* reset_gpio,uint16_t reset_pin,GPIO_TypeDef* int_gpio,uint16_t int_pin){
	
	uint8_t chipid;
	
	if(handle == NULL || hi2c == NULL){
		return 1;
	}
	
	handle -> hi2c = hi2c;
	handle -> int_gpio = int_gpio;
	handle -> int_pin = int_pin;
	handle -> reset_gpio = reset_gpio;
    handle -> reset_pin = reset_pin;
	handle -> initialized = 0;
	
	memset(&handle -> info,0,sizeof (cst816d_info_t));
	
	cst816t_reset(handle);
	
	if(cst816t_get_chip_id(handle,&chipid) != 0){
		return 2;
	}
	
	if(chipid != 0xB5 && chipid != 0xB6){
		return 3;
	}
	
	cst816t_get_version(handle,&handle->info.fw_version);
	
	reg_iic_write(handle,0x00,CST816D_REG_IRQ_CTL);
	reg_iic_write(handle,0x0C,CST816D_REG_AUTO_RST);
	reg_iic_write(handle,0X0A,CST816D_REG_LONG_PRESS);
	
	reg_iic_write(handle,0x00,CST816D_REG_BTL_DIS);
	reg_iic_write(handle,0x00,CST816D_REG_SLEEP);

	handle -> initialized = 1;

	return 0;
}

/**
*@brief 接触数据获取代码
*@param handle:芯片句柄
*@param info:接触点句柄
*@return 0正确码  1参数错误  2发送超时
*/
uint8_t cst816t_touch_value(cst816d_handle_t*handle,cst816d_info_t*info){
	uint8_t result;
	uint8_t buffer[8];
	
	
	if(handle == NULL || info == NULL){
		return 1;
	}

	result = cst816t_iic_read(handle,CST816D_REG_GESTURE_ID,buffer,8);
	if(result != 0){
		return 2;
	}
	
	info -> gesture_id = buffer[0];
	info -> finger_num = buffer[1];
	
	info -> point.x = (uint16_t)(buffer [2] & 0x3F) << 8 | buffer [3];
	info -> point.y = (uint16_t)(buffer [4] & 0xFF) << 8 | buffer [5];
	info -> point.event = (buffer[2] >> 6) & 0x03;
	info -> point.weight = buffer[6];
	info -> point.area = buffer[7];
	
	handle ->info.gesture_id = info -> gesture_id;
	handle ->info.finger_num = info -> finger_num;
	handle ->info.point = info -> point;

	return 0;
}


/**
*@brief 软件触发硬件复位
*@param handle:芯片句柄
*@return 0正确码 
*/
uint8_t cst816t_reset(cst816d_handle_t*handle){
	
	if(handle == NULL){
		return 1;
	}
	
	if(handle -> reset_gpio != NULL){
		HAL_GPIO_WritePin(handle -> reset_gpio,handle -> reset_pin,GPIO_PIN_RESET);
		cst816t_delay(10);
		HAL_GPIO_WritePin(handle -> reset_gpio,handle -> reset_pin,GPIO_PIN_SET);
	    cst816t_delay(50);
	}

    return 0;
}

/**
*@brief 得到芯片地址
*@param handle:芯片句柄
*@param chipid:接收芯片地址
*@return 0正确码 
*/
uint8_t cst816t_get_chip_id(cst816d_handle_t*handle,uint8_t* chipid){
	
	if(handle == NULL || chipid == NULL ){

		return 1;
	}
	
	return reg_iic_read(handle,chipid,CST816D_REG_CHIP_ID);;
}


/**
*@brief 得到版本号
*@param handle:芯片句柄
*@param version:版本号
*@return 0正确码 
*/
uint8_t cst816t_get_version(cst816d_handle_t*handle,uint8_t* version){
	
	if(handle == NULL || version == NULL || !handle ->initialized){
		return 1;
	}
	
	return reg_iic_read(handle,version,CST816D_REG_FW_VERSION);
}

/**
*@brief 设置中断的状态
*@param handle:芯片句柄
*@param reg:地址
*@param mode:中断模式
*@return 0正确码 
*/
uint8_t cst816t_int_station(cst816d_handle_t*handle,uint8_t reg,uint8_t mode){

	if(handle == NULL || !handle ->initialized){
		return 1;
	}
	
	reg_iic_write(handle,mode,CST816D_REG_IRQ_CTL);

	return 0;
}

/**
*@brief 睡眠模式
*@param handle:芯片句柄
*@return 0正确码 
*/
uint8_t cst816t_sleep(cst816d_handle_t*handle){

	if(handle == NULL || !handle ->initialized){
		return 1;
	}
	
	reg_iic_write(handle,0x03,CST816D_REG_SLEEP);

	return 0;
}

/**
*@brief 唤醒芯片
*@param handle:芯片句柄
*@return 0正确码 
*/
uint8_t cst816t_wakeup(cst816d_handle_t*handle){

	if(handle == NULL || !handle ->initialized){
		return 1;
	}
	
	reg_iic_write(handle,0x00,CST816D_REG_SLEEP);
	return 0;

}


/**
*@brief 读取一个寄存器地址
*@param handle:芯片句柄
*@param reg:寄存器地址
*@param date:接收数据的
*@return 0正确码 
*/
uint8_t reg_iic_read(cst816d_handle_t*handle,uint8_t* date,uint8_t reg){

	return cst816t_iic_read(handle,reg,date ,1);

}

/**
*@brief 写入一个寄存器地址
*@param handle:芯片句柄
*@param reg:寄存器地址
*@param date:传入数据
*@return 0正确码 
*/
uint8_t reg_iic_write(cst816d_handle_t*handle,uint8_t date,uint8_t reg){

	return cst816t_iic_write(handle,reg,&date ,1);

}


/**
*@brief 内置芯片iic，写入寄存器
*@param handle:芯片句柄
*@param reg:寄存器地址
*@param date:传入数据
*@param len:数据长度
*@return 0正确码  1参数错误  2发送超时
*/
static uint8_t cst816t_iic_write(cst816d_handle_t*handle,uint8_t reg,uint8_t* date,uint16_t len){
	uint8_t buffer[16];
	
	/*参数检测*/
	if(handle == NULL || handle -> hi2c == NULL || date == NULL || len == 0){
		return 1;
	}

	/*地址+数据，重新构建*/
	buffer[0] = reg;
	for(uint16_t i=0;i < len;i++){
		buffer[i + 1] = date [i];
	}
	
	/*iic发送数据（替换成对应的iic读写函数，使用hal库的不用关）*/
	if(HAL_I2C_Master_Transmit(handle -> hi2c ,CST816D_I2C_ADDRESS << 1,buffer ,len + 1 ,100) != HAL_OK ){
		return 2;
	}
	return 0;
}

/**
*@brief 内置芯片iic，读取寄存器
*@param handle:芯片句柄
*@param reg:寄存器地址
*@param date:接收的数据
*@param len:数据长度
*@return 0正确码  1参数错误  2发送超时 3读取超时
*/
static uint8_t cst816t_iic_read(cst816d_handle_t*handle,uint8_t reg,uint8_t* date,uint16_t len){
	
	/*参数检测*/
    if(handle == NULL || handle -> hi2c == NULL || date == NULL || len == 0){

    	return 1;
    }
	
	/*写寄存器（替换成对应的iic读写函数，使用hal库的不用关）*/
	if(HAL_I2C_Master_Transmit(handle -> hi2c ,CST816D_I2C_ADDRESS << 1,&reg,1,100) != HAL_OK ){

		return 2;
	}
	
	/*读寄存器（替换成对应的iic读写函数，使用hal库的不用关）*/
	if(HAL_I2C_Master_Receive(handle -> hi2c ,CST816D_I2C_ADDRESS << 1 | 0x01,date ,len ,100) != HAL_OK ){

		return 3;
	}
	
	return 0;

}
/**
*@brief 内置芯片延迟函数
*@param ms:延迟时间
*/
static void cst816t_delay(uint32_t ms){
	HAL_Delay (ms);
}










