#ifndef _I2C_H_
#define _I2C_H_
#include "sys.h"
#include "delay.h"

enum
{
	I2C_ACK,
	I2C_NACK
};

/* 批次6：I2C_Direction_* 原由 conf.h→stm32f4xx_i2c.h:138-139 提供（conf.h 已删）。
 * 软 I2C 首字节的 R/W 位语义：7 位地址|0x00=写(Transmitter)、|0x01=读(Receiver)，
 * 数值与原宏一致。 */
#define I2C_Direction_Transmitter  ((uint8_t)0x00)
#define I2C_Direction_Receiver     ((uint8_t)0x01)

/*--------simulate iic config--------*/
//模拟IIC 引脚配置
/* 批次6：RCC_*PeriphClockCmd → __HAL_RCC_*_CLK_ENABLE（写同 ENR 位，行为等价；StdPeriph 退场） */
#define ENABLE_IIC_SCL_PIN_CLOCK __HAL_RCC_GPIOB_CLK_ENABLE()
#define ENABLE_IIC_SDA_PIN_CLOCK __HAL_RCC_GPIOB_CLK_ENABLE()

#define IIC_SCL_PORT      GPIOB
#define IIC_SCL_PIN       GPIO_PIN_9
#define IIC_SCL           PBout(9) //SCL

#define IIC_SDA_PORT      GPIOB
#define IIC_SDA_PIN       GPIO_PIN_8
#define IIC_SDA_PIN_NUM   8          //SDA的引脚号
#define IIC_SDA           PBout(8)   //输出SDA	 
#define READ_SDA          PBin(8)    //输入SDA 
/*----------------------------------*/

//根据用户配置自动适配，无需修改
#define SDA_IN()  {IIC_SDA_PORT->MODER&=~(3<<(IIC_SDA_PIN_NUM*2));IIC_SDA_PORT->MODER|=0<<IIC_SDA_PIN_NUM*2;}	
#define SDA_OUT() {IIC_SDA_PORT->MODER&=~(3<<(IIC_SDA_PIN_NUM*2));IIC_SDA_PORT->MODER|=1<<IIC_SDA_PIN_NUM*2;} 

void I2C_SDAMode(uint8_t Mode);
void I2C_Start(void);
void I2C_Stop(void);
uint8_t I2C_WaiteForAck(void);
void I2C_Ack(void);
void I2C_NAck(void);
uint8_t I2C_WriteOneBit(uint8_t DevAddr, uint8_t RegAddr, uint8_t BitNum, uint8_t Data);
uint8_t I2C_WriteBits(uint8_t DevAddr, uint8_t RegAddr, uint8_t BitStart, uint8_t Length, uint8_t Data);
void I2C_WriteByte(uint8_t Data);
uint8_t I2C_ReadByte(uint8_t Ack);
u8 I2C_WriteOneByte(uint8_t DevAddr, uint8_t RegAddr, uint8_t Data);
uint8_t I2C_ReadOneByte(uint8_t DevAddr, uint8_t RegAddr);
uint8_t I2C_WriteBuff(uint8_t DevAddr, uint8_t RegAddr, uint8_t Num, uint8_t *pBuff);
uint8_t I2C_ReadBuff(uint8_t DevAddr, uint8_t RegAddr, uint8_t Num, uint8_t *pBuff);
void I2C_GPIOInit(void);

#endif

