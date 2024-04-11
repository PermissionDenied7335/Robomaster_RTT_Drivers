#ifndef __RT_IST8310_H__
#define __RT_IST8310_H__

#include <rtthread.h>
#include <stm32f4xx_hal.h>
#include <arm_math.h>

/* STAT1 */
#define IST8310_DATA_AVAILABLE  UINT8_C(1 << 0)
#define IST8310_DATA_OVERFLOW   UINT8_C(1 << 1)

/* CNTL1 */
#define IST8310_STANDBY         UINT8_C(0x00)
#define IST8310_ONESHOT         UINT8_C(0x01)
#define IST8310_CONTINUOUS      UINT8_C(0x03)

/* CNTL2 */
#define IST8310_DRDY_INT_ENABLE UINT8_C(1 << 3)
#define IST8310_DRDY_INT_HIGH   UINT8_C(1 << 2)
#define IST8310_DRDY_INT_LOW    UINT8_C(0 << 2)
#define IST8310_SOFTRESET       UINT8_C(1 << 0)

/* STR */
#define IST8310_SELFTEST_ON     UINT8_C(1 << 6)
#define IST8310_SELFTEST_OFF    UINT8_C(0)

/* TCCNTL */
#define IST8310_TC_ENABLE       UINT8_C(0x00)
#define IST8310_TC_DISABLE      UINT8_C(0x01)

/* AVGCNTL */
#define IST8310_Y_AVG_1         UINT8_C(0x00 << 3)
#define IST8310_Y_AVG_2         UINT8_C(0x01 << 3)
#define IST8310_Y_AVG_4         UINT8_C(0x02 << 3)
#define IST8310_Y_AVG_8         UINT8_C(0x03 << 3)
#define IST8310_Y_AVG_16        UINT8_C(0x04 << 3)
#define IST8310_XZ_AVG_1        UINT8_C(0x00)
#define IST8310_XZ_AVG_2        UINT8_C(0x01)
#define IST8310_XZ_AVG_4        UINT8_C(0x02)
#define IST8310_XZ_AVG_8        UINT8_C(0x03)
#define IST8310_XZ_AVG_16       UINT8_C(0x04)
#define IST8310_AVG_1           (IST8310_Y_AVG_1  | IST8310_XZ_AVG_1)
#define IST8310_AVG_2           (IST8310_Y_AVG_2  | IST8310_XZ_AVG_2)
#define IST8310_AVG_4           (IST8310_Y_AVG_4  | IST8310_XZ_AVG_4)
#define IST8310_AVG_8           (IST8310_Y_AVG_8  | IST8310_XZ_AVG_8)
#define IST8310_AVG_16          (IST8310_Y_AVG_16 | IST8310_XZ_AVG_16)

/* PDCNTL */
#define IST8310_PERFORMANCE     UINT8_C(0xC0)

#define IST8310_ID              UINT8_C(0x10)
#define IST8310_ADDRESS_0       UINT8_C(0x18)
#define IST8310_ADDRESS_1       UINT8_C(0x1A)
#define IST8310_ADDRESS_2       UINT8_C(0x1C)
#define IST8310_ADDRESS_3       UINT8_C(0x1E)

#define IST8310_RESET_DELAY_MS  UINT8_C(50)
#define IST8310_RESOLUTION      (0.3f)

/* Sensor control cmd types */
#ifndef RT_SENSOR_INTF
#define RT_SENSOR_INTF

#define  RT_SENSOR_CTRL_GET_ID         0  /* Get device id */
#define  RT_SENSOR_CTRL_GET_INFO       1  /* Get sensor info */
#define  RT_SENSOR_CTRL_SET_RANGE      2  /* Set the measure range of sensor. */
#define  RT_SENSOR_CTRL_SET_ODR        3  /* Set output date rate. */
#define  RT_SENSOR_CTRL_SET_MODE       4  /* not used */
#define  RT_SENSOR_CTRL_SET_POWER      5  /* Set power mode. args type of sensor power mode. ex. RT_SENSOR_POWER_DOWN, RT_SENSOR_POWER_NORMAL */
#define  RT_SENSOR_CTRL_SELF_TEST      6  /* Take a self test */

#define  RT_SENSOR_POWER_NONE          (0)
#define  RT_SENSOR_POWER_DOWN          (1)  /* power down mode   */
#define  RT_SENSOR_POWER_NORMAL        (2)  /* normal-power mode */
#define  RT_SENSOR_POWER_LOW           (3)  /* low-power mode    */
#define  RT_SENSOR_POWER_HIGH          (4)  /* high-power mode   */

#endif

/**
 * @brief IST8310寄存器名称与地址的对应关系
 * 
 */
typedef enum
{
  MAG_WHO_AM_I  = 0x00,
  MAG_STAT1     = 0x02,
  MAG_DATAXL    = 0x03,
  MAG_DATAXH    = 0x04,
  MAG_DATAYL    = 0x05,
  MAG_DATAYH    = 0x06,
  MAG_DATAZL    = 0x07,
  MAG_DATAZH    = 0x08,
  MAG_STAT2     = 0x09,
  MAG_CNTL1     = 0x0A,
  MAG_CNTL2     = 0x0B,
  MAG_STR       = 0x0C,
  MAG_TEMPL     = 0x1C,
  MAG_TEMPH     = 0x1D,
  MAG_TCCNTL    = 0x40,
  MAG_AVGCNTL   = 0x41,
  MAG_PDCNTL    = 0x42,
  MAG_COMPY11L  = 0x9C,
  MAG_COMPY11H  = 0x9D,
  MAG_COMPY12L  = 0x9E,
  MAG_COMPY12H  = 0x9F,
  MAG_COMPY13L  = 0xA0,
  MAG_COMPY13H  = 0xA1,
  MAG_COMPY21L  = 0xA2,
  MAG_COMPY21H  = 0xA3,
  MAG_COMPY22L  = 0xA4,
  MAG_COMPY22H  = 0xA5,
  MAG_COMPY23L  = 0xA6,
  MAG_COMPY23H  = 0xA7,
  MAG_COMPY31L  = 0xA8,
  MAG_COMPY31H  = 0xA9,
  MAG_COMPY32L  = 0xAA,
  MAG_COMPY32H  = 0xAB,
  MAG_COMPY33L  = 0xAC,
  MAG_COMPY33H  = 0xAD
}ist8310_reg_t;

/**
 * @brief 用于保存IST8310原始输出数据的结构体
 * 
 */
typedef struct ist8310_3axes
{
  /** 16位的X轴分量原始值 */
  rt_int16_t x;

  /** 16位的Y轴分量原始值 */
  rt_int16_t y;

  /** 16位的Z轴分量原始值 */
  rt_int16_t z;

  /** 16位的温度原始值 */
  rt_int16_t temp;
  
}*ist8310_3axes_t;

/**
 * @brief 用于输出IST8310换算后的数据的结构体
 * 
 */
typedef struct rt_ist8310_data
{
  /** X轴分量，单位为uT */
  float x;

  /** Y轴分量，单位为uT */
  float y;

  /** Z轴分量，单位为uT */
  float z;

  /** 温度，单位为℃ */
  float temp;

}*rt_ist8310_data_t;

/**
 * @brief IST8310的GPIO映射
 * 
 */
typedef struct rt_ist8310_gpio_mapping
{
  /** 复位引脚所在的GPIO端口 */
  GPIO_TypeDef *rstn_port;

  /** 复位引脚的引脚序号 */
  uint16_t rstn_pin;

}*rt_ist8310_gpio_mapping_t;


/**
 * @brief IST8310的设备驱动结构体，接入到RT-Thread设备驱动框架
 * 
 */
typedef struct rt_ist8310
{
  /** RT-Thread设备驱动框架必需的结构 */
  struct rt_device parent;

  /** I2C通信端口 */
  I2C_HandleTypeDef *interface;

  /** GPIO映射配置 */
  struct rt_ist8310_gpio_mapping mapping;

  /** I2C设备地址 */
  rt_uint8_t address;
  
  /** IST8310的正交补偿矩阵的矩阵相关定义 */
  arm_matrix_instance_f32 compensation_matrix;

  /** IST8310的正交补偿矩阵的数据存放位置 */
  float compensation_matrix_data[9];

  /** IST8310的原始数据 */
  ist8310_3axes_t data;

  /** IST8310正交补偿运算时的运算缓冲区 */
  struct ist8310_3axes data_buffer[2];

  /** 设备锁，防止多线程写入冲突 */
  struct rt_semaphore lock;
  
}*rt_ist8310_t;

rt_ist8310_t rt_ist8310_create(void);
void rt_ist8310_destroy(rt_ist8310_t mag);
rt_err_t rt_ist8310_register(rt_ist8310_t mag, const char *name);
rt_err_t rt_ist8310_unregister(rt_ist8310_t mag);
void rt_ist8310_recv(rt_ist8310_t mag);
void rt_ist8310_recv_cplt(rt_ist8310_t mag);

#endif