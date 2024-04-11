/* THIS FILE IS PARTLY BASED ON https://github.com/gmyFighting/bmi088/blob/master/inc/bmi088.h */
#ifndef __RT_BMI088_H__
#define __RT_BMI088_H__

#include <rtthread.h>
#include <stdint.h>
#include <stm32f4xx_hal.h>
#include <arm_math.h>
//#include "sensor.h"



/*************************** Common Macros for both Accel and Gyro *****************************/
// Bit #0  : Read/Write bit
// Bit #1-7: Address AD
#define BMI08X_SPI_RD_MASK                          UINT8_C(0x80)
#define BMI08X_SPI_WR_MASK                          UINT8_C(0x7F)

/* CMD: soft reset */
#define BMI08X_SOFT_RESET_CMD                       UINT8_C(0xB6)

/* CMD: accel power save */
#define BMI08X_ACCEL_PWR_ACTIVE_CMD                 UINT8_C(0x00)
#define BMI08X_ACCEL_PWR_SUSPEND_CMD                UINT8_C(0x03)

/* CMD: accel power control */ 
#define BMI08X_ACCEL_POWER_DISABLE_CMD              UINT8_C(0x00)
#define BMI08X_ACCEL_POWER_ENABLE_CMD               UINT8_C(0x04)

/* Accel Power Mode */
#define BMI08X_ACCEL_PM_ACTIVE                      UINT8_C(0x00)
#define BMI08X_ACCEL_PM_SUSPEND                     UINT8_C(0x03)

/* Gyro Power mode */
#define BMI08X_GYRO_PM_NORMAL                       UINT8_C(0x00)
#define BMI08X_GYRO_PM_DEEP_SUSPEND                 UINT8_C(0x20)
#define BMI08X_GYRO_PM_SUSPEND                      UINT8_C(0x80)

/* Accel Bandwidth */
#define BMI08X_ACCEL_BW_OSR4                        UINT8_C(0x00 << 4)
#define BMI08X_ACCEL_BW_OSR2                        UINT8_C(0x01 << 4)
#define BMI08X_ACCEL_BW_NORMAL                      UINT8_C(0x02 << 4)

/* Accel Output Data Rate */
#define BMI08X_ACCEL_ODR_12_5_HZ                    UINT8_C(0x05)
#define BMI08X_ACCEL_ODR_25_HZ                      UINT8_C(0x06)
#define BMI08X_ACCEL_ODR_50_HZ                      UINT8_C(0x07)
#define BMI08X_ACCEL_ODR_100_HZ                     UINT8_C(0x08)
#define BMI08X_ACCEL_ODR_200_HZ                     UINT8_C(0x09)
#define BMI08X_ACCEL_ODR_400_HZ                     UINT8_C(0x0A)
#define BMI08X_ACCEL_ODR_800_HZ                     UINT8_C(0x0B)
#define BMI08X_ACCEL_ODR_1600_HZ                    UINT8_C(0x0C)

#define BMI08X_ACCEL_CONF_SET                       UINT8_C(0x80)

/* Accel Range */
#define BMI08X_ACCEL_RANGE_3G                       UINT8_C(0x00)
#define BMI08X_ACCEL_RANGE_6G                       UINT8_C(0x01)
#define BMI08X_ACCEL_RANGE_12G                      UINT8_C(0x02)
#define BMI08X_ACCEL_RANGE_24G                      UINT8_C(0x03)

/* Accel Self test */
#define BMI08X_ACCEL_DISABLE_SELF_TEST              UINT8_C(0x00)
#define BMI08X_ACCEL_POS_SELF_TEST                  UINT8_C(0x0D)
#define BMI08X_ACCEL_NEG_SELF_TEST                  UINT8_C(0x09)

/* Accel interrupt pin configurations */
#define BMI08X_ACCEL_INT_INPUT                      UINT8_C(0x01 << 4)
#define BMI08X_ACCEL_INT_OUTPUT                     UINT8_C(0x01 << 3)
#define BMI08X_ACCEL_INT_OUTPUT_OD                  UINT8_C(0x01 << 2)
#define BMI08X_ACCEL_INT_OUTPUT_PP                  UINT8_C(0x00 << 2)
#define BMI08X_ACCEL_INT_ACTIVE_HIGH                UINT8_C(0x01 << 1)
#define BMI08X_ACCEL_INT_ACTIVE_LOW                 UINT8_C(0x00 << 1)

/* Accel interrupt mapping */
#define BMI08X_ACCEL_INT1_DRDY_INTERRUPT            UINT8_C(0x01 << 2)
#define BMI08X_ACCEL_INT1_FWM_INTERRUPT             UINT8_C(0x01 << 1)
#define BMI08X_ACCEL_INT1_FFULL_INTERRUPT           UINT8_C(0x01 << 0)
#define BMI08X_ACCEL_INT2_DRDY_INTERRUPT            UINT8_C(0x01 << 6)
#define BMI08X_ACCEL_INT2_FWM_INTERRUPT             UINT8_C(0x01 << 5)
#define BMI08X_ACCEL_INT2_FFULL_INTERRUPT           UINT8_C(0x01 << 4)

/* Gyro Range */
#define BMI08X_GYRO_RANGE_2000_DPS                  UINT8_C(0x00)
#define BMI08X_GYRO_RANGE_1000_DPS                  UINT8_C(0x01)
#define BMI08X_GYRO_RANGE_500_DPS                   UINT8_C(0x02)
#define BMI08X_GYRO_RANGE_250_DPS                   UINT8_C(0x03)
#define BMI08X_GYRO_RANGE_125_DPS                   UINT8_C(0x04)

/* Gyro Output data rate and bandwidth */
#define BMI08X_GYRO_BW_532_ODR_2000_HZ              UINT8_C(0x00)
#define BMI08X_GYRO_BW_230_ODR_2000_HZ              UINT8_C(0x01)
#define BMI08X_GYRO_BW_116_ODR_1000_HZ              UINT8_C(0x02)
#define BMI08X_GYRO_BW_47_ODR_400_HZ                UINT8_C(0x03)
#define BMI08X_GYRO_BW_23_ODR_200_HZ                UINT8_C(0x04)
#define BMI08X_GYRO_BW_12_ODR_100_HZ                UINT8_C(0x05)
#define BMI08X_GYRO_BW_64_ODR_200_HZ                UINT8_C(0x06)
#define BMI08X_GYRO_BW_32_ODR_100_HZ                UINT8_C(0x07)

#define BMI08X_GYRO_CONF_SET                        UINT8_C(0x80)

/* Gryo interrupt configurations */
#define BMI08X_GYRO_INT_DRDY                        UINT8_C(0x80)
#define BMI08X_GYRO_INT_FIFO                        UINT8_C(0x40)

/* Gryo interrupt pin configurations */
#define BMI08X_GYRO_INT3_OUTPUT_OD                  UINT8_C(0x01 << 3)
#define BMI08X_GYRO_INT3_OUTPUT_PP                  UINT8_C(0x00 << 3)
#define BMI08X_GYRO_INT3_ACTIVE_HIGH                UINT8_C(0x01 << 2)
#define BMI08X_GYRO_INT3_ACTIVE_LOW                 UINT8_C(0x00 << 2)
#define BMI08X_GYRO_INT4_OUTPUT_OD                  UINT8_C(0x01 << 1)
#define BMI08X_GYRO_INT4_OUTPUT_PP                  UINT8_C(0x00 << 1)
#define BMI08X_GYRO_INT4_ACTIVE_HIGH                UINT8_C(0x01 << 0)
#define BMI08X_GYRO_INT4_ACTIVE_LOW                 UINT8_C(0x00 << 0)

/* Gryo interrupt mapping */
#define BMI08X_GYRO_INT3_DRDY_INTERRUPT             UINT8_C(0x01 << 0)
#define BMI08X_GYRO_INT3_FIFO_INTERRUPT             UINT8_C(0x01 << 2)
#define BMI08X_GYRO_INT4_DRDY_INTERRUPT             UINT8_C(0x01 << 7)
#define BMI08X_GYRO_INT4_FIFO_INTERRUPT             UINT8_C(0x01 << 5)

/* Gryo Self test */
#define BMI08X_GYRO_SELF_TEST_TRIG                  UINT8_C(0x01 << 0)
#define BMI08X_GYRO_SELF_TEST_RDY                   UINT8_C(0x01 << 1)
#define BMI08X_GYRO_SELF_TEST_FAIL                  UINT8_C(0x01 << 2)
#define BMI08X_GYRO_SELF_TEST_OK                    UINT8_C(0x01 << 4)

#define BMI08X_ACCEL_DATA_SYNC_MODE_OFF 0x00
#define BMI08X_ACCEL_DATA_SYNC_MODE_400HZ 0x01
#define BMI08X_ACCEL_DATA_SYNC_MODE_1000HZ 0x02
#define BMI08X_ACCEL_DATA_SYNC_MODE_2000HZ 0x03

/* Wait Time */
#define BMI08X_ACCEL_SOFTRESET_DELAY_MS             UINT8_C(10) // though it's said that only 1ms needed, it never works when I delay just 1ms
#define BMI08X_GYRO_SOFTRESET_DELAY_MS              UINT8_C(30)
#define BMI08X_GYRO_POWER_MODE_CONFIG_DELAY_MS      UINT8_C(30)
#define BMI08X_ACCEL_POWER_MODE_CONFIG_DELAY_MS     UINT8_C(5)
#define BMI08X_POWER_CONFIG_DELAY_MS                UINT8_C(50)


#define BMI088_ACCEL_CHIP_ID                        UINT8_C(0x1E)
#define BMI088_GYRO_CHIP_ID                         UINT8_C(0x0F)

#define G (9.80665f)
#define deg2rad (PI / 180.0f)
#define rad2deg (180.0f / PI)

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
 * @brief BMI088加速度计部分寄存器名称与地址的对应关系
 * 
 */
typedef enum 
{
  ACC_CHIP_ID_REG             = 0x00,
  ACC_ERR_REG                 = 0x02,
  ACC_STATUS_REG              = 0x03,
  ACC_X_LSB_REG               = 0x12,
  ACC_X_MSB_REG               = 0x13,
  ACC_Y_LSB_REG               = 0x14,
  ACC_Y_MSB_REG               = 0x15,
  ACC_Z_LSB_REG               = 0x16,
  ACC_Z_MSB_REG               = 0x17,
  SENSORTIME_0_REG            = 0x18,
  SENSORTIME_1_REG            = 0x19,
  SENSORTIME_2_REG            = 0x1A,
  TEMP_MSB_REG                = 0x22,
  TEMP_LSB_REG                = 0x23,
  ACC_CONF_REG                = 0x40,
  ACC_RANGE_REG               = 0x41,
  INT1_IO_CTRL_REG            = 0x53,
  INT2_IO_CTRL_REG            = 0x54,
  INT1_INT2_MAP_DATA_REG      = 0x58,
  ACC_SELF_TEST_REG           = 0x6D,
  ACC_PWR_CONF_REG            = 0x7C,
  ACC_PWR_CTRL_REG            = 0x7D,
  ACC_SOFTRESET_REG           = 0x7E
} bmi088a_reg_list_t;

/**
 * @brief BMI088陀螺仪部分寄存器名称与地址的对应关系
 * 
 */
typedef enum 
{
  GYRO_CHIP_ID_REG            = 0x00,
  RATE_X_LSB_REG              = 0x02,
  RATE_X_MSB_REG              = 0x03,
  RATE_Y_LSB_REG              = 0x04,
  RATE_Y_MSB_REG              = 0x05,
  RATE_Z_LSB_REG              = 0x06,
  RATE_Z_MSB_REG              = 0x07,
  GYRO_INT_STAT_1_REG         = 0x0A,
  GYRO_RANGE_REG              = 0x0F,
  GYRO_BANDWIDTH_REG          = 0x10,
  GYRO_LPM1_REG               = 0x11,
  GYRO_SOFTRESET_REG          = 0x14,
  GYRO_INT_CTRL_REG           = 0x15,
  INT3_INT4_IO_CONF           = 0x16,
  INT3_INT4_IO_MAP            = 0x18,
  GYRO_SELF_TEST_REG          = 0x3C
} bmi088g_reg_list_t;

/**
 * @brief 用于保存BMI088原始输出数据的结构体
 * 
 */
struct bmi088_3axes
{
  /** 16位的X轴分量原始值 */
  rt_int16_t x;

  /** 16位的Y轴分量原始值 */
  rt_int16_t y;

  /** 16位的Z轴分量原始值 */
  rt_int16_t z;

  /** 16位的温度原始值 */
  rt_int16_t temp;

  /** 32位的时间戳（实际有效长度为24位） */
  rt_uint32_t timestamp;
};

/**
 * @brief 用于输出BMI088换算后的数据的结构体
 * 
 */
typedef struct rt_bmi088_data
{
  /** X轴分量，单位为g/dps */
  float x;

  /** Y轴分量，单位为g/dps */
  float y;

  /** Z轴分量，单位为g/dps */
  float z;

  /** 传感器温度，单位为℃ */
  float temp;

  /** 32位的时间戳（实际有效长度为24位） */
  rt_uint32_t timestamp;

}*rt_bmi088_data_t;

/**
 * @brief 用于保存BMI088配置的结构体
 * 
 */
typedef struct rt_bmi088_cfg
{
  /** 电源模式，包括正常、休眠等模式，参见RT_SENSOR_POWER系列的宏定义 */
  uint8_t power;

  /** 量程选择，保存了传感器量程寄存器的原始值，参见BMI08X_*_RANGE系列的宏定义 */
  uint8_t range;

  /** 
   * 滤波带宽选择，保存了传感器带宽寄存器的原始值。
   * 对于加速度计部分，数据速率和滤波带宽设置是相对独立的，所以该成员的值也是独立的，参见BMI08X_ACCEL_BW系列的宏定义
   * 但对于陀螺仪部分，数据速率和滤波带宽设置是绑定在同一个寄存器中的，参见BMI08X_GYRO_BW_*_ODR_*系列的宏定义
   * 所以该成员的值与数据速率配置成员的值相同，如果不同以带宽选择的配置为准，以下不再重复解释。
   */
  uint8_t bw;

  /** 数据输出速率选择，保存了传感器数据速率寄存器的原始值，对于加速度计部分，参见BMI08X_ACCEL_ODR系列的宏定义 */
  uint8_t odr;

}*rt_bmi088_cfg_t;

/**
 * @brief BMI088控制块，一个控制块用于控制加速度计或陀螺仪部分，包括传感器配置和传感器数据输出
 * 
 */
typedef struct rt_bmi088_ctrl
{
  /** 用于保存传感器配置的结构体 */
  struct rt_bmi088_cfg config;

  /** 用于保存传感器输出的结构体 */
  struct bmi088_3axes raw;

}*rt_bmi088_ctrl_t;

/**
 * @brief BMI088的GPIO映射
 * 
 */
typedef struct rt_bmi088_gpio_mapping
{
  /** 加速度计的片选引脚所在的GPIO端口 */
  GPIO_TypeDef *accel_cs_port;

  /** 陀螺仪的片选引脚所在的GPIO端口 */
  GPIO_TypeDef *gyro_cs_port;

  /** 加速度计的片选引脚的引脚序号 */
  uint16_t accel_cs_pin;

  /** 陀螺仪的片选引脚的引脚序号 */
  uint16_t gyro_cs_pin;

}*rt_bmi088_gpio_mapping_t;

/**
 * @brief BMI088的设备驱动结构体，接入到RT-Thread设备驱动框架
 * 
 */
typedef struct rt_bmi088
{
  /** RT-Thread设备驱动框架必需的结构 */
  struct rt_device parent;
  
  /** BMI088陀螺仪控制块 */
  struct rt_bmi088_ctrl gyro;
  
  /** BMI088加速度计控制块 */
  struct rt_bmi088_ctrl accel;

  /** 设备锁，防止多线程写入冲突 */
  struct rt_semaphore lock;
  
  /** SPI通信端口 */
  SPI_HandleTypeDef *interface;

  /** GPIO映射配置 */
  struct rt_bmi088_gpio_mapping mapping;
}*rt_bmi088_t;

rt_bmi088_t rt_bmi088_create(void);
void rt_bmi088_destroy(rt_bmi088_t imu);
rt_err_t rt_bmi088_register(rt_bmi088_t imu, const char *name);
rt_err_t rt_bmi088_unregister(rt_bmi088_t imu);
void rt_bmi088_gyro_recv(rt_bmi088_t imu);
void rt_bmi088_accel_recv(rt_bmi088_t imu);

#endif // BMI088_H
