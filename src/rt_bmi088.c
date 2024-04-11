/**
 * @file rt_bmi088.c
 * @author Jacky Wu (grade6_wty@163.com)
 * @brief BMI088惯性测量单元驱动程序
 * @version
 * @date 2024-04-05
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#define RT_BMI088_DEF
#include <rtthread.h>
#include <stm32f4xx_hal.h>
#include <rt_bmi088.h>
#include <stdlib.h>
#include <dwt.h>

/**
 * @brief 对BMI088的SPI总线的单次访问，同时发送和接收一个字节
 * 
 * @param imu BMI088设备结构
 * @param txdata 待发送的字节
 * @return rt_uint8_t 接收回来的字节
 */
static rt_uint8_t _bmi088_spi_access(rt_bmi088_t imu, rt_uint8_t txdata)
{
  rt_uint8_t rxdata;
  HAL_SPI_TransmitReceive(imu->interface, &txdata, &rxdata, 1, 1);
  return rxdata;
}

/**
 * @brief 读取BMI088加速度计部分的寄存器，支持连续读取
 * 
 * @param imu BMI088设备结构
 * @param reg_addr 寄存器地址
 * @param buf 读取得到的数据的存放地址
 * @param len 读取长度
 * @return rt_err_t 读取结果，RT_EOK代表读取成功
 */
static rt_err_t _bmi088a_spi_read(rt_bmi088_t imu, bmi088a_reg_list_t reg_addr, rt_uint8_t *buf, rt_size_t len)
{
  HAL_GPIO_WritePin(imu->mapping.accel_cs_port, imu->mapping.accel_cs_pin, GPIO_PIN_RESET);
  
  reg_addr |= BMI08X_SPI_RD_MASK;
  _bmi088_spi_access(imu, reg_addr);
  _bmi088_spi_access(imu, reg_addr); // dummy read to fit accel spi interface
  for (rt_size_t i = 0; i < len; ++i)
    *(buf + i) = _bmi088_spi_access(imu, reg_addr);
  
  HAL_GPIO_WritePin(imu->mapping.accel_cs_port, imu->mapping.accel_cs_pin, GPIO_PIN_SET);
  
  return RT_EOK;
}

/**
 * @brief 写入BMI088加速度计部分的寄存器，单个写入
 * 
 * @param imu BMI088设备结构
 * @param reg_addr 寄存器地址
 * @param data 待写入的数据，不是地址！
 * @return rt_err_t 写入结果，RT_EOK代表写入成功
 */
static rt_err_t _bmi088a_spi_write(rt_bmi088_t imu, bmi088a_reg_list_t reg_addr, rt_uint8_t data)
{
  HAL_GPIO_WritePin(imu->mapping.accel_cs_port, imu->mapping.accel_cs_pin, GPIO_PIN_RESET);
  
  reg_addr &= BMI08X_SPI_WR_MASK;
  _bmi088_spi_access(imu, reg_addr);
  _bmi088_spi_access(imu, data);
  
  HAL_GPIO_WritePin(imu->mapping.accel_cs_port, imu->mapping.accel_cs_pin, GPIO_PIN_SET);
  
  dwt_delay_us(2);
  return RT_EOK;
}

/**
 * @brief 读取BMI088陀螺仪部分的寄存器，支持连续读取
 * 
 * @param imu BMI088设备结构
 * @param reg_addr 寄存器地址
 * @param buf 读取得到的数据的存放地址
 * @param len 读取长度
 * @return rt_err_t 读取结果，RT_EOK代表读取成功
 */
static rt_err_t _bmi088g_spi_read(rt_bmi088_t imu, bmi088g_reg_list_t reg_addr, rt_uint8_t *buf, rt_size_t len)
{
  HAL_GPIO_WritePin(imu->mapping.gyro_cs_port, imu->mapping.gyro_cs_pin, GPIO_PIN_RESET);
  
  reg_addr |= BMI08X_SPI_RD_MASK;
  _bmi088_spi_access(imu, reg_addr);
  for (rt_size_t i = 0; i < len; ++i)
    *(buf + i) = _bmi088_spi_access(imu, reg_addr);
  
  HAL_GPIO_WritePin(imu->mapping.gyro_cs_port, imu->mapping.gyro_cs_pin, GPIO_PIN_SET);
  return RT_EOK;
}

/**
 * @brief 写入BMI088陀螺仪部分的寄存器，单个写入
 * 
 * @param imu BMI088设备结构
 * @param reg_addr 寄存器地址
 * @param data 待写入的数据，不是地址！
 * @return rt_err_t 写入结果，RT_EOK代表写入成功
 */
static rt_err_t _bmi088g_spi_write(rt_bmi088_t imu, bmi088g_reg_list_t reg_addr, rt_uint8_t data)
{
  HAL_GPIO_WritePin(imu->mapping.gyro_cs_port, imu->mapping.gyro_cs_pin, GPIO_PIN_RESET);
  
  reg_addr &= BMI08X_SPI_WR_MASK;
  _bmi088_spi_access(imu, reg_addr);
  _bmi088_spi_access(imu, data);
  
  HAL_GPIO_WritePin(imu->mapping.gyro_cs_port, imu->mapping.gyro_cs_pin, GPIO_PIN_SET);
  dwt_delay_us(2);
  return RT_EOK;
}

/**
 * @brief BMI088加速度计软件复位
 *
 * @param dev BMI088设备结构
 *
 * @return rt_err_t 复位结果，RT_EOK代表复位成功
 */
static rt_err_t _bmi088a_soft_reset(rt_bmi088_t imu)
{
  rt_uint8_t dummy;
  rt_err_t ret = _bmi088a_spi_write(imu, ACC_SOFTRESET_REG, BMI08X_SOFT_RESET_CMD);
  if (ret == RT_EOK)
    rt_thread_mdelay(BMI08X_ACCEL_SOFTRESET_DELAY_MS); // soft reset of accel part requires 1ms delay
  _bmi088a_spi_read(imu, ACC_CHIP_ID_REG, &dummy, 1);    /* Dummy read */
  return ret;
}

/**
 * @brief BMI088陀螺仪软件复位
 *
 * @param dev BMI088设备结构
 *
 * @return rt_err_t 复位结果，RT_EOK代表复位成功
 */
static rt_err_t _bmi088g_soft_reset(rt_bmi088_t imu)
{
  rt_err_t ret = _bmi088g_spi_write(imu, GYRO_SOFTRESET_REG, BMI08X_SOFT_RESET_CMD);
  if (ret == RT_EOK)
    rt_thread_mdelay(BMI08X_GYRO_SOFTRESET_DELAY_MS); // soft reset of gyro part requires 30ms delay
  return ret;
}

/**
 * @brief 获取加速度计原始数据
 * 
 * @param imu BMI088设备结构
 * @param accel 加速度数据结果存放地址
 * @return rt_err_t 读取结果，RT_EOK代表读取成功
 */
static rt_err_t _bmi088_get_accel_raw(rt_bmi088_t imu, struct bmi088_3axes *accel)
{
  RT_ASSERT(accel != RT_NULL);

  rt_uint8_t buffer[9];
  rt_uint8_t lsb;
  rt_uint16_t msb;
  rt_err_t ret;

  if ((ret = _bmi088a_spi_read(imu, ACC_X_LSB_REG, buffer, 9)) != RT_EOK)
    return ret;
  
  lsb = buffer[0];
  msb = buffer[1];
  accel->x = (msb << 8) | lsb; /* X */
  
  lsb = buffer[2];
  msb = buffer[3];
  accel->y = (msb << 8) | lsb; /* Y */

  lsb = buffer[4];
  msb = buffer[5];
  accel->z = (msb << 8) | lsb; /* Z */

  accel->timestamp = 0;
  for (int i = 0; i < 3; ++i)
    accel->timestamp |= ((rt_uint32_t)buffer[6 + i]) << (8 * i);

  if ((ret = _bmi088a_spi_read(imu, TEMP_MSB_REG, buffer, 2)) != RT_EOK)
    return ret;
  accel->temp = ((uint16_t)(buffer[0])) << 3 | buffer[1] >> 5;
  if (buffer[0] & 0x80)
    accel->temp |= 0xF800;
  
  return RT_EOK;
}

/**
 * @brief 获取陀螺仪原始数据
 * 
 * @param imu BMI088设备结构
 * @param accel 角速度数据结果存放地址
 * @return rt_err_t 读取结果，RT_EOK代表读取成功
 */
static rt_err_t _bmi088_get_gyro_raw(rt_bmi088_t imu, struct bmi088_3axes *gyro)
{
  rt_uint8_t buffer[6];
  rt_uint8_t lsb;
  rt_uint16_t msb;
  rt_err_t ret;
  
  if ((ret = _bmi088g_spi_read(imu, RATE_X_LSB_REG, buffer, 6)) != RT_EOK)
    return ret;
  
  lsb = buffer[0];
  msb = buffer[1];
  gyro->x = (msb << 8) | lsb; /* X */

  lsb = buffer[2];
  msb = buffer[3];
  gyro->y = (msb << 8) + lsb; /* Y */

  lsb = buffer[4];
  msb = buffer[5];
  gyro->z = (msb << 8) + lsb; /* Z */
  
  // temporarily enable accel part to get sensor time and temperature
  if ((ret = _bmi088a_spi_read(imu, SENSORTIME_0_REG, buffer, 3)) != RT_EOK)
    return ret;
  gyro->timestamp = 0;
  for (int i = 0; i < 3; ++i)
    gyro->timestamp |= ((rt_uint32_t)buffer[i]) << (8 * i);
  
  if ((ret = _bmi088a_spi_read(imu, TEMP_MSB_REG, buffer, 2)) != RT_EOK)
    return ret;
  gyro->temp = ((uint16_t)(buffer[0])) << 3 | buffer[1] >> 5;
  if (buffer[0] & 0x80)
    gyro->temp |= 0xF800;

  return RT_EOK;
}

/**
 * @brief 加速度计自检
 * 
 * @param imu BMI088设备结构
 * @return rt_err_t 自检结果，RT_EOK代表自检通过
 */
static rt_err_t _bmi088a_selftest(rt_bmi088_t imu)
{
  static rt_uint8_t selftest_seq[] = {
    ACC_RANGE_REG,  BMI08X_ACCEL_RANGE_24G, // maximum range of ±24g
    ACC_CONF_REG,   BMI08X_ACCEL_CONF_SET | BMI08X_ACCEL_BW_NORMAL | BMI08X_ACCEL_ODR_1600_HZ, // ODR=1.6kHz, continuous sampling, normal mode
  };
  struct bmi088_3axes positive_offset, negative_offset;
  for (int i = 0; i < sizeof(selftest_seq) / 2; i++)
    _bmi088a_spi_write(imu, selftest_seq[i], selftest_seq[i + 1]);
  rt_thread_mdelay(2); // wait for > 2ms

  _bmi088a_spi_write(imu, ACC_SELF_TEST_REG, BMI08X_ACCEL_POS_SELF_TEST); // positive self test
  rt_thread_mdelay(50); // wait for > 50ms
  _bmi088_get_accel_raw(imu, &positive_offset);

  _bmi088a_spi_write(imu, ACC_SELF_TEST_REG, BMI08X_ACCEL_NEG_SELF_TEST); // negative self test
  rt_thread_mdelay(50); // wait for > 50ms
  _bmi088_get_accel_raw(imu, &negative_offset);

  _bmi088a_spi_write(imu, ACC_SELF_TEST_REG, BMI08X_ACCEL_DISABLE_SELF_TEST); // disable self test
  rt_thread_mdelay(50); // wait for > 50ms to let the sensor settle to normal mode

  /**
   * the self test passes only when the differences between positive result and negative result larger than:
   * X: 1000mg
   * Y: 1000mg
   * Z: 500mg
   */
  if (abs((int16_t)positive_offset.x - (int16_t)negative_offset.x) / 32768.0f * 1000.0f * 1.5f * 16.0f < 1000 ||\
      abs((int16_t)positive_offset.y - (int16_t)negative_offset.y) / 32768.0f * 1000.0f * 1.5f * 16.0f < 1000 ||\
      abs((int16_t)positive_offset.z - (int16_t)negative_offset.z) / 32768.0f * 1000.0f * 1.5f * 16.0f < 500)
    return RT_ERROR;
  return RT_EOK;
}

/**
 * @brief 陀螺仪自检
 * 
 * @param imu BMI088设备结构
 * @return rt_err_t 自检结果，RT_EOK代表自检通过
 */
static rt_err_t _bmi088g_selftest(rt_bmi088_t imu)
{
  _bmi088g_spi_write(imu, GYRO_SELF_TEST_REG, BMI08X_GYRO_SELF_TEST_TRIG);
  rt_uint8_t test_result = 0;
  rt_err_t ret = RT_EOK;
  while (1)
  {
    if ((ret = _bmi088g_spi_read(imu, GYRO_SELF_TEST_REG, &test_result, 1)) != RT_EOK)
      return ret;
    
    if (test_result & BMI08X_GYRO_SELF_TEST_RDY)
    {
      if (test_result & BMI08X_GYRO_SELF_TEST_OK)
        return RT_EOK;
      else if (test_result & BMI08X_GYRO_SELF_TEST_FAIL)
        return RT_ERROR;
    }
    rt_thread_mdelay(1);
  }
}

/**
 * @brief 初始化加速度计
 * 
 * @param imu BMI088设备结构
 * @return rt_err_t 初始化结果，RT_EOK代表初始化通过
 */
static rt_err_t _bmi088a_init(rt_bmi088_t imu)
{
  rt_err_t ret = RT_EOK;
  rt_uint8_t chip_acc_id = 0;
  
  if (imu->accel.config.range < BMI08X_ACCEL_RANGE_3G &&\
      imu->accel.config.range > BMI08X_ACCEL_RANGE_24G)
    imu->accel.config.range = BMI08X_ACCEL_RANGE_24G; // maximum range by default
  
  if (imu->accel.config.power != BMI08X_ACCEL_PM_SUSPEND &&\
      imu->accel.config.power != BMI08X_ACCEL_PM_ACTIVE)
    imu->accel.config.power = BMI08X_ACCEL_PM_ACTIVE; // active power by default
  
  if (imu->accel.config.odr < BMI08X_ACCEL_ODR_12_5_HZ &&\
      imu->accel.config.odr > BMI08X_ACCEL_ODR_1600_HZ)
    imu->accel.config.odr = BMI08X_ACCEL_ODR_1600_HZ; // 1600Hz by default
  
  if (imu->accel.config.bw != BMI08X_ACCEL_BW_OSR4 &&\
      imu->accel.config.bw != BMI08X_ACCEL_BW_OSR2 &&\
      imu->accel.config.bw != BMI08X_ACCEL_BW_NORMAL)
    imu->accel.config.bw = BMI08X_ACCEL_BW_NORMAL; // normal bandwidth by default

  /* STEP 1: check communication */
  _bmi088a_spi_read(imu, ACC_CHIP_ID_REG, &chip_acc_id, 1);    /* Dummy read */
  _bmi088a_spi_read(imu, ACC_CHIP_ID_REG, &chip_acc_id, 1);
  if (chip_acc_id != BMI088_ACCEL_CHIP_ID) 
  {
    rt_kprintf("Error: Failed to communicate with acc!\r\n");
    return RT_ERROR;        
  }
  
  /* STEP 2 reset the accelerometer */
  if ((ret = _bmi088a_soft_reset(imu)) != RT_EOK)
    return ret;
  
  /* STEP 3: power on the accelerometer */
  _bmi088a_spi_write(imu, ACC_PWR_CTRL_REG, BMI08X_ACCEL_POWER_ENABLE_CMD);
  rt_thread_mdelay(BMI08X_POWER_CONFIG_DELAY_MS); // a 50ms delay required after the first power on
  
  /* STEP 4: enable normal power mode */
  _bmi088a_spi_write(imu, ACC_PWR_CONF_REG, BMI08X_ACCEL_PM_ACTIVE); // leave suspend mode
  dwt_delay_us(450); // a 450us delay required after the first PM change of power on
  
//  if ((ret = _bmi088a_selftest(imu)) != RT_EOK)
//    return ret;

  /* STEP 5: load configurations */
  _bmi088a_spi_write(imu, ACC_CONF_REG, BMI08X_ACCEL_CONF_SET | imu->accel.config.bw | imu->accel.config.odr);
  _bmi088a_spi_write(imu, ACC_RANGE_REG, imu->accel.config.range);
  _bmi088a_spi_write(imu, INT1_IO_CTRL_REG, BMI08X_ACCEL_INT_OUTPUT | BMI08X_ACCEL_INT_OUTPUT_PP | BMI08X_ACCEL_INT_ACTIVE_HIGH);
  _bmi088a_spi_write(imu, INT1_INT2_MAP_DATA_REG, BMI08X_ACCEL_INT1_DRDY_INTERRUPT);

  // if suspend mode is requested
  _bmi088a_spi_write(imu, ACC_PWR_CONF_REG, imu->accel.config.power);
  rt_thread_mdelay(BMI08X_ACCEL_POWER_MODE_CONFIG_DELAY_MS); // a 5ms delay required after power mode changing
  
  return ret;
}

/**
 * @brief 初始化陀螺仪
 * 
 * @param imu BMI088设备结构
 * @return rt_err_t 初始化结果，RT_EOK代表初始化通过
 */
static rt_err_t _bmi088g_init(rt_bmi088_t imu)
{
	rt_err_t ret = RT_EOK;
  rt_uint8_t id = 0;

  if ((imu->gyro.config.bw & 0xF8) != 0) // bw is 0x00~0x07
    imu->gyro.config.bw = BMI08X_GYRO_BW_230_ODR_2000_HZ; // 230Hz bandwidth @ 2000Hz ODR by default
  
  imu->gyro.config.odr = imu->gyro.config.bw; // odr is bw 
  
  if (imu->gyro.config.power != BMI08X_GYRO_PM_DEEP_SUSPEND &&\
      imu->gyro.config.power != BMI08X_GYRO_PM_SUSPEND &&\
      imu->gyro.config.power != BMI08X_GYRO_PM_NORMAL)
    imu->gyro.config.power = BMI08X_GYRO_PM_NORMAL; // normal power mode by default
  
  if (imu->gyro.config.range < BMI08X_GYRO_RANGE_2000_DPS &&\
      imu->gyro.config.range > BMI08X_GYRO_RANGE_125_DPS)
    imu->gyro.config.range = BMI08X_GYRO_RANGE_2000_DPS; // 2000DPS by default
  
  /* STEP 1: check communication*/
  _bmi088g_spi_read(imu, GYRO_CHIP_ID_REG, &id, 1);
  if (id != BMI088_GYRO_CHIP_ID) 
  {
    rt_kprintf("Error: Failed to communicate with gyro!\n");
    return RT_ERROR;
  }
  
  /* STEP 2: reset the gyroscope */
  if ((ret = _bmi088g_soft_reset(imu)) != RT_EOK)
    return ret;
  
  /* STEP 3: load configurations */
  _bmi088g_spi_write(imu, GYRO_RANGE_REG, imu->gyro.config.range);
  _bmi088g_spi_write(imu, GYRO_BANDWIDTH_REG, BMI08X_GYRO_CONF_SET | imu->gyro.config.bw);
  _bmi088g_spi_write(imu, GYRO_INT_CTRL_REG, BMI08X_GYRO_INT_DRDY);
  _bmi088g_spi_write(imu, INT3_INT4_IO_CONF, BMI08X_GYRO_INT3_OUTPUT_PP | BMI08X_GYRO_INT3_ACTIVE_HIGH);
  _bmi088g_spi_write(imu, INT3_INT4_IO_MAP, BMI08X_GYRO_INT3_DRDY_INTERRUPT);
  
  // if suspend mode is requested
  _bmi088g_spi_write(imu, GYRO_LPM1_REG, imu->gyro.config.power);
  rt_thread_mdelay(BMI08X_GYRO_POWER_MODE_CONFIG_DELAY_MS);
  
  return ret;
}

/**
 * @brief 打开BMI088设备，接入到RT-Thread设备框架
 * 
 * @param dev 设备指针
 * @param oflag 打开标志
 * @return rt_err_t 初始化结果，RT_EOK代表打开成功
 */
static rt_err_t rt_bmi088_instance_open(rt_device_t dev, rt_uint16_t oflag)
{
  rt_bmi088_t imu = (rt_bmi088_t)dev;
  rt_err_t ret = RT_EOK;
  
  rt_sem_take(&imu->lock, RT_WAITING_FOREVER);
  
  ret |= _bmi088a_init(imu);
  ret |= _bmi088g_init(imu);
  
  if (ret == RT_EOK)
    dev->open_flag = (oflag & RT_DEVICE_OFLAG_RDONLY);
  
  rt_sem_release(&imu->lock);
  
  return ret;
}

/**
 * @brief 关闭BMI088设备，接入到RT-Thread设备框架
 * 
 * @param dev 设备指针
 * @return rt_err_t 初始化结果，RT_EOK代表关闭成功
 */
static rt_err_t rt_bmi088_instance_close(rt_device_t dev)
{
  rt_bmi088_t imu = (rt_bmi088_t)dev;
  rt_err_t ret = RT_EOK;

  rt_sem_take(&imu->lock, RT_WAITING_FOREVER);

  ret |= _bmi088a_soft_reset(imu);
  ret |= _bmi088g_soft_reset(imu);

  rt_sem_release(&imu->lock);
  return ret;
}

/**
 * @brief 从设备缓冲区中读取、解码内容，接入到RT-Thread设备框架
 * 
 * @param dev 设备指针
 * @param pos 读取数据类型，0代表加速度，1代表角速度
 * @param buffer 读取缓冲区
 * @param size 数据个数，当读取位置为0且个数为2时，同时读取加速度和角速度
 * @return rt_size_t 读取结果，RT_EOK代表读取成功
 */
static rt_size_t rt_bmi088_instance_read(rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t size)
{
  rt_bmi088_t imu = (rt_bmi088_t)dev;
  rt_sem_take(&imu->lock, RT_WAITING_FOREVER);
  struct rt_bmi088_data *result = (rt_bmi088_data_t)buffer;

  // limit reading range to 0~1
  rt_uint8_t start, end;
  if (pos > 1)
    return 0;
  else if (pos <= 0)
    start = 0;
  else
    start = 1;
  if (size <= 0)
    return 0;
  if (start + size - 1 >= 2)
    end = 1;
  else
    end = start + size - 1;

  if (start == 0)
  {
    result[0].x         = ((float)imu->accel.raw.x) / 32768.0f * (2 << imu->accel.config.range) * 1.5;
    result[0].y         = ((float)imu->accel.raw.y) / 32768.0f * (2 << imu->accel.config.range) * 1.5;
    result[0].z         = ((float)imu->accel.raw.z) / 32768.0f * (2 << imu->accel.config.range) * 1.5;
    result[0].temp      = ((float)imu->accel.raw.temp) * 0.125f + 23;
    result[0].timestamp = imu->accel.raw.timestamp;
  }

  if (end == 1)
  {
    result[end - start].x         = (float)imu->gyro.raw.x / 32767.0f * (2000 >> imu->gyro.config.range);
    result[end - start].y         = (float)imu->gyro.raw.y / 32767.0f * (2000 >> imu->gyro.config.range);
    result[end - start].z         = (float)imu->gyro.raw.z / 32767.0f * (2000 >> imu->gyro.config.range);
    result[end - start].temp      = ((float)imu->gyro.raw.temp) * 0.125f + 23;
    result[end - start].timestamp = imu->gyro.raw.timestamp;
  }

  rt_sem_release(&imu->lock);
  return end - start + 1;
}

/**
 * @brief 配置BMI088设备，接入到RT-Thread设备框架
 * 
 * @param dev 设备指针
 * @param cmd 控制命令
 * @param args 控制参数
 * @return rt_err_t 控制结果，RT_EOK代表控制成功
 */
static rt_err_t rt_bmi088_instance_control(rt_device_t dev, int cmd, void *args)
{
  rt_bmi088_t imu = (rt_bmi088_t)dev;
  rt_uint8_t *power_mode = (rt_uint8_t*)args;
  rt_err_t ret = RT_EOK;
  rt_uint8_t reg;
  
  rt_sem_take(&imu->lock, RT_WAITING_FOREVER);

  switch (cmd)
  {
  case RT_SENSOR_CTRL_GET_ID:
    RT_ASSERT(args != RT_NULL);
    ((rt_uint8_t *)args)[0] = 0x1E;
    ((rt_uint8_t *)args)[1] = 0x0F;
    break;
  case RT_SENSOR_CTRL_SET_ODR: // and also bandwidth
    if (args != RT_NULL)
    {
      rt_bmi088_cfg_t accel_cfg = (rt_bmi088_cfg_t)args, gyro_cfg = (rt_bmi088_cfg_t)args + 1;
      imu->accel.config.odr = accel_cfg->odr;
      imu->accel.config.bw  = accel_cfg->bw;
      imu->gyro.config.odr  = gyro_cfg->odr;
      imu->gyro.config.bw   = gyro_cfg->odr; // the gyroscope's bw is equivalent to odr
    }
    ret |= _bmi088a_spi_write(imu, ACC_CONF_REG, imu->accel.config.odr | imu->accel.config.bw | BMI08X_ACCEL_CONF_SET);
    ret |= _bmi088g_spi_write(imu, GYRO_BANDWIDTH_REG, imu->accel.config.odr | BMI08X_GYRO_CONF_SET);
    break;
  case RT_SENSOR_CTRL_SET_RANGE:
    if (args != RT_NULL)
    {
      rt_bmi088_cfg_t accel_cfg = (rt_bmi088_cfg_t)args, gyro_cfg = (rt_bmi088_cfg_t)args + 1;
      imu->accel.config.range = accel_cfg->range;
      imu->gyro.config.range  = gyro_cfg->range;
    }
    ret |= _bmi088a_spi_write(imu, ACC_RANGE_REG, imu->accel.config.range);
    ret |= _bmi088g_spi_write(imu, GYRO_RANGE_REG, imu->gyro.config.range);
    break;
  case RT_SENSOR_CTRL_SET_POWER:
    RT_ASSERT(power_mode != RT_NULL);
    switch (*power_mode)
    {
    case RT_SENSOR_POWER_NONE:
      break;
    case RT_SENSOR_POWER_DOWN:
      ret |= _bmi088a_spi_read(imu, ACC_PWR_CTRL_REG, &reg, 1);
      if (reg != BMI08X_ACCEL_POWER_DISABLE_CMD)
      {
        ret |= _bmi088a_spi_write(imu, ACC_PWR_CTRL_REG, BMI08X_ACCEL_POWER_DISABLE_CMD);
        rt_thread_mdelay(BMI08X_POWER_CONFIG_DELAY_MS);
      }

      ret |= _bmi088g_spi_read(imu, GYRO_LPM1_REG, &reg, 1);
      if (reg != BMI08X_GYRO_PM_DEEP_SUSPEND)
      {
        ret |= _bmi088g_spi_write(imu, GYRO_LPM1_REG, BMI08X_GYRO_PM_DEEP_SUSPEND);
        rt_thread_mdelay(BMI08X_GYRO_POWER_MODE_CONFIG_DELAY_MS);
      }
      break;
    case RT_SENSOR_POWER_LOW:
      ret |= _bmi088a_spi_read(imu, ACC_PWR_CTRL_REG, &reg, 1);
      if (reg != BMI08X_ACCEL_POWER_ENABLE_CMD)
      {
        ret |= _bmi088a_spi_write(imu, ACC_PWR_CTRL_REG, BMI08X_ACCEL_POWER_ENABLE_CMD);
        rt_thread_mdelay(BMI08X_POWER_CONFIG_DELAY_MS);
      }
      ret |= _bmi088a_spi_read(imu, ACC_PWR_CONF_REG, &reg, 1);
      if (reg != BMI08X_ACCEL_PM_SUSPEND)
      {
        ret |= _bmi088a_spi_write(imu, ACC_PWR_CONF_REG, BMI08X_ACCEL_PM_SUSPEND);
        rt_thread_mdelay(BMI08X_POWER_CONFIG_DELAY_MS);
      }

      ret |= _bmi088g_spi_read(imu, GYRO_LPM1_REG, &reg, 1);
      if (reg != BMI08X_GYRO_PM_SUSPEND)
      {
        ret |= _bmi088g_spi_write(imu, GYRO_LPM1_REG, BMI08X_GYRO_PM_SUSPEND);
        rt_thread_mdelay(BMI08X_GYRO_POWER_MODE_CONFIG_DELAY_MS);
      }
      break;
    case RT_SENSOR_POWER_NORMAL:
    case RT_SENSOR_POWER_HIGH:
      ret |= _bmi088a_spi_read(imu, ACC_PWR_CTRL_REG, &reg, 1);
      if (reg != BMI08X_ACCEL_POWER_ENABLE_CMD)
      {
        ret |= _bmi088a_spi_write(imu, ACC_PWR_CTRL_REG, BMI08X_ACCEL_POWER_ENABLE_CMD);
        rt_thread_mdelay(BMI08X_POWER_CONFIG_DELAY_MS);
      }
      ret |= _bmi088a_spi_read(imu, ACC_PWR_CONF_REG, &reg, 1);
      if (reg != BMI08X_ACCEL_PM_ACTIVE)
      {
        ret |= _bmi088a_spi_write(imu, ACC_PWR_CONF_REG, BMI08X_ACCEL_PM_ACTIVE);
        rt_thread_mdelay(BMI08X_POWER_CONFIG_DELAY_MS);
      }

      ret |= _bmi088g_spi_read(imu, GYRO_LPM1_REG, &reg, 1);
      if (reg != BMI08X_GYRO_PM_NORMAL)
      {
        ret |= _bmi088g_spi_write(imu, GYRO_LPM1_REG, BMI08X_GYRO_PM_NORMAL);
        rt_thread_mdelay(BMI08X_GYRO_POWER_MODE_CONFIG_DELAY_MS);
      }
      break;
    default:
      ret = -RT_EINVAL;
    }
    break;
  case RT_SENSOR_CTRL_SELF_TEST:
    ret |= _bmi088a_selftest(imu);
    ret |= _bmi088g_selftest(imu);
    break;
  default:
    ret = -RT_EINVAL;
  }
  
  rt_sem_release(&imu->lock);
  return ret;
}

#ifdef RT_USING_DEVICE_OPS
static const struct rt_device_ops rt_bmi088_ops {
  .open     = rt_bmi088_instance_open,
  .close    = rt_bmi088_instance_close,
  .read     = rt_bmi088_instance_read,
  .control  = rt_bmi088_instance_control
};
#endif

/**
 * @brief 创建BMI088设备结构
 * 
 * @return rt_bmi088_t BMI088设备指针，非RT_NULL值代表创建成功
 */
rt_bmi088_t rt_bmi088_create(void)
{
  rt_bmi088_t imu = (rt_bmi088_t)rt_device_create(RT_Device_Class_Sensor, sizeof(struct rt_bmi088) - sizeof(struct rt_device));
  if (imu != RT_NULL)
  {
    imu->interface = RT_NULL;
    rt_memset(&imu->mapping, 0, sizeof(imu->mapping));
  }
  return imu;
}

/**
 * @brief 销毁BMI088设备结构
 * 
 * @param imu BMI088设备指针
 */
void rt_bmi088_destroy(rt_bmi088_t imu)
{
  rt_device_destroy((rt_device_t)imu);
}

/**
 * @brief 注册BMI088设备
 * 
 * @param imu BMI088设备指针
 * @param name 设备名称
 * @return rt_err_t 注册结果，RT_EOK代表注册成功
 */
rt_err_t rt_bmi088_register(rt_bmi088_t imu, const char *name)
{
  RT_ASSERT(imu != RT_NULL);
  RT_ASSERT(imu->interface != RT_NULL);
  
  rt_device_t dev = &imu->parent;
  
  // initialize its lock and semaphores
  char lock_name[RT_NAME_MAX] = {0};
  rt_strncpy(lock_name, name, RT_NAME_MAX);
  rt_sem_init(&imu->lock, lock_name, RT_IPC_FLAG_PRIO, 1);
  
  // filling the class methods
  #ifdef RT_USING_DEVICE_OPS
  dev->ops      = &rt_bmi088_ops;
  #else
  dev->open     = rt_bmi088_instance_open;
  dev->close    = rt_bmi088_instance_close;
  dev->read     = rt_bmi088_instance_read;
  dev->control  = rt_bmi088_instance_control;
  #endif
  
  return rt_device_register(dev, name, RT_DEVICE_FLAG_RDONLY | RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_ACTIVATED);
}

/**
 * @brief 反注册BMI088设备
 * 
 * @param imu BMI088设备指针
 * @return rt_err_t 反注册结果，RT_EOK代表反注册成功
 */
rt_err_t rt_bmi088_unregister(rt_bmi088_t imu)
{
  RT_ASSERT(imu != RT_NULL);
  
  rt_sem_detach(&imu->lock);
  
  return rt_device_unregister((rt_device_t)imu);
}

/**
 * @brief 从BMI088陀螺仪设备中读取原始值
 * 
 * @param imu BMI088设备指针
 */
__INLINE void rt_bmi088_gyro_recv(rt_bmi088_t imu)
{
  if (imu == RT_NULL || (imu->parent.open_flag & RT_DEVICE_OFLAG_OPEN) == 0)
    return;
  if (rt_sem_take(&imu->lock, RT_WAITING_NO) != RT_EOK)
    return;
  _bmi088_get_gyro_raw(imu, &imu->gyro.raw);
  if (imu->parent.rx_indicate != RT_NULL)
    imu->parent.rx_indicate(&imu->parent, 1);
  rt_sem_release(&imu->lock);
}


/**
 * @brief 从BMI088加速度计设备中读取原始值
 * 
 * @param imu BMI088设备指针
 */
__INLINE void rt_bmi088_accel_recv(rt_bmi088_t imu)
{
  if (imu == RT_NULL || (imu->parent.open_flag & RT_DEVICE_OFLAG_OPEN) == 0)
    return;
  if (rt_sem_take(&imu->lock, RT_WAITING_NO) != RT_EOK)
    return;
  _bmi088_get_accel_raw(imu, &imu->accel.raw);
  if (imu->parent.rx_indicate != RT_NULL)
    imu->parent.rx_indicate(&imu->parent, 1);
  rt_sem_release(&imu->lock);
}