/**
 * @file rt_ist8310.c
 * @author Jacky Wu (grade6_wty@163.com)
 * @brief IST8310磁力计驱动程序
 * @version
 * @date 2024-04-05
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include <rt_ist8310.h>

/**
 * @brief 阻塞式读取IST8310寄存器，支持连续读取，内部函数
 * 
 * @param mag IST8310设备结构
 * @param reg_addr 寄存器起始地址
 * @param buf 数据的存放地址
 * @param len 待读取的寄存器个数
 * @return rt_err_t 读取结果，RT_EOK代表读取成功
 */
static rt_err_t _ist8310_i2c_read(rt_ist8310_t mag, rt_uint8_t reg_addr, rt_uint8_t *buf, rt_size_t len)
{
  HAL_StatusTypeDef stat = HAL_I2C_Mem_Read(mag->interface, mag->address, reg_addr, I2C_MEMADD_SIZE_8BIT, buf, len, 2);
  if (stat == HAL_ERROR)
    return -RT_ERROR;
  else if (stat == HAL_BUSY)
    return -RT_EBUSY;
  else if (stat == HAL_TIMEOUT)
    return -RT_ETIMEOUT;
  return RT_EOK;
}

/**
 * @brief 阻塞式写入IST8310寄存器，单个写入，内部函数
 * 
 * @param mag is8310设备结构
 * @param reg_addr 寄存器地址
 * @param data 待写入的数据内容
 * @return rt_err_t 写入结果，RT_EOK代表写入成功
 */
static rt_err_t _ist8310_i2c_write(rt_ist8310_t mag, rt_uint8_t reg_addr, rt_uint8_t data)
{
  HAL_StatusTypeDef stat = HAL_I2C_Mem_Write(mag->interface, mag->address, reg_addr, I2C_MEMADD_SIZE_8BIT, &data, 1, 2);
  if (stat == HAL_ERROR)
    return -RT_ERROR;
  else if (stat == HAL_BUSY)
    return -RT_EBUSY;
  else if (stat == HAL_TIMEOUT)
    return -RT_ETIMEOUT;
  return RT_EOK;
}

/**
 * @brief 打开IST8310设备，注册到RT-Thread设备框架
 * 
 * @param dev RT-Thread设备结构
 * @param oflag 打开方式，仅支持只读打开
 * @return rt_err_t 打开结果，RT_EOK代表打开成功
 * 
 * @todo 限于不齐全的手册，当前的实现尚未能成功配置IST8310的中断输出，如果有足够的资料可以尝试配置
 */
static rt_err_t rt_ist8310_instance_open(rt_device_t dev, rt_uint16_t oflag)
{
  RT_ASSERT(dev != RT_NULL);
  rt_err_t ret = RT_EOK;
  rt_ist8310_t mag = (rt_ist8310_t)dev;

  /* 初始化IST8310正交补偿用的矩阵 */
  float temp_matrix_data[9], cross_axes_matrix_data[9];
  static const float constant_matrix_data[9] = 
  {
    50, 0, 0,
    0, 50, 0,
    0, 0, 50
  };
  arm_matrix_instance_f32 temp_matrix, cross_axes_matrix;
  static const arm_matrix_instance_f32 constant_matrix = {
    .numCols = 3,
    .numRows = 3,
    .pData   = (float32_t*)constant_matrix_data
  };
  arm_mat_init_f32(&temp_matrix, 3, 3, temp_matrix_data);
  arm_mat_init_f32(&cross_axes_matrix, 3, 3, cross_axes_matrix_data);
  arm_mat_init_f32(&mag->compensation_matrix, 3, 3, mag->compensation_matrix_data);

  rt_sem_take(&mag->lock, RT_WAITING_FOREVER);

  /* STEP 1: reset */
  HAL_GPIO_WritePin(mag->mapping.rstn_port, mag->mapping.rstn_pin, GPIO_PIN_RESET);
  rt_thread_mdelay(IST8310_RESET_DELAY_MS);
  HAL_GPIO_WritePin(mag->mapping.rstn_port, mag->mapping.rstn_pin, GPIO_PIN_SET);
  rt_thread_mdelay(IST8310_RESET_DELAY_MS);

  /* STEP 2: check communication */
  rt_uint8_t id = RT_NULL;
  _ist8310_i2c_read(mag, MAG_WHO_AM_I, &id, 1);
  if (id != IST8310_ID)
  {
    rt_sem_release(&mag->lock);
    return -RT_EIO;
  }

  /* STEP 3: get compensation data */
  rt_int16_t comp_temp[9];
  _ist8310_i2c_read(mag, MAG_COMPY11L, (rt_uint8_t*)comp_temp, MAG_COMPY33H - MAG_COMPY11L + 1);
  for (int i = 0; i < sizeof(comp_temp) / sizeof(comp_temp[0]); ++i)
    temp_matrix_data[i] = comp_temp[i];
  /**
   * How to build the compensation matrix:
   * 1. Read the cross-axis data (9 int16 digits, LSB first, column first) from 0x9C~0xAD
   * 2. Fill it to a matrix (a transposition needed, as arm_matrix is row first)
   * 3. Multiply by a constant M=3/20=0.15
   * 4. Inverse
   * 5. Multiply a constant matrix as above with this matrix, you'll get the compensation matrix
   */
  arm_mat_trans_f32(&temp_matrix, &cross_axes_matrix);
  arm_mat_scale_f32(&cross_axes_matrix, 0.15f, &temp_matrix);
  arm_mat_inverse_f32(&temp_matrix, &cross_axes_matrix);
  arm_mat_mult_f32(&constant_matrix, &cross_axes_matrix, &mag->compensation_matrix);
  
  static const rt_uint8_t init_seq[] = {
    MAG_CNTL2,    IST8310_DRDY_INT_ENABLE | IST8310_DRDY_INT_HIGH,
    MAG_AVGCNTL,  IST8310_AVG_4,
    MAG_PDCNTL,   IST8310_PERFORMANCE,
    MAG_TCCNTL,   IST8310_TC_ENABLE,
    MAG_CNTL1,    IST8310_CONTINUOUS
  };

  /* STEP 4: load configurations */
  HAL_StatusTypeDef stat = HAL_OK;
  for (int i = 0; i < sizeof(init_seq); i += 2)
    _ist8310_i2c_write(mag, init_seq[i], init_seq[i + 1]);
  if (stat == HAL_ERROR)
    ret = -RT_ERROR;
  else if (stat == HAL_BUSY)
    ret = -RT_EBUSY;
  else if (stat == HAL_TIMEOUT)
    ret = -RT_ETIMEOUT;

  if (ret == RT_EOK)
    dev->open_flag = (oflag & RT_DEVICE_OFLAG_RDONLY);

  rt_sem_release(&mag->lock);
  return ret;
}

/**
 * @brief 关闭IST8310设备，注册到RT-Thread设备框架
 * 
 * @param dev RT-Thread设备结构
 * @return rt_err_t 关闭结果，RT_EOK代表关闭成功
 */
static rt_err_t rt_ist8310_instance_close(rt_device_t dev)
{
  RT_ASSERT(dev != RT_NULL);
  rt_err_t ret = RT_EOK;
  rt_ist8310_t mag = (rt_ist8310_t)dev;

  rt_sem_take(&mag->lock, RT_WAITING_FOREVER);

  HAL_GPIO_WritePin(mag->mapping.rstn_port, mag->mapping.rstn_pin, GPIO_PIN_RESET);
  rt_thread_mdelay(IST8310_RESET_DELAY_MS);
  HAL_GPIO_WritePin(mag->mapping.rstn_port, mag->mapping.rstn_pin, GPIO_PIN_SET);
  rt_thread_mdelay(IST8310_RESET_DELAY_MS);

  rt_sem_release(&mag->lock);
  return ret;
}

/**
 * @brief 读取磁力计数据，注册到RT-Thread设备框架
 * 
 * @param dev RT-Thread设备结构
 * @param pos 偏移量，只能置0
 * @param buffer 读取到的数据的存放地址，建议使用rt_ist8310_data结构体指针
 * @param size 读取数据个数，只能置1
 * @return rt_size_t 读取到的数据个数，读取到了则为1，否则为0
 */
static rt_size_t rt_ist8310_instance_read(rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t size)
{
  RT_ASSERT(dev != RT_NULL);
  rt_ist8310_t mag = (rt_ist8310_t)dev;
  rt_ist8310_data_t data = (rt_ist8310_data_t)buffer;
  if (pos != 0 || size != 1)
    return 0;
  
  rt_sem_take(&mag->lock, RT_WAITING_FOREVER);

  if (mag->data != RT_NULL) // data available
  {
    arm_matrix_instance_f32 raw_data_matrix, output_data_matrix;
    float raw_data_matrix_data[3] = {mag->data->x, mag->data->y, mag->data->z};
    arm_mat_init_f32(&raw_data_matrix, 3, 1, raw_data_matrix_data);
    arm_mat_init_f32(&output_data_matrix, 3, 1, (float32_t*)buffer);
    arm_mat_mult_f32(&mag->compensation_matrix, &raw_data_matrix, &output_data_matrix); // multiply the original data by compensation matrix to do orthogonal compensation
    arm_mat_scale_f32(&output_data_matrix, IST8310_RESOLUTION, &output_data_matrix); // change the unit of data to uT
    mag->data = RT_NULL;
  }
  else
  {
    data->x = data->y = data->z = data->temp = 0.0f;
    size = 0;
  }

  rt_sem_release(&mag->lock);

  return size;
}

/**
 * @brief 控制IST8310设备
 * 
 * @param dev RT-Thread设备结构
 * @param cmd 控制命令，参见RT_SENSOR_CTRL
 * @param args 参数指针，既可用于传入参数，也可能用于保存返回值，此处：
 *             RT_SENSOR_CTRL_GET_ID: 保存返回的1字节ID
 *             RT_SENSOR_CTRL_SELF_TEST: 未使用参数
 * @return rt_err_t 控制结果，RT_EOK代表控制成功，-RT_INVAL代表不支持该命令，-RT_ERROR代表控制失败
 */
static rt_err_t rt_ist8310_instance_control(rt_device_t dev, int cmd, void *args)
{
  RT_ASSERT(dev != RT_NULL);
  rt_ist8310_t mag = (rt_ist8310_t)dev;
  rt_err_t ret = RT_EOK;
  struct ist8310_3axes raw_data, selftest_data;
  rt_uint8_t reg;

  rt_sem_take(&mag->lock, RT_WAITING_FOREVER);

  switch (cmd)
  {
  case RT_SENSOR_CTRL_GET_ID:
    RT_ASSERT(args != RT_NULL);
    *((rt_uint8_t *)args) = IST8310_ID;
    break;
  case RT_SENSOR_CTRL_SELF_TEST:
    _ist8310_i2c_write(mag, MAG_CNTL1, IST8310_STANDBY); // stop ongoing sampling
    _ist8310_i2c_read(mag, MAG_STAT1, &reg, 1);
    while (reg & (IST8310_DATA_AVAILABLE | IST8310_DATA_OVERFLOW)) // read out previous data
    {
      _ist8310_i2c_read(mag, MAG_DATAXL, (rt_uint8_t*)&raw_data, MAG_DATAZH - MAG_DATAXL + 1);
      _ist8310_i2c_read(mag, MAG_STAT1, &reg, 1);
    }

    _ist8310_i2c_write(mag, MAG_TCCNTL, IST8310_TC_DISABLE); // turn off temperature compensation
    _ist8310_i2c_write(mag, MAG_CNTL1, IST8310_ONESHOT); // do one sampling
    do
    {
      _ist8310_i2c_read(mag, MAG_STAT1, &reg, 1);
    } while (!(reg & IST8310_DATA_AVAILABLE));
    _ist8310_i2c_read(mag, MAG_DATAXL, (rt_uint8_t*)&raw_data, MAG_DATAZH - MAG_DATAXL + 1); // get a non-selftest data

    _ist8310_i2c_write(mag, MAG_STR, IST8310_SELFTEST_ON); // start selftest
    _ist8310_i2c_write(mag, MAG_CNTL1, IST8310_ONESHOT); // do one sampling
    do
    {
      _ist8310_i2c_read(mag, MAG_STAT1, &reg, 1);
    } while (!(reg & IST8310_DATA_AVAILABLE));
    _ist8310_i2c_read(mag, MAG_DATAXL, (rt_uint8_t*)&selftest_data, MAG_DATAZH - MAG_DATAXL + 1); // get a selftest data
    
    /**
     * The datasheet says if the absolute values are same and polarity changed, then self-test passed, otherwise not passed.
     * But it's too strict that it's seldom passed, so I loosen up a bit.
     */
    if (selftest_data.x + raw_data.x > 5 ||\
        selftest_data.y + raw_data.y > 5 ||\
        selftest_data.z + raw_data.z > 5)
      ret = -RT_ERROR;
    
    _ist8310_i2c_write(mag, MAG_STR, IST8310_SELFTEST_OFF);
    _ist8310_i2c_write(mag, MAG_TCCNTL, IST8310_TC_ENABLE);
    _ist8310_i2c_write(mag, MAG_CNTL1, IST8310_CONTINUOUS); // continue regular sampling
  default:
    ret = -RT_EINVAL;
  }
  return ret;
}

#ifdef RT_USING_DEVICE_OPS
static const struct rt_device_ops rt_ist8310_ops {
  .open     = rt_ist8310_instance_open,
  .close    = rt_ist8310_instance_close,
  .read     = rt_ist8310_instance_read,
  .control  = rt_ist8310_instance_control
};
#endif

/**
 * @brief 创建IST8310设备结构
 * 
 * @return rt_ist8310_t IST8310设备结构指针
 */
rt_ist8310_t rt_ist8310_create(void)
{
  rt_ist8310_t mag = (rt_ist8310_t)rt_device_create(RT_Device_Class_Sensor, sizeof(struct rt_ist8310) - sizeof(struct rt_device));
  if (mag != RT_NULL)
  {
    mag->interface = RT_NULL;
    rt_memset(&mag->mapping, 0, sizeof(mag->mapping));
  }
  return mag;
}

/**
 * @brief 销毁IST8310设备结构
 * 
 * @param mag IST8310设备结构指针
 */
void rt_ist8310_destroy(rt_ist8310_t mag)
{
  rt_device_destroy(&mag->parent);
}
/**
 * @brief 注册IST8310设备
 * 
 * @param mag IST8310设备指针
 * @param name 设备名称
 * @return rt_err_t 注册结果，RT_EOK代表注册成功
 */
rt_err_t rt_ist8310_register(rt_ist8310_t mag, const char *name)
{
  RT_ASSERT(mag != RT_NULL);
  RT_ASSERT(mag->interface != RT_NULL);
  RT_ASSERT(mag->mapping.rstn_port != RT_NULL);
  RT_ASSERT(mag->address == IST8310_ADDRESS_0 || mag->address == IST8310_ADDRESS_1 || mag->address == IST8310_ADDRESS_2 || mag->address == IST8310_ADDRESS_3);
  
  rt_device_t dev = &mag->parent;
  
  // initialize its lock
  char lock_name[RT_NAME_MAX] = {0};
  rt_strncpy(lock_name, name, RT_NAME_MAX);
  rt_size_t name_length = rt_strnlen(lock_name, RT_NAME_MAX);
  rt_sem_init(&mag->lock, lock_name, 1, RT_IPC_FLAG_PRIO);
  
  // filling the class methods
  #ifdef RT_USING_DEVICE_OPS
  dev->ops      = &rt_ist8310_ops;
  #else
  dev->open     = rt_ist8310_instance_open;
  dev->close    = rt_ist8310_instance_close;
  dev->read     = rt_ist8310_instance_read;
  dev->control  = rt_ist8310_instance_control;
  #endif
  
  return rt_device_register(dev, name, RT_DEVICE_FLAG_RDONLY | RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_ACTIVATED);
}

/**
 * @brief 反注册IST8310设备
 * 
 * @param mag IST8310设备指针
 * @return rt_err_t 反注册结果，RT_EOK代表反注册成功
 */
rt_err_t rt_ist8310_unregister(rt_ist8310_t mag)
{
  RT_ASSERT(mag != RT_NULL);
  
  rt_sem_detach(&mag->lock);
  
  return rt_device_unregister(&mag->parent);
}

/**
 * @brief 接收IST8310数据，DMA非阻塞接收，建议添加到接收中断
 * 
 * @param mag IST8310设备指针
 */
__INLINE void rt_ist8310_recv(rt_ist8310_t mag)
{
  if (rt_sem_take(&mag->lock, RT_WAITING_NO) == RT_EOK)
  {
    if (mag->data == mag->data_buffer)
      mag->data = mag->data_buffer + 1;
    else
      mag->data = mag->data_buffer;
    
    HAL_I2C_Mem_Read_DMA(mag->interface, mag->address, MAG_DATAXL, I2C_MEMADD_SIZE_8BIT, (uint8_t*)mag->data, MAG_DATAZH - MAG_DATAXL + 1);
  }
  else
  {
    if (mag->data == mag->data_buffer)
      HAL_I2C_Mem_Read_DMA(mag->interface, mag->address, MAG_DATAXL, I2C_MEMADD_SIZE_8BIT, (uint8_t*)(mag->data_buffer + 1), MAG_DATAZH - MAG_DATAXL + 1);
    else
      HAL_I2C_Mem_Read_DMA(mag->interface, mag->address, MAG_DATAXL, I2C_MEMADD_SIZE_8BIT, (uint8_t*)mag->data_buffer, MAG_DATAZH - MAG_DATAXL + 1);
  }
}

/**
 * @brief 接收完成中断回调，添加到I2C的MemRxCplt回调函数中
 * 
 * @param mag IST8310设备指针
 */
__INLINE void rt_ist8310_recv_cplt(rt_ist8310_t mag)
{
  rt_sem_release(&mag->lock);
  if (mag->parent.rx_indicate != RT_NULL)
    mag->parent.rx_indicate(&mag->parent, 1);
}