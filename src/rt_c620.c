/**
 * @file rt_c620.c
 * @author Jacky Wu (grade6_wty@163.com)
 * @brief C620电调驱动程序（适用于M3508动力电机系统）
 * @version
 * @date 2024-04-05
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#define RT_C620_DEF
#include <rt_c620.h>
#include <rtthread.h>
#include <rthw.h>
#include <stdlib.h>

/**
 * @brief 从接收缓冲区更新电调状态，内部函数，不导出
 * 
 * @param ec 电调设备对象
 */
__STATIC_FORCEINLINE void rt_c620_recv_parser(rt_c620_t ec)
{
  if (ec->rx_buff_ptr == RT_NULL)
    return; // nothing to recv, skip
  
  uint8_t id = (*(uint16_t*)(ec->rx_buff_ptr + 8) - C620_HEADER_RET_BASE);
  
  // totally invalid id
  if (id < 1 || id > 8)
    return;
  
  // id mismatches with device configuration
  if (id > 4 && ec->header != C620_HEADER_CTRL_GROUP_2 || id <= 4 && ec->header != C620_HEADER_CTRL_GROUP_1)
    return;
  
  // convert id to ctrl block index
  id--;
  if (id >= 4)
    id -= 4;
  
  // byte order transfer
  ec->ctrl[id].angle.u8[1]        = ec->rx_buff_ptr[0];
  ec->ctrl[id].angle.u8[0]        = ec->rx_buff_ptr[1];
  ec->ctrl[id].speed.u8[1]        = ec->rx_buff_ptr[2];
  ec->ctrl[id].speed.u8[0]        = ec->rx_buff_ptr[3];
  ec->ctrl[id].torque.u8[1]       = ec->rx_buff_ptr[4];
  ec->ctrl[id].torque.u8[0]       = ec->rx_buff_ptr[5];
  ec->ctrl[id].temp.i8            = ec->rx_buff_ptr[6];
  ec->ctrl[id].rx_tick            = rt_tick_get();

  ec->rx_buff_ptr = RT_NULL;
}

/**
 * @brief 将电调控制值打包为CAN帧，内部函数，不导出
 * 
 * @param ec 电调设备对象
 */
__STATIC_FORCEINLINE void rt_c620_send_packer(rt_c620_t ec)
{
  ec->tx_buff_ptr = ec->tx_buff;
  ec->tx_buff_ptr[0] = ec->ctrl[0].current.u8[1];
  ec->tx_buff_ptr[1] = ec->ctrl[0].current.u8[0];
  ec->tx_buff_ptr[2] = ec->ctrl[1].current.u8[1];
  ec->tx_buff_ptr[3] = ec->ctrl[1].current.u8[0];
  ec->tx_buff_ptr[4] = ec->ctrl[2].current.u8[1];
  ec->tx_buff_ptr[5] = ec->ctrl[2].current.u8[0];
  ec->tx_buff_ptr[6] = ec->ctrl[3].current.u8[1];
  ec->tx_buff_ptr[7] = ec->ctrl[3].current.u8[0];
}

/**
 * @brief 刷新电调状态，因为当电调离线过久（应该是1秒）会强制停车
 * 
 * @param ec 电调设备对象
 */
static void rt_c620_refresh(rt_c620_t ec)
{
  // avoid unneeded refresh
  if (rt_tick_get() - ec->tx_tick <= 250 / 2)
    return;
  
  // take its mutex lock
  rt_sem_take(&ec->lock, RT_WAITING_FOREVER);

  // make a ready-to-send buffer
  rt_c620_send_packer(ec);
  
  // create the message
  CAN_TxHeaderTypeDef header;
  header.IDE                = CAN_ID_STD;
  header.RTR                = CAN_RTR_DATA;
  header.StdId              = ec->header;
  header.ExtId              = 0;
  header.DLC                = 8;
  header.TransmitGlobalTime = DISABLE;
  
  uint32_t mailbox;
  // try to send it
  if (HAL_CAN_AddTxMessage(ec->interface, &header, ec->tx_buff_ptr, &mailbox) == HAL_OK)
  {
    ec->tx_buff_ptr = RT_NULL; // if send successfully, then clear the flag
    ec->tx_tick = rt_tick_get(); // save update time
  }
  
  rt_c620_recv_parser(ec);
  
  rt_sem_release(&ec->lock);
}

/**
 * @brief 打开电调设备，启动自动刷新定时器
 * 
 * @param dev 设备对象
 * @param oflag 打开模式
 * @return rt_err_t 操作结果，目前没做异常返回
 */
static rt_err_t rt_c620_instance_open(rt_device_t dev, rt_uint16_t oflag)
{
  RT_ASSERT(dev != RT_NULL);
  rt_c620_t ec = (rt_c620_t)dev;
  
  rt_sem_take(&ec->lock, RT_WAITING_FOREVER);
  
  dev->open_flag = (oflag & RT_DEVICE_OFLAG_MASK);
  rt_timer_start(&ec->refresh_timer);
  
  rt_sem_release(&ec->lock);
  return RT_EOK;
}

/**
 * @brief 关闭电调设备，停止自动刷新定时器
 * 
 * @param dev 设备对象
 * @return rt_err_t 操作结果，目前没做异常返回
 */
static rt_err_t rt_c620_instance_close(rt_device_t dev)
{
  RT_ASSERT(dev != RT_NULL);
  rt_c620_t ec = (rt_c620_t)dev;
  
  rt_sem_take(&ec->lock, RT_WAITING_FOREVER);
  
  dev->open_flag = RT_DEVICE_OFLAG_CLOSE;
  rt_timer_stop(&ec->refresh_timer);
  
  rt_sem_release(&ec->lock);
  return RT_EOK;
}

/**
 * @brief 读取电调状态到buffer中，格式为struct rt_c620_ctrl
 * 
 * @param dev 设备对象
 * @param pos 列表起始位置，0代表id1（或5）
 * @param buffer 接收缓冲区
 * @param size 电调个数
 * @return rt_size_t 成功读取信息的电调个数
 */
static rt_size_t rt_c620_instance_read(rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t size)
{
  RT_ASSERT(dev != RT_NULL);
  RT_ASSERT(buffer != RT_NULL);

  rt_c620_t ec = (rt_c620_t)dev;
  
  // checking if the handler has read permission
  if (!(dev->open_flag & RT_DEVICE_OFLAG_RDONLY))
    return 0;

  // calculate copy range
  uint8_t start, end;
  if (pos >= 4)
    return 0; // there are nothing after block 3
  else if (pos < 0)
    start = 0; // only block #0~#3 are valid
  else
    start = pos;
  
  if (size + start > 4)
    end = 3; // only blocks #0~#3 are valid
  else if (size == 0)
    return 0;
  else
    end = start + size - 1;
    

  // take its mutex lock
  rt_sem_take(&ec->lock, RT_WAITING_FOREVER);

  rt_memcpy(buffer, &ec->ctrl[start], sizeof(end - start + 1) * sizeof(struct rt_c620_ctrl));

  rt_ubase_t dummy; // to test the mailbox
  rt_c620_recv_parser(ec);
  
  rt_sem_release(&ec->lock);

  return end - start + 1;
}

/**
 * @brief 向电调发送控制值
 * 
 * @param dev 设备对象
 * @param pos 列表起始位置，0代表id1（或5）
 * @param buffer 控制值指针
 * @param size 电调个数
 * @return rt_size_t 成功设置的电调个数
 */
static rt_size_t rt_c620_instance_write(rt_device_t dev, rt_off_t pos, const void *buffer, rt_size_t size)
{
  RT_ASSERT(dev != RT_NULL);
  RT_ASSERT(buffer != RT_NULL);

  rt_c620_t ec = (rt_c620_t)dev;
  
  // checking if the handler has write permission
  if (!(dev->open_flag & RT_DEVICE_OFLAG_WRONLY))
    return 0;

  // calculate copy range
  uint8_t start, end;
  if (pos >= 4)
    return 0; // there are nothing after block 3
  else if (pos < 0)
    start = 0; // only block #0~#3 are valid
  else
    start = pos;
  
  if (size + start > 4)
    end = 3; // only blocks #0~#3 are valid
  else if (size == 0)
    return 0;
  else
    end = start + size - 1;
  
  // take its mutex lock
  rt_sem_take(&ec->lock, RT_WAITING_FOREVER);

  const int16_t *i16_buffer = (const int16_t*)buffer;
  for (int i = start; i <= end; i++)
    ec->ctrl[i].current.i16 = i16_buffer[i - start];

  // make a ready-to-send buffer
  rt_c620_send_packer(ec);
  
  // create the message
  CAN_TxHeaderTypeDef header;
  header.IDE                = CAN_ID_STD;
  header.RTR                = CAN_RTR_DATA;
  header.StdId              = ec->header;
  header.ExtId              = 0;
  header.DLC                = 8;
  header.TransmitGlobalTime = DISABLE;
  
  uint32_t mailbox;
  // try to send it
  if (HAL_CAN_AddTxMessage(ec->interface, &header, ec->tx_buff_ptr, &mailbox) == HAL_OK)
  {
    ec->tx_buff_ptr = RT_NULL; // if send successfully, then clear the flag
    ec->tx_tick = rt_tick_get(); // save update time
  }

  rt_c620_recv_parser(ec);
  
  rt_sem_release(&ec->lock);

  return end - start + 1;
}

#ifdef RT_USING_DEVICE_OPS
static const struct rt_device_ops rt_c620_ops = {
  .open   = rt_c620_instance_open,
  .close  = rt_c620_instance_close,
  .read   = rt_c620_instance_read,
  .write  = rt_c620_instance_write
};
#endif

/**
 * @brief 创建C620电调设备对象
 * 
 * @return rt_c620_t 设备对象指针
 */
rt_c620_t rt_c620_create(void)
{
  rt_c620_t ec = (rt_c620_t)rt_device_create(RT_Device_Class_Miscellaneous, sizeof(struct rt_c620) - sizeof(struct rt_device));
  if (ec != RT_NULL)
    ec->interface = RT_NULL; // only clear pointers in order to keep parent structure
  ec->ctrl[0].current.i16 = 0;
  ec->ctrl[1].current.i16 = 0;
  ec->ctrl[2].current.i16 = 0;
  ec->ctrl[3].current.i16 = 0;
  return ec;
}

/**
 * @brief 销毁C620电调设备对象（本函数不进行反注册）
 * 
 * @param ec 设备对象指针
 */
void rt_c620_destroy(rt_c620_t ec)
{
  RT_ASSERT(ec != RT_NULL);
  
  rt_device_t dev = (rt_device_t)ec;
  rt_device_destroy(dev);
}

/**
 * @brief 注册C620电调设备对象（4个一组注册）
 * 
 * @param ec 设备指针
 * @param name 设备名
 * @return rt_err_t 注册结果，RT_EOK为注册成功
 */
rt_err_t rt_c620_register(rt_c620_t ec, const char *name)
{
  RT_ASSERT(ec != RT_NULL);
  RT_ASSERT(ec->interface != RT_NULL);
  RT_ASSERT(ec->header == C620_HEADER_CTRL_GROUP_1 || ec->header == C620_HEADER_CTRL_GROUP_2);
  
  rt_device_t dev = &ec->parent;
  
  // initialize its lock
  static char lock_name[RT_NAME_MAX], timer_name[RT_NAME_MAX];
  rt_strncpy(lock_name, name, RT_NAME_MAX);
  rt_strncpy(timer_name, name, RT_NAME_MAX);
  rt_size_t name_length = rt_strnlen(lock_name, RT_NAME_MAX);
  if (name_length < RT_NAME_MAX)
    timer_name[name_length] = '_';
  if (name_length < RT_NAME_MAX - 1)
    timer_name[name_length + 1] = 'T';
  
  
  // filling the class methods
  #ifdef RT_USING_DEVICE_OPS
  dev->ops      = &rt_c620_ops;
  #else
  dev->open     = rt_c620_instance_open;
  dev->close    = rt_c620_instance_close;
  dev->read     = rt_c620_instance_read;
  dev->write    = rt_c620_instance_write;
  #endif
  
  rt_err_t ret = rt_device_register(dev, name, RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_INT_TX | RT_DEVICE_FLAG_ACTIVATED);
  if (ret != RT_EOK)
    return ret;
  
  rt_sem_init(&ec->lock, lock_name, RT_IPC_FLAG_PRIO, 1);
  rt_timer_init(&ec->refresh_timer, timer_name, (void(*)(void*))rt_c620_refresh, ec, 250, RT_TIMER_FLAG_PERIODIC);
  return RT_EOK;
}

/**
 * @brief 反注册C620电调设备对象
 * 
 * @param ec 设备对象指针
 * @return rt_err_t 解除注册结果，RT_EOK为成功解除
 */
rt_err_t rt_c620_unregister(rt_c620_t ec)
{
  RT_ASSERT(ec != RT_NULL);
  
  // deinit these os objects
  rt_sem_detach(&ec->lock);
  
  return rt_device_unregister((rt_device_t)ec);
}

/**
 * @brief 从CAN更新数据到C620电调设备对象，建议添加到CAN接收中断中运行
 * 
 * @param ec 设备对象指针
 * @param hcan CAN接口对象
 * @param header CAN帧头
 * @param data CAN数据
 */
__INLINE void rt_c620_recv(rt_c620_t ec, CAN_HandleTypeDef *hcan, CAN_RxHeaderTypeDef *header, uint8_t *data)
{
  RT_ASSERT(ec != RT_NULL);
  RT_ASSERT(hcan !- RT_NULL);
  RT_ASSERT(header != RT_NULL);
  RT_ASSERT(data != RT_NULL);

  rt_device_t dev = &ec->parent;
  
  // if interface and/or frame type do(es) not match, ignore it
  if (hcan->Instance != ec->interface->Instance ||\
      header->IDE != CAN_ID_STD ||\
      header->RTR != CAN_RTR_DATA ||\
      (header->StdId & 0xFFF8) != C620_HEADER_RET_BASE ||\
      header->DLC != 8)
    return;
  
  if (ec->rx_buff_ptr == RT_NULL) // if there is no pending data, then fill the buffer now
  {
    rt_memcpy(ec->rx_buff, data, header->DLC);
    *(uint16_t*)(ec->rx_buff + 8) = header->StdId;
    ec->rx_buff_ptr = ec->rx_buff;

    if (rt_sem_take(&ec->lock, RT_WAITING_NO) == RT_EOK) // if the instance is vacant, then update it now
    {
      rt_c620_recv_parser(ec);
      rt_sem_release(&ec->lock);
      if (dev->rx_indicate != RT_NULL)
        dev->rx_indicate(dev, 4);
    }
  }
  else
  {
    if (rt_sem_take(&ec->lock, RT_WAITING_NO) == RT_EOK) // if there is data pending, then fill it when convenient
    {
      rt_memcpy(ec->rx_buff, data, header->DLC);
      *(uint16_t*)(ec->rx_buff + 8) = header->StdId;
      ec->rx_buff_ptr = ec->rx_buff;
      rt_c620_recv_parser(ec);
      rt_sem_release(&ec->lock);
      if (dev->rx_indicate != RT_NULL)
        dev->rx_indicate(dev, 4);
    }
    // otherwise ignore it
  }
}

/**
 * @brief 向CAN发送缓冲区中的数据，用于CAN繁忙，未能及时发送的情况，将此函数添加到CAN发送中断中运行可实现成功发包后将数据添加到邮箱
 * 
 * @param ec 设备对象指针
 * @param hcan CAN接口对象
 */
__INLINE void rt_c620_send(rt_c620_t ec, CAN_HandleTypeDef *hcan)
{
  RT_ASSERT(ec != RT_NULL);
  RT_ASSERT(hcan != RT_NULL);

  rt_device_t dev = &ec->parent;
  
  // checking if the Tx buffer is refreshing
  if (rt_sem_take(&ec->lock, RT_WAITING_NO) != RT_EOK)
    return;
  
  // checking if there is anything to transmit
  if (ec->tx_buff_ptr == RT_NULL)
  {
    rt_sem_release(&ec->lock);
    return;
  }
  
  // create the message
  CAN_TxHeaderTypeDef header;
  header.IDE                = CAN_ID_STD;
  header.RTR                = CAN_RTR_DATA;
  header.StdId              = ec->header;
  header.ExtId              = 0;
  header.DLC                = 8;
  header.TransmitGlobalTime = DISABLE;
  
  // try to send it
  uint32_t mailbox;
  if (HAL_CAN_AddTxMessage(ec->interface, &header, ec->tx_buff_ptr, &mailbox) == HAL_OK)
  {
    ec->tx_buff_ptr = RT_NULL; // if send successfully, then clear the flag
    ec->tx_tick = rt_tick_get(); // save update time
  }

  // release the device object
  rt_sem_release(&ec->lock);

  if (dev->tx_complete != RT_NULL) //Tx callback
    dev->tx_complete(dev, RT_NULL);
}

#include <stdio.h>
static char output_buffer[128];
/**
 * @brief C620电调状态显示命令（接入到RT-Thread shell）
 * 
 * @param argc 命令参数个数
 * @param argv 命令参数字符串
 */
static void c620_show_stat(int argc, char *argv[])
{
  rt_device_t dev = RT_NULL;
  rt_c620_t ec = RT_NULL;
  extern UART_HandleTypeDef huart6;
  
  if (argc != 2 || !rt_strncmp(argv[1], "--help", 6))
  {
    while (HAL_UART_GetState(&huart6) != HAL_UART_STATE_READY && HAL_UART_GetState(&huart6) != HAL_UART_STATE_BUSY_RX)
      rt_thread_mdelay(1);
    snprintf(output_buffer, sizeof(output_buffer), "Usage: %s <DEVICE_NAME>\n\r", argv[0]);
    HAL_UART_Transmit_DMA(&huart6, (const uint8_t*)output_buffer, rt_strnlen(output_buffer, sizeof(output_buffer)));
    return;
  }
  
  dev = rt_device_find(argv[1]);
  
  if (dev == RT_NULL)
  {
    while (HAL_UART_GetState(&huart6) != HAL_UART_STATE_READY && HAL_UART_GetState(&huart6) != HAL_UART_STATE_BUSY_RX)
      rt_thread_mdelay(1);
    snprintf(output_buffer, sizeof(output_buffer), "%s device not found\n\r", argv[1]);
    HAL_UART_Transmit_DMA(&huart6, (const uint8_t*)output_buffer, rt_strnlen(output_buffer, sizeof(output_buffer)));
    return;
  }
  
  ec = (rt_c620_t)dev;
  
  while (HAL_UART_GetState(&huart6) != HAL_UART_STATE_READY && HAL_UART_GetState(&huart6) != HAL_UART_STATE_BUSY_RX)
    rt_thread_mdelay(1);
  rt_strncpy(output_buffer, "id\t|angle/deg\t|speed/rpm\t|current/A\t|temp/C\t|ping/tick\n\r", sizeof(output_buffer));
  HAL_UART_Transmit_DMA(&huart6, (const uint8_t*)output_buffer, rt_strnlen(output_buffer, sizeof(output_buffer)));
  for (int i = 0; i < 4; i++)
  {
    int id;
    if (ec->header == C620_HEADER_CTRL_GROUP_1)
      id = i + 1;
    else
      id = i + 5;
    
    while (HAL_UART_GetState(&huart6) != HAL_UART_STATE_READY && HAL_UART_GetState(&huart6) != HAL_UART_STATE_BUSY_RX)
      rt_thread_mdelay(1);
    
    snprintf(output_buffer, sizeof(output_buffer), "%d\t|%f\t|%d\t\t|%f\t|%d\t|%u\n\r",\
    id,\
    ec->ctrl[i].angle.u16 * 360.0f / 8192.0f,\
    (int)ec->ctrl[i].speed.i16,\
    ec->ctrl[i].torque.i16 * 20.0f / 16384.0f,\
    (int)ec->ctrl[i].temp.i8,\
    rt_tick_get() - ec->ctrl[i].rx_tick);
    
    HAL_UART_Transmit_DMA(&huart6, (const uint8_t*)output_buffer, rt_strnlen(output_buffer, sizeof(output_buffer))); // 请勿使用rt_kprintf，否则会出现Hard Fault，且rt_kprintf功能不完整，不支持浮点
    //rt_kprintf("%d\t|%f\t\t|%f\t\t|%f\t\t|%d\n", id, ctrl.angle.u16 * 360.0f / 8192.0f, ctrl.speed.i16 * 20.0f / 16384.0f, ctrl.torque.i16, (int)ctrl.temp.i8);
  }
}
MSH_CMD_EXPORT(c620_show_stat, display c620 ec status by giving device name.);

/**
 * @brief 设置C620电调输出电流（接入到RT-Thread shell）
 * 
 * @param argc 命令参数个数
 * @param argv 命令参数字符串
 */
static void c620_set_amp(int argc, char *argv[])
{
  rt_device_t dev = RT_NULL;
  rt_c620_t ec = RT_NULL;
  extern UART_HandleTypeDef huart6;
  
  if (argc != 4 || !rt_strncmp(argv[1], "--help", 6))
  {
    while (HAL_UART_GetState(&huart6) != HAL_UART_STATE_READY && HAL_UART_GetState(&huart6) != HAL_UART_STATE_BUSY_RX)
      rt_thread_mdelay(1);
    snprintf(output_buffer, sizeof(output_buffer), "Usage: %s <DEVICE_NAME> <ec_id> <current>\n\r", argv[0]);
    HAL_UART_Transmit_DMA(&huart6, (const uint8_t*)output_buffer, rt_strnlen(output_buffer, sizeof(output_buffer)));
    return;
  }
  
  dev = rt_device_find(argv[1]);
  
  if (dev == RT_NULL)
  {
    while (HAL_UART_GetState(&huart6) != HAL_UART_STATE_READY && HAL_UART_GetState(&huart6) != HAL_UART_STATE_BUSY_RX)
      rt_thread_mdelay(1);
    snprintf(output_buffer, sizeof(output_buffer), "%s device not found\n\r", argv[1]);
    HAL_UART_Transmit_DMA(&huart6, (const uint8_t*)output_buffer, rt_strnlen(output_buffer, sizeof(output_buffer)));
    return;
  }

  ec = (rt_c620_t)dev;
  
  uint8_t id = (atoi(argv[2]) - 1) % 4;
  float current = atof(argv[3]);
  int16_t i16_current = current * 16384.0f / 20.0f;
  rt_device_write(dev, id, &i16_current, 1);
}
MSH_CMD_EXPORT(c620_set_amp, set c620 ec output current by giving device name and id.);