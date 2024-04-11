#ifndef __RT_C620_H__
#define __RT_C620_H__

#include <rtthread.h>
#include <stm32f4xx_hal.h>
#include <stdint.h>

#ifdef RT_C620_DEF
#define RT_C620_EXT
#else
#define RT_C620_EXT extern
#endif

#define C620_HEADER_CTRL_GROUP_1 0x200
#define C620_HEADER_CTRL_GROUP_2 0x1FF

#define C620_HEADER_RET_BASE 0x200

#define C620_ID_1_5 0x01
#define C620_ID_2_6 0x02
#define C620_ID_3_7 0x04
#define C620_ID_4_8 0x08

/**
 * @brief C620电调控制块
 * 
 */
struct rt_c620_ctrl
{

  /** 转子角度反馈值，0~8191代表0~360° */
  union
  {
    uint16_t u16;
    uint8_t u8[2];
  }angle;

  /** 转子转速反馈值，单位rpm */
  union
  {
    int16_t i16;
    uint8_t u8[2];
  }speed;

  /** 转矩电流反馈值，-16384~16384代表-20A~20A */
  union
  {
    int16_t i16;
    uint8_t u8[2];
  }torque;

  /** 电压设置值，此处没有用到 */
  union
  {
    int16_t i16;
    uint8_t u8[2];
  }voltage;

  /** 电流设置值，-16384~16384代表-20A~20A */
  union
  {
    int16_t i16;
    uint8_t u8[2];
  }current;

  /** 电机温度，-128~127，单位℃ */
  union
  {
    uint16_t dummy;
    int8_t i8;
  }temp;

  /** 电机数据更新时间，可用于判断是否掉线 */
  rt_tick_t rx_tick;

};

/**
 * @brief C620电调设备结构，每个结构控制对应控制ID的4个电机，接入到RT-Thread设备驱动框架
 * 
 */
typedef struct rt_c620
{

  /** RT-Thread设备驱动框架必需的结构 */
  struct rt_device parent;

  /** CAN通信端口 */
  CAN_HandleTypeDef *interface;

  /** 电机控制块，分别对应该ID下的4个电机 */
  struct rt_c620_ctrl ctrl[4];

  /** CAN控制数据的帧ID */
  uint16_t header;

  /** 用于4字节对齐的无效半字 */
  uint16_t dummy;

  /** 接收缓冲区，与控制块配合形成双缓冲 */
  uint8_t rx_buff[8 + 2]; // 2 extra bytes to save frame id

  /** 用于4字节对齐的无效半字 */
  uint16_t dummy2; // for alignment

  /** 发送缓冲区，与控制块配合形成双缓冲 */
  uint8_t tx_buff[8];

  /** 
   * 接收缓冲区指针，用于指示接收缓冲区数据是否有效
   * 其值为RT_NULL时缓冲区数据无效，可以安全覆盖
   * 其值为rx_buff时缓冲区数据有效，需要判断缓冲区是否繁忙才能覆盖
   * 以后如果需要也可将其指向其他地址，但不保证是否可用
   */
  uint8_t *rx_buff_ptr;

  /** 
   * 发送缓冲区指针，用于指示发送缓冲区数据是否有效
   * 其值为RT_NULL时缓冲区数据无效，可以安全覆盖
   * 其值为tx_buff时缓冲区数据有效，需要判断缓冲区是否繁忙才能覆盖
   * 以后如果需要也可将其指向其他地址，但不保证是否可用
   */
  uint8_t *tx_buff_ptr;
  
  /** 设备锁，防止多线程写入冲突 */
  struct rt_semaphore lock;

  /** 
   * 刷新定时器，用于在长时间没写入新控制值时维持之前的状态
   * 如果不定时刷新，那么电机将在停止发送控制值的一段时间后停转
   */
  struct rt_timer refresh_timer;

  /** 电机控制数据发送时间，用于判断是否有刷新的必要 */
  rt_tick_t tx_tick;
  
} *rt_c620_t;

rt_c620_t rt_c620_create(void);
void rt_c620_destroy(rt_c620_t ec);
rt_err_t rt_c620_register(rt_c620_t ec, const char *name);
rt_err_t rt_c620_unregister(rt_c620_t ec);

// Add this function into CAN Rx ISR
void rt_c620_recv(rt_c620_t ec, CAN_HandleTypeDef *hcan, CAN_RxHeaderTypeDef *header, uint8_t *data);

// Add this function into CAN Tx ISR
void rt_c620_send(rt_c620_t ec, CAN_HandleTypeDef *hcan);

#endif