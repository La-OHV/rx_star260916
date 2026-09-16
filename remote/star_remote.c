//
// Created by 23906 on 2026/9/16.
//

#include "star_remote.h"
#include <string.h>

#define FRAME_HEADER_1 0xAAU
#define FRAME_HEADER_2 0x55U

volatile RemoteControlData_t g_remote_control = {0};

static UART_HandleTypeDef *s_remote_uart = NULL;
static uint8_t s_rx_byte = 0U;
static uint8_t s_frame[REMOTE_CONTROL_FRAME_SIZE] = {0};
static uint8_t s_frame_index = 0U;

static uint8_t RemoteControl_Crc8(const uint8_t *data, uint8_t length);
static bool RemoteControl_DecodeFrame(const uint8_t *frame);
static void RemoteControl_ProcessByte(uint8_t byte);
static HAL_StatusTypeDef RemoteControl_ArmReceive(void);

HAL_StatusTypeDef RemoteControl_Start(UART_HandleTypeDef *huart)
{
  if (huart == NULL)
  {
    return HAL_ERROR;
  }

  s_remote_uart = huart;
  s_frame_index = 0U;
  memset((void *)&g_remote_control, 0, sizeof(g_remote_control));
  return RemoteControl_ArmReceive();
}

void RemoteControl_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if ((huart == NULL) || (huart != s_remote_uart))
  {
    return;
  }

  RemoteControl_ProcessByte(s_rx_byte);
  (void)RemoteControl_ArmReceive();
}

void RemoteControl_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if ((huart == NULL) || (huart != s_remote_uart))
  {
    return;
  }

  s_frame_index = 0U;
  __HAL_UART_CLEAR_OREFLAG(huart);
  (void)RemoteControl_ArmReceive();
}

bool RemoteControl_GetSnapshot(RemoteControlData_t *data)
{
  uint32_t primask;

  if (data == NULL)
  {
    return false;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  memcpy(data, (const void *)&g_remote_control, sizeof(*data));
  if (primask == 0U)
  {
    __enable_irq();
  }

  return (data->valid_frame_count > 0U);
}

bool RemoteControl_IsOnline(uint32_t timeout_ms)
{
  RemoteControlData_t snapshot;

  if (!RemoteControl_GetSnapshot(&snapshot))
  {
    return false;
  }

  return ((uint32_t)(HAL_GetTick() - snapshot.last_update_ms) <= timeout_ms);
}

static HAL_StatusTypeDef RemoteControl_ArmReceive(void)
{
  if (s_remote_uart == NULL)
  {
    return HAL_ERROR;
  }

  return HAL_UART_Receive_IT(s_remote_uart, &s_rx_byte, 1U);
}

static void RemoteControl_ProcessByte(uint8_t byte)
{
  switch (s_frame_index)
  {
    case 0U:
      if (byte == FRAME_HEADER_1)
      {
        s_frame[0] = byte;
        s_frame_index = 1U;
      }
      break;

    case 1U:
      if (byte == FRAME_HEADER_2)
      {
        s_frame[1] = byte;
        s_frame_index = 2U;
      }
      else if (byte == FRAME_HEADER_1)
      {
        s_frame[0] = byte;
      }
      else
      {
        s_frame_index = 0U;
      }
      break;

    default:
      s_frame[s_frame_index++] = byte;
      if (s_frame_index >= REMOTE_CONTROL_FRAME_SIZE)
      {
        (void)RemoteControl_DecodeFrame(s_frame);
        s_frame_index = 0U;
      }
      break;
  }
}

static bool RemoteControl_DecodeFrame(const uint8_t *frame)
{
  uint8_t calculated_crc;
  bool power_on;
  int8_t left_x;
  int8_t left_y;
  int8_t right_x;
  int8_t right_y;

  if ((frame[0] != FRAME_HEADER_1) ||
      (frame[1] != FRAME_HEADER_2) ||
      (frame[2] != REMOTE_CONTROL_PROTOCOL_VERSION))
  {
    return false;
  }

  calculated_crc = RemoteControl_Crc8(&frame[2], 9U);
  if (calculated_crc != frame[11])
  {
    g_remote_control.crc_error_count++;
    return false;
  }

  if (frame[4] > 1U)
  {
    return false;
  }

  power_on = (frame[4] == 1U);
  left_x = (int8_t)frame[5];
  left_y = (int8_t)frame[6];
  right_x = (int8_t)frame[7];
  right_y = (int8_t)frame[8];

  if ((left_x < -100) || (left_x > 100) ||
      (left_y < -100) || (left_y > 100) ||
      (right_x < -100) || (right_x > 100) ||
      (right_y < -100) || (right_y > 100))
  {
    return false;
  }

  if (power_on)
  {
    if ((frame[9] < 1U) || (frame[9] > 2U) ||
        (frame[10] < 1U) || (frame[10] > 2U))
    {
      return false;
    }
  }
  else if ((left_x != 0) || (left_y != 0) ||
           (right_x != 0) || (right_y != 0) ||
           (frame[9] != 0U) || (frame[10] != 0U))
  {
    return false;
  }

  g_remote_control.sequence = frame[3];
  g_remote_control.power_on = power_on;
  g_remote_control.left_x = left_x;
  g_remote_control.left_y = left_y;
  g_remote_control.right_x = right_x;
  g_remote_control.right_y = right_y;
  g_remote_control.left_gear = frame[9];
  g_remote_control.right_gear = frame[10];
  g_remote_control.last_update_ms = HAL_GetTick();
  g_remote_control.valid_frame_count++;

  return true;
}

static uint8_t RemoteControl_Crc8(const uint8_t *data, uint8_t length)
{
  uint8_t crc = 0U;
  uint8_t index;
  uint8_t bit;

  for (index = 0U; index < length; index++)
  {
    crc ^= data[index];
    for (bit = 0U; bit < 8U; bit++)
    {
      if ((crc & 0x80U) != 0U)
      {
        crc = (uint8_t)((crc << 1U) ^ 0x07U);
      }
      else
      {
        crc <<= 1U;
      }
    }
  }

  return crc;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART6)   // 改成你的遥控串口
  {
    RemoteControl_RxCpltCallback(huart);
  }
  // 其他串口在这里分派
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART6)
  {
    RemoteControl_UART_ErrorCallback(huart);
  }
}