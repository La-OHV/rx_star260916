//
// Created by 23906 on 2026/9/16.
//

#ifndef RX_STAR260916_STAR_REMOTE_H
#define RX_STAR260916_STAR_REMOTE_H
#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

#define REMOTE_CONTROL_FRAME_SIZE       12U
#define REMOTE_CONTROL_PROTOCOL_VERSION 0x01U
#define REMOTE_CONTROL_TIMEOUT_MS       300U

typedef struct
{
    uint8_t sequence;
    bool power_on;
    int8_t left_x;
    int8_t left_y;
    int8_t right_x;
    int8_t right_y;
    uint8_t left_gear;
    uint8_t right_gear;
    uint32_t last_update_ms;
    uint32_t valid_frame_count;
    uint32_t crc_error_count;
} RemoteControlData_t;

extern volatile RemoteControlData_t g_remote_control;

HAL_StatusTypeDef RemoteControl_Start(UART_HandleTypeDef *huart);
void RemoteControl_RxCpltCallback(UART_HandleTypeDef *huart);
void RemoteControl_UART_ErrorCallback(UART_HandleTypeDef *huart);
bool RemoteControl_GetSnapshot(RemoteControlData_t *data);
bool RemoteControl_IsOnline(uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif
#endif //RX_STAR260916_STAR_REMOTE_H
