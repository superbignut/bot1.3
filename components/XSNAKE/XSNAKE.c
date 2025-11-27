/**
 * @file MONKEY.c
 * @author bignut
 * @brief
 * @version 0.1
 * @date 2025-08-5
 *
 * @copyright Copyright (c) 2025
 *
 */
#include "XSNAKE.h"
#include "stdio.h"
#include "esp_timer.h"
#include "PCA9685.h"
#include "math.h"
#include "lwip/sys.h"
#include "esp_log.h"
#include "lwip/sockets.h"

struct SNAKE global_snake;
static const char *TAG = "snake";
static const char* CONST_CMD[] = {"MOVE_FORWARD", "MOVE_BACK", "MOVE_LEFT", "MOVE_RIGHT", "CIRCLE", "NULL"};

static inline int convertChat2Num(char c) {
    if (c >= '0' && c<= '9') {
        int tmp_cmd = c - '0';
        return tmp_cmd;
    } else {
        return -1;
    }
}


// 接收socket连接后的处理函数
void snake_socket_CB(void *pvParameters) {
    // 客户端 socket
    int client_sock = *(int *)pvParameters;
    // 配置接收 buffer
    char recv_buf[64];
    while (1) {
        // 读取socket
        int len = read(client_sock, recv_buf, sizeof(recv_buf) - 1);
        if (len < 0) {
            ESP_LOGE(TAG, "Recv failed: errno %d", errno);
            break;
        } else if (len == 0) {
            ESP_LOGI(TAG, "Connection closed by client");
            break;
        } else {
            recv_buf[len] = 0;  // Null-terminate
            ESP_LOGI(TAG, "Received: %s, Len: %d", recv_buf, len);
            if(len != 1) {
                ESP_LOGI(TAG, "Received invalid cmd, Len is: %d", len);
                continue;
            }
            int tmp_cmd = convertChat2Num(recv_buf[0]);
            if (tmp_cmd == -1) {
                ESP_LOGI(TAG, "Received invalid cmd, char is: %c", recv_buf[0]);
                continue;
            } else {
                // 防止超出范围
                tmp_cmd = tmp_cmd % (sizeof(CONST_CMD) / sizeof(CONST_CMD[0]));
                ESP_LOGI(TAG, "Current CMD is: %s", CONST_CMD[tmp_cmd]);
                global_snake.socketCmd = tmp_cmd;
            }
        }
    }
    // 关闭 socket
    shutdown(client_sock, 0);
    close(client_sock);
    // 删除任务
    vTaskDelete(NULL);  
}