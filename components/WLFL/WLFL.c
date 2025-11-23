/**
 * @file WLFL.c
 * @author bignut
 * @brief 
 * @details
 * @version 0.1
 * @date 2025-03-28
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#include "WLFL.h"
#include "driver/gpio.h"
#include "esp_sleep.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "string.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"

#define PORT 1234  // 监听端口

// Connect the same WIFI with host-ubuntu.

#define EXAMPLE_ESP_MAXIMUM_RETRY  5

// Passward needless.
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_OPEN

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

// Two EventGroup Bit.
#define WIFI_CONNECTED_BIT BIT0 
#define WIFI_FAIL_BIT      BIT1

static const char *TAG = "wifi station";

static int s_retry_num = 0;


/// @brief Wlfl connection handler.
/// @param arg 
/// @param event_base 
/// @param event_id 
/// @param event_data 
static void wlfl_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) 
    {
        esp_wifi_connect();
    } 
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) 
    {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) 
        {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } 
        else 
        {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);  // EVENT GROUP BIT
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } 
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) 
    {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

/// @brief WLFL init function. Register EventGroup, Register Handler, WLFL start, and Check WLFL connection.
/// @param  
void wlfl_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();               // EventGroup

    ESP_ERROR_CHECK(esp_netif_init());                      // tcpip stack

    ESP_ERROR_CHECK(esp_event_loop_create_default());       // loop create
    
    esp_netif_create_default_wifi_sta();                    // tcpip to wifi

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,       // ANY WIFI EVENT 
                                                        &wlfl_event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,    // GOT IP EVENT
                                                        &wlfl_event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    // WLFL START.
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS,
            .threshold.authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");


    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) 
    {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } 
    else if (bits & WIFI_FAIL_BIT) 
    {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } 
    else 
    {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
}


/// @brief addr[4] represent (127.0.0.1).split('.')
/// @param addr 
void get_ip_address_uint16(uint16_t *addr)
{
    esp_netif_ip_info_t ip_info;

    esp_netif_t *netif = esp_netif_get_default_netif();

    if(esp_netif_get_ip_info(netif, &ip_info) == ESP_OK)
    {
        addr[0] = esp_ip4_addr1_16(&ip_info.ip);
        addr[1] = esp_ip4_addr2_16(&ip_info.ip);
        addr[2] = esp_ip4_addr3_16(&ip_info.ip);
        addr[3] = esp_ip4_addr4_16(&ip_info.ip);

        /*         ESP_LOGI("GOT ", "IP Address:" IPSTR, IP2STR(&ip_info.ip));
        ESP_LOGI("GOT ", "Subnet Mask:" IPSTR, IP2STR(&ip_info.netmask));
        ESP_LOGI("GOT ", "Gateway Address:" IPSTR, IP2STR(&ip_info.gw));
        
        char tmp_addr[16];

        printf("this method is: %s\n ", esp_ip4addr_ntoa(&ip_info.ip, tmp_addr, sizeof(tmp_addr))); */
    }
    else
    {
        ESP_LOGI("Error ", "Failed to get ip addr.");
    }
}

/// @brief Return str of ipaddr.
/// @param addr 
/// @param len 
void get_ip_address_str(char *addr, int len)
{
    esp_netif_ip_info_t ip_info;

    esp_netif_t *netif = esp_netif_get_default_netif();

    if(esp_netif_get_ip_info(netif, &ip_info) == ESP_OK)
    {

        esp_ip4addr_ntoa(&ip_info.ip, addr, len);
        // snprintf(ip_str, sizeof(ip_str), IPSTR, IP2STR(&ip_info.ip));
    }
    else
    {
        ESP_LOGI("Error ", "Failed to get ip addr.");
    }
}

// 接收socket连接后的处理函数
void client_socket_task(void *pvParameters) {
    int client_sock = *(int *)pvParameters;
    
    // 配置接收 buffer
    char recv_buf[64];
    while (1) {
        int len = read(client_sock, recv_buf, sizeof(recv_buf) - 1);
        if (len < 0) {
            ESP_LOGE(TAG, "Recv failed: errno %d", errno);
            break;
        } else if (len == 0) {
            ESP_LOGI(TAG, "Connection closed by client");
            break;
        } else {
            recv_buf[len] = 0;  // Null-terminate
            ESP_LOGI(TAG, "Received: %s, Len: %d", recv_buf, strlen(recv_buf));
            // 在这里可以添加处理接收到的消息的逻辑
        }
    }

    // 关闭 socket
    shutdown(client_sock, 0);
    close(client_sock);
    vTaskDelete(NULL);  // 删除任务
}

// 监听进程
void my_tcp_server_task(void *pvParameters) {
    char addr_str[128];
    int addr_family = AF_INET;
    int ip_protocol = IPPROTO_IP;
    struct sockaddr_in dest_addr;

    // 创建本地 socket
    int sock = socket(addr_family, SOCK_STREAM, ip_protocol);
    if (sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }

    // 配置 socket 监听地址
    dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(PORT);

    // socket 绑定 监听地址
    if (bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr)) < 0) {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        close(sock);
        vTaskDelete(NULL);
        return;
    }

    // 预备监听
    if (listen(sock, 1) < 0) {
        ESP_LOGE(TAG, "Error occurred during listen: errno %d", errno);
        close(sock);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Socket listening on port %d", PORT);

    // 循环等待连接
    while (1) {
        struct sockaddr_in source_addr;
        socklen_t addr_len = sizeof(source_addr);
        // 阻塞等待连接
        int client_sock = accept(sock, (struct sockaddr *)&source_addr, &addr_len);
        if (client_sock < 0) {
            ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
            break;
        }

        ESP_LOGI(TAG, "Socket accepted");
        // 获取连接ip地址
        inet_ntoa_r(source_addr.sin_addr.s_addr, addr_str, sizeof(addr_str) - 1);
        ESP_LOGI(TAG, "Client IP: %s", addr_str);

        // 配置接收 buffer
        /*char recv_buf[64];
        int len = read(client_sock, recv_buf, sizeof(recv_buf) - 1);
        if (len < 0) {
            ESP_LOGE(TAG, "Recv failed: errno %d", errno);
        } else {
            recv_buf[len] = 0;  // Null-terminate
            ESP_LOGI(TAG, "Received: %s", recv_buf);
        } */

        int *client_sock_ptr = (int *)malloc(sizeof(int));
        *client_sock_ptr = client_sock;
        if (xTaskCreate(client_socket_task, "client_socket_task", 4096, client_sock_ptr, 5, NULL) != pdPASS) {
            ESP_LOGE(TAG, "Failed to create task");
            free(client_sock_ptr);
            shutdown(client_sock, 0);
            close(client_sock);
        }
    }

    close(sock);
    vTaskDelete(NULL);
}

