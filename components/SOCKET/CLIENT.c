/**
 * @file CLIENT.c
 * @author bignut
 * @brief 
 * @details
 * @version 0.1
 * @date 2025-03-28
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#include "SOCKET.h"
#include "driver/gpio.h"
#include "esp_sleep.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "string.h"
#include "esp_system.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "netdb.h"
#include "errno.h"
#include "WLFL.h"

// #define CONFIG_EXAMPLE_TCP_CLIENT_CONNECT_ADDRESS "192.168.198.17"
#define CONFIG_EXAMPLE_TCP_CLIENT_CONNECT_PORT "5557"

/**
 * @brief Indicates that the file descriptor represents an invalid (uninitialized or closed) socket
 *
 * Used in the TCP server structure `sock[]` which holds list of active clients we serve.
 */
#define INVALID_SOCK (-1)

/**
 * @brief Time in ms to yield to all tasks when a non-blocking socket would block
 *
 * Non-blocking socket operations are typically executed in a separate task validating
 * the socket status. Whenever the socket returns `EAGAIN` (idle status, i.e. would block)
 * we have to yield to all tasks to prevent lower priority tasks from starving.
 */
#define YIELD_TO_ALL_MS 50

/**
 * @brief Utility to log socket errors
 *
 * @param[in] tag Logging tag
 * @param[in] sock Socket number
 * @param[in] err Socket errno
 * @param[in] message Message to print
 */
static void log_socket_error(const char *tag, const int sock, const int err, const char *message)
{
    ESP_LOGE(tag, "[sock=%d]: %s\n"
                  "error=%d: %s", sock, message, err, strerror(err));
}

/**
 * @brief Tries to receive data from specified sockets in a non-blocking way,
 *        i.e. returns immediately if no data.
 *
 * @param[in] tag Logging tag
 * @param[in] sock Socket for reception
 * @param[out] data Data pointer to write the received data
 * @param[in] max_len Maximum size of the allocated space for receiving data
 * @return
 *          >0 : Size of received data
 *          =0 : No data available
 *          -1 : Error occurred during socket read operation
 *          -2 : Socket is not connected, to distinguish between an actual socket error and active disconnection
 */
static int try_receive(const char *tag, const int sock, char * data, size_t max_len)
{
    int len = recv(sock, data, max_len, 0);
    if (len < 0) 
    {
        if (errno == EINPROGRESS || errno == EAGAIN || errno == EWOULDBLOCK) 
        {
            return 0;   // Not an error
        }
        if (errno == ENOTCONN) 
        {
            ESP_LOGW(tag, "[sock=%d]: Connection closed", sock);
            return -2;  // Socket has been disconnected
        }
        log_socket_error(tag, sock, errno, "Error occurred during receiving");
        return -1;
    }

    return len;
}

/**
 * @brief Sends the specified data to the socket. This function blocks until all bytes got sent.
 *
 * @param[in] tag Logging tag
 * @param[in] sock Socket to write data
 * @param[in] data Data to be written
 * @param[in] len Length of the data
 * @return
 *          >0 : Size the written data
 *          -1 : Error occurred during socket write operation
 */
static int socket_send(const char *tag, const int sock, const char * data, const size_t len)
{
    int to_write = len;
    while (to_write > 0) 
    {
        int written = send(sock, data + (len - to_write), to_write, 0);
        if (written < 0 && errno != EINPROGRESS && errno != EAGAIN && errno != EWOULDBLOCK) 
        {
            log_socket_error(tag, sock, errno, "Error occurred during sending");
            return -1;
        }
        to_write -= written;
    }
    return len;
}



static void tcp_client_task(void *pvParameters)
{
    static const char *TAG = "nonblocking-socket-client";
    static const char *payload = "GET / HTTP/1.1\r\n\r\n";
    static char rx_buffer[128];

    struct addrinfo hints = { .ai_socktype = SOCK_STREAM };
    struct addrinfo *address_info;
    int sock = INVALID_SOCK;

    char ip_str[16];

    get_ip_address_str(ip_str, 16);     // Get ip address.

    int res = getaddrinfo(ip_str, CONFIG_EXAMPLE_TCP_CLIENT_CONNECT_PORT, &hints, &address_info);
    if (res != 0 || address_info == NULL) {
        ESP_LOGE(TAG, "couldn't get hostname for `%s` "
                      "getaddrinfo() returns %d, addrinfo=%p", ip_str, res, address_info);
        goto error;
    }

    // Creating client's socket
    sock = socket(address_info->ai_family, address_info->ai_socktype, address_info->ai_protocol);
    if (sock < 0) {
        log_socket_error(TAG, sock, errno, "Unable to create socket");
        goto error;
    }
    ESP_LOGI(TAG, "Socket created, connecting to %s:%s", ip_str, CONFIG_EXAMPLE_TCP_CLIENT_CONNECT_PORT);

    // Marking the socket as non-blocking
    int flags = fcntl(sock, F_GETFL);
    if (fcntl(sock, F_SETFL, flags | O_NONBLOCK) == -1) {
        log_socket_error(TAG, sock, errno, "Unable to set socket non blocking");
    }

    if (connect(sock, address_info->ai_addr, address_info->ai_addrlen) != 0) {
        if (errno == EINPROGRESS) {
            ESP_LOGD(TAG, "connection in progress");
            fd_set fdset;
            FD_ZERO(&fdset);
            FD_SET(sock, &fdset);

            // Connection in progress -> have to wait until the connecting socket is marked as writable, i.e. connection completes
            res = select(sock+1, NULL, &fdset, NULL, NULL);
            if (res < 0) {
                log_socket_error(TAG, sock, errno, "Error during connection: select for socket to be writable");
                goto error;
            } else if (res == 0) {
                log_socket_error(TAG, sock, errno, "Connection timeout: select for socket to be writable");
                goto error;
            } else {
                int sockerr;
                socklen_t len = (socklen_t)sizeof(int);

                if (getsockopt(sock, SOL_SOCKET, SO_ERROR, (void*)(&sockerr), &len) < 0) {
                    log_socket_error(TAG, sock, errno, "Error when getting socket error using getsockopt()");
                    goto error;
                }
                if (sockerr) {
                    log_socket_error(TAG, sock, sockerr, "Connection error");
                    goto error;
                }
            }
        } else {
            log_socket_error(TAG, sock, errno, "Socket is unable to connect");
            goto error;
        }
    }

    ESP_LOGI(TAG, "Client sends data to the server...");
    int len = socket_send(TAG, sock, payload, strlen(payload));
    if (len < 0) {
        ESP_LOGE(TAG, "Error occurred during socket_send");
        goto error;
    }
    ESP_LOGI(TAG, "Written: %.*s", len, payload);

    // Keep receiving until we have a reply
    do {
        len = try_receive(TAG, sock, rx_buffer, sizeof(rx_buffer));
        if (len < 0) {
            ESP_LOGE(TAG, "Error occurred during try_receive");
            goto error;
        }
        vTaskDelay(pdMS_TO_TICKS(YIELD_TO_ALL_MS));
    } while (len == 0);
    ESP_LOGI(TAG, "Received: %.*s", len, rx_buffer);

error:
    if (sock != INVALID_SOCK) {
        close(sock);
    }
    free(address_info);
    vTaskDelete(NULL);

}


void client_task()
{
    
}