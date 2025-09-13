/**
 * @file LD14.h
 * @author bignut
 * @brief 
 * @details
 * @version 0.1
 * @date 2025-03-28
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#ifndef __LTL_WLFL__
#define __LTL_WLFL__

#include "stdio.h"

#define EXAMPLE_ESP_WIFI_SSID      "CMCC-1002"
#define EXAMPLE_ESP_WIFI_PASS      "5qyb4x4z"

#ifdef __cplusplus
extern "C" {
#endif

void wlfl_init_sta(void);

void get_ip_address_uint16(uint16_t *addr);

void get_ip_address_str(char *addr, int len);

#ifdef __cplusplus
}
#endif

#endif