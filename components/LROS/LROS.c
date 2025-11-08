/**
 * @file LROS.c
 * @author bignut
 * @brief 
 * @version 0.1
 * @date 2025-03-29
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#include "LROS.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <inttypes.h>
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include <string.h>
#include <stdio.h>
#include <unistd.h>

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif


// 定义检查define
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d:fn:%s: %d. Aborting.\n",__LINE__, #fn, (int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d:fn:%s: %d. Continuing.\n",__LINE__, #fn, (int)temp_rc);}}

// ROS publisher subscriber
rcl_publisher_t publisher;
rcl_subscription_t subscriber;

// 发送和接受
std_msgs__msg__Int32 msg;
std_msgs__msg__Int32 recv_msg;

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	RCLC_UNUSED(last_call_time);
	if (timer != NULL) {
		printf("Publishing: %d\n", (int) msg.data);
		RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));   // 这里会检查发送消息的类型与pub是否一致
		msg.data++;
	}
}

void subscription_callback(const void * msgin)
{
	const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
	printf("Received: %d\n",  (int)  msg->data);
}

void micro_ros_task(void * arg)
{
	rcl_allocator_t allocator = rcl_get_default_allocator();    // 内存分配
	rclc_support_t support;     // ros资源管理 结构体

	rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();  // support 初始化 选项
	RCCHECK(rcl_init_options_init(&init_options, allocator));                   // 将allocator 存在init_option

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
	rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options); // 获取中间件 配置

	// Static Agent IP and port can be used instead of autodisvery.
	RCCHECK(rmw_uros_options_set_udp_address("192.168.1.10", "35557", rmw_options)); // 配置中间件的IP !!!
	//RCCHECK(rmw_uros_discover_agent(rmw_options));
#endif

	// create init_options
	RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));      // 初始化 support

	// create node
	
	rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, "int32_publisher_subscriber_rclc", "", &support));      // 创建ROS 节点

	// create publisher
	RCCHECK(rclc_publisher_init_default(        //初始化 pub
		&publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),  // 获取消息类型
		"l_int32_publisher"));
			// Create subscriber.
	RCCHECK(rclc_subscription_init_default(
		&subscriber,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"l_int32_subpub"));

	// create timer,
	rcl_timer_t timer;      // 初始化定时器
	const unsigned int timer_timeout = 1000;
	RCCHECK(rclc_timer_init_default2(
		&timer,
		&support,
		RCL_MS_TO_NS(timer_timeout),
		timer_callback,             // 添加回调函数
		true));

	// create executor
	rclc_executor_t executor;       // 执行器
	RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
	
	// 增加了一个过期时间
	unsigned int rcl_wait_timeout = 1000;   // in ms
	RCCHECK(rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(rcl_wait_timeout)));
	
	// 给执行器，添加定时器和订阅者
	RCCHECK(rclc_executor_add_timer(&executor, &timer));
	RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &recv_msg, &subscription_callback, ON_NEW_DATA));

	msg.data = 0;

	while(1){
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1000));      // 执行器启动
		usleep(10000);
	}

	// free resources
	RCCHECK(rcl_subscription_fini(&subscriber, &node));
	RCCHECK(rcl_publisher_fini(&publisher, &node));
	RCCHECK(rcl_node_fini(&node));

  	vTaskDelete(NULL);
}

void LRosInit()
{
	#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
		ESP_ERROR_CHECK(uros_network_interface_initialize());		// 这里会初始化 wifi 连接 
	#endif 
    
	//pin micro-ros task in APP_CPU to make PRO_CPU to deal with wifi:
    xTaskCreate(micro_ros_task,
            "uros_task",
            48000,
            NULL,
            5,
            NULL);

}