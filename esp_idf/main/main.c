#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"  // Include LEDC for PWM control
#include "esp_log.h"
#include "esp_system.h"

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <geometry_msgs/msg/twist.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#define AIN1 GPIO_NUM_17
#define AIN2 GPIO_NUM_16
#define BIN1 GPIO_NUM_18
#define BIN2 GPIO_NUM_19
#define STBY GPIO_NUM_23
#define PWMA GPIO_NUM_21
#define PWMB GPIO_NUM_4

// Define the PWM settings
#define PWM_FREQ 5000                     // PWM frequency in Hz
#define PWM_RESOLUTION LEDC_TIMER_8_BIT   // 8-bit resolution, 0-255 duty cycle
#define MAX_SPEED 255                     // Maximum PWM duty cycle
#define TURNING_SCALE 0.3                 // Adjust this value to reduce turning speed

// Set a constant or variable speed for motors
int motor_speed = 100;                    // Set motor speed independently

// Define the publisher and subscriber
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist recv_msg;

void init_pwm() {
    // Configure timer for PWM
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = PWM_RESOLUTION,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = PWM_FREQ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    // Configure PWM channels
    ledc_channel_config_t ledc_channel_a = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = PWMA,
        .duty = 0,  // Start with 0 duty cycle
        .hpoint = 0
    };
    ledc_channel_config(&ledc_channel_a);

    ledc_channel_config_t ledc_channel_b = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_1,
        .timer_sel = LEDC_TIMER_0,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = PWMB,
        .duty = 0,  // Start with 0 duty cycle
        .hpoint = 0
    };
    ledc_channel_config(&ledc_channel_b);
}

void set_motor_speed(float linear_x, float angular_z) {
    int forward_speed = (int)(fabs(linear_x) * MAX_SPEED);
    int turning_speed = (int)(fabs(angular_z) * MAX_SPEED * TURNING_SCALE);

    if (forward_speed > MAX_SPEED) forward_speed = MAX_SPEED;
    if (turning_speed > MAX_SPEED) turning_speed = MAX_SPEED;

    // Adjust motor speeds for turning and forward motion
    if (linear_x > 0) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, forward_speed + turning_speed);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, forward_speed - turning_speed);
    } else if (linear_x < 0) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, forward_speed - turning_speed);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, forward_speed + turning_speed);
    } else {
        // For purely turning
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, turning_speed);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, turning_speed);
    }

    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
}

void set_motor_direction(float linear_x, float angular_z) {
    if (linear_x > 0) { // Forward
        gpio_set_level(AIN1, 1);
        gpio_set_level(AIN2, 0);
        gpio_set_level(BIN1, 1);
        gpio_set_level(BIN2, 0);
        printf("Moving Forward\n");
    } else if (linear_x < 0) { // Backward
        gpio_set_level(AIN1, 0);
        gpio_set_level(AIN2, 1);
        gpio_set_level(BIN1, 0);
        gpio_set_level(BIN2, 1);
        printf("Moving Backward\n");
    } else if (angular_z > 0) { // Left
        gpio_set_level(AIN1, 1);
        gpio_set_level(AIN2, 0);
        gpio_set_level(BIN1, 0);
        gpio_set_level(BIN2, 1);
        printf("Turning Left\n");
    } else if (angular_z < 0) { // Right
        gpio_set_level(AIN1, 0);
        gpio_set_level(AIN2, 1);
        gpio_set_level(BIN1, 1);
        gpio_set_level(BIN2, 0);
        printf("Turning Right\n");
    } else if (linear_x == 0 && angular_z == 0) { // Backward
        gpio_set_level(AIN1, 0);
        gpio_set_level(AIN2, 0);
        gpio_set_level(BIN1, 0);
        gpio_set_level(BIN2, 0);
        printf("Stopped\n");
    }
}

void subscription_callback(const void * msgin)
{
    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
    gpio_set_level(STBY, 1);

    set_motor_direction(msg->linear.x, msg->angular.z); 
    set_motor_speed(msg->linear.x, msg->angular.z);   // Set speed independently
}

void micro_ros_task(void * arg)
{
    // Initialize GPIO pins and PWM
    esp_rom_gpio_pad_select_gpio(AIN1);
    esp_rom_gpio_pad_select_gpio(AIN2);
    esp_rom_gpio_pad_select_gpio(BIN1);
    esp_rom_gpio_pad_select_gpio(BIN2);
    esp_rom_gpio_pad_select_gpio(STBY);

    gpio_set_direction(AIN1, GPIO_MODE_OUTPUT);
    gpio_set_direction(AIN2, GPIO_MODE_OUTPUT);
    gpio_set_direction(BIN1, GPIO_MODE_OUTPUT);
    gpio_set_direction(BIN2, GPIO_MODE_OUTPUT);
    gpio_set_direction(STBY, GPIO_MODE_OUTPUT);

    init_pwm();  // Initialize PWM channels

    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    // Initialize ROS 2 and set up the network
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    rcl_init_options_init(&init_options, allocator);

    #ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
    rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
    rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options);
    #endif

    rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);

    // Create a new node
    rcl_node_t node = rcl_get_zero_initialized_node();
    rclc_node_init_default(&node, "Ecobot_esp32", "", &support);

    // Initialize subscriber to listen for Twist messages
    rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/cmd_vel");

    // Create executor and add subscription
    rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_subscription(&executor, &subscriber, &recv_msg, &subscription_callback, ON_NEW_DATA);

    // Spin to handle incoming messages
    while (1) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        usleep(100000);  // Small delay
    }

    // Clean up (though this part will likely never be reached)
    rcl_subscription_fini(&subscriber, &node);
    rcl_node_fini(&node);
    vTaskDelete(NULL);
}

void app_main(void)
{
    #if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
        ESP_ERROR_CHECK(uros_network_interface_initialize());
    #endif

    xTaskCreate(micro_ros_task, "uros_task", 8192, NULL, 5, NULL);
}
