#include <Arduino.h>

// ローカル
#include "global_config.hpp"
#include "robot_manage_task_main.hpp"

#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/u_int32.h>
#include <msg_legrobot/msg/testmessage.h>
#include <msg_legrobot/msg/order_leg_state.h>
#include <msg_legrobot/msg/info_leg_state.h>

namespace RMT {

// publisher
rcl_publisher_t pb_info_leg_state;
msg_legrobot__msg__InfoLegState msg_pb_info_leg_state;

// subscriber
rcl_subscription_t sb_odr_command;
rcl_subscription_t sb_odr_leg_state;
std_msgs__msg__UInt32 msg_sb_odr_cmd;
msg_legrobot__msg__OrderLegState msg_sb_odr_leg_state;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
  

void error_loop(){
  while(1){
    delay(100);
  }
}

void sb_order_cmd_callback(const void* msgin){

}

void sb_order_legstate_callback(const void* msgin){

}


static void create_microros_entities(){
  allocator = rcl_get_default_allocator();
  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  // create node
  RCCHECK(rclc_node_init_default(&node, "leg_robot", "", &support));
  // create publisher
  RCCHECK(rclc_publisher_init_best_effort(
    &pb_info_leg_state,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(msg_legrobot, msg, InfoLegState),
    "InfoLegState"));
  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &sb_odr_command,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt32),
    "OrderCommand"));
  RCCHECK(rclc_subscription_init_default(
    &sb_odr_leg_state,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(msg_legrobot, msg, OrderLegState),
    "OrderLegState"));

  // create executor
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &sb_odr_command, &msg_sb_odr_cmd, &sb_order_cmd_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sb_odr_leg_state, &msg_sb_odr_leg_state, &sb_order_legstate_callback, ON_NEW_DATA));
}

static void destroy_microros_entities(){
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  RCSOFTCHECK(rcl_publisher_fini(&pb_info_leg_state, &node));
  RCSOFTCHECK(rcl_subscription_fini(&sb_odr_command, &node));
  RCSOFTCHECK(rcl_subscription_fini(&sb_odr_leg_state, &node));
  RCSOFTCHECK(rclc_executor_fini(&executor));
  RCSOFTCHECK(rcl_node_fini(&node));
  RCSOFTCHECK(rclc_support_fini(&support));
}


/**
 * @brief タスク起動前の準備用関数
 *
 */
void prepare_task() {
  set_microros_transports();

  while(true) {
    if(RMW_RET_OK == rmw_uros_ping_agent(50, 2)) {
      create_microros_entities();
      break;
    } else {
      // 何もしない
    }
    // vTaskDelay(500);
    delay(500);
  }

}

/**
 * @brief タスク処理
 *
 * @param params
 */
void main(void *params) {
  uint32_t loop_tick = (int)configTICK_RATE_HZ / LOOP_RATE_RMT_HZ;
  RMT::prepare_task();
  

  auto xLastWakeTime = xTaskGetTickCount();
  while(1) {
    vTaskDelayUntil(&xLastWakeTime, loop_tick);
    DEBUG_PRINT_PRC_START(DBG_PRC_ID::RMT_MAIN);  // 処理時間計測開始

    msg_pb_info_leg_state.fault++;
    RCSOFTCHECK(rcl_publish(&pb_info_leg_state, &msg_pb_info_leg_state, NULL));

    //rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
    DEBUG_PRINT_PRC_FINISH(DBG_PRC_ID::RMT_MAIN); // 処理時間計測停止
  }
}


}