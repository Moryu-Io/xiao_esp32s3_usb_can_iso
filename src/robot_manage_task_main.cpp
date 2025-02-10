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

bool is_microros_init_successful = false;
enum ConnectionStatus{
  WAITING_AGENT,
  AVAILABLE_AGENT,
  CONNECTED,
  DISCONNECTED,
  UNKNOWN,
};
ConnectionStatus UROS_AGENT_STATUS = WAITING_AGENT;
uint32_t U32_UROS_PING_COUNTER_MATCH = 15;  // この回数に一回、Pingを打つ
uint32_t U32_UROS_PING_COUNTER       = 0; // Pingを打つまでのカウンター

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
  DEBUG_PRINT_STR_RMT("[RMT]OrderLegState msg recieve\n");

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
  //RCSOFTCHECK(rcl_subscription_fini(&sb_odr_command, &node));
  //RCSOFTCHECK(rcl_subscription_fini(&sb_odr_leg_state, &node));
  RCSOFTCHECK(rclc_executor_fini(&executor));
  RCSOFTCHECK(rcl_node_fini(&node));
  RCSOFTCHECK(rclc_support_fini(&support));
}


void routine_ros(){
  msg_pb_info_leg_state.fault++;
  RCSOFTCHECK(rcl_publish(&pb_info_leg_state, &msg_pb_info_leg_state, NULL));

  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
}


/**
 * @brief タスク起動前の準備用関数
 *
 */
void prepare_task() {
  set_microros_transports();

  for(int i=0;i<5;i++){
    if(RMW_RET_OK == rmw_uros_ping_agent(5, 2)) {
      create_microros_entities();
      UROS_AGENT_STATUS = CONNECTED;
      break;
    } else {
      UROS_AGENT_STATUS = WAITING_AGENT;
    }
  }

}

/**
 * @brief タスク処理
 *
 * @param params
 */
void main(void *params) {
  uint32_t loop_tick = (int)configTICK_RATE_HZ / LOOP_RATE_RMT_HZ;
  

  auto xLastWakeTime = xTaskGetTickCount();
  while(1) {
    vTaskDelayUntil(&xLastWakeTime, loop_tick);
    DEBUG_PRINT_PRC_START(DBG_PRC_ID::RMT_MAIN);  // 処理時間計測開始


    switch (UROS_AGENT_STATUS)
    {
    case WAITING_AGENT:
      /* 切断後のAgentからのPing応答待ち状態 */
      DEBUG_PRINT_STR_RMT("[RMT]waiting uros agent response\n");
      //set_microros_transports();
      if(RMW_RET_OK == rmw_uros_ping_agent(5, 2)){
        UROS_AGENT_STATUS = AVAILABLE_AGENT;
      }
      break;
    case AVAILABLE_AGENT:
      DEBUG_PRINT_STR_RMT("[RMT]Recreate uros entities\n");
      create_microros_entities();
      UROS_AGENT_STATUS = CONNECTED;
      break;
    case CONNECTED:
      if(U32_UROS_PING_COUNTER >= U32_UROS_PING_COUNTER_MATCH){
        U32_UROS_PING_COUNTER = 0;
        DEBUG_PRINT_STR_RMT("[RMT]ping uros\n");
        if(RMW_RET_OK == rmw_uros_ping_agent(5, 2)){
          routine_ros();
        } else {
          DEBUG_PRINT_STR_RMT("[RMT]uros disconnect\n");
          UROS_AGENT_STATUS = DISCONNECTED;
        }
      } else {
          routine_ros();
        U32_UROS_PING_COUNTER++;
      }
      break;
    case DISCONNECTED:
      DEBUG_PRINT_STR_RMT("[RMT]destroy uros entities\n");
      destroy_microros_entities();
      UROS_AGENT_STATUS = WAITING_AGENT;
      break;
    default:
      break;
    }

    DEBUG_PRINT_PRC_FINISH(DBG_PRC_ID::RMT_MAIN); // 処理時間計測停止
  }
}


}