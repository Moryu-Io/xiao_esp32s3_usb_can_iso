// arduino
#include <Arduino.h>
#include <ESP32-TWAI-CAN.hpp>

// user code
#include "global_config.hpp"
#include "Debug_task_main.hpp"
#include "util_gptimer.hpp"

#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;
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

hw_timer_t* debug_gp_timer = NULL;

// RTOS hundle
TaskHandle_t tskHndl_can;
TaskHandle_t tskHndl_rs485;
TaskHandle_t tskHndl_ui;
TaskHandle_t tskHndl_debug;


void main_can(void *params){
  uint32_t loop_tick = (int)configTICK_RATE_HZ / LOOP_RATE_CAN_HZ;

  auto xLastWakeTime = xTaskGetTickCount();
  while (true) {
    vTaskDelayUntil(&xLastWakeTime, loop_tick);
    DEBUG_PRINT_PRC_START(DBG_PRC_ID::CAN_MAIN);  // 処理時間計測開始



    DEBUG_PRINT_PRC_FINISH(DBG_PRC_ID::CAN_MAIN); // 処理時間計測停止
  }
}

void main_rs485(void *params){
  uint32_t loop_tick = (int)configTICK_RATE_HZ / LOOP_RATE_RS485_HZ;

  auto xLastWakeTime = xTaskGetTickCount();
  while (true) {
    vTaskDelayUntil(&xLastWakeTime, loop_tick);
    DEBUG_PRINT_PRC_START(DBG_PRC_ID::RS485_MAIN);  // 処理時間計測開始

    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    msg.data++;
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));

    DEBUG_PRINT_PRC_FINISH(DBG_PRC_ID::RS485_MAIN); // 処理時間計測停止
  }
}

void main_ui(void *params){
  uint32_t loop_tick = (int)configTICK_RATE_HZ / LOOP_RATE_UI_HZ;

  auto xLastWakeTime = xTaskGetTickCount();
  while (true) {
    vTaskDelayUntil(&xLastWakeTime, loop_tick);
    DEBUG_PRINT_PRC_START(DBG_PRC_ID::UI_MAIN);  // 処理時間計測開始

    digitalWrite(PIN::DEBUG_USER_LED, digitalRead(PIN::DEBUG_USER_LED)^1);

    DEBUG_PRINT_PRC_FINISH(DBG_PRC_ID::UI_MAIN); // 処理時間計測停止
  }
}


void setup() {

  set_microros_transports();
  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "micro_ros_arduino_node_publisher"));
  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));

  // general purpose timer (8MHz count) 
  debug_gp_timer = timerBegin(0, 10, true);
  init_debug_timer(debug_gp_timer);

  // USB uart
  Serial.begin(460800);  // USB
  Serial1.begin(460800,SERIAL_8N1,PIN::DEBUG_UART_RX,PIN::DEBUG_UART_TX);  // debug ext pin

  // PinConfig
  pinMode(PIN::DEBUG_USER_LED, OUTPUT);
  pinMode(PIN::PWR_COM_DEVICE, OUTPUT);
  pinMode(PIN::RS485_RE,       OUTPUT);

  // CAN Config
  ESP32Can.begin(TWAI_SPEED_1000KBPS, PIN::CAN_TX, PIN::CAN_RX);

  // 電源投入後、一旦LOWに明示的に落とす
  digitalWrite(PIN::DEBUG_USER_LED, LOW);
  digitalWrite(PIN::PWR_COM_DEVICE, LOW);

  delay(300);

  // 外部デバイスの電源ON
  digitalWrite(PIN::PWR_COM_DEVICE, HIGH);

  // Task Config
  DEBUG::prepare_task();

  xTaskCreatePinnedToCore(main_can,     "CAN", CAN_STACK_SIZE,   NULL, 4, &tskHndl_can,   APP_CPU_NUM);
  xTaskCreatePinnedToCore(main_rs485,  "RS485", RS485_STACK_SIZE, NULL, 3, &tskHndl_rs485, APP_CPU_NUM);
  xTaskCreatePinnedToCore(main_ui,       "UI", UI_STACK_SIZE,    NULL, 2, &tskHndl_ui,    PRO_CPU_NUM);
  xTaskCreatePinnedToCore(DEBUG::main, "DEBUG", DEBUG_STACK_SIZE, NULL, 1, &tskHndl_debug, PRO_CPU_NUM);
}

void loop() {
}


