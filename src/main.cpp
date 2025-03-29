// arduino
#include <Arduino.h>
#include <ESP32-TWAI-CAN.hpp>

// user code
#include "global_config.hpp"
#include "robot_manage_task_main.hpp"
#include "rs485_imu_task_main.hpp"
#include "Debug_task_main.hpp"
#include "util_gptimer.hpp"



hw_timer_t* debug_gp_timer = NULL;

// RTOS hundle
TaskHandle_t tskHndl_can;
TaskHandle_t tskHndl_rs485;
TaskHandle_t tskHndl_ui;
TaskHandle_t tskHndl_rmt;
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

/**
 * @brief init function
 * @note loopTask外で実行される
 * @note cores/esp32/main.cpp -> app_main関数 ->
 *       cores/esp32/esp32-hal-misc.c -> initArduino関数内で呼び出し
 * 
 */
void init(){
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
  RSI::prepare_task();
  RMT::prepare_task();
  Serial.begin(460800);  // USB

  xTaskCreatePinnedToCore(main_can,      "CAN", CAN_STACK_SIZE,   NULL, 4, &tskHndl_can,   APP_CPU_NUM);
  xTaskCreatePinnedToCore(RSI::main,   "RS485", RS485_STACK_SIZE, NULL, 3, &tskHndl_rs485, APP_CPU_NUM);
  xTaskCreatePinnedToCore(main_ui,        "UI", UI_STACK_SIZE,    NULL, 2, &tskHndl_ui,    PRO_CPU_NUM);
  xTaskCreatePinnedToCore(RMT::main,     "RMT", RMT_STACK_SIZE,   NULL, 3, &tskHndl_rmt,   PRO_CPU_NUM);
  xTaskCreatePinnedToCore(DEBUG::main, "DEBUG", DEBUG_STACK_SIZE, NULL, 1, &tskHndl_debug, PRO_CPU_NUM);

}

/**
 * @brief setup function
 * @note loopTask内の最初で1回実行されることに注意
 * 
 */
void setup() {}

void loop() {}


