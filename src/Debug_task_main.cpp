#include <Arduino.h>

// ローカル
#include "Debug_task_main.hpp"
#include "global_config.hpp"
#include "util_gptimer.hpp"

extern TaskHandle_t tskHndl_can;
extern TaskHandle_t tskHndl_rs485;
extern TaskHandle_t tskHndl_ui;
extern TaskHandle_t tskHndl_debug;

namespace DEBUG {

static Stream* P_DBG_SERIAL = &DEBUG_SERIAL_MOD;

#define DEBUGPRINT_BUFLEN (4096)
#define PROCLOAD_BUFLEN   (4096*4)

template <int buflen>
struct PrintBuffer {
  uint8_t  u8_buf_[buflen];
  uint32_t u32_head_;
};

PrintBuffer<DEBUGPRINT_BUFLEN> DEBUG_PRINT_BUF[2];
uint8_t                        U8_WRITE_PAGE = 0;

// 外部でSprintfするためのBuffer
char EXT_PRINT_BUF[1024];

// 処理負荷計測用
constexpr uint8_t            U8_PROCLOAD_DATA_LEN = 6;
PrintBuffer<PROCLOAD_BUFLEN> PROCLOAD_BUF[2];
volatile uint8_t                      U8_PL_WRITE_PAGE = 0;
volatile bool                         IS_SATRT_RECORD  = false;
volatile SemaphoreHandle_t semaphore_procload;

// RTOS debug variable
char CR_RTOS_RUNTIME_STATUS_BUF[512] = {};
bool IS_RTOS_RUNTIME_MEASURED        = false;

uint32_t       U32_RTOS_RUNTIME_MEAS_MS_CNT = 0;
const uint32_t CU32_RTOS_RUNTIME_MEAS_MS    = 10000;

bool IS_DEBUG_TASK_START = false;

void process_inputchar();

/**
 * @brief タスク起動前の準備用関数
 *
 */
void prepare_task() {
  semaphore_procload = xSemaphoreCreateBinary();
  xSemaphoreGive(semaphore_procload);
}

/**
 * @brief タスク処理
 *
 * @param params
 */
void main(void *params) {
  uint32_t loop_tick = (int)configTICK_RATE_HZ / LOOP_RATE_DEBUG_HZ;

  IS_DEBUG_TASK_START = true;

  auto xLastWakeTime = xTaskGetTickCount();
  while(1) {
    vTaskDelayUntil(&xLastWakeTime, loop_tick);
    //DEBUG_PRINT_PRC_START(DBG_PRC_ID::DBG_MAIN);  // なぜか上手く動作しない

    /* Debug処理 */
    if(IS_RTOS_RUNTIME_MEASURED && ((millis() - U32_RTOS_RUNTIME_MEAS_MS_CNT) > CU32_RTOS_RUNTIME_MEAS_MS)) {
      IS_RTOS_RUNTIME_MEASURED = false;

    }

    /* 入力処理 */
    process_inputchar();

    /* 出力処理 */
    /* 書き込みページ切り替え */
    uint8_t _u8_read_page = U8_WRITE_PAGE;
    U8_WRITE_PAGE         = U8_WRITE_PAGE ^ 1;

    if(DEBUG_PRINT_BUF[_u8_read_page].u32_head_ != 0) {
      /* 書き込みデータがある場合 */
      P_DBG_SERIAL->write(DEBUG_PRINT_BUF[_u8_read_page].u8_buf_, DEBUG_PRINT_BUF[_u8_read_page].u32_head_);
      DEBUG_PRINT_BUF[_u8_read_page].u32_head_ = 0;
    }

    if(IS_SATRT_RECORD) {
      xSemaphoreTake(semaphore_procload, 100);
      uint8_t _u8_pl_read_page = U8_PL_WRITE_PAGE;
      U8_PL_WRITE_PAGE         = U8_PL_WRITE_PAGE ^ 1;
      xSemaphoreGive(semaphore_procload);

      if(PROCLOAD_BUF[_u8_pl_read_page].u32_head_ != 0) {
        /* 書き込みデータがある場合 */
        P_DBG_SERIAL->write(PROCLOAD_BUF[_u8_pl_read_page].u8_buf_, PROCLOAD_BUF[_u8_pl_read_page].u32_head_);
        PROCLOAD_BUF[_u8_pl_read_page].u32_head_ = 0;
      }
    }

    
    //DEBUG_PRINT_PRC_FINISH(DBG_PRC_ID::DBG_MAIN);
  }
}

/**
 * @brief
 *
 * @param _buf
 * @param _size
 */
void print(char *_buf, uint32_t _size) {
  if(IS_DEBUG_TASK_START){
    uint32_t _u32_head     = DEBUG_PRINT_BUF[U8_WRITE_PAGE].u32_head_;
    uint32_t _u32_rest_len = DEBUGPRINT_BUFLEN - _u32_head;
    if(_u32_rest_len >= _size) {
      /* バッファ残りより少ない量の書き込みの場合 */
      memcpy(&DEBUG_PRINT_BUF[U8_WRITE_PAGE].u8_buf_[_u32_head], _buf, _size);
      DEBUG_PRINT_BUF[U8_WRITE_PAGE].u32_head_ += _size;
    } else {
      /* バッファ残りより多い場合、書き込める場所まで書き込む */
      memcpy(&DEBUG_PRINT_BUF[U8_WRITE_PAGE].u8_buf_[_u32_head], _buf, _u32_rest_len);
      DEBUG_PRINT_BUF[U8_WRITE_PAGE].u32_head_ += _u32_rest_len;
    }
  } else {
    // Debug Taskが始まっていないので普通にSerialで投げる
    P_DBG_SERIAL->write(_buf, _size);
  }
}

/**
 * @brief
 *
 * @param _buf
 * @param _size
 */
void record_proc_load(uint8_t prc_id, uint8_t is_start) {
  if(!IS_SATRT_RECORD) return;

  xSemaphoreTake(semaphore_procload, 100);
  volatile uint8_t  _u8_wpage = U8_PL_WRITE_PAGE;
  volatile uint32_t _u32_head      = PROCLOAD_BUF[_u8_wpage].u32_head_;
  volatile uint32_t _u32_rest_len  = PROCLOAD_BUFLEN - _u32_head;
  if(_u32_rest_len >= U8_PROCLOAD_DATA_LEN) {
    /* バッファ残りより少ない量の書き込みの場合 */
    // 先にheadを進めておく（割り込み対策）
    PROCLOAD_BUF[_u8_wpage].u32_head_ = _u32_head + U8_PROCLOAD_DATA_LEN;

    volatile uint32_t _u32_now_count = (uint32_t)get_debug_cnt();

    PROCLOAD_BUF[_u8_wpage].u8_buf_[_u32_head]     = prc_id;
    PROCLOAD_BUF[_u8_wpage].u8_buf_[_u32_head + 1] = is_start;
    PROCLOAD_BUF[_u8_wpage].u8_buf_[_u32_head + 2] = _u32_now_count & 0x000000FF;
    PROCLOAD_BUF[_u8_wpage].u8_buf_[_u32_head + 3] = (uint8_t)((_u32_now_count & 0x0000FF00) >> 8);
    PROCLOAD_BUF[_u8_wpage].u8_buf_[_u32_head + 4] = (uint8_t)((_u32_now_count & 0x00FF0000) >> 16);
    PROCLOAD_BUF[_u8_wpage].u8_buf_[_u32_head + 5] = (uint8_t)((_u32_now_count & 0xFF000000) >> 24);

  } else {
    /* バッファ残りより多い場合、無効 */
  }
  xSemaphoreGive(semaphore_procload);
}

/**
 *
 */
static void subproc_debug_menu() {
  P_DBG_SERIAL->printf("[DEBUG]DEBUG MENU\n");
  P_DBG_SERIAL->printf("[DEBUG]r:rtos proctime, s:stacksize, p:pl start, f:pl finish\n");
  while(P_DBG_SERIAL->available() < 1) {};
  char _c = P_DBG_SERIAL->read();

  switch(_c) {
  case 'r':
    /* RTOS処理時間測定開始 */
    IS_RTOS_RUNTIME_MEASURED     = true;
    //U32_RTOS_RUNTIME_MEAS_MS_CNT = millis();
    //start_gptimer_cnt();
    break;
  case 's':
    /* stack size */
  {
    int can_max_stack_size   = CAN_STACK_SIZE   - uxTaskGetStackHighWaterMark(tskHndl_can);
    int rs485_max_stack_size = RS485_STACK_SIZE - uxTaskGetStackHighWaterMark(tskHndl_rs485);
    int ui_max_stack_size    = UI_STACK_SIZE    - uxTaskGetStackHighWaterMark(tskHndl_ui);
    int debug_max_stack_size = DEBUG_STACK_SIZE - uxTaskGetStackHighWaterMark(tskHndl_debug);


    P_DBG_SERIAL->println("***** Stack Coms *****");
    P_DBG_SERIAL->printf("Can   : %d byte / %d byte (rest: %d[%%])\n",
                  can_max_stack_size, CAN_STACK_SIZE, can_max_stack_size*100/CAN_STACK_SIZE);
    P_DBG_SERIAL->printf("RS485 : %d byte / %d byte (rest: %d[%%])\n",
                  rs485_max_stack_size, RS485_STACK_SIZE, rs485_max_stack_size*100/RS485_STACK_SIZE);
    P_DBG_SERIAL->printf("UI    : %d byte / %d byte (rest: %d[%%])\n",
                  ui_max_stack_size, UI_STACK_SIZE, ui_max_stack_size*100/UI_STACK_SIZE);
    P_DBG_SERIAL->printf("Debug : %d byte / %d byte (rest: %d[%%])\n",
                  debug_max_stack_size, DEBUG_STACK_SIZE, debug_max_stack_size*100/DEBUG_STACK_SIZE);
  }
    break;
  case 'p':
    /* 全処理時間測定開始 */
    start_debug_cnt();
    IS_SATRT_RECORD = true;
    break;
  case 'f':
    /* 全処理時間測定終了 */
    IS_SATRT_RECORD = false;
    stop_debug_cnt();
    break;
  default:
    break;
  }
};

/**
 * @brief シリアル入力処理
 *
 */
void process_inputchar() {
  if(P_DBG_SERIAL->available()) {
    char _c = P_DBG_SERIAL->read();
    switch(_c) {
    case 't':
      subproc_debug_menu();
      break;
    default:
      break;
    }
  }
}

}; // namespace DEBUG
