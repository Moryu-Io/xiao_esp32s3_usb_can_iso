#ifndef UTIL_GPTIMER_HPP_
#define UTIL_GPTIMER_HPP_

#include <Arduino.h>

/* タスク処理時間計測用 */
void     init_debug_timer(hw_timer_t* _p_timer);
void     start_debug_cnt();
void     stop_debug_cnt();
uint32_t get_debug_cnt();   // 8MHzカウンタ

#endif