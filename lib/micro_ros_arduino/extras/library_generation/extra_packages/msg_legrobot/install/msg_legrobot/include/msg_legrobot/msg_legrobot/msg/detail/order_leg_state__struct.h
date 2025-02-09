// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from msg_legrobot:msg/OrderLegState.idl
// generated code does not contain a copyright notice

#ifndef MSG_LEGROBOT__MSG__DETAIL__ORDER_LEG_STATE__STRUCT_H_
#define MSG_LEGROBOT__MSG__DETAIL__ORDER_LEG_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/OrderLegState in the package msg_legrobot.
typedef struct msg_legrobot__msg__OrderLegState
{
  uint32_t mode_lf;
  float force_lf[3];
  float pos_lf[3];
  uint32_t mode_rf;
  float force_rf[3];
  float pos_rf[3];
  uint32_t mode_lr;
  float force_lr[3];
  float pos_lr[3];
  uint32_t mode_rr;
  float force_rr[3];
  float pos_rr[3];
} msg_legrobot__msg__OrderLegState;

// Struct for a sequence of msg_legrobot__msg__OrderLegState.
typedef struct msg_legrobot__msg__OrderLegState__Sequence
{
  msg_legrobot__msg__OrderLegState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} msg_legrobot__msg__OrderLegState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MSG_LEGROBOT__MSG__DETAIL__ORDER_LEG_STATE__STRUCT_H_
