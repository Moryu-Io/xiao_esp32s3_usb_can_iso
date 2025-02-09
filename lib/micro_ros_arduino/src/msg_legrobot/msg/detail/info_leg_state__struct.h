// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from msg_legrobot:msg/InfoLegState.idl
// generated code does not contain a copyright notice

#ifndef MSG_LEGROBOT__MSG__DETAIL__INFO_LEG_STATE__STRUCT_H_
#define MSG_LEGROBOT__MSG__DETAIL__INFO_LEG_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/InfoLegState in the package msg_legrobot.
typedef struct msg_legrobot__msg__InfoLegState
{
  float force_lf[3];
  float pos_lf[3];
  float force_rf[3];
  float pos_rf[3];
  float force_lr[3];
  float pos_lr[3];
  float force_rr[3];
  float pos_rr[3];
  float pos_body[3];
  float vel_body[3];
  float acc_imu[3];
  float gyro_imu[3];
  uint32_t fault;
} msg_legrobot__msg__InfoLegState;

// Struct for a sequence of msg_legrobot__msg__InfoLegState.
typedef struct msg_legrobot__msg__InfoLegState__Sequence
{
  msg_legrobot__msg__InfoLegState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} msg_legrobot__msg__InfoLegState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MSG_LEGROBOT__MSG__DETAIL__INFO_LEG_STATE__STRUCT_H_
