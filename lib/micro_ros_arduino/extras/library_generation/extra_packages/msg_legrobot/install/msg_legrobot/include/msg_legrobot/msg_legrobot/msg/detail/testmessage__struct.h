// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from msg_legrobot:msg/Testmessage.idl
// generated code does not contain a copyright notice

#ifndef MSG_LEGROBOT__MSG__DETAIL__TESTMESSAGE__STRUCT_H_
#define MSG_LEGROBOT__MSG__DETAIL__TESTMESSAGE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Testmessage in the package msg_legrobot.
typedef struct msg_legrobot__msg__Testmessage
{
  int32_t x;
  float y;
} msg_legrobot__msg__Testmessage;

// Struct for a sequence of msg_legrobot__msg__Testmessage.
typedef struct msg_legrobot__msg__Testmessage__Sequence
{
  msg_legrobot__msg__Testmessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} msg_legrobot__msg__Testmessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MSG_LEGROBOT__MSG__DETAIL__TESTMESSAGE__STRUCT_H_
