// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from msg_legrobot:msg/Testmessage.idl
// generated code does not contain a copyright notice

#ifndef MSG_LEGROBOT__MSG__DETAIL__TESTMESSAGE__FUNCTIONS_H_
#define MSG_LEGROBOT__MSG__DETAIL__TESTMESSAGE__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "msg_legrobot/msg/rosidl_generator_c__visibility_control.h"

#include "msg_legrobot/msg/detail/testmessage__struct.h"

/// Initialize msg/Testmessage message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * msg_legrobot__msg__Testmessage
 * )) before or use
 * msg_legrobot__msg__Testmessage__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_msg_legrobot
bool
msg_legrobot__msg__Testmessage__init(msg_legrobot__msg__Testmessage * msg);

/// Finalize msg/Testmessage message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_msg_legrobot
void
msg_legrobot__msg__Testmessage__fini(msg_legrobot__msg__Testmessage * msg);

/// Create msg/Testmessage message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * msg_legrobot__msg__Testmessage__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_msg_legrobot
msg_legrobot__msg__Testmessage *
msg_legrobot__msg__Testmessage__create();

/// Destroy msg/Testmessage message.
/**
 * It calls
 * msg_legrobot__msg__Testmessage__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_msg_legrobot
void
msg_legrobot__msg__Testmessage__destroy(msg_legrobot__msg__Testmessage * msg);

/// Check for msg/Testmessage message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_msg_legrobot
bool
msg_legrobot__msg__Testmessage__are_equal(const msg_legrobot__msg__Testmessage * lhs, const msg_legrobot__msg__Testmessage * rhs);

/// Copy a msg/Testmessage message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_msg_legrobot
bool
msg_legrobot__msg__Testmessage__copy(
  const msg_legrobot__msg__Testmessage * input,
  msg_legrobot__msg__Testmessage * output);

/// Initialize array of msg/Testmessage messages.
/**
 * It allocates the memory for the number of elements and calls
 * msg_legrobot__msg__Testmessage__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_msg_legrobot
bool
msg_legrobot__msg__Testmessage__Sequence__init(msg_legrobot__msg__Testmessage__Sequence * array, size_t size);

/// Finalize array of msg/Testmessage messages.
/**
 * It calls
 * msg_legrobot__msg__Testmessage__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_msg_legrobot
void
msg_legrobot__msg__Testmessage__Sequence__fini(msg_legrobot__msg__Testmessage__Sequence * array);

/// Create array of msg/Testmessage messages.
/**
 * It allocates the memory for the array and calls
 * msg_legrobot__msg__Testmessage__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_msg_legrobot
msg_legrobot__msg__Testmessage__Sequence *
msg_legrobot__msg__Testmessage__Sequence__create(size_t size);

/// Destroy array of msg/Testmessage messages.
/**
 * It calls
 * msg_legrobot__msg__Testmessage__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_msg_legrobot
void
msg_legrobot__msg__Testmessage__Sequence__destroy(msg_legrobot__msg__Testmessage__Sequence * array);

/// Check for msg/Testmessage message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_msg_legrobot
bool
msg_legrobot__msg__Testmessage__Sequence__are_equal(const msg_legrobot__msg__Testmessage__Sequence * lhs, const msg_legrobot__msg__Testmessage__Sequence * rhs);

/// Copy an array of msg/Testmessage messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_msg_legrobot
bool
msg_legrobot__msg__Testmessage__Sequence__copy(
  const msg_legrobot__msg__Testmessage__Sequence * input,
  msg_legrobot__msg__Testmessage__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // MSG_LEGROBOT__MSG__DETAIL__TESTMESSAGE__FUNCTIONS_H_
