// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from msg_legrobot:msg/InfoLegState.idl
// generated code does not contain a copyright notice
#include "msg_legrobot/msg/detail/info_leg_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
msg_legrobot__msg__InfoLegState__init(msg_legrobot__msg__InfoLegState * msg)
{
  if (!msg) {
    return false;
  }
  // force_lf
  // pos_lf
  // force_rf
  // pos_rf
  // force_lr
  // pos_lr
  // force_rr
  // pos_rr
  // pos_body
  // vel_body
  // acc_imu
  // gyro_imu
  // fault
  return true;
}

void
msg_legrobot__msg__InfoLegState__fini(msg_legrobot__msg__InfoLegState * msg)
{
  if (!msg) {
    return;
  }
  // force_lf
  // pos_lf
  // force_rf
  // pos_rf
  // force_lr
  // pos_lr
  // force_rr
  // pos_rr
  // pos_body
  // vel_body
  // acc_imu
  // gyro_imu
  // fault
}

bool
msg_legrobot__msg__InfoLegState__are_equal(const msg_legrobot__msg__InfoLegState * lhs, const msg_legrobot__msg__InfoLegState * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // force_lf
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->force_lf[i] != rhs->force_lf[i]) {
      return false;
    }
  }
  // pos_lf
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->pos_lf[i] != rhs->pos_lf[i]) {
      return false;
    }
  }
  // force_rf
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->force_rf[i] != rhs->force_rf[i]) {
      return false;
    }
  }
  // pos_rf
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->pos_rf[i] != rhs->pos_rf[i]) {
      return false;
    }
  }
  // force_lr
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->force_lr[i] != rhs->force_lr[i]) {
      return false;
    }
  }
  // pos_lr
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->pos_lr[i] != rhs->pos_lr[i]) {
      return false;
    }
  }
  // force_rr
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->force_rr[i] != rhs->force_rr[i]) {
      return false;
    }
  }
  // pos_rr
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->pos_rr[i] != rhs->pos_rr[i]) {
      return false;
    }
  }
  // pos_body
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->pos_body[i] != rhs->pos_body[i]) {
      return false;
    }
  }
  // vel_body
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->vel_body[i] != rhs->vel_body[i]) {
      return false;
    }
  }
  // acc_imu
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->acc_imu[i] != rhs->acc_imu[i]) {
      return false;
    }
  }
  // gyro_imu
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->gyro_imu[i] != rhs->gyro_imu[i]) {
      return false;
    }
  }
  // fault
  if (lhs->fault != rhs->fault) {
    return false;
  }
  return true;
}

bool
msg_legrobot__msg__InfoLegState__copy(
  const msg_legrobot__msg__InfoLegState * input,
  msg_legrobot__msg__InfoLegState * output)
{
  if (!input || !output) {
    return false;
  }
  // force_lf
  for (size_t i = 0; i < 3; ++i) {
    output->force_lf[i] = input->force_lf[i];
  }
  // pos_lf
  for (size_t i = 0; i < 3; ++i) {
    output->pos_lf[i] = input->pos_lf[i];
  }
  // force_rf
  for (size_t i = 0; i < 3; ++i) {
    output->force_rf[i] = input->force_rf[i];
  }
  // pos_rf
  for (size_t i = 0; i < 3; ++i) {
    output->pos_rf[i] = input->pos_rf[i];
  }
  // force_lr
  for (size_t i = 0; i < 3; ++i) {
    output->force_lr[i] = input->force_lr[i];
  }
  // pos_lr
  for (size_t i = 0; i < 3; ++i) {
    output->pos_lr[i] = input->pos_lr[i];
  }
  // force_rr
  for (size_t i = 0; i < 3; ++i) {
    output->force_rr[i] = input->force_rr[i];
  }
  // pos_rr
  for (size_t i = 0; i < 3; ++i) {
    output->pos_rr[i] = input->pos_rr[i];
  }
  // pos_body
  for (size_t i = 0; i < 3; ++i) {
    output->pos_body[i] = input->pos_body[i];
  }
  // vel_body
  for (size_t i = 0; i < 3; ++i) {
    output->vel_body[i] = input->vel_body[i];
  }
  // acc_imu
  for (size_t i = 0; i < 3; ++i) {
    output->acc_imu[i] = input->acc_imu[i];
  }
  // gyro_imu
  for (size_t i = 0; i < 3; ++i) {
    output->gyro_imu[i] = input->gyro_imu[i];
  }
  // fault
  output->fault = input->fault;
  return true;
}

msg_legrobot__msg__InfoLegState *
msg_legrobot__msg__InfoLegState__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  msg_legrobot__msg__InfoLegState * msg = (msg_legrobot__msg__InfoLegState *)allocator.allocate(sizeof(msg_legrobot__msg__InfoLegState), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(msg_legrobot__msg__InfoLegState));
  bool success = msg_legrobot__msg__InfoLegState__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
msg_legrobot__msg__InfoLegState__destroy(msg_legrobot__msg__InfoLegState * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    msg_legrobot__msg__InfoLegState__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
msg_legrobot__msg__InfoLegState__Sequence__init(msg_legrobot__msg__InfoLegState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  msg_legrobot__msg__InfoLegState * data = NULL;

  if (size) {
    data = (msg_legrobot__msg__InfoLegState *)allocator.zero_allocate(size, sizeof(msg_legrobot__msg__InfoLegState), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = msg_legrobot__msg__InfoLegState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        msg_legrobot__msg__InfoLegState__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
msg_legrobot__msg__InfoLegState__Sequence__fini(msg_legrobot__msg__InfoLegState__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      msg_legrobot__msg__InfoLegState__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

msg_legrobot__msg__InfoLegState__Sequence *
msg_legrobot__msg__InfoLegState__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  msg_legrobot__msg__InfoLegState__Sequence * array = (msg_legrobot__msg__InfoLegState__Sequence *)allocator.allocate(sizeof(msg_legrobot__msg__InfoLegState__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = msg_legrobot__msg__InfoLegState__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
msg_legrobot__msg__InfoLegState__Sequence__destroy(msg_legrobot__msg__InfoLegState__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    msg_legrobot__msg__InfoLegState__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
msg_legrobot__msg__InfoLegState__Sequence__are_equal(const msg_legrobot__msg__InfoLegState__Sequence * lhs, const msg_legrobot__msg__InfoLegState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!msg_legrobot__msg__InfoLegState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
msg_legrobot__msg__InfoLegState__Sequence__copy(
  const msg_legrobot__msg__InfoLegState__Sequence * input,
  msg_legrobot__msg__InfoLegState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(msg_legrobot__msg__InfoLegState);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    msg_legrobot__msg__InfoLegState * data =
      (msg_legrobot__msg__InfoLegState *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!msg_legrobot__msg__InfoLegState__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          msg_legrobot__msg__InfoLegState__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!msg_legrobot__msg__InfoLegState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
