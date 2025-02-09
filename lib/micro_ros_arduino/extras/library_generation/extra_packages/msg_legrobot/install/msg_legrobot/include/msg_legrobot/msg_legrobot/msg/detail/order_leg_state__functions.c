// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from msg_legrobot:msg/OrderLegState.idl
// generated code does not contain a copyright notice
#include "msg_legrobot/msg/detail/order_leg_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
msg_legrobot__msg__OrderLegState__init(msg_legrobot__msg__OrderLegState * msg)
{
  if (!msg) {
    return false;
  }
  // mode_lf
  // force_lf
  // pos_lf
  // mode_rf
  // force_rf
  // pos_rf
  // mode_lr
  // force_lr
  // pos_lr
  // mode_rr
  // force_rr
  // pos_rr
  return true;
}

void
msg_legrobot__msg__OrderLegState__fini(msg_legrobot__msg__OrderLegState * msg)
{
  if (!msg) {
    return;
  }
  // mode_lf
  // force_lf
  // pos_lf
  // mode_rf
  // force_rf
  // pos_rf
  // mode_lr
  // force_lr
  // pos_lr
  // mode_rr
  // force_rr
  // pos_rr
}

bool
msg_legrobot__msg__OrderLegState__are_equal(const msg_legrobot__msg__OrderLegState * lhs, const msg_legrobot__msg__OrderLegState * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // mode_lf
  if (lhs->mode_lf != rhs->mode_lf) {
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
  // mode_rf
  if (lhs->mode_rf != rhs->mode_rf) {
    return false;
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
  // mode_lr
  if (lhs->mode_lr != rhs->mode_lr) {
    return false;
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
  // mode_rr
  if (lhs->mode_rr != rhs->mode_rr) {
    return false;
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
  return true;
}

bool
msg_legrobot__msg__OrderLegState__copy(
  const msg_legrobot__msg__OrderLegState * input,
  msg_legrobot__msg__OrderLegState * output)
{
  if (!input || !output) {
    return false;
  }
  // mode_lf
  output->mode_lf = input->mode_lf;
  // force_lf
  for (size_t i = 0; i < 3; ++i) {
    output->force_lf[i] = input->force_lf[i];
  }
  // pos_lf
  for (size_t i = 0; i < 3; ++i) {
    output->pos_lf[i] = input->pos_lf[i];
  }
  // mode_rf
  output->mode_rf = input->mode_rf;
  // force_rf
  for (size_t i = 0; i < 3; ++i) {
    output->force_rf[i] = input->force_rf[i];
  }
  // pos_rf
  for (size_t i = 0; i < 3; ++i) {
    output->pos_rf[i] = input->pos_rf[i];
  }
  // mode_lr
  output->mode_lr = input->mode_lr;
  // force_lr
  for (size_t i = 0; i < 3; ++i) {
    output->force_lr[i] = input->force_lr[i];
  }
  // pos_lr
  for (size_t i = 0; i < 3; ++i) {
    output->pos_lr[i] = input->pos_lr[i];
  }
  // mode_rr
  output->mode_rr = input->mode_rr;
  // force_rr
  for (size_t i = 0; i < 3; ++i) {
    output->force_rr[i] = input->force_rr[i];
  }
  // pos_rr
  for (size_t i = 0; i < 3; ++i) {
    output->pos_rr[i] = input->pos_rr[i];
  }
  return true;
}

msg_legrobot__msg__OrderLegState *
msg_legrobot__msg__OrderLegState__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  msg_legrobot__msg__OrderLegState * msg = (msg_legrobot__msg__OrderLegState *)allocator.allocate(sizeof(msg_legrobot__msg__OrderLegState), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(msg_legrobot__msg__OrderLegState));
  bool success = msg_legrobot__msg__OrderLegState__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
msg_legrobot__msg__OrderLegState__destroy(msg_legrobot__msg__OrderLegState * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    msg_legrobot__msg__OrderLegState__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
msg_legrobot__msg__OrderLegState__Sequence__init(msg_legrobot__msg__OrderLegState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  msg_legrobot__msg__OrderLegState * data = NULL;

  if (size) {
    data = (msg_legrobot__msg__OrderLegState *)allocator.zero_allocate(size, sizeof(msg_legrobot__msg__OrderLegState), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = msg_legrobot__msg__OrderLegState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        msg_legrobot__msg__OrderLegState__fini(&data[i - 1]);
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
msg_legrobot__msg__OrderLegState__Sequence__fini(msg_legrobot__msg__OrderLegState__Sequence * array)
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
      msg_legrobot__msg__OrderLegState__fini(&array->data[i]);
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

msg_legrobot__msg__OrderLegState__Sequence *
msg_legrobot__msg__OrderLegState__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  msg_legrobot__msg__OrderLegState__Sequence * array = (msg_legrobot__msg__OrderLegState__Sequence *)allocator.allocate(sizeof(msg_legrobot__msg__OrderLegState__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = msg_legrobot__msg__OrderLegState__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
msg_legrobot__msg__OrderLegState__Sequence__destroy(msg_legrobot__msg__OrderLegState__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    msg_legrobot__msg__OrderLegState__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
msg_legrobot__msg__OrderLegState__Sequence__are_equal(const msg_legrobot__msg__OrderLegState__Sequence * lhs, const msg_legrobot__msg__OrderLegState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!msg_legrobot__msg__OrderLegState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
msg_legrobot__msg__OrderLegState__Sequence__copy(
  const msg_legrobot__msg__OrderLegState__Sequence * input,
  msg_legrobot__msg__OrderLegState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(msg_legrobot__msg__OrderLegState);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    msg_legrobot__msg__OrderLegState * data =
      (msg_legrobot__msg__OrderLegState *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!msg_legrobot__msg__OrderLegState__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          msg_legrobot__msg__OrderLegState__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!msg_legrobot__msg__OrderLegState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
