// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from msg_legrobot:msg/OrderLegState.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "msg_legrobot/msg/detail/order_leg_state__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace msg_legrobot
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void OrderLegState_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) msg_legrobot::msg::OrderLegState(_init);
}

void OrderLegState_fini_function(void * message_memory)
{
  auto typed_message = static_cast<msg_legrobot::msg::OrderLegState *>(message_memory);
  typed_message->~OrderLegState();
}

size_t size_function__OrderLegState__force_lf(const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * get_const_function__OrderLegState__force_lf(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<float, 3> *>(untyped_member);
  return &member[index];
}

void * get_function__OrderLegState__force_lf(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<float, 3> *>(untyped_member);
  return &member[index];
}

void fetch_function__OrderLegState__force_lf(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__OrderLegState__force_lf(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__OrderLegState__force_lf(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__OrderLegState__force_lf(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

size_t size_function__OrderLegState__pos_lf(const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * get_const_function__OrderLegState__pos_lf(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<float, 3> *>(untyped_member);
  return &member[index];
}

void * get_function__OrderLegState__pos_lf(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<float, 3> *>(untyped_member);
  return &member[index];
}

void fetch_function__OrderLegState__pos_lf(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__OrderLegState__pos_lf(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__OrderLegState__pos_lf(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__OrderLegState__pos_lf(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

size_t size_function__OrderLegState__force_rf(const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * get_const_function__OrderLegState__force_rf(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<float, 3> *>(untyped_member);
  return &member[index];
}

void * get_function__OrderLegState__force_rf(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<float, 3> *>(untyped_member);
  return &member[index];
}

void fetch_function__OrderLegState__force_rf(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__OrderLegState__force_rf(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__OrderLegState__force_rf(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__OrderLegState__force_rf(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

size_t size_function__OrderLegState__pos_rf(const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * get_const_function__OrderLegState__pos_rf(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<float, 3> *>(untyped_member);
  return &member[index];
}

void * get_function__OrderLegState__pos_rf(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<float, 3> *>(untyped_member);
  return &member[index];
}

void fetch_function__OrderLegState__pos_rf(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__OrderLegState__pos_rf(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__OrderLegState__pos_rf(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__OrderLegState__pos_rf(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

size_t size_function__OrderLegState__force_lr(const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * get_const_function__OrderLegState__force_lr(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<float, 3> *>(untyped_member);
  return &member[index];
}

void * get_function__OrderLegState__force_lr(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<float, 3> *>(untyped_member);
  return &member[index];
}

void fetch_function__OrderLegState__force_lr(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__OrderLegState__force_lr(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__OrderLegState__force_lr(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__OrderLegState__force_lr(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

size_t size_function__OrderLegState__pos_lr(const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * get_const_function__OrderLegState__pos_lr(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<float, 3> *>(untyped_member);
  return &member[index];
}

void * get_function__OrderLegState__pos_lr(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<float, 3> *>(untyped_member);
  return &member[index];
}

void fetch_function__OrderLegState__pos_lr(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__OrderLegState__pos_lr(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__OrderLegState__pos_lr(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__OrderLegState__pos_lr(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

size_t size_function__OrderLegState__force_rr(const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * get_const_function__OrderLegState__force_rr(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<float, 3> *>(untyped_member);
  return &member[index];
}

void * get_function__OrderLegState__force_rr(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<float, 3> *>(untyped_member);
  return &member[index];
}

void fetch_function__OrderLegState__force_rr(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__OrderLegState__force_rr(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__OrderLegState__force_rr(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__OrderLegState__force_rr(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

size_t size_function__OrderLegState__pos_rr(const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * get_const_function__OrderLegState__pos_rr(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<float, 3> *>(untyped_member);
  return &member[index];
}

void * get_function__OrderLegState__pos_rr(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<float, 3> *>(untyped_member);
  return &member[index];
}

void fetch_function__OrderLegState__pos_rr(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__OrderLegState__pos_rr(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__OrderLegState__pos_rr(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__OrderLegState__pos_rr(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember OrderLegState_message_member_array[12] = {
  {
    "mode_lf",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(msg_legrobot::msg::OrderLegState, mode_lf),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "force_lf",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(msg_legrobot::msg::OrderLegState, force_lf),  // bytes offset in struct
    nullptr,  // default value
    size_function__OrderLegState__force_lf,  // size() function pointer
    get_const_function__OrderLegState__force_lf,  // get_const(index) function pointer
    get_function__OrderLegState__force_lf,  // get(index) function pointer
    fetch_function__OrderLegState__force_lf,  // fetch(index, &value) function pointer
    assign_function__OrderLegState__force_lf,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "pos_lf",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(msg_legrobot::msg::OrderLegState, pos_lf),  // bytes offset in struct
    nullptr,  // default value
    size_function__OrderLegState__pos_lf,  // size() function pointer
    get_const_function__OrderLegState__pos_lf,  // get_const(index) function pointer
    get_function__OrderLegState__pos_lf,  // get(index) function pointer
    fetch_function__OrderLegState__pos_lf,  // fetch(index, &value) function pointer
    assign_function__OrderLegState__pos_lf,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "mode_rf",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(msg_legrobot::msg::OrderLegState, mode_rf),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "force_rf",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(msg_legrobot::msg::OrderLegState, force_rf),  // bytes offset in struct
    nullptr,  // default value
    size_function__OrderLegState__force_rf,  // size() function pointer
    get_const_function__OrderLegState__force_rf,  // get_const(index) function pointer
    get_function__OrderLegState__force_rf,  // get(index) function pointer
    fetch_function__OrderLegState__force_rf,  // fetch(index, &value) function pointer
    assign_function__OrderLegState__force_rf,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "pos_rf",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(msg_legrobot::msg::OrderLegState, pos_rf),  // bytes offset in struct
    nullptr,  // default value
    size_function__OrderLegState__pos_rf,  // size() function pointer
    get_const_function__OrderLegState__pos_rf,  // get_const(index) function pointer
    get_function__OrderLegState__pos_rf,  // get(index) function pointer
    fetch_function__OrderLegState__pos_rf,  // fetch(index, &value) function pointer
    assign_function__OrderLegState__pos_rf,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "mode_lr",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(msg_legrobot::msg::OrderLegState, mode_lr),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "force_lr",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(msg_legrobot::msg::OrderLegState, force_lr),  // bytes offset in struct
    nullptr,  // default value
    size_function__OrderLegState__force_lr,  // size() function pointer
    get_const_function__OrderLegState__force_lr,  // get_const(index) function pointer
    get_function__OrderLegState__force_lr,  // get(index) function pointer
    fetch_function__OrderLegState__force_lr,  // fetch(index, &value) function pointer
    assign_function__OrderLegState__force_lr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "pos_lr",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(msg_legrobot::msg::OrderLegState, pos_lr),  // bytes offset in struct
    nullptr,  // default value
    size_function__OrderLegState__pos_lr,  // size() function pointer
    get_const_function__OrderLegState__pos_lr,  // get_const(index) function pointer
    get_function__OrderLegState__pos_lr,  // get(index) function pointer
    fetch_function__OrderLegState__pos_lr,  // fetch(index, &value) function pointer
    assign_function__OrderLegState__pos_lr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "mode_rr",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(msg_legrobot::msg::OrderLegState, mode_rr),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "force_rr",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(msg_legrobot::msg::OrderLegState, force_rr),  // bytes offset in struct
    nullptr,  // default value
    size_function__OrderLegState__force_rr,  // size() function pointer
    get_const_function__OrderLegState__force_rr,  // get_const(index) function pointer
    get_function__OrderLegState__force_rr,  // get(index) function pointer
    fetch_function__OrderLegState__force_rr,  // fetch(index, &value) function pointer
    assign_function__OrderLegState__force_rr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "pos_rr",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(msg_legrobot::msg::OrderLegState, pos_rr),  // bytes offset in struct
    nullptr,  // default value
    size_function__OrderLegState__pos_rr,  // size() function pointer
    get_const_function__OrderLegState__pos_rr,  // get_const(index) function pointer
    get_function__OrderLegState__pos_rr,  // get(index) function pointer
    fetch_function__OrderLegState__pos_rr,  // fetch(index, &value) function pointer
    assign_function__OrderLegState__pos_rr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers OrderLegState_message_members = {
  "msg_legrobot::msg",  // message namespace
  "OrderLegState",  // message name
  12,  // number of fields
  sizeof(msg_legrobot::msg::OrderLegState),
  OrderLegState_message_member_array,  // message members
  OrderLegState_init_function,  // function to initialize message memory (memory has to be allocated)
  OrderLegState_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t OrderLegState_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &OrderLegState_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace msg_legrobot


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<msg_legrobot::msg::OrderLegState>()
{
  return &::msg_legrobot::msg::rosidl_typesupport_introspection_cpp::OrderLegState_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, msg_legrobot, msg, OrderLegState)() {
  return &::msg_legrobot::msg::rosidl_typesupport_introspection_cpp::OrderLegState_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
