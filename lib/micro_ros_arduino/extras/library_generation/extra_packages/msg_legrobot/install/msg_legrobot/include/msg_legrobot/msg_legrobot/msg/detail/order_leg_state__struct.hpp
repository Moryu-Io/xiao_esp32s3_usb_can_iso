// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from msg_legrobot:msg/OrderLegState.idl
// generated code does not contain a copyright notice

#ifndef MSG_LEGROBOT__MSG__DETAIL__ORDER_LEG_STATE__STRUCT_HPP_
#define MSG_LEGROBOT__MSG__DETAIL__ORDER_LEG_STATE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__msg_legrobot__msg__OrderLegState __attribute__((deprecated))
#else
# define DEPRECATED__msg_legrobot__msg__OrderLegState __declspec(deprecated)
#endif

namespace msg_legrobot
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct OrderLegState_
{
  using Type = OrderLegState_<ContainerAllocator>;

  explicit OrderLegState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->mode_lf = 0ul;
      std::fill<typename std::array<float, 3>::iterator, float>(this->force_lf.begin(), this->force_lf.end(), 0.0f);
      std::fill<typename std::array<float, 3>::iterator, float>(this->pos_lf.begin(), this->pos_lf.end(), 0.0f);
      this->mode_rf = 0ul;
      std::fill<typename std::array<float, 3>::iterator, float>(this->force_rf.begin(), this->force_rf.end(), 0.0f);
      std::fill<typename std::array<float, 3>::iterator, float>(this->pos_rf.begin(), this->pos_rf.end(), 0.0f);
      this->mode_lr = 0ul;
      std::fill<typename std::array<float, 3>::iterator, float>(this->force_lr.begin(), this->force_lr.end(), 0.0f);
      std::fill<typename std::array<float, 3>::iterator, float>(this->pos_lr.begin(), this->pos_lr.end(), 0.0f);
      this->mode_rr = 0ul;
      std::fill<typename std::array<float, 3>::iterator, float>(this->force_rr.begin(), this->force_rr.end(), 0.0f);
      std::fill<typename std::array<float, 3>::iterator, float>(this->pos_rr.begin(), this->pos_rr.end(), 0.0f);
    }
  }

  explicit OrderLegState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : force_lf(_alloc),
    pos_lf(_alloc),
    force_rf(_alloc),
    pos_rf(_alloc),
    force_lr(_alloc),
    pos_lr(_alloc),
    force_rr(_alloc),
    pos_rr(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->mode_lf = 0ul;
      std::fill<typename std::array<float, 3>::iterator, float>(this->force_lf.begin(), this->force_lf.end(), 0.0f);
      std::fill<typename std::array<float, 3>::iterator, float>(this->pos_lf.begin(), this->pos_lf.end(), 0.0f);
      this->mode_rf = 0ul;
      std::fill<typename std::array<float, 3>::iterator, float>(this->force_rf.begin(), this->force_rf.end(), 0.0f);
      std::fill<typename std::array<float, 3>::iterator, float>(this->pos_rf.begin(), this->pos_rf.end(), 0.0f);
      this->mode_lr = 0ul;
      std::fill<typename std::array<float, 3>::iterator, float>(this->force_lr.begin(), this->force_lr.end(), 0.0f);
      std::fill<typename std::array<float, 3>::iterator, float>(this->pos_lr.begin(), this->pos_lr.end(), 0.0f);
      this->mode_rr = 0ul;
      std::fill<typename std::array<float, 3>::iterator, float>(this->force_rr.begin(), this->force_rr.end(), 0.0f);
      std::fill<typename std::array<float, 3>::iterator, float>(this->pos_rr.begin(), this->pos_rr.end(), 0.0f);
    }
  }

  // field types and members
  using _mode_lf_type =
    uint32_t;
  _mode_lf_type mode_lf;
  using _force_lf_type =
    std::array<float, 3>;
  _force_lf_type force_lf;
  using _pos_lf_type =
    std::array<float, 3>;
  _pos_lf_type pos_lf;
  using _mode_rf_type =
    uint32_t;
  _mode_rf_type mode_rf;
  using _force_rf_type =
    std::array<float, 3>;
  _force_rf_type force_rf;
  using _pos_rf_type =
    std::array<float, 3>;
  _pos_rf_type pos_rf;
  using _mode_lr_type =
    uint32_t;
  _mode_lr_type mode_lr;
  using _force_lr_type =
    std::array<float, 3>;
  _force_lr_type force_lr;
  using _pos_lr_type =
    std::array<float, 3>;
  _pos_lr_type pos_lr;
  using _mode_rr_type =
    uint32_t;
  _mode_rr_type mode_rr;
  using _force_rr_type =
    std::array<float, 3>;
  _force_rr_type force_rr;
  using _pos_rr_type =
    std::array<float, 3>;
  _pos_rr_type pos_rr;

  // setters for named parameter idiom
  Type & set__mode_lf(
    const uint32_t & _arg)
  {
    this->mode_lf = _arg;
    return *this;
  }
  Type & set__force_lf(
    const std::array<float, 3> & _arg)
  {
    this->force_lf = _arg;
    return *this;
  }
  Type & set__pos_lf(
    const std::array<float, 3> & _arg)
  {
    this->pos_lf = _arg;
    return *this;
  }
  Type & set__mode_rf(
    const uint32_t & _arg)
  {
    this->mode_rf = _arg;
    return *this;
  }
  Type & set__force_rf(
    const std::array<float, 3> & _arg)
  {
    this->force_rf = _arg;
    return *this;
  }
  Type & set__pos_rf(
    const std::array<float, 3> & _arg)
  {
    this->pos_rf = _arg;
    return *this;
  }
  Type & set__mode_lr(
    const uint32_t & _arg)
  {
    this->mode_lr = _arg;
    return *this;
  }
  Type & set__force_lr(
    const std::array<float, 3> & _arg)
  {
    this->force_lr = _arg;
    return *this;
  }
  Type & set__pos_lr(
    const std::array<float, 3> & _arg)
  {
    this->pos_lr = _arg;
    return *this;
  }
  Type & set__mode_rr(
    const uint32_t & _arg)
  {
    this->mode_rr = _arg;
    return *this;
  }
  Type & set__force_rr(
    const std::array<float, 3> & _arg)
  {
    this->force_rr = _arg;
    return *this;
  }
  Type & set__pos_rr(
    const std::array<float, 3> & _arg)
  {
    this->pos_rr = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    msg_legrobot::msg::OrderLegState_<ContainerAllocator> *;
  using ConstRawPtr =
    const msg_legrobot::msg::OrderLegState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<msg_legrobot::msg::OrderLegState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<msg_legrobot::msg::OrderLegState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      msg_legrobot::msg::OrderLegState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<msg_legrobot::msg::OrderLegState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      msg_legrobot::msg::OrderLegState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<msg_legrobot::msg::OrderLegState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<msg_legrobot::msg::OrderLegState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<msg_legrobot::msg::OrderLegState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__msg_legrobot__msg__OrderLegState
    std::shared_ptr<msg_legrobot::msg::OrderLegState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__msg_legrobot__msg__OrderLegState
    std::shared_ptr<msg_legrobot::msg::OrderLegState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const OrderLegState_ & other) const
  {
    if (this->mode_lf != other.mode_lf) {
      return false;
    }
    if (this->force_lf != other.force_lf) {
      return false;
    }
    if (this->pos_lf != other.pos_lf) {
      return false;
    }
    if (this->mode_rf != other.mode_rf) {
      return false;
    }
    if (this->force_rf != other.force_rf) {
      return false;
    }
    if (this->pos_rf != other.pos_rf) {
      return false;
    }
    if (this->mode_lr != other.mode_lr) {
      return false;
    }
    if (this->force_lr != other.force_lr) {
      return false;
    }
    if (this->pos_lr != other.pos_lr) {
      return false;
    }
    if (this->mode_rr != other.mode_rr) {
      return false;
    }
    if (this->force_rr != other.force_rr) {
      return false;
    }
    if (this->pos_rr != other.pos_rr) {
      return false;
    }
    return true;
  }
  bool operator!=(const OrderLegState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct OrderLegState_

// alias to use template instance with default allocator
using OrderLegState =
  msg_legrobot::msg::OrderLegState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace msg_legrobot

#endif  // MSG_LEGROBOT__MSG__DETAIL__ORDER_LEG_STATE__STRUCT_HPP_
