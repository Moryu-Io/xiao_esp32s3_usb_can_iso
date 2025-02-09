// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from msg_legrobot:msg/OrderLegState.idl
// generated code does not contain a copyright notice

#ifndef MSG_LEGROBOT__MSG__DETAIL__ORDER_LEG_STATE__BUILDER_HPP_
#define MSG_LEGROBOT__MSG__DETAIL__ORDER_LEG_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "msg_legrobot/msg/detail/order_leg_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace msg_legrobot
{

namespace msg
{

namespace builder
{

class Init_OrderLegState_pos_rr
{
public:
  explicit Init_OrderLegState_pos_rr(::msg_legrobot::msg::OrderLegState & msg)
  : msg_(msg)
  {}
  ::msg_legrobot::msg::OrderLegState pos_rr(::msg_legrobot::msg::OrderLegState::_pos_rr_type arg)
  {
    msg_.pos_rr = std::move(arg);
    return std::move(msg_);
  }

private:
  ::msg_legrobot::msg::OrderLegState msg_;
};

class Init_OrderLegState_force_rr
{
public:
  explicit Init_OrderLegState_force_rr(::msg_legrobot::msg::OrderLegState & msg)
  : msg_(msg)
  {}
  Init_OrderLegState_pos_rr force_rr(::msg_legrobot::msg::OrderLegState::_force_rr_type arg)
  {
    msg_.force_rr = std::move(arg);
    return Init_OrderLegState_pos_rr(msg_);
  }

private:
  ::msg_legrobot::msg::OrderLegState msg_;
};

class Init_OrderLegState_mode_rr
{
public:
  explicit Init_OrderLegState_mode_rr(::msg_legrobot::msg::OrderLegState & msg)
  : msg_(msg)
  {}
  Init_OrderLegState_force_rr mode_rr(::msg_legrobot::msg::OrderLegState::_mode_rr_type arg)
  {
    msg_.mode_rr = std::move(arg);
    return Init_OrderLegState_force_rr(msg_);
  }

private:
  ::msg_legrobot::msg::OrderLegState msg_;
};

class Init_OrderLegState_pos_lr
{
public:
  explicit Init_OrderLegState_pos_lr(::msg_legrobot::msg::OrderLegState & msg)
  : msg_(msg)
  {}
  Init_OrderLegState_mode_rr pos_lr(::msg_legrobot::msg::OrderLegState::_pos_lr_type arg)
  {
    msg_.pos_lr = std::move(arg);
    return Init_OrderLegState_mode_rr(msg_);
  }

private:
  ::msg_legrobot::msg::OrderLegState msg_;
};

class Init_OrderLegState_force_lr
{
public:
  explicit Init_OrderLegState_force_lr(::msg_legrobot::msg::OrderLegState & msg)
  : msg_(msg)
  {}
  Init_OrderLegState_pos_lr force_lr(::msg_legrobot::msg::OrderLegState::_force_lr_type arg)
  {
    msg_.force_lr = std::move(arg);
    return Init_OrderLegState_pos_lr(msg_);
  }

private:
  ::msg_legrobot::msg::OrderLegState msg_;
};

class Init_OrderLegState_mode_lr
{
public:
  explicit Init_OrderLegState_mode_lr(::msg_legrobot::msg::OrderLegState & msg)
  : msg_(msg)
  {}
  Init_OrderLegState_force_lr mode_lr(::msg_legrobot::msg::OrderLegState::_mode_lr_type arg)
  {
    msg_.mode_lr = std::move(arg);
    return Init_OrderLegState_force_lr(msg_);
  }

private:
  ::msg_legrobot::msg::OrderLegState msg_;
};

class Init_OrderLegState_pos_rf
{
public:
  explicit Init_OrderLegState_pos_rf(::msg_legrobot::msg::OrderLegState & msg)
  : msg_(msg)
  {}
  Init_OrderLegState_mode_lr pos_rf(::msg_legrobot::msg::OrderLegState::_pos_rf_type arg)
  {
    msg_.pos_rf = std::move(arg);
    return Init_OrderLegState_mode_lr(msg_);
  }

private:
  ::msg_legrobot::msg::OrderLegState msg_;
};

class Init_OrderLegState_force_rf
{
public:
  explicit Init_OrderLegState_force_rf(::msg_legrobot::msg::OrderLegState & msg)
  : msg_(msg)
  {}
  Init_OrderLegState_pos_rf force_rf(::msg_legrobot::msg::OrderLegState::_force_rf_type arg)
  {
    msg_.force_rf = std::move(arg);
    return Init_OrderLegState_pos_rf(msg_);
  }

private:
  ::msg_legrobot::msg::OrderLegState msg_;
};

class Init_OrderLegState_mode_rf
{
public:
  explicit Init_OrderLegState_mode_rf(::msg_legrobot::msg::OrderLegState & msg)
  : msg_(msg)
  {}
  Init_OrderLegState_force_rf mode_rf(::msg_legrobot::msg::OrderLegState::_mode_rf_type arg)
  {
    msg_.mode_rf = std::move(arg);
    return Init_OrderLegState_force_rf(msg_);
  }

private:
  ::msg_legrobot::msg::OrderLegState msg_;
};

class Init_OrderLegState_pos_lf
{
public:
  explicit Init_OrderLegState_pos_lf(::msg_legrobot::msg::OrderLegState & msg)
  : msg_(msg)
  {}
  Init_OrderLegState_mode_rf pos_lf(::msg_legrobot::msg::OrderLegState::_pos_lf_type arg)
  {
    msg_.pos_lf = std::move(arg);
    return Init_OrderLegState_mode_rf(msg_);
  }

private:
  ::msg_legrobot::msg::OrderLegState msg_;
};

class Init_OrderLegState_force_lf
{
public:
  explicit Init_OrderLegState_force_lf(::msg_legrobot::msg::OrderLegState & msg)
  : msg_(msg)
  {}
  Init_OrderLegState_pos_lf force_lf(::msg_legrobot::msg::OrderLegState::_force_lf_type arg)
  {
    msg_.force_lf = std::move(arg);
    return Init_OrderLegState_pos_lf(msg_);
  }

private:
  ::msg_legrobot::msg::OrderLegState msg_;
};

class Init_OrderLegState_mode_lf
{
public:
  Init_OrderLegState_mode_lf()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_OrderLegState_force_lf mode_lf(::msg_legrobot::msg::OrderLegState::_mode_lf_type arg)
  {
    msg_.mode_lf = std::move(arg);
    return Init_OrderLegState_force_lf(msg_);
  }

private:
  ::msg_legrobot::msg::OrderLegState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::msg_legrobot::msg::OrderLegState>()
{
  return msg_legrobot::msg::builder::Init_OrderLegState_mode_lf();
}

}  // namespace msg_legrobot

#endif  // MSG_LEGROBOT__MSG__DETAIL__ORDER_LEG_STATE__BUILDER_HPP_
