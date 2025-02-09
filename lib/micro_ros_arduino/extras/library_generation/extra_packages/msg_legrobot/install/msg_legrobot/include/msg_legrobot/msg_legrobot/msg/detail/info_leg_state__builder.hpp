// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from msg_legrobot:msg/InfoLegState.idl
// generated code does not contain a copyright notice

#ifndef MSG_LEGROBOT__MSG__DETAIL__INFO_LEG_STATE__BUILDER_HPP_
#define MSG_LEGROBOT__MSG__DETAIL__INFO_LEG_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "msg_legrobot/msg/detail/info_leg_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace msg_legrobot
{

namespace msg
{

namespace builder
{

class Init_InfoLegState_fault
{
public:
  explicit Init_InfoLegState_fault(::msg_legrobot::msg::InfoLegState & msg)
  : msg_(msg)
  {}
  ::msg_legrobot::msg::InfoLegState fault(::msg_legrobot::msg::InfoLegState::_fault_type arg)
  {
    msg_.fault = std::move(arg);
    return std::move(msg_);
  }

private:
  ::msg_legrobot::msg::InfoLegState msg_;
};

class Init_InfoLegState_gyro_imu
{
public:
  explicit Init_InfoLegState_gyro_imu(::msg_legrobot::msg::InfoLegState & msg)
  : msg_(msg)
  {}
  Init_InfoLegState_fault gyro_imu(::msg_legrobot::msg::InfoLegState::_gyro_imu_type arg)
  {
    msg_.gyro_imu = std::move(arg);
    return Init_InfoLegState_fault(msg_);
  }

private:
  ::msg_legrobot::msg::InfoLegState msg_;
};

class Init_InfoLegState_acc_imu
{
public:
  explicit Init_InfoLegState_acc_imu(::msg_legrobot::msg::InfoLegState & msg)
  : msg_(msg)
  {}
  Init_InfoLegState_gyro_imu acc_imu(::msg_legrobot::msg::InfoLegState::_acc_imu_type arg)
  {
    msg_.acc_imu = std::move(arg);
    return Init_InfoLegState_gyro_imu(msg_);
  }

private:
  ::msg_legrobot::msg::InfoLegState msg_;
};

class Init_InfoLegState_vel_body
{
public:
  explicit Init_InfoLegState_vel_body(::msg_legrobot::msg::InfoLegState & msg)
  : msg_(msg)
  {}
  Init_InfoLegState_acc_imu vel_body(::msg_legrobot::msg::InfoLegState::_vel_body_type arg)
  {
    msg_.vel_body = std::move(arg);
    return Init_InfoLegState_acc_imu(msg_);
  }

private:
  ::msg_legrobot::msg::InfoLegState msg_;
};

class Init_InfoLegState_pos_body
{
public:
  explicit Init_InfoLegState_pos_body(::msg_legrobot::msg::InfoLegState & msg)
  : msg_(msg)
  {}
  Init_InfoLegState_vel_body pos_body(::msg_legrobot::msg::InfoLegState::_pos_body_type arg)
  {
    msg_.pos_body = std::move(arg);
    return Init_InfoLegState_vel_body(msg_);
  }

private:
  ::msg_legrobot::msg::InfoLegState msg_;
};

class Init_InfoLegState_pos_rr
{
public:
  explicit Init_InfoLegState_pos_rr(::msg_legrobot::msg::InfoLegState & msg)
  : msg_(msg)
  {}
  Init_InfoLegState_pos_body pos_rr(::msg_legrobot::msg::InfoLegState::_pos_rr_type arg)
  {
    msg_.pos_rr = std::move(arg);
    return Init_InfoLegState_pos_body(msg_);
  }

private:
  ::msg_legrobot::msg::InfoLegState msg_;
};

class Init_InfoLegState_force_rr
{
public:
  explicit Init_InfoLegState_force_rr(::msg_legrobot::msg::InfoLegState & msg)
  : msg_(msg)
  {}
  Init_InfoLegState_pos_rr force_rr(::msg_legrobot::msg::InfoLegState::_force_rr_type arg)
  {
    msg_.force_rr = std::move(arg);
    return Init_InfoLegState_pos_rr(msg_);
  }

private:
  ::msg_legrobot::msg::InfoLegState msg_;
};

class Init_InfoLegState_pos_lr
{
public:
  explicit Init_InfoLegState_pos_lr(::msg_legrobot::msg::InfoLegState & msg)
  : msg_(msg)
  {}
  Init_InfoLegState_force_rr pos_lr(::msg_legrobot::msg::InfoLegState::_pos_lr_type arg)
  {
    msg_.pos_lr = std::move(arg);
    return Init_InfoLegState_force_rr(msg_);
  }

private:
  ::msg_legrobot::msg::InfoLegState msg_;
};

class Init_InfoLegState_force_lr
{
public:
  explicit Init_InfoLegState_force_lr(::msg_legrobot::msg::InfoLegState & msg)
  : msg_(msg)
  {}
  Init_InfoLegState_pos_lr force_lr(::msg_legrobot::msg::InfoLegState::_force_lr_type arg)
  {
    msg_.force_lr = std::move(arg);
    return Init_InfoLegState_pos_lr(msg_);
  }

private:
  ::msg_legrobot::msg::InfoLegState msg_;
};

class Init_InfoLegState_pos_rf
{
public:
  explicit Init_InfoLegState_pos_rf(::msg_legrobot::msg::InfoLegState & msg)
  : msg_(msg)
  {}
  Init_InfoLegState_force_lr pos_rf(::msg_legrobot::msg::InfoLegState::_pos_rf_type arg)
  {
    msg_.pos_rf = std::move(arg);
    return Init_InfoLegState_force_lr(msg_);
  }

private:
  ::msg_legrobot::msg::InfoLegState msg_;
};

class Init_InfoLegState_force_rf
{
public:
  explicit Init_InfoLegState_force_rf(::msg_legrobot::msg::InfoLegState & msg)
  : msg_(msg)
  {}
  Init_InfoLegState_pos_rf force_rf(::msg_legrobot::msg::InfoLegState::_force_rf_type arg)
  {
    msg_.force_rf = std::move(arg);
    return Init_InfoLegState_pos_rf(msg_);
  }

private:
  ::msg_legrobot::msg::InfoLegState msg_;
};

class Init_InfoLegState_pos_lf
{
public:
  explicit Init_InfoLegState_pos_lf(::msg_legrobot::msg::InfoLegState & msg)
  : msg_(msg)
  {}
  Init_InfoLegState_force_rf pos_lf(::msg_legrobot::msg::InfoLegState::_pos_lf_type arg)
  {
    msg_.pos_lf = std::move(arg);
    return Init_InfoLegState_force_rf(msg_);
  }

private:
  ::msg_legrobot::msg::InfoLegState msg_;
};

class Init_InfoLegState_force_lf
{
public:
  Init_InfoLegState_force_lf()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_InfoLegState_pos_lf force_lf(::msg_legrobot::msg::InfoLegState::_force_lf_type arg)
  {
    msg_.force_lf = std::move(arg);
    return Init_InfoLegState_pos_lf(msg_);
  }

private:
  ::msg_legrobot::msg::InfoLegState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::msg_legrobot::msg::InfoLegState>()
{
  return msg_legrobot::msg::builder::Init_InfoLegState_force_lf();
}

}  // namespace msg_legrobot

#endif  // MSG_LEGROBOT__MSG__DETAIL__INFO_LEG_STATE__BUILDER_HPP_
