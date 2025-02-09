// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from msg_legrobotrx:msg/Testmessage.idl
// generated code does not contain a copyright notice

#ifndef MSG_LEGROBOTRX__MSG__DETAIL__TESTMESSAGE__BUILDER_HPP_
#define MSG_LEGROBOTRX__MSG__DETAIL__TESTMESSAGE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "msg_legrobotrx/msg/detail/testmessage__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace msg_legrobotrx
{

namespace msg
{

namespace builder
{

class Init_Testmessage_y
{
public:
  explicit Init_Testmessage_y(::msg_legrobotrx::msg::Testmessage & msg)
  : msg_(msg)
  {}
  ::msg_legrobotrx::msg::Testmessage y(::msg_legrobotrx::msg::Testmessage::_y_type arg)
  {
    msg_.y = std::move(arg);
    return std::move(msg_);
  }

private:
  ::msg_legrobotrx::msg::Testmessage msg_;
};

class Init_Testmessage_x
{
public:
  Init_Testmessage_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Testmessage_y x(::msg_legrobotrx::msg::Testmessage::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_Testmessage_y(msg_);
  }

private:
  ::msg_legrobotrx::msg::Testmessage msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::msg_legrobotrx::msg::Testmessage>()
{
  return msg_legrobotrx::msg::builder::Init_Testmessage_x();
}

}  // namespace msg_legrobotrx

#endif  // MSG_LEGROBOTRX__MSG__DETAIL__TESTMESSAGE__BUILDER_HPP_
