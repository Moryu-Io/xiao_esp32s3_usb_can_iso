// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from msg_legrobot:msg/Testmessage.idl
// generated code does not contain a copyright notice

#ifndef MSG_LEGROBOT__MSG__DETAIL__TESTMESSAGE__TRAITS_HPP_
#define MSG_LEGROBOT__MSG__DETAIL__TESTMESSAGE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "msg_legrobot/msg/detail/testmessage__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace msg_legrobot
{

namespace msg
{

inline void to_flow_style_yaml(
  const Testmessage & msg,
  std::ostream & out)
{
  out << "{";
  // member: x
  {
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << ", ";
  }

  // member: y
  {
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Testmessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << "\n";
  }

  // member: y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Testmessage & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace msg_legrobot

namespace rosidl_generator_traits
{

[[deprecated("use msg_legrobot::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const msg_legrobot::msg::Testmessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  msg_legrobot::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use msg_legrobot::msg::to_yaml() instead")]]
inline std::string to_yaml(const msg_legrobot::msg::Testmessage & msg)
{
  return msg_legrobot::msg::to_yaml(msg);
}

template<>
inline const char * data_type<msg_legrobot::msg::Testmessage>()
{
  return "msg_legrobot::msg::Testmessage";
}

template<>
inline const char * name<msg_legrobot::msg::Testmessage>()
{
  return "msg_legrobot/msg/Testmessage";
}

template<>
struct has_fixed_size<msg_legrobot::msg::Testmessage>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<msg_legrobot::msg::Testmessage>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<msg_legrobot::msg::Testmessage>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MSG_LEGROBOT__MSG__DETAIL__TESTMESSAGE__TRAITS_HPP_
