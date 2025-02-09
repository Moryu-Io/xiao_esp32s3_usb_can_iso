// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from msg_legrobot:msg/OrderLegState.idl
// generated code does not contain a copyright notice

#ifndef MSG_LEGROBOT__MSG__DETAIL__ORDER_LEG_STATE__TRAITS_HPP_
#define MSG_LEGROBOT__MSG__DETAIL__ORDER_LEG_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "msg_legrobot/msg/detail/order_leg_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace msg_legrobot
{

namespace msg
{

inline void to_flow_style_yaml(
  const OrderLegState & msg,
  std::ostream & out)
{
  out << "{";
  // member: mode_lf
  {
    out << "mode_lf: ";
    rosidl_generator_traits::value_to_yaml(msg.mode_lf, out);
    out << ", ";
  }

  // member: force_lf
  {
    if (msg.force_lf.size() == 0) {
      out << "force_lf: []";
    } else {
      out << "force_lf: [";
      size_t pending_items = msg.force_lf.size();
      for (auto item : msg.force_lf) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: pos_lf
  {
    if (msg.pos_lf.size() == 0) {
      out << "pos_lf: []";
    } else {
      out << "pos_lf: [";
      size_t pending_items = msg.pos_lf.size();
      for (auto item : msg.pos_lf) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: mode_rf
  {
    out << "mode_rf: ";
    rosidl_generator_traits::value_to_yaml(msg.mode_rf, out);
    out << ", ";
  }

  // member: force_rf
  {
    if (msg.force_rf.size() == 0) {
      out << "force_rf: []";
    } else {
      out << "force_rf: [";
      size_t pending_items = msg.force_rf.size();
      for (auto item : msg.force_rf) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: pos_rf
  {
    if (msg.pos_rf.size() == 0) {
      out << "pos_rf: []";
    } else {
      out << "pos_rf: [";
      size_t pending_items = msg.pos_rf.size();
      for (auto item : msg.pos_rf) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: mode_lr
  {
    out << "mode_lr: ";
    rosidl_generator_traits::value_to_yaml(msg.mode_lr, out);
    out << ", ";
  }

  // member: force_lr
  {
    if (msg.force_lr.size() == 0) {
      out << "force_lr: []";
    } else {
      out << "force_lr: [";
      size_t pending_items = msg.force_lr.size();
      for (auto item : msg.force_lr) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: pos_lr
  {
    if (msg.pos_lr.size() == 0) {
      out << "pos_lr: []";
    } else {
      out << "pos_lr: [";
      size_t pending_items = msg.pos_lr.size();
      for (auto item : msg.pos_lr) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: mode_rr
  {
    out << "mode_rr: ";
    rosidl_generator_traits::value_to_yaml(msg.mode_rr, out);
    out << ", ";
  }

  // member: force_rr
  {
    if (msg.force_rr.size() == 0) {
      out << "force_rr: []";
    } else {
      out << "force_rr: [";
      size_t pending_items = msg.force_rr.size();
      for (auto item : msg.force_rr) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: pos_rr
  {
    if (msg.pos_rr.size() == 0) {
      out << "pos_rr: []";
    } else {
      out << "pos_rr: [";
      size_t pending_items = msg.pos_rr.size();
      for (auto item : msg.pos_rr) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const OrderLegState & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: mode_lf
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "mode_lf: ";
    rosidl_generator_traits::value_to_yaml(msg.mode_lf, out);
    out << "\n";
  }

  // member: force_lf
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.force_lf.size() == 0) {
      out << "force_lf: []\n";
    } else {
      out << "force_lf:\n";
      for (auto item : msg.force_lf) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: pos_lf
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.pos_lf.size() == 0) {
      out << "pos_lf: []\n";
    } else {
      out << "pos_lf:\n";
      for (auto item : msg.pos_lf) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: mode_rf
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "mode_rf: ";
    rosidl_generator_traits::value_to_yaml(msg.mode_rf, out);
    out << "\n";
  }

  // member: force_rf
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.force_rf.size() == 0) {
      out << "force_rf: []\n";
    } else {
      out << "force_rf:\n";
      for (auto item : msg.force_rf) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: pos_rf
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.pos_rf.size() == 0) {
      out << "pos_rf: []\n";
    } else {
      out << "pos_rf:\n";
      for (auto item : msg.pos_rf) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: mode_lr
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "mode_lr: ";
    rosidl_generator_traits::value_to_yaml(msg.mode_lr, out);
    out << "\n";
  }

  // member: force_lr
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.force_lr.size() == 0) {
      out << "force_lr: []\n";
    } else {
      out << "force_lr:\n";
      for (auto item : msg.force_lr) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: pos_lr
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.pos_lr.size() == 0) {
      out << "pos_lr: []\n";
    } else {
      out << "pos_lr:\n";
      for (auto item : msg.pos_lr) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: mode_rr
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "mode_rr: ";
    rosidl_generator_traits::value_to_yaml(msg.mode_rr, out);
    out << "\n";
  }

  // member: force_rr
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.force_rr.size() == 0) {
      out << "force_rr: []\n";
    } else {
      out << "force_rr:\n";
      for (auto item : msg.force_rr) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: pos_rr
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.pos_rr.size() == 0) {
      out << "pos_rr: []\n";
    } else {
      out << "pos_rr:\n";
      for (auto item : msg.pos_rr) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const OrderLegState & msg, bool use_flow_style = false)
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
  const msg_legrobot::msg::OrderLegState & msg,
  std::ostream & out, size_t indentation = 0)
{
  msg_legrobot::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use msg_legrobot::msg::to_yaml() instead")]]
inline std::string to_yaml(const msg_legrobot::msg::OrderLegState & msg)
{
  return msg_legrobot::msg::to_yaml(msg);
}

template<>
inline const char * data_type<msg_legrobot::msg::OrderLegState>()
{
  return "msg_legrobot::msg::OrderLegState";
}

template<>
inline const char * name<msg_legrobot::msg::OrderLegState>()
{
  return "msg_legrobot/msg/OrderLegState";
}

template<>
struct has_fixed_size<msg_legrobot::msg::OrderLegState>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<msg_legrobot::msg::OrderLegState>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<msg_legrobot::msg::OrderLegState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MSG_LEGROBOT__MSG__DETAIL__ORDER_LEG_STATE__TRAITS_HPP_
