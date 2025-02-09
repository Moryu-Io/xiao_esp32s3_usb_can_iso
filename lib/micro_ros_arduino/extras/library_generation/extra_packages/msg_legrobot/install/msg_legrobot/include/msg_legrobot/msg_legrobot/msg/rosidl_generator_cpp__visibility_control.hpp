// generated from rosidl_generator_cpp/resource/rosidl_generator_cpp__visibility_control.hpp.in
// generated code does not contain a copyright notice

#ifndef MSG_LEGROBOT__MSG__ROSIDL_GENERATOR_CPP__VISIBILITY_CONTROL_HPP_
#define MSG_LEGROBOT__MSG__ROSIDL_GENERATOR_CPP__VISIBILITY_CONTROL_HPP_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROSIDL_GENERATOR_CPP_EXPORT_msg_legrobot __attribute__ ((dllexport))
    #define ROSIDL_GENERATOR_CPP_IMPORT_msg_legrobot __attribute__ ((dllimport))
  #else
    #define ROSIDL_GENERATOR_CPP_EXPORT_msg_legrobot __declspec(dllexport)
    #define ROSIDL_GENERATOR_CPP_IMPORT_msg_legrobot __declspec(dllimport)
  #endif
  #ifdef ROSIDL_GENERATOR_CPP_BUILDING_DLL_msg_legrobot
    #define ROSIDL_GENERATOR_CPP_PUBLIC_msg_legrobot ROSIDL_GENERATOR_CPP_EXPORT_msg_legrobot
  #else
    #define ROSIDL_GENERATOR_CPP_PUBLIC_msg_legrobot ROSIDL_GENERATOR_CPP_IMPORT_msg_legrobot
  #endif
#else
  #define ROSIDL_GENERATOR_CPP_EXPORT_msg_legrobot __attribute__ ((visibility("default")))
  #define ROSIDL_GENERATOR_CPP_IMPORT_msg_legrobot
  #if __GNUC__ >= 4
    #define ROSIDL_GENERATOR_CPP_PUBLIC_msg_legrobot __attribute__ ((visibility("default")))
  #else
    #define ROSIDL_GENERATOR_CPP_PUBLIC_msg_legrobot
  #endif
#endif

#ifdef __cplusplus
}
#endif

#endif  // MSG_LEGROBOT__MSG__ROSIDL_GENERATOR_CPP__VISIBILITY_CONTROL_HPP_
