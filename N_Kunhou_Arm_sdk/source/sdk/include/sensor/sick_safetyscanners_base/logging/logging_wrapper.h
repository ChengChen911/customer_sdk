#ifndef SICK_SAFETYSCANNERS_BASE_LOGGING_LOGGINGWRAPPER_H
#define SICK_SAFETYSCANNERS_BASE_LOGGING_LOGGINGWRAPPER_H

#define NO_ROS_LOGGING
#ifdef NO_ROS_LOGGING
// clang-format off
#include <cstdio>

#define ROS_INFO(...) std::printf(__VA_ARGS__)
#define ROS_WARN(...) std::printf(__VA_ARGS__)
#define ROS_ERROR(...) std::fprintf(stderr, __VA_ARGS__)

#else

#include <ros/console.h>

//clang-format on

#endif // NO_ROS_LOGGING

#endif // SICK_SAFETYSCANNERS_BASE_LOGGING_LOGGINGWRAPPER_H
