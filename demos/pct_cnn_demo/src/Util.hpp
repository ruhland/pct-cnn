#ifndef UTIL_HPP_
#define UTIL_HPP_

#include <pcl/console/print.h>

#ifndef pr_fmt
#define pr_fmt(fmt) fmt
#endif

#define INFO(fmt, ...) PCL_INFO("[%s] " pr_fmt(fmt) "\n", __func__, \
                ##__VA_ARGS__)

#define ERROR(fmt, ...) PCL_ERROR("[%s:%s:%d]" pr_fmt(fmt) "\n", \
                __FILE__, __func__, __LINE__, ##__VA_ARGS__)

#endif
