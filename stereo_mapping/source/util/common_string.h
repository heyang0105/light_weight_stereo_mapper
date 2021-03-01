#ifndef SRC_UTIL_STRING_H_
#define SRC_UTIL_STRING_H_

#include <string>
#include <vector>

namespace UTIL {
// Format string by replacing embedded format specifiers with their respective
// values, see `printf` for more details. This is a modified implementation
// of Google's BSD-licensed StringPrintf function.
std::string StringPrintf(const char* format, ...);
}
#endif
