#pragma once

namespace librmcs::utility {

#ifdef _MSC_VER
# define PACKED_STRUCT(...) __pragma(pack(push, 1)) struct __VA_ARGS__ __pragma(pack(pop))
#elif defined(__GNUC__)
# define PACKED_STRUCT(...) struct __attribute__((packed)) __VA_ARGS__ //用于告诉编译器不要进行字节对齐填充。因为硬件发送的数据是紧凑排列的，如果编译器为了优化访问速度插入了填充字节，会导致数据错位。
#endif

constexpr static inline bool is_linux() {
#ifdef __linux__
    return true;
#else
    return false;
#endif
}

constexpr static inline bool is_windows() {
#if defined(_WIN32) || defined(WIN32)
    return true;
#else
    return false;
#endif
}

} // namespace librmcs::utility