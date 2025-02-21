#pragma once

#include <lvr2/util/Logging.hpp>
#include <format>
#include <filesystem>

#define _LOG(LEVEL, ...)\
    do {\
        lvr2::logout::get() << LEVEL << "[" << __func__ << "] " << std::format(__VA_ARGS__) << lvr2::endl;\
    }\
    while(false)

#define LOG_INFO(...) _LOG(lvr2::info, __VA_ARGS__)
#define LOG_WARNING(...) _LOG(lvr2::warning, __VA_ARGS__)
#define LOG_ERROR(...) _LOG(lvr2::error, __VA_ARGS__)


// Custom formatters

// C++26 has support for formatting paths
#if __cplusplus <= 202302L
template <>
struct std::formatter<std::filesystem::path>
: std::formatter<std::string>
{
    auto format(const std::filesystem::path& path, format_context& ctx) const
    {
        return std::formatter<std::string>::format(
            std::format("{}", path.string()), ctx
        );
    }

};

#endif
