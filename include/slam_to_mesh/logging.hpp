#pragma once

#include <lvr2/util/Logging.hpp>
#include <lvr2/geometry/BaseVector.hpp>
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
template <typename Scalar>
struct std::formatter<lvr2::BaseVector<Scalar>>
{
    constexpr auto parse(std::format_parse_context& ctx)
    {
        // Do not parse anything for now
        auto pos = ctx.begin();
        while(pos != ctx.end() && *pos != '}')
        {
            pos++;
        }
        return pos;
    }

    auto format(const lvr2::BaseVector<Scalar>& vec, format_context& ctx) const
    {
        return std::format_to(ctx.out(), "[{}, {}, {}]", vec.x, vec.y, vec.z);
    }

};

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
