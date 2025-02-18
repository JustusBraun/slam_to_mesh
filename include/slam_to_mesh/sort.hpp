#pragma once
#include <algorithm>
#include <filesystem>
#include <string>

/**
*   @brief Implements natural sorting of file paths.
*   
*   Only supports positive numbers.
*   file1.xyz is ordered before file10.xyz etc
*   
*   Example:
*   file2.ply
*   file1.xyz
*   file1.abc
*   file1.ply
*
*   =>
*
*   file1.abc
*   file1.ply
*   file1.xyz
*   file2.ply
*/
struct natural_compare
{

inline bool operator()(const std::filesystem::path& a, const std::filesystem::path& b)
{
    // Compare without extensions
    const std::string a_str = a.stem();
    const std::string b_str = b.stem();
    auto it_a = a_str.begin();
    auto it_b = b_str.begin();

    for(;it_a != a_str.end() && it_b != b_str.end();)
    {
        if (std::isdigit(*it_a) && std::isdigit(*it_b))
        {
            // Parse and compare numbers
            auto a_num_end = std::find_if_not(it_a, a_str.end(), [](char c){ return std::isdigit(c);});
            auto b_num_end = std::find_if_not(it_b, b_str.end(), [](char c){ return std::isdigit(c);});
            unsigned int a_num = std::stoi(std::string(it_a, a_num_end));
            unsigned int b_num = std::stoi(std::string(it_b, b_num_end));

            if (a_num == b_num)
            {
                it_a = a_num_end;
                it_b = b_num_end;
                continue;
            }
            else
            {
                return a_num < b_num;
            }
        }
        else if (!std::isdigit(*it_a) && !std::isdigit(*it_b))
        {
            // Normal string compare
            if (*it_a == *it_b)
            { 
                it_a++, it_b++;
                continue;
            }
            else
            {
                return *it_a < *it_b;
            }
        }
        else if(std::isdigit(*it_a))
        {
            return true;
        }
        else if (std::isdigit(*it_b)) {
            return false;
        }
    }

    // At this point the strings matched until one ended
    if (a_str.length() == b_str.length())
    {
        return a.extension().string() < b.extension().string();
    }

    return a_str.length() < b_str.length();
}

};
