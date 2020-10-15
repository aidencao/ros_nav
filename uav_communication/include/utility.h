#ifndef GS_UTILITY_H
#define GS_UTILITY_H

#include "uavtype.h"
#include "uav_inc.h"

inline void PRINT_STRING_TO_BINARY(const string& str) {
    for (auto it : str)
        printf("%02x ", (UINT8)it);
    printf("\n");
}

//字符串分割函数
static std::vector<std::string>
split(std::string str, std::string pattern)
{
    std::string::size_type pos;
    std::vector<std::string> result;
    str += pattern; //扩展字符串以方便操作
    int size = str.size();

    for (int i = 0; i < size; i++)
    {
        pos = str.find(pattern, i);
        if (pos < size)
        {
            std::string s = str.substr(i, pos - i);
            result.push_back(s);
            i = pos + pattern.size() - 1;
        }
    }
    return result;
}


#endif
