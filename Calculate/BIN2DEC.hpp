//
// Created by 39894 on 2023/3/14.
//

#pragma once

#include <type_traits>

/// 二进制转FieldType类型
/// \tparam FieldType 字段数据类型
/// \param array 指向一块连续的内存(需保证内存对齐,即转double要保证指向的连续内存至少有八个字节)
/// \return 转换后的字段
template<typename FieldType> requires std::is_arithmetic_v<FieldType>
inline FieldType GetField(unsigned char* array)
{
    return *reinterpret_cast<FieldType*>(array);
}

template<typename OutType, typename InType, std::size_t Cut_Count> requires std::is_integral_v<OutType>&& std::is_integral_v<InType>
inline OutType BitCut(InType ul, int startIndex)
{
    return static_cast<OutType>(((ul >> startIndex) & (~(~0 << Cut_Count))));
}