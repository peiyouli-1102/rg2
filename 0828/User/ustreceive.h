#ifndef USTRECEIVE_H
#define USTRECEIVE_H

#include <stdint.h>
#include <stdbool.h>

// 定义可能的前缀类型
typedef enum {
    PREFIX_RIGHT,
    PREFIX_FORWARD,
	PREFIX_TURN,
	PREFIX_STEP,
    PREFIX_UNKNOWN
} prefix_type_t;

/**
 * @brief 从字符串中提取指定前缀后面的数字
 *        支持在数字前使用'$'表示负数，例如 "right$20--" → -20
 *
 * @param buf 输入缓冲区
 * @param buf_size 缓冲区大小
 * @param prefix_type 输出参数，返回匹配到的前缀类型
 * @return int32_t 匹配到的数字，如果没有匹配到则返回0
 */
int32_t extract_number(const uint8_t *buf, uint32_t buf_size, prefix_type_t *prefix_type);

#endif // USTRECEIVE_H
