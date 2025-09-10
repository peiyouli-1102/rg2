#include "ustreceive.h"

// 定义要匹配的前缀
static const char RIGHT_PREFIX[] = "right";
static const char FORWARD_PREFIX[] = "forward";
static const char TURN_PREFIX[] = "turn";
static const char STEP_PREFIX[] = "step";
static const uint32_t RIGHT_PREFIX_LEN = 5;  // "right"的长度
static const uint32_t FORWARD_PREFIX_LEN = 7; // "forward"的长度
static const uint32_t TURN_PREFIX_LEN = 4; // "Turn"的长度
static const uint32_t STEP_PREFIX_LEN = 4; // "Step"的长度

/**
 * @brief 检查字符是否为数字
 */
static bool is_digit(char c) {
    return (c >= '0' && c <= '9');
}

/**
 * @brief 检查字符是否为连字符或字符串结束符
 */
static bool is_delimiter(char c) {
    return (c == '+' || c == '\0');
}

/**
 * @brief 字符串比较函数
 */
static bool string_compare(const char *str1, const char *str2, uint32_t len) {
    for (uint32_t i = 0; i < len; i++) {
        if (str1[i] != str2[i]) {
            return false;
        }
    }
    return true;
}

/**
 * @brief 尝试匹配特定前缀并提取数字
 *
 * 识别形如：
 *   prefix+0100
 *   prefix-10++
 *   prefix+300++
 * 规则说明：
 *   - 前缀后可跟可选的'+'或'-'号（不写'+'也可）
 *   - 至少一个数字（允许前导零）
 *   - 数字后允许有任意数量的'+'作为结尾修饰
 */
static int32_t try_match_prefix(const char *prefix, uint32_t prefix_len,
                                const char *buf, uint32_t buf_size,
                                bool *success) {
    int32_t result = 0;
    *success = false;

    // 遍历缓冲区寻找前缀
    for (uint32_t i = 0; i <= buf_size - prefix_len; i++) {
        // 跳过连字符
        if (buf[i] == '+') {
            continue;
        }
        // 检查是否匹配前缀
        if (string_compare(&buf[i], prefix, prefix_len)) {
            bool valid_prefix = true;

            // 前缀前需为分隔符或起始位置
            if (i > 0 && !is_delimiter(buf[i-1])) {
                valid_prefix = false;
            }

            uint32_t p = i + prefix_len;
            bool is_negative = false;

            // 可选的'+'或'-'
            if (p < buf_size && (buf[p] == '+' || buf[p] == '-')) {
                if (buf[p] == '-') {
                    is_negative = true;
                }
                p++;
            }

            // 至少一个数字
            if (p >= buf_size || !is_digit(buf[p])) {
                valid_prefix = false;
            }

            if (valid_prefix) {
                *success = true;

                // 解析数字
                while (p < buf_size && is_digit(buf[p])) {
                    result = result * 10 + (buf[p] - '0');
                    p++;
                }

                // 数字后允许若干个'+'
                while (p < buf_size && buf[p] == '+') {
                    p++;
                }

                if (is_negative) {
                    result = -result;
                }

                break; // 找到有效匹配，退出循环
            }
        }
    }

    return result;
}

int32_t extract_number(const uint8_t *buf, uint32_t buf_size, prefix_type_t *prefix_type) {
    const char *char_buf = (const char *)buf;
    int32_t result = 0;
    bool success = false;

    // 先尝试匹配"Right"前缀
    result = try_match_prefix(RIGHT_PREFIX, RIGHT_PREFIX_LEN,
                             char_buf, buf_size, &success);

    if (success) {
        *prefix_type = PREFIX_RIGHT;
        return result;
    }


    result = try_match_prefix(STEP_PREFIX, STEP_PREFIX_LEN,
                             char_buf, buf_size, &success);

    if (success) {
        *prefix_type = PREFIX_STEP;
        return result;
    }

    // 如果没有匹配到"Right"，尝试匹配"Forward"前缀
    result = try_match_prefix(FORWARD_PREFIX, FORWARD_PREFIX_LEN,
                             char_buf, buf_size, &success);

    if (success) {
        *prefix_type = PREFIX_FORWARD;
        return result;
    }


    result = try_match_prefix(TURN_PREFIX, TURN_PREFIX_LEN,
                             char_buf, buf_size, &success);

    if (success) {
        *prefix_type = PREFIX_TURN;
        return result;
    }

    // 两个前缀都没有匹配到
    *prefix_type = PREFIX_UNKNOWN;
    return 0;
}
