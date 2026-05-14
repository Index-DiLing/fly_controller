#pragma once

#include "stm32f4xx.h"
#include <cstring>

namespace dl
{
    template<typename T>
    uint8_t removeOutliers(T data[], uint16_t size){
        if (size < 4) {
            return size;  // 数据太少，无法计算四分位数
        }

        // 创建临时数组用于排序
        T* sorted = new T[size];
        std::memcpy(sorted, data, size * sizeof(T));
        
        // 冒泡排序（用于小数据集）
        for (uint16_t i = 0; i < size - 1; i++) {
            for (uint16_t j = 0; j < size - i - 1; j++) {
                if (sorted[j] > sorted[j + 1]) {
                    T temp = sorted[j];
                    sorted[j] = sorted[j + 1];
                    sorted[j + 1] = temp;
                }
            }
        }
        
        // 计算第一四分位数（Q1）- 25% 位置
        uint16_t q1_idx = size / 4;
        T q1 = sorted[q1_idx];
        
        // 计算第三四分位数（Q3）- 75% 位置
        uint16_t q3_idx = (3 * size) / 4;
        T q3 = sorted[q3_idx];
        
        // 计算 IQR 和离群范围（使用2.0倍IQR以保留更多数据）
        T iqr = q3 - q1;
        T lower_bound = q1 - (iqr * 2.0);
        T upper_bound = q3 + (iqr * 2.0);
        
        delete[] sorted;
        
        // 移除离群点，保持数组连续
        uint16_t valid_count = 0;
        for (uint16_t i = 0; i < size; i++) {
            if (data[i] >= lower_bound && data[i] <= upper_bound) {
                data[valid_count++] = data[i];
            }
        }
        
        return valid_count;
    }
    template<typename T>
    T mean(T data[], uint16_t size){
        T sum = 0;
        for (uint16_t i = 0; i < size; i++) {
            sum += data[i];
        }
        return sum / size;
    }
    template<typename T>
    T sum(T data[], uint16_t size){
        T sum = 0;
        for (uint16_t i = 0; i < size; i++) {
            sum += data[i];
        }
        return sum;
    }

    template<typename T>
    T max(T data[], uint16_t size){
        T max_val = data[0];
        for (uint16_t i = 1; i < size; i++) {
            if (data[i] > max_val) {
                max_val = data[i];
            }
        return max_val;
        }
    }
    template<typename T>
    T min(T data[], uint16_t size){
        T min_val = data[0];
        for (uint16_t i = 1; i < size; i++) {
            if (data[i] < min_val) {
                min_val = data[i];
            }
        }
    }
} // namespace dl
