#ifndef VISION_PIXEL_CLASSIFIER_H_
#define VISION_PIXEL_CLASSIFIER_H_

#include <cstdint>
#include <algorithm>
#include <cstring>

enum vision_binarization_mode_enum
{
    VISION_BINARIZATION_PRECOMPUTED = 0,
    VISION_BINARIZATION_ADAPTIVE_MEAN = 1,
    VISION_BINARIZATION_ADAPTIVE_FULL = 2
};

// 像素分类器：统一预计算二值图查表 / 按需自适应阈值两种策略。
// 生命周期：每帧在 process_step 中构造，巡线结束后析构。
// 线程模型：仅 vision 处理线程使用，无需加锁。
//
// PRECOMPUTED 模式：构造后直接使用 is_white()，查预计算二值图。
//
// ADAPTIVE_MEAN 模式两阶段：
//   1. 构造 → 巡线（is_white 内部自动收集局部阈值到 threshold_sum/count）
//   2. 巡线结束 → average_threshold() 取均值 → 外部用均值做全图二值化
struct PixelClassifier
{
    int mode;
    const uint8_t *gray;
    int width;
    int height;
    int window_size;
    int constant;

    const uint8_t *binary;

    uint8_t *cache_results;
    uint8_t *cache_computed;
    int cache_stride_bytes;

    uint64_t threshold_sum;
    int threshold_count;

    inline bool is_white(int x, int y)
    {
        if (mode == VISION_BINARIZATION_PRECOMPUTED)
        {
            return binary[y * width + x] > 127;
        }

        const int idx = y * width + x;

        const int byte_idx = idx >> 3;
        const int bit_mask = 1 << (idx & 7);
        if (cache_computed[byte_idx] & bit_mask)
        {
            return cache_results[idx] > 127;
        }

        const int half = window_size / 2;
        const int x0 = std::max(1, x - half);
        const int x1 = std::min(width - 2, x + half);
        const int y0 = std::max(1, y - half);
        const int y1 = std::min(height - 2, y + half);

        int sum = 0;
        for (int yy = y0; yy <= y1; ++yy)
        {
            for (int xx = x0; xx <= x1; ++xx)
            {
                sum += gray[yy * width + xx];
            }
        }
        const int count = (x1 - x0 + 1) * (y1 - y0 + 1);
        const int mean = sum / count;
        int threshold = mean - constant;
        if (threshold < 0) threshold = 0;
        if (threshold > 255) threshold = 255;

        threshold_sum += static_cast<uint64_t>(threshold);
        threshold_count++;

        const bool white = gray[idx] > static_cast<uint8_t>(threshold);
        cache_results[idx] = white ? 255 : 0;
        cache_computed[byte_idx] |= static_cast<uint8_t>(bit_mask);

        return white;
    }

    inline bool is_wall(int x, int y, bool wall_is_white)
    {
        return wall_is_white ? is_white(x, y) : !is_white(x, y);
    }

    inline bool is_path(int x, int y, bool wall_is_white)
    {
        return !is_wall(x, y, wall_is_white);
    }

    inline uint8_t average_threshold() const
    {
        if (threshold_count == 0) return 127;
        return static_cast<uint8_t>(threshold_sum / static_cast<uint64_t>(threshold_count));
    }

    void reset_cache()
    {
        if (mode == VISION_BINARIZATION_ADAPTIVE_MEAN && cache_computed != nullptr)
        {
            std::memset(cache_computed, 0, static_cast<size_t>(cache_stride_bytes) * static_cast<size_t>(height));
        }
        threshold_sum = 0;
        threshold_count = 0;
    }

    static PixelClassifier make_precomputed(const uint8_t *gray_img,
                                            const uint8_t *binary_img,
                                            int w, int h)
    {
        PixelClassifier c{};
        c.mode = VISION_BINARIZATION_PRECOMPUTED;
        c.gray = gray_img;
        c.binary = binary_img;
        c.width = w;
        c.height = h;
        return c;
    }

    static PixelClassifier make_adaptive(const uint8_t *gray_img,
                                         int w, int h,
                                         int window, int constant_val,
                                         uint8_t *cache_results,
                                         uint8_t *cache_computed)
    {
        PixelClassifier c{};
        c.mode = VISION_BINARIZATION_ADAPTIVE_MEAN;
        c.gray = gray_img;
        c.width = w;
        c.height = h;
        c.window_size = window;
        c.constant = constant_val;
        c.cache_results = cache_results;
        c.cache_computed = cache_computed;
        c.cache_stride_bytes = (w + 7) / 8;
        c.threshold_sum = 0;
        c.threshold_count = 0;
        return c;
    }
};

#endif
