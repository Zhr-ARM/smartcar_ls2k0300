# TinyClassifier NCNN 部署指南

## 模型信息

| 项目 | 值 |
|------|-----|
| 输入尺寸 | 64×64 RGB |
| 输入格式 | NCHW float32，已归一化 |
| 输出 | 6 个 logits，取 argmax 为类别索引 |
| 参数量 | 4,440 |
| NCNN 文件 | `tiny_classifier_fp32.ncnn.param` (1.7KB) + `.bin` (17KB) |

## 类别顺序

`labels.txt` 中的顺序即模型输出索引：

| 索引 | 类别 |
|------|------|
| 0 | ambulance |
| 1 | armored_car |
| 2 | bomb |
| 3 | gun |
| 4 | medicine |
| 5 | telescope |

---

## 1. 准备模型文件

将以下文件拷贝到板端：

```
tiny_classifier_fp32.ncnn.param
tiny_classifier_fp32.ncnn.bin
labels.txt
```

---

## 2. C++ 推理代码

### 2.1 完整单图推理示例

```cpp
#include <cstdio>
#include <vector>
#include <algorithm>
#include "net.h"

// 类别名称，与 labels.txt 顺序一致
const char* class_names[] = {
    "ambulance",
    "armored_car",
    "bomb",
    "gun",
    "medicine",
    "telescope"
};

int main(int argc, char** argv)
{
    if (argc != 2)
    {
        fprintf(stderr, "Usage: %s <image.jpg>\n", argv[0]);
        return -1;
    }

    // ======== 第1步：加载模型 ========
    ncnn::Net net;
    net.opt.use_vulkan_compute = false;  // 龙芯久久派无 GPU，必须关掉
    net.opt.num_threads = 1;             // 单线程即可，模型很小

    if (net.load_param("tiny_classifier_fp32.ncnn.param") != 0)
    {
        fprintf(stderr, "Failed to load param\n");
        return -1;
    }
    if (net.load_model("tiny_classifier_fp32.ncnn.bin") != 0)
    {
        fprintf(stderr, "Failed to load model\n");
        return -1;
    }

    // ======== 第2步：读取图像并预处理 ========
    // 用 OpenCV 读取
    cv::Mat bgr = cv::imread(argv[1], cv::IMREAD_COLOR);
    if (bgr.empty())
    {
        fprintf(stderr, "Failed to read image: %s\n", argv[1]);
        return -1;
    }

    // 转为 RGB（PyTorch 训练用的是 RGB）
    cv::Mat rgb;
    cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);

    // 缩放到模型输入尺寸
    cv::resize(rgb, rgb, cv::Size(64, 64));

    // 转换为 ncnn::Mat
    // from_pixels 将 uint8 [0,255] 转为 float32 [0,255]
    ncnn::Mat in = ncnn::Mat::from_pixels(rgb.data, ncnn::Mat::PIXEL_RGB, 64, 64);

    // ======== 第3步：归一化（关键！） ========
    //
    // ncnn 公式：output = (pixel - mean) * norm
    // pixel 范围是 [0, 255]，所以 mean 和 norm 必须适配这个范围。
    //
    // PyTorch 训练时的公式：
    //   output = (pixel/255 - mean_pt) / std_pt
    //
    // 要让两者等价：
    //   mean_ncnn = mean_pt * 255
    //   norm_ncnn = 1 / (std_pt * 255)
    //
    // RGB 顺序的均值：
    //   mean_pt = [0.485, 0.456, 0.406]
    //   mean_ncnn = [0.485*255=123.675, 0.456*255=116.28, 0.406*255=103.53]
    //   norm_ncnn = [1/(255*0.229)=1/58.395, 1/(255*0.224)=1/57.12, 1/(255*0.225)=1/57.375]

    const float mean_vals[3] = {123.675f, 116.28f,  103.53f};
    const float norm_vals[3] = {1.0f / 58.395f, 1.0f / 57.12f, 1.0f / 57.375f};
    in.substract_mean_normalize(mean_vals, norm_vals);

    // ======== 第4步：推理 ========
    ncnn::Extractor ex = net.create_extractor();
    ex.input("in0", in);

    ncnn::Mat out;
    ex.extract("out0", out);

    // out 是 6 个 float 的 logits
    // 找出最大值的索引
    int pred_class = 0;
    float max_score = out[0];
    for (int i = 1; i < 6; i++)
    {
        if (out[i] > max_score)
        {
            max_score = out[i];
            pred_class = i;
        }
    }

    printf("Predicted class: %d (%s), score: %.4f\n",
           pred_class, class_names[pred_class], max_score);

    // 打印所有类别的 logits
    printf("All logits:\n");
    for (int i = 0; i < 6; i++)
    {
        printf("  %s: %.4f\n", class_names[i], out[i]);
    }

    return 0;
}
```

### 2.2 如果不使用 OpenCV（用 stb_image 等）

```cpp
// 如果用 stb_image 或其他库读取，注意：
// 1. 确保是 RGB 三通道
// 2. 像素数据是 uint8 连续内存 (HWC 布局)
// 3. 已经 resize 到 64x64

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

int w, h, c;
unsigned char* pixels = stbi_load(argv[1], &w, &h, &c, 3);  // 强制 3 通道 RGB
if (!pixels) { /* error */ }

// 手动 resize 到 64x64...（或用 stb_image_resize）

ncnn::Mat in = ncnn::Mat::from_pixels(pixels, ncnn::Mat::PIXEL_RGB, 64, 64);
// 后续归一化和推理同上...

stbi_image_free(pixels);
```

### 2.3 BGR 模式（直接用 OpenCV 不转 RGB）

```cpp
// 如果不想做 BGR->RGB 转换，可以用 BGR 模式，
// 但 mean/norm 的顺序必须相应调整：
cv::Mat bgr = cv::imread(argv[1]);
cv::resize(bgr, bgr, cv::Size(64, 64));

// 注意：PIXEL_BGR，mean 顺序也是 BGR
ncnn::Mat in = ncnn::Mat::from_pixels(bgr.data, ncnn::Mat::PIXEL_BGR, 64, 64);

// BGR 顺序：原来 RGB [123.675, 116.28, 103.53] → BGR [103.53, 116.28, 123.675]
const float mean_vals[3] = {103.53f,  116.28f,  123.675f};
const float norm_vals[3] = {1.0f / 57.375f, 1.0f / 57.12f, 1.0f / 58.395f};
in.substract_mean_normalize(mean_vals, norm_vals);
```

---

## 3. CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.10)
project(tiny_classifier LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# OpenCV（读取图像用）
find_package(OpenCV REQUIRED)

# ncnn
set(ncnn_DIR "/path/to/ncnn/build/install/lib/cmake/ncnn" CACHE PATH "ncnn CMake config dir")
find_package(ncnn REQUIRED)

add_executable(tiny_cls main.cpp)
target_link_libraries(tiny_cls ncnn ${OpenCV_LIBS})
```

---

## 4. 龙芯久久派 (LoongArch) 编译 ncnn

```bash
git clone https://github.com/Tencent/ncnn.git
cd ncnn
mkdir build && cd build

cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DNCNN_BUILD_TOOLS=OFF \
    -DNCNN_BUILD_EXAMPLES=OFF \
    -DNCNN_BUILD_BENCHMARK=OFF \
    -DNCNN_VULKAN=OFF \
    -DNCNN_OPENMP=OFF \
    ..

make -j$(nproc)
make install
```

如果龙芯久久派的 CPU 不支持 MSA SIMD，ncnn 会自动回退到标量计算。模型只有 4K 参数，标量推理也足够快（预计 10-20ms）。

---

## 5. 常见错误排查

### 5.1 准确率极差（接近随机猜测）

**原因：归一化参数错误。** 这是最常见的问题。

```cpp
// ❌ 错误 — 直接用了 PyTorch 的 mean/std
const float mean_vals[3] = {0.485f, 0.456f, 0.406f};
const float norm_vals[3] = {1.0f/0.229f, 1.0f/0.224f, 1.0f/0.225f};

// ✅ 正确 — 适配了 ncnn 的 [0,255] 像素范围
const float mean_vals[3] = {123.675f, 116.28f, 103.53f};   // mean_pt * 255
const float norm_vals[3] = {1.0f/58.395f, 1.0f/57.12f, 1.0f/57.375f};  // 1/(std_pt * 255)
```

### 5.2 部分类别全错

**原因：RGB/BGR 通道顺序不匹配。**

- 如果 `from_pixels` 用 `PIXEL_RGB`，mean 必须是 RGB 顺序：`[123.675, 116.28, 103.53]`
- 如果 `from_pixels` 用 `PIXEL_BGR`（直接用 OpenCV 的 BGR），mean 必须是 BGR 顺序：`[103.53, 116.28, 123.675]`

### 5.3 程序崩溃或输出异常

- 输入图像没有 resize 到 64×64
- 模型文件路径错误（.param 和 .bin 必须同时存在）
- 输入 blob 名称不对（确认是 `"in0"` 和 `"out0"`）

### 5.4 推理速度很慢

- 检查 `net.opt.num_threads` 设置是否合理（单核设 1）
- 检查是否意外开启了 Vulkan（龙芯久久派不支持，必须设为 `false`）
- 检查是否链接了 debug 版本的 ncnn 库

---

## 6. Python 端验证命令

在 PC 上部署前，先用以下命令验证 NCNN 模型与 PyTorch/ONNX 精度一致：

```bash
python -m src.evaluate \
    --data-root dataset \
    --checkpoint model/best_model.pt \
    --img-size 64 \
    --max-samples 500
```

这会输出 PyTorch / ONNX / NCNN 三方精度对比。
