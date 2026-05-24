# WASM Dependency Inventory

这个清单只服务一个目标：

- 在不修改 `project/` 现有算法文件的前提下
- 尽快把“同源 C++ 视觉算法编到 WebAssembly”推进到可验证状态

## 当前目标分层

### 第一阶段：先拉通较纯的一层

优先纳入：

- `project/code/driver/vision/vision_config.c`
- `project/code/driver/vision/vision_line_error_layer.cpp`

原因：

- 这两部分已经覆盖了 `line_error`、跟踪点、决策点、中线曲率等核心输出
- 相比 `vision_image_processor.cpp`，它们的外部耦合更少
- 可以先验证“同源源码 + compat 桩 + bridge target”的编译链

### 第二阶段：再接主视觉处理链

后续纳入：

- `project/code/driver/vision/vision_image_processor.cpp`

原因：

- 它同时耦合了 OpenCV、采图入口、静态全局状态和大量 getter
- 是最接近最终目标的一层，但不是最适合第一个编译闭环的一层

## 已识别的关键耦合点

### 1. `zf_common_headfile.h`

风险：

- `vision_image_processor.h` 和 `vision_line_error_layer.h` 的上游会带入 SDK 总头
- 总头继续引入 GPIO、PWM、ADC、UVC、UDP/TCP 等大量板级依赖

当前策略：

- 在 `wasm_bridge/compat` 中提供最小版 `zf_common_headfile.h`
- 通过 include path 优先级覆盖真实头文件
- 只保留当前桥接阶段需要的基础类型定义

### 2. `line_follow_thread_base_speed()`

风险：

- `vision_line_error_layer.cpp` 在速度索引模式下直接依赖控制线程速度
- 只传图像不足以保证结果与车端一致

当前策略：

- 在 `wasm_bridge/compat` 中提供 `line_follow_thread.h/.cpp`
- 由 bridge 层把网页收到的原始速度值写入 compat 状态

### 3. `vision_frame_capture_*`

风险：

- `vision_image_processor.cpp` 主入口默认走采图线程
- 浏览器侧不应复用真实相机线程实现

当前策略：

- 第一阶段先不纳入 `vision_frame_capture.cpp`
- 第二阶段在 `wasm_bridge/compat` 中提供同名采图适配层
- 由 JS/worker 把浏览器帧缓冲喂给 compat 实现

### 4. OpenCV

风险：

- `vision_image_processor.cpp` 直接依赖 `cv::Mat`、`remap`、`cvtColor`、`threshold`、`connectedComponentsWithStats`
- 这是接入主视觉链的主要技术门槛之一

当前策略：

- 第一阶段暂不把 `vision_image_processor.cpp` 拉进 probe target
- 后续优先尝试“真实 OpenCV for WASM”，而不是手写兼容算法

## 第一版 probe target 的目标

### 需要成功的事情

- 同源编译 `vision_config.c`
- 同源编译 `vision_line_error_layer.cpp`
- 用 compat 桩满足其外部依赖
- 保持 `project/` 源码零改动

### 暂时不要求的事情

- `vision_image_processor.cpp` 编译通过
- 真实图像主链跑通
- OpenCV WASM 构建

## 第一批 compat 清单

### 已落

- `compat/zf_common_headfile.h`
- `compat/line_follow_thread.h`
- `compat/line_follow_thread.cpp`

### 下一批

- `compat/vision_frame_capture.h`
- `compat/vision_frame_capture.cpp`

## 当前结论

最合理的推进顺序是：

1. 先验证 `vision_line_error_layer.cpp` 的同源编译
2. 再补 `vision_frame_capture` compat
3. 再把 `vision_image_processor.cpp` 拉进 bridge target
4. 最后处理 OpenCV for WASM 与真实 worker 调用闭环
