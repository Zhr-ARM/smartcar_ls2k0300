# Local Compute Architecture

这个目录后续按下面的边界推进：

## 原则

1. 不修改 `project/` 里现有车端算法文件
2. 单一传输通道，同时服务两个网页界面
3. 网页端优先复用同一份 C++ 算法源码，而不是维护一份手写 JS 副本

## 结构

- `server.js`
  - 单一接收通道
  - 同时服务“传统展示页”和“本地复算页”
- `public/`
  - 浏览器 UI
  - worker 调度
- `wasm_bridge/`
  - WebAssembly 侧的只读桥接层
  - 允许引用 `project/` 中已有源码
  - 但不直接修改这些源码文件

## 推荐实现策略

### 1. 保持 `project/` 算法源码不动

后续如需给 WebAssembly 编译使用：

- 在 `tools/pc_receiver_local_compute/wasm_bridge` 中新增适配层
- 由适配层引入 `project/code/driver/vision/...` 中的现有源码
- 如果现有源码依赖硬件接口，则在 `wasm_bridge/compat` 中提供桩实现

### 2. 单一接收服务，双页面展示

同一个 `server.js` 同时提供：

- 传统全结果展示页
- 本地复算展示页

这样主板只维护一条传输链路。

### 3. 双编译，而不是双实现

- 主板端：正常编译现有 C++ 工程
- Web 端：用同一份 C++ 算法源码编译出 WASM

目标是“一份算法，两种产物”，而不是“两份算法，两边同步改”。

### 4. 不再把简化版 JS 识别结果作为目标路线

- 浏览器端可以临时保留最小联调桩
- 但正式目标不是“近似结果”
- 正式目标是：
  - 输入同一帧图像
  - 输入同一组原始采集值
  - 调同一份 C++ 视觉算法源码
  - 输出尽可能一模一样的边界 / 中线 / 曲率 / line_error / 角点 / 辅助线

也就是说，网页端本地计算最终应是“同源算法的另一次编译产物”，而不是单独维护一份手写 JS 算法。

## 后续落地顺序

1. 定义 Web 端真正需要的最小原始状态集
2. 在 `server.js` 中把两个页面统一挂到一个接收服务下
3. 在 `wasm_bridge` 中建立只读编译清单
4. 对 `project` 中算法依赖的硬件/线程接口做桥接桩
5. 编出第一版可调用的 WASM 模块

## 明确约束

如果后续某一步必须动 `project/` 内现有算法文件，我会先停下来和你确认，不会直接改。

## 当前里程碑进度

- 已完成：单通道双页面服务
- 已完成：`FULL / RAW_MINIMAL` 传输档位
- 已完成：网页接收层与基础绘图共享
- 已完成：`wasm_bridge` 稳定接口骨架
- 已完成：第一版 probe 编译链
  - 当前已能在 `tools/pc_receiver_local_compute/wasm_bridge` 下同源编译：
    - `vision_config.c`
    - `vision_line_error_layer.cpp`
  - 当前使用的 compat：
    - `zf_common_headfile.h`
    - `line_follow_thread.h/.cpp`
- 下一步：补 `vision_frame_capture` compat，并把 `vision_image_processor.cpp` 纳入 probe
