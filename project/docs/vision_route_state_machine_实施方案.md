# 视觉状态机实施方案

## 当前状态

这份文档原本记录的是一版早期实施方案，里面包含较多已经过时的内容，例如：

1. 十字状态机作为当前主线的一部分。
2. `circle_left_begin / in / running / out / end` 这一套旧阶段命名。
3. 若干尚未按当前代码落地的状态转移规则。

为避免继续把历史方案误当成当前实现，这里不再保留旧方案正文。

## 现行参考

当前状态机请以实际代码和以下文档为准：

1. `project/code/driver/vision/vision_route_state_machine.h`
2. `project/code/driver/vision/vision_route_state_machine.cpp`
3. `project/docs/圆环状态机.md`
4. `project/docs/当前代码视觉处理概述.md`

## 说明

当前主线已实际落地的是左右圆环状态机及其补线流程；十字相关枚举和部分辅助逻辑仍有保留，但不应继续按旧方案文档理解为“当前已经启用的完整状态机”。
