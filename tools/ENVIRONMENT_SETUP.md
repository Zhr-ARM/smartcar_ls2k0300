# `tools` 环境安装与统一构建说明

本文面向整个 [`tools`](/home/terrisa/LoongCar/LS2K0300_Library-2k0300_99pi_wifi/Example/Motherboard_Demo/smartcar_ls2k0300/tools) 目录，说明：

- 需要哪些环境
- 已经做好的本地长期环境
- 以后最常用的统一命令

## 1. `tools` 目录里有哪些项目

当前主要有 2 个可运行/可构建的工具目录：

1. [`tools/pc_receiver_js`](/home/terrisa/LoongCar/LS2K0300_Library-2k0300_99pi_wifi/Example/Motherboard_Demo/smartcar_ls2k0300/tools/pc_receiver_js)
   需要：`Node.js`
2. [`tools/pc_receiver_local_compute`](/home/terrisa/LoongCar/LS2K0300_Library-2k0300_99pi_wifi/Example/Motherboard_Demo/smartcar_ls2k0300/tools/pc_receiver_local_compute)
   运行需要：`Node.js + npm`
   重编 WASM 需要：`Node.js + npm + CMake + Python3 + make + Emscripten + OpenCV(for WASM)`

## 2. 现在已经准备好的长期环境

当前仓库已经按“本地长期可复用”的方式准备好了这些内容：

- `emsdk` 安装在 [`tools/.local/emsdk`](/home/terrisa/LoongCar/LS2K0300_Library-2k0300_99pi_wifi/Example/Motherboard_Demo/smartcar_ls2k0300/tools/.local/emsdk)
- Emscripten 环境已写入 [`~/.bashrc`](/home/terrisa/.bashrc)
- 本地 WASM 版 OpenCV 安装在 [`tools/.local/opencv-wasm/install`](/home/terrisa/LoongCar/LS2K0300_Library-2k0300_99pi_wifi/Example/Motherboard_Demo/smartcar_ls2k0300/tools/.local/opencv-wasm/install)
- `build_wasm.sh` 会自动优先使用这份本地 OpenCV-for-WASM

这意味着：

- 新开一个 Bash 终端后，通常不需要再手动 `source emsdk_env.sh`
- 以后重编 `vision_pipeline.js/.wasm` 时，不需要再自己手动填 `OpenCV_DIR`

## 3. 推荐版本

仓库当前已验证通过的方向：

- `Node.js 18+`
- `npm 9+`
- `CMake 3.16+`
- `Python 3`
- `make`
- `emsdk / emcc`
- `OpenCV 4.x`

## 4. 以后最常用的统一入口

统一入口脚本是：

[`tools/build_all_tools.sh`](/home/terrisa/LoongCar/LS2K0300_Library-2k0300_99pi_wifi/Example/Motherboard_Demo/smartcar_ls2k0300/tools/build_all_tools.sh)

先进入仓库根目录：

```bash
cd /home/terrisa/LoongCar/LS2K0300_Library-2k0300_99pi_wifi/Example/Motherboard_Demo/smartcar_ls2k0300
```

### 4.1 安装 `tools` 目录下的 Node 依赖

```bash
tools/build_all_tools.sh setup
```

它会：

- 给 `tools/pc_receiver_local_compute` 执行 `npm ci`
- 跳过 `tools/pc_receiver_js`，因为它当前没有声明 npm 依赖

### 4.2 构建本地 WASM 版 OpenCV

```bash
tools/build_all_tools.sh wasm-opencv
```

它会调用：

- [`tools/pc_receiver_local_compute/build_opencv_wasm.sh`](/home/terrisa/LoongCar/LS2K0300_Library-2k0300_99pi_wifi/Example/Motherboard_Demo/smartcar_ls2k0300/tools/pc_receiver_local_compute/build_opencv_wasm.sh)

产物安装到：

- [`tools/.local/opencv-wasm/install`](/home/terrisa/LoongCar/LS2K0300_Library-2k0300_99pi_wifi/Example/Motherboard_Demo/smartcar_ls2k0300/tools/.local/opencv-wasm/install)

说明：

- 只有第一次准备环境，或你想重建本地 OpenCV-for-WASM 时，才需要跑这一步

### 4.3 重编网页侧 WASM 视觉桥

```bash
tools/build_all_tools.sh wasm
```

它会调用：

- [`tools/pc_receiver_local_compute/build_wasm.sh`](/home/terrisa/LoongCar/LS2K0300_Library-2k0300_99pi_wifi/Example/Motherboard_Demo/smartcar_ls2k0300/tools/pc_receiver_local_compute/build_wasm.sh)

生成这些产物：

- [`tools/pc_receiver_js/public/wasm/vision_pipeline.js`](/home/terrisa/LoongCar/LS2K0300_Library-2k0300_99pi_wifi/Example/Motherboard_Demo/smartcar_ls2k0300/tools/pc_receiver_js/public/wasm/vision_pipeline.js)
- [`tools/pc_receiver_js/public/wasm/vision_pipeline.wasm`](/home/terrisa/LoongCar/LS2K0300_Library-2k0300_99pi_wifi/Example/Motherboard_Demo/smartcar_ls2k0300/tools/pc_receiver_js/public/wasm/vision_pipeline.wasm)
- [`tools/pc_receiver_js/public/wasm/vision_pipeline.sync.json`](/home/terrisa/LoongCar/LS2K0300_Library-2k0300_99pi_wifi/Example/Motherboard_Demo/smartcar_ls2k0300/tools/pc_receiver_js/public/wasm/vision_pipeline.sync.json)

### 4.4 一次做完整套准备

```bash
tools/build_all_tools.sh all
```

它会依次执行：

1. `setup`
2. `wasm-opencv`
3. `wasm`

如果你换了新机器，或者把 `tools/.local` 删除了，最推荐直接跑这个。

## 5. 平时最常用的命令

### 5.1 只想重新编译 WASM

```bash
tools/build_all_tools.sh wasm
```

### 5.2 想把 `tools` 的环境整体补齐

```bash
tools/build_all_tools.sh all
```

### 5.3 单独运行某个工具

`pc_receiver_local_compute`：

```bash
cd tools/pc_receiver_local_compute
npm run dev
```

`pc_receiver_js`：

```bash
cd tools/pc_receiver_js
npm run dev
```

## 6. 如果你不想记总入口，底层脚本分别是什么

安装 Node 依赖：

```bash
cd tools/pc_receiver_local_compute && npm ci
```

构建本地 OpenCV-for-WASM：

```bash
tools/pc_receiver_local_compute/build_opencv_wasm.sh
```

构建 `vision_pipeline`：

```bash
tools/pc_receiver_local_compute/build_wasm.sh
```

## 7. 常见问题

### 7.1 `npm run dev` 提示找不到模块

先执行：

```bash
tools/build_all_tools.sh setup
```

### 7.2 `build_wasm.sh` 提示 `Emscripten toolchain not found`

通常新开终端就会自动生效。

如果当前 shell 是旧会话，可以执行一次：

```bash
source ~/.bashrc
```

### 7.3 `build_wasm.sh` 提示找不到 OpenCV

先执行：

```bash
tools/build_all_tools.sh wasm-opencv
```

### 7.4 换机器后最省事的做法是什么

直接执行：

```bash
cd /home/terrisa/LoongCar/LS2K0300_Library-2k0300_99pi_wifi/Example/Motherboard_Demo/smartcar_ls2k0300
tools/build_all_tools.sh all
```
