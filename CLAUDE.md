# 龙芯 LS2K0300 智能车项目

## 本地编译

本机已安装龙芯交叉编译工具链，无需连接主板即可编译验证。

### 工具链路径

| 组件 | 路径 |
|------|------|
| 编译器 (GCC 8.3.0) | `/opt/ls_2k0300_env/loongson-gnu-toolchain-8.3-x86_64-loongarch64-linux-gnu-rc1.6/` |
| C++ 编译器 | `loongarch64-linux-gnu-g++` |
| C 编译器 | `loongarch64-linux-gnu-gcc` |
| OpenCV 4.10.0 | `/opt/ls_2k0300_env/opencv_4_10_build/` |
| 交叉编译配置文件 | `project/user/cross.cmake` |

### 编译命令

```bash
# 1. 准备输出目录
mkdir -p project/out && cd project/out

# 2. CMake 配置（关闭 NCNN 可加快编译）
cmake ../user \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
  -DUVC_RES_PRESET=1 \
  -DUVC_FORMAT_PRESET=0 \
  -DENABLE_NCNN=0

# 3. 编译（-j12 并行）
make -j12
```

### CMake 选项

| 选项 | 值 | 说明 |
|------|-----|------|
| `UVC_RES_PRESET` | `0`=160x120, `1`=320x240 | 摄像头采图分辨率 |
| `UVC_FORMAT_PRESET` | `0`=YUY2, `1`=MJPG | 摄像头输出格式 |
| `ENABLE_NCNN` | `0`=关闭, `1`=开启 | ncnn 推理开关 |

### 产物

编译输出为 LoongArch 64-bit ELF，位于 `project/out/project`。

### 部署到主板

使用 `project/user/build.sh` 一键编译+传输+部署：

```bash
cd project/user
./build.sh --preset hotspot_a   # 根据 build_target.env 配置预设
```

`build.sh` 会自动：
1. CMake 配置 + make 编译
2. scp 可执行文件到主板
3. scp 配置文件 `smartcar_config.toml` 到主板
4. （如开启 ncnn）scp 模型文件到主板
