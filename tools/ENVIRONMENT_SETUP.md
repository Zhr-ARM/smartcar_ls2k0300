# `tools` 环境安装与统一构建说明

本文面向当前保留的电脑端工具：

- `tools/pc_receiver_js`

本地复算 / WASM 相关工具已经移除，网页端只显示主板发送的实时图像与状态数据。

## 环境要求

- Node.js 18+
- npm 9+

## 安装依赖

在仓库根目录执行：

```bash
tools/build_all_tools.sh setup
```

这个命令会进入 `tools/pc_receiver_js` 并执行 `npm ci`。

## 启动网页接收器

```bash
cd tools/pc_receiver_js
npm run dev
```

默认端口：

- UDP 图像：`10000`
- TCP 状态：`10001`
- HTTP 页面：`9090`

可以通过环境变量覆盖：

```bash
BIND_HOST=0.0.0.0 UDP_PORT=10000 TCP_PORT=10001 HTTP_PORT=9090 npm run dev
```

## 常见问题

### `npm run dev` 提示找不到模块

先执行：

```bash
tools/build_all_tools.sh setup
```

### 网页有结构但没有数据

确认主板配置中的电脑接收端 IP 是主板能访问到的地址，并确认防火墙已放行：

- UDP `10000`
- TCP `10001`
- TCP `9090`
