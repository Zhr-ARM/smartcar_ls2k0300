# Vision JS Receiver

Node.js + npm 版本的电脑端接收器：

- UDP 收图像分片（JPEG）
- TCP 收状态 JSON（每行一条）
- 网页展示实时图像与状态

## 1. 启动

```bash
cd tools/pc_receiver_js
npm run dev
```

默认端口：

- UDP 视频：`10000`
- TCP 状态：`10001`
- HTTP 页面：`9090`

网页地址：

`http://<电脑IP>:9090/`

## 2. 录制布局测试源数据

网页里的录制功能适合人工回放检查，会保存 canvas 视频和状态快照。做前端布局自测时，建议再录一份固定 fixture：它直接从接收器 HTTP API 抓 `/api/status` 和当前帧图像，不改主板端和后端数据流。

```bash
cd tools/pc_receiver_js
npm run record:fixture -- --base http://127.0.0.1:9090 --duration 10 --interval 200
```

输出默认在 `tools/pc_receiver_js/recordings/live_fixture_<timestamp>/`：

- `status.json`：每次采样的状态 JSON、时间戳、图像引用
- `meta.json`：录制摘要
- `frames/`：灰度、RGB、二值、ROI 图像去重后的源帧

如果只需要状态数据：

```bash
npm run record:fixture -- --status-only --duration 5
```

## 3. 模拟主板数据

前端布局自动测试建议使用模拟主板：它按真实协议向接收器发送 TCP 状态 JSON 和 UDP 图像帧，让页面仍然走完整链路。

先启动接收器：

```bash
cd tools/pc_receiver_js
npm run dev
```

再开另一个终端启动合成数据源：

```bash
cd tools/pc_receiver_js
npm run mock:board -- --duration 30
```

也可以回放上一节录到的 fixture：

```bash
npm run mock:board -- --fixture recordings/live_fixture_20260525T090208Z --duration 30
```

常用隔离端口，适合自动化测试：

```bash
BIND_HOST=127.0.0.1 UDP_PORT=19000 TCP_PORT=19001 HTTP_PORT=19090 npm run dev
npm run mock:board -- --udp-port 19000 --tcp-port 19001 --duration 30
```

这样测试脚本访问 `http://127.0.0.1:19090/`，不用依赖真实主板在线，也不会影响默认 `10000/10001/9090` 端口。

最小自动化烟测：

```bash
npm run test:mock:pipeline
```

它会启动隔离接收器、启动模拟主板、检查 `/api/status`、`/api/frame_gray.jpg` 和主页可访问性，然后自动退出。默认会优先回放 `recordings/live_fixture_*` 里最新的已录制 fixture；如果没有 fixture，才退回合成模拟数据。这个测试主要拦截“后端没数据、API 坏了、主页没起来”的问题；布局截图回归可以在这个基础上再接 Playwright。

也可以显式指定或强制使用合成数据：

```bash
npm run test:mock:pipeline -- --fixture recordings/live_fixture_20260525T090208Z
npm run test:mock:pipeline -- --synthetic
```

## 4. 可选环境变量

```bash
BIND_HOST=0.0.0.0 UDP_PORT=10000 TCP_PORT=10001 HTTP_PORT=9080 npm run dev
```
