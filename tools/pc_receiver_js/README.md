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
- HTTP 页面：`8080`

网页地址：

`http://<电脑IP>:8080/`

## 2. 可选环境变量

```bash
BIND_HOST=0.0.0.0 UDP_PORT=10000 TCP_PORT=10001 HTTP_PORT=8080 npm run dev
```
