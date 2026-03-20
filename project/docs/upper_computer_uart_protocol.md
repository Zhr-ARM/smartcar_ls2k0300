# 上位机串口通信说明

## 1. 用途

该协议用于上位机和下位机之间通过串口完成以下功能：

- 下位机持续上报左右电机目标值、左轮误差和右轮误差
- 上位机修改左右电机 PID 参数

当前工程中速度单位统一为 `counts/5ms`。

编译模式说明：

- 定义 `left_mototr`：进入左轮单独调参模式
- 定义 `right_mototr`：进入右轮单独调参模式
- 两者都不定义：保持当前双轮兼容模式

## 2. 串口参数

- 波特率：`115200`
- 数据位：`8`
- 停止位：`1`
- 校验位：`None`
- 发送方式：`十六进制原始字节`

注意：

- 上位机发送时必须开启 `HEX` 发送
- 不能发送 ASCII 字符串 `"55 01 21 ..."`
- 不能自动追加 `CR`、`LF` 或空格

## 3. 帧格式

每一帧格式固定为：

| 字段 | 字节数 | 说明 |
| --- | --- | --- |
| `head` | 1 | 帧头，固定 `0x55` |
| `ver` | 1 | 协议版本，固定 `0x01` |
| `cmd` | 1 | 命令字 |
| `seq` | 1 | 帧序号，请求和应答保持一致 |
| `len` | 1 | `payload` 长度 |
| `payload` | N | 数据区 |
| `checksum` | 1 | 从 `ver` 到 `payload` 末字节的异或校验 |
| `tail` | 1 | 帧尾，固定 `0xCC` |

浮点数一律使用：

- 类型：`float32`
- 字节序：`小端序`

## 4. 下位机主动发送

### 4.1 状态推送 `0x10`

下位机每 `5ms` 主动发送一帧状态数据。

`payload` 根据编译宏变化：

| 编译模式 | `len` | `payload` |
| --- | --- | --- |
| 定义 `left_mototr` | `0x04` | `left_error(float32)` |
| 定义 `right_mototr` | `0x04` | `right_error(float32)` |
| 都不定义 | `0x10` | `left_target_count(float32) -> right_target_count(float32) -> left_error(float32) -> right_error(float32)` |

误差定义：

```text
left_error  = left_target_count  - left_current_count
right_error = right_target_count - right_current_count
```

双轮兼容模式示例：

```text
55 01 10 01 10 00 00 E1 43 00 00 E1 43 00 00 20 41 00 00 20 41 00 CC
```

含义：

- 左目标：`450.0`
- 右目标：`450.0`
- 左误差：`10.0`
- 右误差：`10.0`

单轮模式示例：

- `left_mototr` 已定义，左轮误差为 `12.5`

```text
55 01 10 01 04 00 00 48 41 1D CC
```

- `right_mototr` 已定义，右轮误差为 `8.0`

```text
55 01 10 01 04 00 00 00 41 55 CC
```

### 4.2 PID 上电通知 `0xA1`

- 命令字：`0xA1`
- `payload` 长度随编译模式变化

`payload` 定义：

| 编译模式 | `len` | `payload` |
| --- | --- | --- |
| 定义 `left_mototr` | `0x0C` | `left_kp -> left_ki -> left_kd` |
| 定义 `right_mototr` | `0x0C` | `right_kp -> right_ki -> right_kd` |
| 都不定义 | `0x18` | `left_kp -> left_ki -> left_kd -> right_kp -> right_ki -> right_kd` |

双轮兼容模式默认返回示例：

```text
55 01 A1 01 18 CD CC 4C 3D 17 B7 51 38 00 00 00 00 CD CC 4C 3D 17 B7 51 38 00 00 00 00 B9 CC
```

对应参数：

- 左轮：`kp=0.05` `ki=0.00005` `kd=0.0`
- 右轮：`kp=0.05` `ki=0.00005` `kd=0.0`

说明：

- 该帧只在下位机启动后主动发送一次
- 后续不会再通过串口回传完整 PID 参数

单轮模式上电示例：

- `left_mototr` 已定义，默认左轮 PID 为 `0.05 / 0.00005 / 0.0`

```text
55 01 A1 00 0C CD CC 4C 3D 17 B7 51 38 00 00 00 00 15 CC
```

- `right_mototr` 已定义，默认右轮 PID 为 `0.05 / 0.00005 / 0.0`

```text
55 01 A1 00 0C CD CC 4C 3D 17 B7 51 38 00 00 00 00 15 CC
```

## 5. 上位机修改 PID

### 5.1 发送 `PID_SET_REQ`

- 命令字：`0x20`
- `payload` 长度随编译模式变化

`payload` 定义：

| 编译模式 | `len` | `payload` |
| --- | --- | --- |
| 定义 `left_mototr` | `0x0C` | `left_kp -> left_ki -> left_kd` |
| 定义 `right_mototr` | `0x0C` | `right_kp -> right_ki -> right_kd` |
| 都不定义 | `0x18` | `left_kp -> left_ki -> left_kd -> right_kp -> right_ki -> right_kd` |

双轮兼容模式示例：将左右 PID 都改为 `kp=0.06`、`ki=0.00006`、`kd=0.0`

```text
55 01 20 02 18 8F C2 75 3D 82 A8 7B 38 00 00 00 00 8F C2 75 3D 82 A8 7B 38 00 00 00 00 3B CC
```

单轮模式示例：

- `left_mototr` 已定义，只修改左轮 PID 为 `0.06 / 0.00006 / 0.0`

```text
55 01 20 02 0C 8F C2 75 3D 82 A8 7B 38 00 00 00 00 43 CC
```

- `right_mototr` 已定义，只修改右轮 PID 为 `0.08 / 0.00008 / 0.0`

```text
55 01 20 02 0C 0A D7 A3 3D AC C5 A7 38 00 00 00 00 9A CC
```

### 5.2 接收 `PID_SET_ACK`

- 命令字：`0xA0`
- `payload` 长度：`0`

ACK 示例：

```text
55 01 A0 02 00 A3 CC
```

当下位机成功改参后：

- 会返回 `PID_SET_ACK`
- 会在本地终端打印 `左目标 右目标 左当前 右当前`

## 6. 不再支持的命令

### `0x21` `PID_GET_REQ`

当前版本不再支持上位机主动读取 PID。

如果发送：

```text
55 01 21 01 00 21 CC
```

下位机将返回错误帧，表示命令不支持。

## 7. 错误帧

当请求帧有问题时，下位机会返回 `0xE0`。

`payload` 格式：

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `error_code` | `uint8` | 错误码 |
| `related_cmd` | `uint8` | 关联命令字 |

错误码：

- `0x01`：长度错误
- `0x02`：命令不支持
- `0x03`：参数非法
- `0x04`：版本不支持

## 8. 上位机对接建议

- 接收程序必须自己按帧头、长度、校验和帧尾组帧
- 一次串口读取可能收到半帧，也可能收到多帧拼接
- 如果只做界面显示，状态帧可以自行抽样显示，不必每帧刷新 UI
- 调 PID 后以 `PID_SET_ACK` 和终端打印为准，不再通过串口读取完整 PID 参数
- 上位机必须先确认板端是 `left_mototr`、`right_mototr` 还是双轮兼容模式，再按对应 `payload` 长度解包
