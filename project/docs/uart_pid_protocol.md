# UART PID 通信协议

## 1. 目的

该协议用于实现：

- 下位机通过串口持续向上位机发送左右电机目标转动值、左轮误差和右轮误差
- 上位机通过串口修改左右电机 PID 参数

当前工程中所有速度相关量统一使用 `counts/5ms`。

编译模式说明：

- 定义 `left_mototr`：进入左轮单独调参模式
- 定义 `right_mototr`：进入右轮单独调参模式
- 两者都不定义：保持当前双轮兼容模式

## 2. 通用帧格式

每一帧统一采用如下格式：

| 字段 | 字节数 | 说明 |
| --- | --- | --- |
| `head` | 1 | 固定为 `0x55` |
| `ver` | 1 | 协议版本，当前固定为 `0x01` |
| `cmd` | 1 | 命令字 |
| `seq` | 1 | 帧序号，请求和应答保持一致 |
| `len` | 1 | `payload` 长度，单位：字节 |
| `payload` | N | 数据区 |
| `checksum` | 1 | `ver ^ cmd ^ seq ^ len ^ payload...` 的异或校验 |
| `tail` | 1 | 固定为 `0xCC` |

说明：

- 浮点数统一使用 `float32` 小端序
- 状态推送帧由下位机自动递增 `seq`
- 上位机下发命令时建议自行递增 `seq`

## 3. 命令字定义

### 3.1 下位机 -> 上位机

#### `0x10` `MOTOR_STATUS_PUSH`

下位机固定周期主动发送，当前周期为 `5ms`。

`payload` 根据编译宏变化：

| 编译模式 | `len` | `payload` |
| --- | --- | --- |
| 定义 `left_mototr` | `0x04` | `left_error(float32)` |
| 定义 `right_mototr` | `0x04` | `right_error(float32)` |
| 都不定义 | `0x10` | `left_target_count(float32) -> right_target_count(float32) -> left_error(float32) -> right_error(float32)` |

其中：

`left_error = left_target_count - left_current_count`

`right_error = right_target_count - right_current_count`

单轮模式整帧总长度为 `11` 字节，双轮兼容模式整帧总长度为 `23` 字节。

单轮模式示例：

- `left_mototr` 已定义，左轮误差 `12.5`

```text
55 01 10 01 04 00 00 48 41 1D CC
```

- `right_mototr` 已定义，右轮误差 `8.0`

```text
55 01 10 01 04 00 00 00 41 55 CC
```

#### `0xA0` `PID_SET_ACK`

收到上位机写 PID 成功后返回。

`payload` 长度固定为 `0`，整帧总长度为 `7` 字节。

#### `0xA1` `PID_BOOT_REPORT`

仅在串口初始化完成时主动发送一次，用于通知上位机当前 PID 初值。

`payload` 根据编译宏变化：

| 编译模式 | `len` | `payload` |
| --- | --- | --- |
| 定义 `left_mototr` | `0x0C` | `left_kp -> left_ki -> left_kd` |
| 定义 `right_mototr` | `0x0C` | `right_kp -> right_ki -> right_kd` |
| 都不定义 | `0x18` | `left_kp -> left_ki -> left_kd -> right_kp -> right_ki -> right_kd` |

单轮模式整帧总长度为 `19` 字节，双轮兼容模式整帧总长度为 `31` 字节。

单轮模式上电示例：

- `left_mototr` 已定义

```text
55 01 A1 00 0C CD CC 4C 3D 17 B7 51 38 00 00 00 00 15 CC
```

- `right_mototr` 已定义

```text
55 01 A1 00 0C CD CC 4C 3D 17 B7 51 38 00 00 00 00 15 CC
```

#### `0xE0` `ERROR_REPORT`

协议错误或参数错误时返回。

`payload` 格式：

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `error_code` | `uint8` | 错误码 |
| `related_cmd` | `uint8` | 出错的命令字 |

错误码定义：

- `0x01`：长度错误
- `0x02`：命令不支持
- `0x03`：PID 参数非法
- `0x04`：协议版本不支持

整帧总长度为 `9` 字节。

### 3.2 上位机 -> 下位机

#### `0x20` `PID_SET_REQ`

用于修改左右电机 PID 参数。

`payload` 根据编译宏变化：

| 编译模式 | `len` | `payload` |
| --- | --- | --- |
| 定义 `left_mototr` | `0x0C` | `left_kp -> left_ki -> left_kd` |
| 定义 `right_mototr` | `0x0C` | `right_kp -> right_ki -> right_kd` |
| 都不定义 | `0x18` | `left_kp -> left_ki -> left_kd -> right_kp -> right_ki -> right_kd` |

单轮模式整帧总长度为 `19` 字节，双轮兼容模式整帧总长度为 `31` 字节。

单轮模式改参示例：

- `left_mototr` 已定义，只修改左轮 PID 为 `0.06 / 0.00006 / 0.0`

```text
55 01 20 02 0C 8F C2 75 3D 82 A8 7B 38 00 00 00 00 43 CC
```

- `right_mototr` 已定义，只修改右轮 PID 为 `0.08 / 0.00008 / 0.0`

```text
55 01 20 02 0C 0A D7 A3 3D AC C5 A7 38 00 00 00 00 9A CC
```

下位机处理规则：

- 6 个参数必须全部发送
- 参数必须为有限正数或 `0`
- 更新成功后会立即清空 PID 的积分项和历史误差
- 更新成功后返回空载 `PID_SET_ACK`

#### `0x21` `PID_GET_REQ`

该命令当前版本不启用。

如果上位机发送该命令，下位机会返回 `ERROR_REPORT(error_code=0x02)`。

## 4. 字段顺序

PID 相关帧中的 6 个参数顺序必须严格固定为：

`left_kp -> left_ki -> left_kd -> right_kp -> right_ki -> right_kd`

## 5. 当前下位机行为

当前代码中的串口线程行为如下：

- 串口初始化成功后，立即启动状态推送线程
- 下位机每 `5ms` 主动发送一帧 `MOTOR_STATUS_PUSH`
- 串口初始化完成后，下位机会主动发送一帧 `PID_BOOT_REPORT(seq=0)`，用于通知当前 PID 初值
- 上位机可以随时发送 `PID_SET_REQ` 修改 PID
- 改参成功后，下位机只返回空载 `PID_SET_ACK`，不再回传 PID 参数帧
- 板端如果定义了 `left_mototr` 或 `right_mototr`，则协议自动切换为对应单轮模式

## 6. 上位机对接建议

- 接收端不要依赖“单次串口读取就是一整帧”，必须按 `head + len + checksum + tail` 组帧
- 如果收到 `ERROR_REPORT`，优先检查 `len`、`checksum`、`cmd` 和 `payload` 浮点格式
- 如果上位机界面只需要低频显示，可以自行对 `MOTOR_STATUS_PUSH` 做抽样显示
- 上位机必须先确认板端编译的是 `left_mototr`、`right_mototr` 还是双轮兼容模式，再按对应 `payload` 长度解包
