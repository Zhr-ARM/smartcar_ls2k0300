# 位置环模糊自整定 PID 工作原理

这份文档解释当前工程里的位置环模糊 PID。它不是“纯模糊控制器”，而是“模糊自整定 PID”：模糊模块只负责在线计算 `Kp`、`Ki`、`Kd`，真正输出转向差速的仍然是位置式 PID。

当前控制链路可以压缩成一句话：

```text
视觉中线误差 -> e/de -> 模糊规则 -> Kp/Ki/Kd -> 位置 PID -> 转向差速 steering -> 左右轮目标速度
```

对应代码主要在：

```text
project/code/app/line_follow_thread/line_follow_thread.cpp
project/user/smartcar_config.toml
```

## 1. 总体控制框架

当前位置控制框架是：

```text
外环：视觉位置环
输入：line_error / control_error_px
输出：steering_output 差速转向量

内环：电机速度环
输入：left_target_count / right_target_count
输出：电机 PWM
```

位置环的最终左右轮目标为：

```text
left_target  = base_speed - steering_output
right_target = base_speed + steering_output
```

其中：

```text
steering_output > 0：右轮更快，左轮更慢，车辆向左修正
steering_output < 0：左轮更快，右轮更慢，车辆向右修正
```

注意：模糊 PID 不直接输出 `steering_output`。它只输出当前帧应该使用的 PID 增益。

## 2. 从视觉误差到控制误差

视觉模块先给出 `line_error`。巡线线程里做了方向统一：

```text
raw_error_px = -line_error
```

然后对误差做一阶 IIR 滤波：

```text
filtered_error[k] = (1 - alpha_e) * filtered_error[k-1] + alpha_e * raw_error[k]
```

这里 `alpha_e` 由配置 `pid.line_follow.error_filter_alpha` 转换得到，并根据真实视觉帧间隔修正。

之后进入死区和小误差降增益：

```text
if |filtered_error| < error_deadzone_px:
    control_error = 0
else if |filtered_error| < error_low_gain_limit_px:
    control_error = filtered_error * error_low_gain
else:
    control_error = filtered_error
```

所以模糊 PID 使用的 `e` 不是最原始的 `line_error`，而是处理后的：

```text
e = control_error_px
```

## 3. de 是怎么来的

`de` 是误差变化率，单位是 `px/s`。

只在新视觉帧到来时更新：

```text
de[k] = (e[k] - e[k-1]) / dt_vision[k]
```

其中：

```text
dt_vision[k] = 当前视觉帧时间 - 上一视觉帧时间
```

如果没有新视觉帧：

```text
e、de、Kp/Ki/Kd、PID 输出都会保持上一帧状态
```

这样做是为了避免 20ms 控制循环重复吃同一帧视觉数据，造成虚假的滤波和微分。

## 4. 输入归一化

模糊规则表希望输入范围固定在 `[-1, 1]`，所以先把 `e` 和 `de` 归一化：

```text
e_norm  = clamp(e / e_scale, -1, 1)
de_norm = clamp(de / de_scale, -1, 1)
```

其中：

```text
e_scale  单位是 px
de_scale 单位是 px/s
```

含义是：

```text
当 |e| >= e_scale 时，模糊系统认为横向误差已经很大，e_norm 饱和到 ±1
当 |de| >= de_scale 时，模糊系统认为误差变化很快，de_norm 饱和到 ±1
```

举例：

```text
当前 e_scale = 20
如果 e = 10 px，则 e_norm = 10 / 20 = 0.5
如果 e = 30 px，则 e_norm = clamp(1.5, -1, 1) = 1
```

再举一个 `de_scale` 的例子：

```text
摄像头 60 FPS，dt ≈ 1 / 60 = 0.0167 s
两帧误差从 5 px 变成 10 px

de = (10 - 5) / 0.0167 = 300 px/s

如果 de_scale = 300，则 de_norm = 1
```

所以 `de_scale` 可以按这个公式估算：

```text
de_scale = 你认为明显的单帧误差变化量(px/frame) * 视觉帧率(fps)
```

例如：

```text
5 px/frame @ 60 FPS -> de_scale = 5 * 60 = 300
4 px/frame @ 60 FPS -> de_scale = 4 * 60 = 240
8 px/frame @ 50 FPS -> de_scale = 8 * 50 = 400
```

## 5. 9 个模糊语言档位

当前使用 9 档语言变量，中心点为：

```text
[-1.00, -0.75, -0.50, -0.25, 0.00, 0.25, 0.50, 0.75, 1.00]
```

可以理解为：

```text
NB2  NB1  NM   NS   ZO   PS   PM   PB1  PB2
-1  -0.75 -0.5 -0.25  0  0.25 0.5 0.75  1
```

工程里没有强制使用这些名字，但理解时可以这样记。

对任意输入 `x`，也就是 `e_norm` 或 `de_norm`，都会计算它对这 9 个档位的隶属度：

```text
mu[0], mu[1], ..., mu[8]
```

每个隶属度范围是：

```text
0 <= mu[i] <= 1
```

并且通常只有相邻几个档位非 0。

## 6. 隶属函数

中间 7 个档位使用三角形隶属函数。

对于中心点 `c_i`，左中心 `c_{i-1}`，右中心 `c_{i+1}`：

```text
tri(x; a,b,c) =
    0,                  x <= a 或 x >= c
    (x - a) / (b - a),  a < x < b
    1,                  x = b
    (c - x) / (c - b),  b < x < c
```

其中：

```text
a = c_{i-1}
b = c_i
c = c_{i+1}
```

最左和最右两个边界档使用梯形隶属函数，避免 `x = -1` 或 `x = 1` 时落不到规则上。

梯形函数：

```text
trap(x; a,b,c,d) =
    0,                  x <= a 或 x >= d
    (x - a) / (b - a),  a < x < b
    1,                  b <= x <= c
    (d - x) / (d - c),  c < x < d
```

当前边界大致是：

```text
最左档：trap(x; -1.5, -1.0, -1.0, -0.75)
最右档：trap(x;  0.75, 1.0, 1.0, 1.5)
```

## 7. 9x9 规则表怎么读

每组规则表有 81 个数：

```text
rule_dkp = [81 个数]
rule_dki = [81 个数]
rule_dkd = [81 个数]
```

它们分别用于计算：

```text
dKp, dKi, dKd
```

规则表按 `e 行、de 列` 展平。

第 `i` 行、第 `j` 列的索引是：

```text
idx = i * 9 + j
```

其中：

```text
i = e_norm 所在语言档位索引，范围 0..8
j = de_norm 所在语言档位索引，范围 0..8
```

也就是说：

```text
rule[0]  是 e=-1.00, de=-1.00 的规则值
rule[4]  是 e=-1.00, de= 0.00 的规则值
rule[40] 是 e= 0.00, de= 0.00 的规则值
rule[80] 是 e= 1.00, de= 1.00 的规则值
```

规则值范围要求是：

```text
-1 <= rule_value <= 1
```

它还不是最终的 `dKp`，只是一个归一化建议。

## 8. 模糊推理

当前实现使用乘积权重。

如果 `e_norm` 对第 `i` 档的隶属度是：

```text
mu_e[i]
```

`de_norm` 对第 `j` 档的隶属度是：

```text
mu_de[j]
```

那么规则 `(i,j)` 的激活权重为：

```text
w_ij = mu_e[i] * mu_de[j]
```

每条规则的贡献为：

```text
contribution_ij = w_ij * rule[i,j]
```

## 9. 解模糊

当前使用加权平均，可以看成简化的质心法：

```text
fuzzy_output = sum(w_ij * rule[i,j]) / sum(w_ij)
```

如果所有权重之和非常小，则输出 0：

```text
if sum(w_ij) <= 1e-6:
    fuzzy_output = 0
```

分别计算三次：

```text
fuzzy_dkp_norm = defuzz(rule_dkp, mu_e, mu_de)
fuzzy_dki_norm = defuzz(rule_dki, mu_e, mu_de)
fuzzy_dkd_norm = defuzz(rule_dkd, mu_e, mu_de)
```

这三个结果仍然在大约 `[-1, 1]` 之间。

## 10. 从模糊输出到 PID 增益

先乘以增量尺度：

```text
dKp = fuzzy_dkp_norm * dkp_scale
dKi = fuzzy_dki_norm * dki_scale
dKd = fuzzy_dkd_norm * dkd_scale
```

再叠加基础 PID：

```text
Kp_raw = kp_base + dKp
Ki_raw = ki_base + dKi
Kd_raw = kd_base + dKd
```

再做限幅：

```text
Kp_raw = clamp(Kp_raw, kp_min, kp_max)
Ki_raw = clamp(Ki_raw, ki_min, ki_max)
Kd_raw = clamp(Kd_raw, kd_min, kd_max)
```

最后通过一阶平滑，得到真正应用到 PID 的增益：

```text
Kp_applied[k] = (1 - alpha_g) * Kp_applied[k-1] + alpha_g * Kp_raw[k]
Ki_applied[k] = (1 - alpha_g) * Ki_applied[k-1] + alpha_g * Ki_raw[k]
Kd_applied[k] = (1 - alpha_g) * Kd_applied[k-1] + alpha_g * Kd_raw[k]
```

其中：

```text
alpha_g = fuzzy_pid.gain_update_alpha
```

如果是第一次运行，还没有历史值，则直接使用当前 raw 值初始化。

## 11. 位置 PID 怎么使用这些增益

模糊模块得到当前帧的：

```text
Kp_applied, Ki_applied, Kd_applied
```

然后设置给位置式 PID：

```text
position_pid.set_params(Kp_applied, Ki_applied, Kd_applied)
```

位置 PID 的概念表达式为：

```text
P[k] = Kp * e[k]
I[k] = I[k-1] + Ki * e[k] * dt
D[k] = Kd * (e[k] - e[k-1]) / dt

pid_output[k] = P[k] + I[k] + D[k]
```

然后做积分限幅和输出限幅：

```text
I[k] = clamp(I[k], -position_max_integral, position_max_integral)
pid_output[k] = clamp(pid_output[k], -position_max_output, position_max_output)
```

当前配置里：

```text
position_max_integral = 0
```

所以即使 `Ki` 不为 0，积分项也会被限制住。也就是说当前主要还是 P/D 行为，I 基本不起作用。

## 12. 位置前馈如何叠加

如果开启位置前馈，工程会额外根据误差的一阶差分和二阶差分生成一份前馈输出：

```text
first_diff  = e[k] - e[k-1]
second_diff = e[k] - 2 * e[k-1] + e[k-2]
```

基础前馈：

```text
ff_raw = first_diff_gain * first_diff + second_diff_gain * second_diff
```

再乘上速度和趋势相关因子：

```text
ff = ff_raw * speed_scale * trend_scale
```

再限幅：

```text
ff = clamp(ff, -position_feedforward_max_output, position_feedforward_max_output)
```

最终位置环输出为：

```text
position_output = clamp(pid_output + ff, -position_max_output, position_max_output)
```

然后再经过总转向限幅：

```text
raw_steering_output = clamp(position_output, -steering_max_output, steering_max_output)
```

## 13. 车轮速度余量限幅

最终转向还会受左右轮目标速度上下限约束。

先计算当前基础速度下还能给多少差速：

```text
left_turn_room  = base_speed - target_count_min
right_turn_room = target_count_max - base_speed
available_steering_limit = min(left_turn_room, right_turn_room)
```

再限幅：

```text
steering_output = clamp(raw_steering_output,
                        -available_steering_limit,
                         available_steering_limit)
```

如果你看到：

```text
raw_steering_output 很大
applied_steering_output 很小
```

那不是模糊 PID 没响应，而是轮速目标上下限把转向差速吃掉了。

## 14. 每个 fuzzy_pid 参数的作用

### enable_fuzzy_pid

```toml
fuzzy_pid.enable_fuzzy_pid = true
```

开关。

```text
true：使用模糊自整定 Kp/Ki/Kd
false：回退到旧的动态 Kp/Kd 逻辑
```

### kp_base / ki_base / kd_base

```toml
fuzzy_pid.kp_base = 10.0
fuzzy_pid.ki_base = 0.0
fuzzy_pid.kd_base = 0.02
```

基础 PID 增益。

模糊输出为 0 时：

```text
Kp = kp_base
Ki = ki_base
Kd = kd_base
```

调参理解：

```text
kp_base 决定基础纠偏力度
ki_base 决定长期偏差修正能力，但当前 position_max_integral=0 时基本不起作用
kd_base 决定基础阻尼和抑制过冲能力
```

### kp_min / kp_max

```toml
fuzzy_pid.kp_min = 0.0
fuzzy_pid.kp_max = 40.0
```

`Kp` 的安全限幅。

```text
Kp = clamp(kp_base + dKp, kp_min, kp_max)
```

现象判断：

```text
Kp 经常顶到 kp_max：规则或 dkp_scale 太猛，可能振荡
Kp 经常贴近 kp_min：规则或基础值太保守，可能无响应
```

### ki_min / ki_max

```toml
fuzzy_pid.ki_min = 0.0
fuzzy_pid.ki_max = 1.0
```

`Ki` 的安全限幅。

如果积分限幅仍为 0：

```text
position_max_integral = 0
```

那么 `Ki` 调大也不会明显生效。

### kd_min / kd_max

```toml
fuzzy_pid.kd_min = 0.0
fuzzy_pid.kd_max = 2.0
```

`Kd` 的安全限幅。

```text
Kd 太小：连续弯、反打、回正时可能没有阻尼
Kd 太大：视觉噪声会被放大，转向可能抖动
```

### e_scale

```toml
fuzzy_pid.e_scale = 20.0
```

误差归一化尺度，单位 `px`。

```text
e_norm = clamp(e / e_scale, -1, 1)
```

调参理解：

```text
e_scale 小：同样误差会更快进入大误差规则，响应更激进
e_scale 大：同样误差只算中小误差，响应更温和
```

建议看遥测：

```text
pid_common_fuzzy_e_norm
```

判断：

```text
经常 ±1：e_scale 太小，规则过早饱和
很少超过 ±0.3：e_scale 太大，模糊规则用不起来
```

### de_scale

```toml
fuzzy_pid.de_scale = 300.0
```

误差变化率归一化尺度，单位 `px/s`。

```text
de_norm = clamp(de / de_scale, -1, 1)
```

调参理解：

```text
de_scale 小：对变化率很敏感，容易快速进入 de 边界规则
de_scale 大：对变化率不敏感，规则主要看 e
```

建议看遥测：

```text
pid_common_fuzzy_de_norm
```

判断：

```text
连续弯时 de_norm 经常 ±1：de_scale 偏小
车身明显摆动但 de_norm 只有 ±0.1：de_scale 偏大
```

### dkp_scale

```toml
fuzzy_pid.dkp_scale = 10.0
```

`dKp` 的放大倍率。

```text
dKp = fuzzy_dkp_norm * dkp_scale
```

如果规则输出是 `0.6`，`dkp_scale = 10`：

```text
dKp = 0.6 * 10 = 6
Kp_raw = kp_base + 6
```

调参理解：

```text
dkp_scale 大：大误差时 Kp 提升明显，转向更有力
dkp_scale 小：模糊调参影响弱，更像固定 Kp
```

### dki_scale

```toml
fuzzy_pid.dki_scale = 0.0
```

`dKi` 的放大倍率。

当前 normal 档为 0，表示模糊规则不会改变 `Ki`。

即使设大，也要同时允许积分：

```text
position_max_integral > 0
```

否则积分项被限幅为 0。

### dkd_scale

```toml
fuzzy_pid.dkd_scale = 0.0
```

`dKd` 的放大倍率。

当前 normal 档为 0，表示模糊规则不会改变 `Kd`。

如果连续弯回正过冲明显，或者进入第二个弯时车身姿态收不住，可以考虑让 D 项参与：

```text
dkd_scale = 0.05 ~ 0.15
kd_base = 0.03 ~ 0.06
```

但 D 项会放大视觉噪声，调的时候要慢慢加。

### gain_update_alpha

```toml
fuzzy_pid.gain_update_alpha = 0.35
```

PID 增益平滑系数。

```text
K_applied[k] = (1 - alpha) * K_applied[k-1] + alpha * K_raw[k]
```

调参理解：

```text
alpha 大：参数变化快，响应快，但可能跳动
alpha 小：参数变化慢，更稳，但连续弯可能跟不上
```

一般范围：

```text
0.2 ~ 0.5
```

### rule_dkp

```toml
fuzzy_pid.rule_dkp = [...]
```

决定 `Kp` 如何随 `e` 和 `de` 改变。

现在这张表已经改成左右对称思路：

```text
|e| 大：Kp 增大
|e| 小：Kp 接近基础值
e 与 de 同号：误差在发散，Kp 略增
e 与 de 异号：误差在回收，Kp 不继续猛增
```

为什么要左右对称：

```text
Kp 是增益，不是方向输出
方向应该由 e 的正负决定
如果 Kp 跟着方向变正负，很容易出现某一侧弯道 Kp 被压没
```

### rule_dki

```toml
fuzzy_pid.rule_dki = [...]
```

决定 `Ki` 如何随 `e` 和 `de` 改变。

但当前 normal 档：

```text
dki_scale = 0
position_max_integral = 0
```

所以这张表暂时基本不会影响实际控制。

### rule_dkd

```toml
fuzzy_pid.rule_dkd = [...]
```

决定 `Kd` 如何随 `e` 和 `de` 改变。

但当前 normal 档：

```text
dkd_scale = 0
```

所以这张表暂时不会影响 normal 档实际控制。

straight 档目前 `dkd_scale = 0.18`，所以 straight 下 D 项模糊调参是生效的。

## 15. 当前 normal 档的实际效果

当前 normal 关键配置为：

```toml
fuzzy_pid.kp_base = 10.0
fuzzy_pid.ki_base = 0.0
fuzzy_pid.kd_base = 0.02
fuzzy_pid.dkp_scale = 10.0
fuzzy_pid.dki_scale = 0.0
fuzzy_pid.dkd_scale = 0.0
```

因此 normal 档实际可以简化理解为：

```text
Kp = 10 + 模糊调节量
Ki = 0
Kd = 0.02
```

也就是：

```text
模糊主要只在调 Kp
D 项只有一个固定基础值
I 项不工作
```

这套参数的优点是简单、不容易因为 I 项累积而乱跑。缺点是连续弯、反向弯时，对误差变化趋势的阻尼和预判偏弱。

## 16. 当前 straight 档的实际效果

当前 straight 关键配置为：

```toml
fuzzy_pid.kp_base = 4.0
fuzzy_pid.ki_base = 0.0
fuzzy_pid.kd_base = 0.0
fuzzy_pid.dkp_scale = 8.0
fuzzy_pid.dki_scale = 0.05
fuzzy_pid.dkd_scale = 0.18
```

但因为：

```text
position_max_integral = 0
```

所以 `Ki` 实际仍然不明显。

straight 档主要是：

```text
Kp 模糊调节 + Kd 模糊调节
```

## 17. 如何用遥测判断问题

重点看这些字段：

```text
pid_common_fuzzy_enabled
pid_common_fuzzy_e_norm
pid_common_fuzzy_de_norm
pid_common_fuzzy_dkp
pid_common_fuzzy_dki
pid_common_fuzzy_dkd
pid_common_fuzzy_kp_applied
pid_common_fuzzy_ki_applied
pid_common_fuzzy_kd_applied
pid_common_position_pid_output
pid_common_position_feedforward_output
pid_common_raw_steering_output
pid_common_clamped_steering_output
pid_common_applied_steering_output
```

### 情况 A：后续弯道无响应

如果看到：

```text
|e_norm| 很大
Kp_applied 很小
position_pid_output 很小
```

说明模糊规则或 `kp_base/dkp_scale` 太保守。

如果看到：

```text
raw_steering_output 很大
applied_steering_output 很小
```

说明不是 PID 不响应，而是轮速上下限限制了差速。

如果看到：

```text
e_norm 还没起来，但车已经需要转弯
```

说明 line_error 取点偏近、滤波太慢，或者视觉中线没有提前看到弯道。

### 情况 B：左右摆动

如果看到：

```text
Kp_applied 很高
steering_output 正负快速切换
```

可以尝试：

```text
降低 dkp_scale
降低 kp_base
增大 e_scale
增加一点 kd_base 或 dkd_scale
```

### 情况 C：进弯慢半拍

如果看到：

```text
e_norm 上升慢
Kp_applied 上升也慢
```

可以尝试：

```text
降低 e_scale
提高 gain_update_alpha
提高 dkp_scale
调整 line_error_prefix_ratio，让误差取点更靠前或更适合弯道
```

### 情况 D：回正过冲

如果看到：

```text
e 已经开始回收，但 steering 还很大
```

可以尝试：

```text
降低 dkp_scale
增加 kd_base
增加 dkd_scale
降低 position_feedforward_first_diff_gain 或 second_diff_gain
```

## 18. 推荐调参顺序

不要一次动太多。推荐顺序：

1. 先固定 `Ki = 0`，积分先别开。
2. 先调 `e_scale`，让 `e_norm` 在正常弯道大约能到 `0.5 ~ 0.9`。
3. 再调 `de_scale`，让连续弯和反向弯时 `de_norm` 能明显变化，但不要长期顶到 `±1`。
4. 再调 `kp_base`，确定基础响应够不够。
5. 再调 `dkp_scale`，确定大误差时是否有足够纠偏力度。
6. 如果过冲或连续弯姿态收不住，再加 `kd_base/dkd_scale`。
7. 最后才考虑 `Ki` 和 `position_max_integral`，用于修长期偏差。

## 19. 一组实用判断公式

如果你想从日志估算 `e_scale`：

```text
e_scale ≈ P90(|e|)
```

也就是取一段正常跑车日志里 `|control_error_px|` 的 90 分位数。

如果你想从日志估算 `de_scale`：

```text
de[k] = (e[k] - e[k-1]) / dt[k]
de_scale ≈ P90(|de|)
```

没有日志时可用帧率估算：

```text
de_scale = 目标单帧误差变化量 * FPS
```

例如：

```text
5 px/frame * 60 FPS = 300 px/s
```

## 20. 最重要的理解

模糊 PID 的关键不是让规则表“直接告诉车往哪边打方向”。

当前工程里，方向由误差 `e` 的正负决定：

```text
PID 输出方向 ≈ sign(e)
```

模糊表应该回答的是：

```text
在当前误差大小和变化趋势下，PID 增益应该更强还是更弱？
```

所以对于 `rule_dkp` 来说，通常应该更关注：

```text
|e| 大不大？
误差是在发散还是回收？
```

而不是：

```text
e 是正还是负？
```

这就是为什么现在把 `rule_dkp` 改成左右对称表：它更符合“调增益”的本质。
