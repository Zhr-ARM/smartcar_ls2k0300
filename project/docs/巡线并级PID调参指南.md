# 巡线并级 PID 调参指南

本文针对当前工程里的控制结构：

- 速度环：左右电机速度闭环
- 位置环：根据视觉横向误差回中线
- 角速度环：根据视觉生成的目标横摆角速度抑制甩尾、提升高速过弯稳定性
- 巡线层：做视觉误差整形、弯道降速、左右轮目标合成

当前代码对应关系：

- 参数文件：[code/driver/pid/pid_tuning.h](code/driver/pid/pid_tuning.h)
- 巡线主逻辑：[code/app/line_follow_thread/line_follow_thread.cpp](code/app/line_follow_thread/line_follow_thread.cpp)
- IMU 线程：[code/app/imu_thread/imu_thread.cpp](code/app/imu_thread/imu_thread.cpp)

## 1. 当前控制结构

当前巡线控制核心是“位置环 PID + 角速度环 PID 并级”：

```text
视觉误差 e --------------------> 位置环 PID ----------\
                                                      +--> steering_output --> 左右轮目标 --> 速度环
视觉误差/视觉曲率 --> 目标角速度 r_ref --> 角速度环 PID --/
                                     ^
                                     |
                                  IMU gyro_z
```

可简化写成：

```text
u_pos  = PID_pos(e)
r_ref  = k1 * e + k2 * curvature
u_rate = PID_yaw_rate(r_ref - r)
u      = u_pos + u_rate
```

其中：

- `u_pos` 负责“贴中线”
- `u_rate` 负责“让车身转得对，不要甩”
- `u` 是最终差速量

## 2. 参数分区

当前 `pid_tuning.h` 已经按下面 5 个区域拆开：

### 2.1 IMU

命名空间：`pid_tuning::imu`

- `kGyroBiasSampleCount`
- `kGyroYawRateSign`
- `kGyroYawRateFilterAlpha`

用途：

- 决定陀螺仪零偏校准
- 决定角速度正负方向是否和控制方向一致
- 决定 `gyro_z` 是否过抖

### 2.2 速度环

命名空间：`pid_tuning::motor_speed`

这一层先保证左右轮速度能跟住目标，不是本文重点，但它不稳的话，上层很难调好。

### 2.3 位置环

命名空间：`pid_tuning::position_loop`

- `kDynamicKpBase`
- `kDynamicKpQuadA`
- `kDynamicKpMin`
- `kDynamicKpMax`
- `kKi`
- `kKd`
- `kMaxOutput`

用途：

- 决定小车“偏离中线后要多快拉回来”
- 决定大误差时修正有多猛
- 决定是否容易过冲、蛇形

### 2.4 角速度环

命名空间：`pid_tuning::yaw_rate_loop`

- `kVisualCurvatureFilterAlpha`
- `kRefFromErrorGainDps`
- `kRefFromCurvatureGainDps`
- `kRefLimitDps`
- `kKp`
- `kKi`
- `kKd`
- `kMaxIntegral`
- `kMaxOutput`

用途：

- 视觉决定“应该转多快”
- IMU 负责“实际转得是否跟上”
- 主要用来抑制甩尾和过摆

### 2.5 巡线层

命名空间：`pid_tuning::line_follow`

- `kErrorFilterAlpha`
- `kTargetCountMin`
- `kTargetCountMax`
- `kTurnSlowdownStartPx`
- `kTurnSlowdownFullPx`
- `kTurnMinSpeedScale`
- `kTurnSlowdownMaxDropRatioPerCycle`
- `kTurnSlowdownMaxRiseRatioPerCycle`
- `kNormalizedErrorLimit`
- `kErrorDeadzonePx`
- `kErrorLowGainLimitPx`
- `kErrorLowGain`

用途：

- 处理视觉误差毛刺
- 决定弯道降速策略
- 决定最终目标速度限幅

## 3. 调参前必须先确认的事

这一步不要省，否则后面很容易越调越乱。

### 3.1 确认速度环先稳定

如果左右轮速度环本身跟不住目标，会出现：

- 明明位置环输出不大，车还是左右晃
- 直道也抖
- 大弯时一边轮子明显跟不上

如果速度环明显不稳，请先调 `motor_speed`，再调上层。

### 3.2 确认陀螺仪正负方向

最重要的是 `pid_tuning::imu::kGyroYawRateSign`。

判断方法：

1. 抬车或低速慢跑。
2. 让车朝左修正。
3. 看 `gyro_z` 和控制方向是不是同号。

现象判断：

- 一开角速度环就更容易甩：大概率符号反了
- 此时先把 `kGyroYawRateSign` 从 `1.0f` 改成 `-1.0f` 再试

### 3.3 调参时尽量只动一组参数

建议每一轮只动 1 到 2 个参数，不要位置环、角速度环、降速一起改。

### 3.4 建议固定测试环境

如果测试环境一直变，调参结论会很乱。

建议尽量固定：

- 同一块场地
- 同一组电池电量区间
- 同一条赛道
- 同样的灯光
- 同样的轮胎状态

特别是下面这些变化，会明显影响你对参数的判断：

- 电池从满电跑到快没电
- 地面摩擦力变化
- 摄像头曝光变化
- 轮胎表面沾灰或磨损

建议做法：

- 一轮调参尽量在较接近的电量下完成
- 每次测试前把轮胎和赛道表面检查一下
- 如果今天光照变化很大，优先先确认视觉误差是不是也变了

### 3.5 每轮测试前的固定检查清单

建议每次上车前按下面快速过一遍：

1. IMU 是否已正常初始化
2. 上电静止阶段是否足够稳定，便于零偏标定
3. 摄像头画面和赛道中线识别是否正常
4. 左右电机转向是否一致
5. 基础速度是否是本轮计划值
6. 本轮只准备改哪 1 到 2 个参数

如果这 6 项里有一项不确定，最好先别直接跑高速。

## 4. 总体调参顺序

强烈建议按这个顺序来：

1. 先把速度环调稳
2. 先关掉角速度环，只调位置环
3. 位置环能中速稳定跑后，再开角速度环
4. 角速度环先只开 `P`
5. 再加视觉曲率前瞻
6. 最后提速，并微调弯道降速

## 5. 第一步：只调位置环

### 5.1 先临时关闭角速度环

把下面参数先设成 0：

```cpp
pid_tuning::yaw_rate_loop::kKp = 0
pid_tuning::yaw_rate_loop::kKi = 0
pid_tuning::yaw_rate_loop::kKd = 0
```

这样车只靠位置环贴中线，便于先把“基本巡线能力”调出来。

### 5.2 先调哪些参数

重点看这几个：

```cpp
pid_tuning::position_loop::kDynamicKpBase
pid_tuning::position_loop::kDynamicKpQuadA
pid_tuning::position_loop::kKd
pid_tuning::position_loop::kKi
```

建议：

- `kKi` 先保持 `0`
- 先调 `kDynamicKpBase`
- 再调 `kDynamicKpQuadA`
- 最后调 `kKd`

### 5.3 位置环调参方法

#### A. 调 `kDynamicKpBase`

它主要决定：

- 小误差时回中线的积极程度
- 直道附近的跟线能力

现象与处理：

- 直道慢悠悠回不来：加大
- 直道轻微左右摆：减小

建议步长：

- 每次改 `50 ~ 100`

#### B. 调 `kDynamicKpQuadA`

它主要决定：

- 大误差时修正力度
- 入弯或出弯大偏差时，能不能果断拉回来

现象与处理：

- 大弯里拉不回来、总在外侧：加大
- 一进弯就猛打、容易切过头：减小

建议步长：

- 每次改 `50 ~ 150`

#### C. 调 `kKd`

它主要决定：

- 压过冲
- 压“出弯后左右来回摆”

现象与处理：

- 出弯会多摆两下：加一点
- 车开始发硬、发抖、转向不跟手：减一点

建议步长：

- 每次改 `50 ~ 120`

### 5.4 位置环调到什么程度算合格

目标不是一开始就追高速极限，而是先达到：

- 中低速稳定贴线
- 大弯不会完全甩出去
- 允许有一点高速过冲，但不能乱摆

如果这一层还没稳，不建议进入角速度环调参。

### 5.5 位置环推荐实操流程

这一段建议你真的按顺序来，不要跳。

#### 第一步：固定一个偏低基础速度

目的：

- 让问题主要暴露在“位置控制”上
- 不让速度太高掩盖位置环本身的问题

要求：

- 这个速度下车必须能完整跑圈
- 就算出弯有点摆，也不能频繁冲出去

#### 第二步：先只看直道

观察这三件事：

- 直道上有没有左右连续摆
- 小误差能不能自然收回
- 车头是不是总在画小蛇

如果直道都不稳，不要急着去看大弯。

#### 第三步：再看中等弯

观察这三件事：

- 入弯会不会明显慢半拍
- 弯中会不会一直压外侧
- 出弯会不会多摆一两下

这个阶段主要还是位置环主导，所以看到这些问题时，优先先动位置环。

#### 第四步：最后再看大误差恢复能力

可以人为制造一个稍微偏一点的入弯姿态，看它能不能回来。

重点观察：

- 回正是否果断
- 会不会因为修正太猛直接打过头

### 5.6 位置环各参数更细的作用边界

很多时候“感觉不对”不代表该改同一个参数，下面把边界讲细一点。

#### `kDynamicKpBase` 更像“小误差主力”

主要管：

- 直道
- 中线附近
- 轻微偏差修正

不太适合用它解决：

- 大弯拉不回来
- 高速甩尾

如果你用它去硬解决大弯问题，常见结果是：

- 直道变得很敏感
- 小误差也一直左右抽

#### `kDynamicKpQuadA` 更像“大误差强化器”

主要管：

- 大偏差
- 大弯
- 出弯偏出去后的强力拉回

不太适合用它解决：

- 直道细抖
- 出弯连续蛇形

如果 `kDynamicKpQuadA` 太大，常见结果是：

- 一旦误差稍大，车就突然很猛
- 入弯或出弯像被“拽”一下

#### `kKd` 更像“刹车感”

主要管：

- 过冲
- 回正过头
- 出弯蛇形

如果它太小：

- 容易打过头
- 出弯会来回摆

如果它太大：

- 车会发僵
- 方向不跟手
- 有时会出现细碎抖动

### 5.7 位置环常见误区

#### 误区 1：一看到弯里偏外就只加 `kDynamicKpBase`

这通常会让直道先坏掉。

更合理的是先判断：

- 是大误差不够猛，还是
- 根本是高速时车身姿态没跟上

前者更像 `kDynamicKpQuadA`
后者更多是后面角速度环的问题

#### 误区 2：位置环还没稳，就急着加角速度环

这样会出现：

- 你以为是角速度环在救车
- 实际是位置环本身就没建立好

最后两个环会互相掩盖问题。

## 6. 第二步：打开角速度环

位置环跑顺后，再开始调角速度环。

### 6.1 初始策略

一开始不要全开，建议：

- `kKi = 0`
- `kKd = 0`
- 先只调 `kKp`
- `kRefFromCurvatureGainDps` 先可以小一些，甚至先设 `0`

这样最容易看清角速度环到底在帮忙还是在捣乱。

### 6.2 先调目标角速度生成

角速度环不是直接盯 `0`，而是跟踪视觉给的 `r_ref`。

重点看这几个：

```cpp
pid_tuning::yaw_rate_loop::kRefFromErrorGainDps
pid_tuning::yaw_rate_loop::kRefFromCurvatureGainDps
pid_tuning::yaw_rate_loop::kRefLimitDps
```

#### A. 调 `kRefFromErrorGainDps`

它表示：

- 视觉误差越大，要给角速度环多大的“转向速度目标”

现象与处理：

- 车偏了但建立转向姿态太慢：加大
- 一有一点误差就立刻很激进：减小

建议步长：

- 每次改 `10 ~ 30`

#### B. 调 `kRefFromCurvatureGainDps`

它表示：

- 看到前方赛道弯曲时，要不要提前建立横摆速度

现象与处理：

- 入弯总是慢半拍、要到弯里才开始拐：加大
- 还没到弯心就提前切太多：减小

建议步长：

- 每次改 `100 ~ 300`

#### C. 调 `kRefLimitDps`

它表示：

- 视觉最多允许给多大的目标横摆角速度

现象与处理：

- 高速大弯时总觉得“想转但转不起来”：适当加大
- 异常帧时容易突然给很猛的转向：适当减小

建议：

- 这个一般不作为第一优先项
- 先调好前两个，再补这个上限

### 6.3 再调角速度环 PID

重点看：

```cpp
pid_tuning::yaw_rate_loop::kKp
pid_tuning::yaw_rate_loop::kKi
pid_tuning::yaw_rate_loop::kKd
pid_tuning::yaw_rate_loop::kMaxOutput
```

#### A. 先调 `kKp`

这是最关键的参数。

它表示：

- 实际横摆率没跟上目标时，角速度环要补多少差速

现象与处理：

- 高速出弯甩尾、车尾跟不住：加大
- 车变得很僵、入弯不愿意转：减小
- 出现高频细抖：减小

建议步长：

- 每次改 `0.05 ~ 0.20`

#### B. `kKi` 一开始先别动

通常建议：

- 先保持 `0`

因为角速度环的输入是 `gyro_z`，本身更新快、对零偏敏感。

过早加 `Ki` 容易出现：

- 方向慢慢累歪
- 出弯后还残留补偿

只有当你确认：

- `gyro` 零偏稳定
- 长弯中一直有稳定跟踪误差

再少量增加 `kKi`。

建议步长：

- 每次只改很小，例如 `0.001 ~ 0.01`

#### C. `kKd` 通常最后再考虑

因为你控的已经是角速度，天然就比位置误差更接近“变化量”。

所以很多时候：

- 角速度环只用 `P`
- 或 `PI`

就够了。

只有当你发现：

- 横摆响应还是略有拖尾
- `P` 再加大就开始抖

再考虑少量加 `kKd`。

### 6.4 角速度环推荐实操流程

#### 第一步：先确认它是在“帮忙”还是“捣乱”

做法：

- 保持位置环已经能中低速稳定跑
- 只开一个较小的 `yaw_rate_loop::kKp`
- 先不要动 `kKi`、`kKd`

观察：

- 出弯有没有比以前更稳
- 入弯会不会突然不愿意转
- 直道会不会比以前更紧张

结论判断：

- 更稳了：方向大体正确
- 更甩了：先查 `kGyroYawRateSign`
- 更僵了：`kKp` 可能偏大，或者 `r_ref` 给太激进

#### 第二步：先调“目标角速度”，再调“跟踪力度”

很多人会直接猛加 `kKp`，但如果 `r_ref` 本身就不合理，`kKp` 再大也只是更强地跟错目标。

建议顺序：

1. 先让 `kRefFromErrorGainDps` 合理
2. 再让 `kRefFromCurvatureGainDps` 合理
3. 最后调 `kKp`

#### 第三步：区分“进弯问题”和“出弯问题”

这一步非常关键。

如果主要问题是：

- 进弯慢半拍
- 车头不肯建立姿态

优先看：

- `kRefFromCurvatureGainDps`
- `kRefFromErrorGainDps`

如果主要问题是：

- 出弯甩尾
- 车尾跟不上
- 明明方向收了但车身还在摆

优先看：

- `kKp`

#### 第四步：角速度环什么程度算合格

不需要追求“完全没有任何摆动”，更现实的目标是：

- 高速出弯不明显甩尾
- 车头和车身方向比较一致
- 位置环不用再靠特别大的修正去救车

### 6.5 角速度环常见误区

#### 误区 1：把 `kKp` 当万能药

如果 `r_ref` 太小，`kKp` 再大也没用；
如果 `r_ref` 太大，`kKp` 再大只会更紧张。

所以：

- 先看目标角速度给得对不对
- 再看跟踪力度够不够

#### 误区 2：过早加 `kKi`

角速度环的对象是 `gyro_z`，对零偏很敏感。

如果你太早加 `Ki`，常见结果是：

- 长时间跑后输出慢慢偏一边
- 出弯已经回正了，补偿还不肯退

#### 误区 3：把位置环问题误判成角速度环问题

比如：

- 直道一直偏一点
- 小误差贴不住中线

这更像位置环的问题，不要先去猛加角速度环。

## 7. 第三步：联调提速

当位置环和角速度环都基本稳定后，再逐步提基础速度。

建议：

1. 每次只提高一点基础速度
2. 每提一次，只看一个主要问题
3. 先看是否甩尾，再看是否贴中线

常见规律：

- 低速稳、高速甩：优先加角速度环 `kKp`
- 高速不甩但过弯偏外：优先加 `kRefFromCurvatureGainDps`
- 高速能过但出弯蛇形：先加位置环 `kKd`

### 7.1 提速时建议的节奏

不要一次把基础速度提很多。

建议：

- 每次只提高一个小台阶
- 每提高一次，只验证一个结论

例如：

1. 先验证“不会甩”
2. 再验证“还能贴中线”
3. 最后再验证“直道不会开始新一轮摆动”

### 7.2 提速后先看什么

建议优先级：

1. 是否甩尾
2. 是否出弯蛇形
3. 是否弯中贴线
4. 是否直道抖动

因为：

- 甩尾是最先要解决的稳定性问题
- 贴中线是第二层
- 舒适程度和细抖放后面再优化

## 8. 巡线层参数怎么配合

巡线层不是主控制，但它会影响你调参感受。

### 8.1 `kErrorFilterAlpha`

作用：

- 过滤视觉误差抖动

现象与处理：

- 图像误差一帧一跳、方向跟着抽：减小一点
- 响应太慢：加大一点

### 8.2 `kErrorDeadzonePx`

作用：

- 直道小抖动忽略不管

现象与处理：

- 直道一直轻微摆：适当加大
- 中线附近不够贴：适当减小

### 8.3 弯道降速参数

重点：

```cpp
kTurnSlowdownStartPx
kTurnSlowdownFullPx
kTurnMinSpeedScale
```

现象与处理：

- 一进弯就慢太多：减小降速强度
- 高速大弯明显抓不住：加大降速强度

注意：

- 降速策略是辅助
- 真正抑制甩尾还是靠角速度环

## 9. 常见现象与优先调整项

### 9.1 直道轻微左右摆

优先看：

- `position_loop::kDynamicKpBase` 先减
- `position_loop::kKd` 适当加
- `line_follow::kErrorDeadzonePx` 适当加

### 9.2 大弯进不去，总在弯外

优先看：

- `yaw_rate_loop::kRefFromCurvatureGainDps` 加大
- `yaw_rate_loop::kRefFromErrorGainDps` 加大
- `position_loop::kDynamicKpQuadA` 加大

### 9.3 出弯甩尾

优先看：

- `yaw_rate_loop::kKp` 加大
- `imu::kGyroYawRateSign` 是否正确
- `line_follow` 的弯道降速是否太弱

### 9.4 车很僵，不愿意转

优先看：

- `yaw_rate_loop::kKp` 减小
- `yaw_rate_loop::kRefFromCurvatureGainDps` 减小
- `position_loop::kKd` 是否过大

### 9.5 大误差时回正太慢

优先看：

- `position_loop::kDynamicKpQuadA` 加大
- `position_loop::kMaxOutput` 是否卡住

### 9.6 已经不甩，但总贴不住中线

优先看：

- 先调位置环，不要急着继续加角速度环
- `position_loop::kDynamicKpBase`
- `position_loop::kDynamicKpQuadA`
- `position_loop::kKd`

### 9.7 直道稳，但一到连续弯就乱

优先看：

- `yaw_rate_loop::kRefFromCurvatureGainDps` 是否太小
- `yaw_rate_loop::kVisualCurvatureFilterAlpha` 是否太小或太大
- `line_follow::kTurnMinSpeedScale` 是否太高，导致连续弯里速度始终偏快

### 9.8 明明能过弯，但总切内线太多

优先看：

- `yaw_rate_loop::kRefFromCurvatureGainDps` 减小
- `yaw_rate_loop::kRefFromErrorGainDps` 减小
- `position_loop::kDynamicKpQuadA` 是否过大

### 9.9 方向不抖，但左右轮目标经常顶满

优先看：

- `position_loop::kMaxOutput`
- `yaw_rate_loop::kMaxOutput`
- `line_follow::kTargetCountMax`

这类现象说明：

- 上层可能一直在要更大的差速
- 但底层已经到极限

如果一直顶满，调大增益不一定有用，反而要重新平衡“基础速度”和“可用差速范围”。

## 10. 推荐实际调参流程

建议上车按下面执行：

### 第 1 轮

- 角速度环关掉
- 低速跑
- 只调位置环

目标：

- 中低速稳定跟线

### 第 2 轮

- 只开角速度环 `P`
- `kRefFromCurvatureGainDps` 先小一点

目标：

- 出弯不再明显甩尾

### 第 3 轮

- 加曲率前瞻

目标：

- 入弯更提前
- 高速大弯更贴中线

### 第 4 轮

- 提基础速度
- 微调巡线层降速

目标：

- 在更高速度下仍能稳过弯

## 11. 调参时建议做的测试动作

如果每次只是“随便跑一圈”，往往很难判断结论。

建议至少分成下面几类动作来观察：

### 11.1 直道保持

观察：

- 中线附近是否来回摆
- 输出是否一直在小范围震荡

适合验证：

- `position_loop::kDynamicKpBase`
- `position_loop::kKd`
- `line_follow::kErrorDeadzonePx`

### 11.2 单个中等弯

观察：

- 入弯时机
- 弯中贴线
- 出弯是否需要二次修正

适合验证：

- `position_loop::kDynamicKpQuadA`
- `yaw_rate_loop::kRefFromCurvatureGainDps`
- `yaw_rate_loop::kKp`

### 11.3 连续弯

观察：

- 一次弯刚结束，下一次弯是否还能快速建立姿态
- 是否出现节奏跟不上

适合验证：

- `yaw_rate_loop::kVisualCurvatureFilterAlpha`
- `yaw_rate_loop::kRefFromCurvatureGainDps`
- 巡线层降速是否偏保守或偏激进

### 11.4 故意给一点初始偏差

做法：

- 让车不是完美居中地进入一个弯

观察：

- 能不能稳稳拉回
- 是平顺回正，还是猛打猛收

适合验证：

- `position_loop::kDynamicKpQuadA`
- `position_loop::kKd`
- `yaw_rate_loop::kKp`

## 12. 调参记录建议

调参最怕“感觉好像比刚才好一点”，但记不住改了什么。

建议每次试车都记下面这些：

| 轮次 | 基础速度 | 改了哪些参数 | 改前值 | 改后值 | 直道表现 | 入弯表现 | 出弯表现 | 是否甩尾 | 结论 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 1 | 500 | `position_loop::kDynamicKpBase` | 900 | 1000 | 稍稳 | 无明显变化 | 略摆 | 否 | 继续加一点 |
| 2 | 500 | `position_loop::kKd` | 920 | 1000 | 更稳 | 正常 | 摆动减小 | 否 | 保留 |

你也可以用下面这个文本模板：

```text
日期：
赛道：
电量状态：
基础速度：

本轮只改参数：
1.
2.

改前现象：
- 

改后现象：
- 直道：
- 入弯：
- 弯中：
- 出弯：

结论：
- 保留 / 回退 / 下轮继续加 / 下轮继续减
```

## 13. 一组更实用的判断口诀

当你在现场没时间翻完整文档时，可以先记这组短判断：

- 直道摆：先看位置环
- 进弯慢：先看目标角速度
- 出弯甩：先看角速度环 `P`
- 不甩但偏线：回去看位置环
- 一切都想要但轮子顶满：先降速或重分配输出范围

## 14. 当前工程里最值得优先看的参数

如果你不想一开始看太多参数，建议先盯住这 8 个：

### 位置环

- `position_loop::kDynamicKpBase`
- `position_loop::kDynamicKpQuadA`
- `position_loop::kKd`

### 角速度环

- `yaw_rate_loop::kRefFromErrorGainDps`
- `yaw_rate_loop::kRefFromCurvatureGainDps`
- `yaw_rate_loop::kKp`

### IMU / 巡线层

- `imu::kGyroYawRateSign`
- `line_follow::kTurnMinSpeedScale`

如果这 8 个都还没摸清楚，先别急着动更细的参数。

## 15. 推荐的首轮调参策略

如果你希望按“最稳妥”的方式开始，建议这样：

### 首轮

- 角速度环先关闭
- 基础速度用保守值
- 只让位置环达到“稳稳能跑”

### 第二轮

- 打开较小的 `yaw_rate_loop::kKp`
- `kRefFromCurvatureGainDps` 先不加太大

目标：

- 先确认它能减轻甩尾，而不是引入新问题

### 第三轮

- 加曲率前瞻
- 开始提速

目标：

- 提前入弯
- 高速弯也不容易发飘

## 16. 一句话总结

可以把这套调参思路记成一句话：

> 先用位置环把“能跟线”调出来，再用角速度环把“高速不甩”调出来，最后用视觉曲率和降速策略把“高速也能贴中线”补齐。
