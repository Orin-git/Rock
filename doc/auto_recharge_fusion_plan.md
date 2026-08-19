# 自动回充融合方案 —— 修改建议与实施手册

> **方案名称**：Laser-Aided Centerline Dock（激光反光条辅助的中心线对接）
> **目标机器**：192.168.0.217（`auto_recharge_ros2`，ROS2 / C++）
> **参考实现**：本地 `auto_charger.py`（ROS1 / Python，激光反光条检测）
> **文档日期**：2026-08-05
> **状态**：待评审 → Phase 1 验证

---

## 目录

1. [背景与动机](#1-背景与动机)
2. [总体设计](#2-总体设计)
3. [传感器优先级与仲裁规则](#3-传感器优先级与仲裁规则)
4. [具体修改建议（代码级）](#4-具体修改建议代码级)
5. [新增/修改的参数配置](#5-新增修改的参数配置)
6. [Topic 与接口设计](#6-topic-与接口设计)
7. [实施计划（三阶段）](#7-实施计划三阶段)
8. [测试验证清单](#8-测试验证清单)
9. [风险与回退策略](#9-风险与回退策略)
10. [附录：核心代码参考实现](#10-附录核心代码参考实现)

---

## 1. 背景与动机

### 1.1 217 现状痛点

217 当前 `auto_recharge_ros2`（Centerline Dock FSM）在实车中暴露的核心问题：

| # | 痛点 | 根因 | 当前的补偿手段 |
|---|------|------|----------------|
| P1 | 贴桩时 AMCL 卡死，地图坐标不可信 | 充电桩附近特征退化，AMCL 粒子不更新 | 里程计累计兜底、开环拉开 |
| P2 | 搜桩需原地转满 ~360°，耗时 30~45s | 单通道红外（0~3）无方向信息，只能扫圈找峰值 | `Scan/SpinFull` 子状态 |
| P3 | 横向偏差无法感知 | 单通道红外只有强度，没有左右 | 只能依赖 Nav2 到点精度 + 重导航 |
| P4 | 强依赖充电桩航点标定精度 | 几何完全来自 `charger.yaml` / Web 下发的 yaw | 无 |

### 1.2 本地 `auto_charger.py` 的可借鉴资产

本地 Python 版虽然整体架构（纯开环、无重试、无状态上报）不如 217，但其**桩体检测器**很有价值：

- `ReflactionDetector`：从激光扫描中提取高强度反光段（intensity > 200），按编码 `[0.06, 0.025, 0.08, 0.025, 0.06]`（宽/间隔交替）做模式匹配，直接输出充电桩在 `base_laser_link` 坐标系下的 (x, y, yaw)。
- **一帧激光即得桩位**（~50ms），无需转圈，无需地图，天然免疫 AMCL 卡死。

### 1.3 融合的核心思想

> **激光做定位修正，红外做对接触发，MCU 做最终闭环。各司其职，互不替代。**

- 激光反光条：解决 P1/P2/P3（近场桩位直接可观）
- 红外信号：保留为对接授权的唯一硬件依据（MCU 闭环的前提）
- Nav2 + 航点：保留为远距离粗定位手段（不变）

---

## 2. 总体设计

### 2.1 修改后的 FSM

```
                    ┌──────────────────────────────────────┐
                    │              Idle                     │
                    └──────────────┬───────────────────────┘
                                   │ auto_charge_command=true / 低电压自动触发
                                   ▼
                    ┌──────────────────────────────────────┐
                    │  Nav — Nav2 导航到 centerline 接近点    │  ← 不变
                    │  (standoff=0.60m, 朝向=yaw+π)          │
                    └──────────────┬───────────────────────┘
                                   ▼
                    ┌──────────────────────────────────────┐
                    │  Align — 原地转到车尾对桩              │  ← 不变
                    └──────────────┬───────────────────────┘
                                   ▼
        ┌─────────────────────────────────────────────────────┐
        │  Acquire — ★ 改造点 ★                                │
        │                                                       │
        │   读激光帧 → ReflactionDetector 检测桩位              │
        │      │                          │                     │
        │   检测到(带置信度)            未检测到                  │
        │      │                          │                     │
        │      ▼                          ▼                     │
        │   用激光位姿修正            回退: 原红外流程            │
        │   目标位姿(base_link       (Scan 扫圈, 现状逻辑)       │
        │   坐标系, 不依赖AMCL)            │                     │
        │      │                          │                     │
        │      └──────────┬───────────────┘                     │
        │                 ▼                                     │
        │         IR >= min_red_for_dock 且稳定 red_stable_sec?  │
        └────────────────┬──────────────────────────────────────┘
                         ▼
        ┌─────────────────────────────────────────────────────┐
        │  Scan — 降级为 fallback（激光失效时才进入）            │
        └────────────────┬─────────────────────────────────────┘
                         ▼
        ┌─────────────────────────────────────────────────────┐
        │  Dock — armChassisDock → /set_charge → MCU 闭环      │  ← 不变
        └────────────────┬─────────────────────────────────────┘
                         ▼
              charging_flag=true → 成功
                         │
              超时/失败 → Retreat → 重试（现状逻辑不变）
```

### 2.2 关键设计决策

| 决策点 | 选择 | 理由 |
|--------|------|------|
| 激光检测结果使用的坐标系 | `base_laser_link` → `base_link`，**不走 map** | 规避 AMCL 卡死（P1 的根因） |
| Scan 阶段是否删除 | **保留但降级为 fallback** | 激光失效（脏污/遮挡/新桩未贴条）时不丢失现有能力 |
| IR 是否仍是对接必要条件 | **是** | MCU 对接闭环依赖红外，ROS 层不重新发明 |
| 反光条编码 | **做成 yaml 可配置** | 不同场地桩体贴法不同，硬编码是本地版的缺陷 |
| 检测失败的表现 | 静默回退红外流程，**不新增失败模式** | 融合不能降低现有成功率 |

---

## 3. 传感器优先级与仲裁规则

### 3.1 仲裁表

| 激光检测 | 红外信号 | 行为 |
|----------|----------|------|
| ✅ 有效（高置信） | ≥ 阈值且稳定 | **用激光位姿微调对准 → armChassisDock**（最快路径） |
| ✅ 有效 | 不足/无 | 用激光位姿做横向纠偏（沿 centerline 横移修正），等待 IR；N 秒内 IR 仍无 → 短距前后微挪 |
| ❌ 未检测到 | ≥ 阈值且稳定 | 忽略激光缺失，走现有逻辑直接对接（信 IR） |
| ❌ 未检测到 | 不足/无 | 回退 `Scan` 扫圈（现有 fallback） |

### 3.2 置信度定义（激光检测）

一帧激光的检测结果满足以下条件才视为「有效」：

1. 编码模式完整匹配：5 段（宽/间隔）误差均在 `laser_code_tol`(默认 0.02m) 内
2. 桩位落在合理窗口：x ∈ [0.15, 1.5]m，y ∈ [-0.8, 0.8]m（base_laser_link 系）
3. **多帧一致性**：连续 `laser_confirm_frames`(默认 3) 帧检测结果的桩中心位置 std < 0.03m

> 多帧一致性是抗误检的关键：偶发的环境反光（玻璃、金属腿）不会连续多帧稳定在同一位置。

### 3.3 坐标换算链

```
LaserScan (base_laser_link)
    │  ReflactionDetector::detect()
    ▼
桩位 PoseStamped (base_laser_link)
    │  tf2: base_laser_link → base_link   ← 静态外参，永远可信
    ▼
桩位 (base_link)  ←── 用于近场对准/纠偏
    │
    ├─(可选, 仅调试用)─ tf2: base_link → map → 发布 marker
```

**注意**：整个近场流程**禁止**经过 `map` 坐标系使用激光检测结果，否则重新引入 AMCL 依赖。

---

## 4. 具体修改建议（代码级）

涉及文件（217 上）：

```
njau/new_nav2_ws/src/auto_recharge_ros2/
├── include/auto_recharge_ros2/
│   ├── auto_recharger.hpp              ← 修改
│   └── reflaction_detector.hpp         ← 新增
├── src/
│   ├── auto_recharger.cpp              ← 修改（Acquire/Align 注入激光修正）
│   └── reflaction_detector.cpp         ← 新增
├── robot_info.yaml                     ← 新增参数段
└── CMakeLists.txt                      ← 新增源文件 + laser_scan 依赖
```

### 4.1 新增 `ReflactionDetector`（移植本地 Python 逻辑）

**职责单一**：输入一帧 `sensor_msgs::msg::LaserScan`，输出 `std::optional<Pose2D>`（base_laser_link 系下的桩位）。不订阅 topic、不碰 TF、不持有状态（多帧一致性由调用方维护）。

接口设计：

```cpp
// reflaction_detector.hpp
#pragma once
#include <optional>
#include <vector>
#include "sensor_msgs/msg/laser_scan.hpp"

namespace auto_recharge_ros2
{

struct ReflactionSegment {
  double theta1, rho1;   // 段首
  double theta2, rho2;   // 段尾
};

struct LaserChargerDetection {
  double x;        // base_laser_link 系
  double y;
  double yaw;      // 桩朝向（垂直于反光条连线）
  double range;    // 桩中心距离
  int    matched_segments;
};

class ReflactionDetector
{
public:
  struct Params {
    double intensity_threshold = 200.0;       // 反光强度阈值
    std::vector<double> code =                // 编码: [宽,间隔,宽,间隔,宽]
      {0.06, 0.025, 0.08, 0.025, 0.06};
    double code_tol = 0.02;                   // 每段匹配容差
    double min_x = 0.15, max_x = 1.5;         // 合理窗口
    double min_y = -0.8, max_y = 0.8;
    size_t min_points_per_segment = 2;
  };

  explicit ReflactionDetector(Params p = Params{});

  std::optional<LaserChargerDetection> detect(
    const sensor_msgs::msg::LaserScan & scan) const;

private:
  std::vector<ReflactionSegment> findSegments(
    const sensor_msgs::msg::LaserScan & scan) const;
  Params params_;
};

}  // namespace auto_recharge_ros2
```

检测算法（与本地版等价，逐一对应）：

```
findSegments:  遍历 ranges/intensities
               intensity > threshold 的连续点聚成段
               段长 >= min_points_per_segment 才保留
detect:        1) segments 数量 < (code.size()+1)/2 → nullopt
               2) 过滤窗口外的段（段中心 x/y 超界丢弃）
               3) 滑动匹配 code: 段宽 ↔ code[偶数位], 段间距 ↔ code[奇数位]
                  全部命中 → 用首段首点 + 末段末点构成桩位
               4) yaw = atan2(y2-y1, x2-x1) + π/2   (反光条连线的法线)
```

> ⚠️ 移植时注意本地 Python 版的一个已知遗留问题：`Reflaction.push_forward()` 引用了被注释掉的 `scipy R`，移植时**不要**带这个包袱——C++ 版根本不需要该方法。

### 4.2 修改 `AutoRecharger`：注入激光修正

在 `auto_recharger.hpp` 新增成员：

```cpp
// --- laser-aided dock (新增段) ---
std::unique_ptr<ReflactionDetector> laser_detector_;
rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;   // 只存最新一帧
std::deque<LaserChargerDetection> recent_detections_;  // 多帧一致性窗口

void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
std::optional<Pose2D> laserChargerInBaseLink();   // 带多帧一致性确认
bool tryLaserRefineAlign();                        // Align/Acquire 中调用
```

修改点 1 — `runAlign()`：对准收敛后、进入 Acquire 前，插入激光精修：

```cpp
// runAlign() 内, aligned == true 之后:
if (aligned) {
  publishStopCmd();
  // ★ 新增: 激光二次确认/精修
  if (tryLaserRefineAlign()) {
    RCLCPP_INFO(get_logger(), "激光精修完成, 横向残差已消除");
  }
  enterAcquire();
  return;
}
```

修改点 2 — `runAcquire()`：用激光做横向纠偏，替代"干等红外"：

```cpp
void AutoRecharger::runAcquire()
{
  // ★ 新增: 激光纠偏优先于红外等待
  if (auto laser_pose = laserChargerInBaseLink()) {
    if (std::abs(laser_pose->y) > laser_lateral_correct_min_) {  // 默认 0.04m
      // 桩在横向有偏差: 小角速度旋转消横向(差速车只能以转代移)
      publishLateralCorrection(*laser_pose);
      return;
    }
    // 横向已居中: 记录精确桩距, 供 Dock 阶段日志/判定
    laser_dock_range_ = laser_pose->x;
  }

  // 以下为现有逻辑保持不变:
  if (red_count_ >= 2) { ... startScan(); ... }   // 注意: Scan 内部也要改(见下)
  ...
}
```

修改点 3 — `startScan()`：改为先问激光，激光有答案就不转圈：

```cpp
void AutoRecharger::startScan()
{
  // ★ 新增: 激光已给出桩位 → 直接对准, 跳过转圈
  if (auto laser_pose = laserChargerInBaseLink()) {
    RCLCPP_INFO(get_logger(),
      "激光直接定位桩体(x=%.2f y=%.2f), 跳过360°扫圈",
      laser_pose->x, laser_pose->y);
    // 直接按激光 yaw 精对准, 然后视 IR 决定是否 armChassisDock
    scan_step_ = ScanStep::SettlePeak;
    scan_best_yaw_ = current_pose_->yaw + laserRelativeYaw(*laser_pose);
    scan_best_red_ = red_count_;
    return;
  }
  // ---- 以下为现有转圈逻辑, 原样保留(fallback) ----
  scan_step_ = ScanStep::SpinFull;
  ...
}
```

修改点 4 — 失败原因上报扩展：激光通道的状态纳入 `recharge_fail_reason`，便于网页/现场诊断：

```
现有:  "对接|对接超时未充上电"
新增:  "对接|对接超时未充上电|laser=ok(0.42m) ir=2"
       "搜桩|激光未检出且扫圈无红外|laser=none ir=0"
```

### 4.3 Dock 阶段不动

`armChassisDock()` / `runDockWatch()` / Retreat 体系**完全不修改**。MCU 闭环和 IR 授权链保持原样——这是融合方案"不增加失败模式"承诺的核心。

---

## 5. 新增/修改的参数配置

`robot_info.yaml` 新增段（全部有默认值，缺省不启用激光辅助 = 行为与现状完全一致）：

```yaml
robot_info:
  # ===== 现有参数保持不变 =====
  approach_standoff: 0.60
  min_red_for_dock: 2
  # ... 略 ...

  # ===== 新增: 激光反光条辅助对接 =====
  laser_dock_enabled: true            # 总开关; false = 与现状完全一致
  laser_intensity_threshold: 200.0    # 反光强度阈值(按雷达型号实测调)
  laser_code: [0.06, 0.025, 0.08, 0.025, 0.06]   # 桩体反光条编码
  laser_code_tol: 0.02
  laser_window_min_x: 0.15
  laser_window_max_x: 1.5
  laser_window_min_y: -0.8
  laser_window_max_y: 0.8
  laser_confirm_frames: 3             # 多帧一致性帧数
  laser_confirm_std_m: 0.03           # 多帧位置标准差上限
  laser_lateral_correct_min: 0.04     # 横向纠偏触发阈值(m)
  laser_lateral_correct_omega: 0.15   # 纠偏角速度上限(rad/s)
```

**调参指引**：

| 参数 | 怎么调 |
|------|--------|
| `laser_intensity_threshold` | 实车对桩录一帧 scan，`rostopic echo /scan` 看反光条 intensity 与背景的比值，取背景最大值与反光最小值之间 |
| `laser_code` | 卡尺量桩上每段反光条宽度和间隔，按 [宽,隔,宽,隔,宽] 填 |
| `laser_confirm_frames` | 环境反光滑多（玻璃门厅）→ 调大到 5；稳定环境保持 3 |

---

## 6. Topic 与接口设计

### 6.1 新增 Topic

| Topic | 类型 | 方向 | 用途 |
|-------|------|------|------|
| `/charger_laser_detection` | `geometry_msgs::msg::PoseStamped` | 发布 | 激光检出的桩位（base_link 系），供 RViz/录包诊断 |
| `/charger_laser_marker` | `visualization_msgs::msg::MarkerArray` | 发布 | 桩位 + 反光段可视化（低频率, 1Hz） |

### 6.2 不修改的接口（兼容性承诺）

- `/auto_charge_command` (Bool) — Web 触发，语义不变
- `/recharge_check` (Int8) — 阶段码 0/1/2/3/4/5 数值与含义全部保留（laser_safety 和网页依赖）
- `/recharge_fail_reason` (String) — 仅扩展后缀，不改前缀格式
- `/robot_recharge_flag`、`/set_charge` — MCU 链路不动
- 桩位标定链（`charger.yaml` / `/charger_position_update` / JSON）不动

### 6.3 订阅新增

| Topic | 类型 | 说明 |
|-------|------|------|
| `/scan` | `sensor_msgs::msg::LaserScan` | 仅缓存最新帧；检测只在 Acquire/Scan 阶段按需调用，**不是每帧都算**（CPU 友好） |

---

## 7. 实施计划（三阶段）

### Phase 1 — 零风险数据验证（预计 0.5 天）

**目标**：确认 217 的雷达 + 217 的充电桩，反光条编码检测在原理上可行。

- [ ] 1.1 确认 217 充电桩是否已贴反光条；未贴则按编码 `[0.06, 0.025, 0.08, 0.025, 0.06]` 补贴（总宽约 20cm，贴在激光扫描高度）
- [ ] 1.2 实车将机器人置于桩前 0.4m / 0.8m / 1.2m 三个距离，正对着录 `ros2 bag record /scan` 各 10 秒
- [ ] 1.3 斜向 30°/45° 各录一组（验证斜视角下编码是否仍可匹配）
- [ ] 1.4 离线跑 Python 版检测器（直接复用本地 `ReflactionDetector` 逻辑读 bag）统计：检出率、位姿抖动 std、误检次数
- [ ] 1.5 记录环境背景 intensity 分布，确定 `laser_intensity_threshold`

**通过标准**：正对检出率 ≥ 95%，位姿 std < 3cm，无误检。
**不通过则**：换反光条材质/调整编码后重测；仍不行 → 终止方案，无代码改动，零损失。

### Phase 2 — 检测器移植与独立验证（预计 1 天）

**目标**：C++ 检测器上线，但**不接 FSM**，只发布诊断 topic。

- [ ] 2.1 新增 `reflaction_detector.hpp/.cpp`（按 §4.1 接口）
- [ ] 2.2 单元测试：用 Phase 1 录的 bag 做回归（检出位姿与离线 Python 结果差 < 1cm）
- [ ] 2.3 接入 `AutoRecharger`：仅订阅 `/scan`、发布 `/charger_laser_detection` + marker
- [ ] 2.4 实车对比验证：机器人在桩前缓慢横移，激光桩位 vs 实际尺量偏差 < 3cm
- [ ] 2.5 观察 1~2 天正常运行期间的误检日志（`laser_dock_enabled` 此时仍为 false，只记录不动作）

**通过标准**：连续运行无误检导致的异常日志；CPU 占用增加 < 2%。
**回退成本**：关闭开关或 revert 两个文件即可。

### Phase 3 — FSM 融合与实车回归（预计 1.5 天）

**目标**：激光修正进入 Acquire/Scan，全链路实车验证。

- [ ] 3.1 实现 §4.2 的四个修改点（runAlign 精修 / runAcquire 纠偏 / startScan 短路 / fail_reason 扩展）
- [ ] 3.2 `laser_dock_enabled: true` 灰度开启
- [ ] 3.3 多方位回归矩阵（见 §8）
- [ ] 3.4 故障注入：遮挡反光条 → 确认自动回退扫圈流程且最终对接成功
- [ ] 3.5 连续 50 次回充成功率统计，与基线（融合前 50 次）对比
- [ ] 3.6 更新网页/运维文档：`recharge_fail_reason` 新后缀说明

**通过标准**：成功率不低于基线；平均回充耗时下降（预期省掉扫圈 20~40s）；激光路径占比 > 70%。

### 里程碑与工时汇总

| 阶段 | 工时 | 风险 | 产出 |
|------|------|------|------|
| Phase 1 | 0.5 天 | 零（不改代码） | 可行性结论 + 阈值参数 |
| Phase 2 | 1.0 天 | 低（只加不动作的诊断） | C++ 检测器 + 精度报告 |
| Phase 3 | 1.5 天 | 中（有开关可秒级回退） | 融合上线 + 对比报告 |
| **合计** | **3 天** | | |

---

## 8. 测试验证清单

### 8.1 功能回归矩阵（Phase 3 必测）

| # | 起始条件 | 预期行为 |
|---|----------|----------|
| T1 | 桩正前方 2m，正对 | Nav→Align→激光直接定位→Dock，全程无扫圈 |
| T2 | 桩侧方 1.5m，横向偏 0.4m | Nav 到点残留横向误差 → Acquire 激光纠偏 → Dock |
| T3 | 桩前 0.3m 贴桩启动 | 开环拉开（现状逻辑）→ 激光定位 → Dock |
| T4 | 反光条完全遮挡 | 激光不检出 → 自动回退扫圈 → IR 对接成功 |
| T5 | 环境强反光干扰（玻璃旁） | 多帧一致性过滤误检 → 不纠偏或正确纠偏 |
| T6 | 低电压自动触发 | 全流程与手动触发一致 |
| T7 | 对接中拔掉充电（IR 有/无两种情况） | Retreat→重试→成功或按现有规则失败 |
| T8 | 回充中 Web 下发停止 | 立即 hardStop，与现状一致 |
| T9 | AMCL 故意卡死（贴桩时） | 激光路径不依赖 map→base，照常对接（核心验证项） |
| T10 | 无反光条的老桩 | `laser_dock_enabled` 下行为与现状完全一致 |

### 8.2 性能指标

| 指标 | 基线（融合前） | 目标（融合后） |
|------|----------------|----------------|
| 平均回充总耗时 | ___ s（实测填写） | 下降 ≥ 15s |
| 扫圈触发率 | ___ % | < 30% |
| 50 次成功率 | ___ % | ≥ 基线 |
| 对接横向残差 | ___ cm | < 4 cm |

---

## 9. 风险与回退策略

| 风险 | 等级 | 缓解措施 |
|------|------|----------|
| 反光条编码与桩体强耦合，换桩失效 | 中 | 编码全部 yaml 化；Phase 1 先行验证 |
| 环境反光误检导致错误纠偏 | 中 | 编码模式匹配 + 窗口过滤 + 多帧一致性三重防线；IR 始终是最终授权 |
| 激光与 IR 结论冲突 | 低 | §3.1 仲裁表：IR 优先于激光做对接决策，激光只做位置修正 |
| 增加 /scan 订阅的 CPU 开销 | 低 | 仅缓存最新帧，检测按需调用（Acquire/Scan 阶段 5Hz 足够） |
| FSM 改动引入新死锁 | 中 | 所有激光分支都有超时兜底落回现有逻辑；session 硬超时 180s 不变 |
| 融合后行为与 laser_safety 假设不符 | 低 | phase 语义完全保留，laser_safety 无感知 |

**一键回退**：`robot_info.yaml` 设 `laser_dock_enabled: false` → 行为与融合前逐字节一致（Phase 2/3 所有代码路径都被该开关短路）。

---

## 10. 附录：核心代码参考实现

### 10.1 `reflaction_detector.cpp` 核心算法（C++ 移植）

```cpp
#include "auto_recharge_ros2/reflaction_detector.hpp"
#include <cmath>

namespace auto_recharge_ros2
{

static double segWidth(const ReflactionSegment & s)
{
  // 余弦定理: 段首末点弦长
  return std::sqrt(s.rho1 * s.rho1 + s.rho2 * s.rho2 -
                   2.0 * s.rho1 * s.rho2 * std::cos(s.theta1 - s.theta2));
}

static double segGap(const ReflactionSegment & a, const ReflactionSegment & b)
{
  // a 末点 到 b 首点 的弦长
  return std::sqrt(a.rho2 * a.rho2 + b.rho1 * b.rho1 -
                   2.0 * a.rho2 * b.rho1 * std::cos(a.theta2 - b.theta1));
}

std::vector<ReflactionSegment> ReflactionDetector::findSegments(
  const sensor_msgs::msg::LaserScan & scan) const
{
  std::vector<ReflactionSegment> out;
  std::vector<size_t> cur;
  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    const float inten = (i < scan.intensities.size()) ? scan.intensities[i] : 0.0f;
    if (inten > params_.intensity_threshold &&
        std::isfinite(scan.ranges[i]) && scan.ranges[i] > 0.01f) {
      cur.push_back(i);
    } else {
      if (cur.size() >= params_.min_points_per_segment) {
        out.push_back({
          scan.angle_min + scan.angle_increment * static_cast<double>(cur.front()),
          scan.ranges[cur.front()],
          scan.angle_min + scan.angle_increment * static_cast<double>(cur.back()),
          scan.ranges[cur.back()]});
      }
      cur.clear();
    }
  }
  return out;
}

std::optional<LaserChargerDetection> ReflactionDetector::detect(
  const sensor_msgs::msg::LaserScan & scan) const
{
  const auto segs = findSegments(scan);
  const size_t need = (params_.code.size() + 1) / 2;   // 需要的段数
  if (segs.size() < need) return std::nullopt;

  // 窗口过滤后的有效段
  std::vector<ReflactionSegment> valid;
  for (const auto & s : segs) {
    const double cx = (s.rho1 * std::cos(s.theta1) + s.rho2 * std::cos(s.theta2)) / 2.0;
    const double cy = (s.rho1 * std::sin(s.theta1) + s.rho2 * std::sin(s.theta2)) / 2.0;
    if (cx < params_.min_x || cx > params_.max_x ||
        cy < params_.min_y || cy > params_.max_y) continue;
    valid.push_back(s);
  }
  if (valid.size() < need) return std::nullopt;

  // 滑动窗口匹配编码: [宽0, 隔0, 宽1, 隔1, 宽2]
  for (size_t start = 0; start + need <= valid.size(); ++start) {
    size_t ci = 0;
    bool ok = true;
    for (size_t k = 0; k < need && ok; ++k) {
      // 段宽匹配 code[偶数位]
      if (std::fabs(segWidth(valid[start + k]) - params_.code[ci]) > params_.code_tol) {
        ok = false; break;
      }
      ++ci;
      // 段间隔匹配 code[奇数位] (最后一段后无间隔)
      if (k + 1 < need) {
        if (std::fabs(segGap(valid[start + k], valid[start + k + 1]) -
                      params_.code[ci]) > params_.code_tol) {
          ok = false; break;
        }
        ++ci;
      }
    }
    if (!ok) continue;

    // 命中: 首段首点 + 末段末点 构成桩位
    const auto & first = valid[start];
    const auto & last  = valid[start + need - 1];
    const double x1 = first.rho1 * std::cos(first.theta1);
    const double y1 = first.rho1 * std::sin(first.theta1);
    const double x2 = last.rho2  * std::cos(last.theta2);
    const double y2 = last.rho2  * std::sin(last.theta2);

    LaserChargerDetection d;
    d.x   = (x1 + x2) / 2.0;
    d.y   = (y1 + y2) / 2.0;
    d.yaw = std::atan2(y2 - y1, x2 - x1) + M_PI / 2.0;  // 反光条法线 = 桩朝向
    d.range = std::hypot(d.x, d.y);
    d.matched_segments = static_cast<int>(need);
    return d;
  }
  return std::nullopt;
}

}  // namespace auto_recharge_ros2
```

### 10.2 多帧一致性确认（AutoRecharger 侧）

```cpp
std::optional<Pose2D> AutoRecharger::laserChargerInBaseLink()
{
  if (!laser_dock_enabled_ || !latest_scan_ || !laser_detector_) return std::nullopt;

  auto det = laser_detector_->detect(*latest_scan_);
  if (!det) { recent_detections_.clear(); return std::nullopt; }

  recent_detections_.push_back(*det);
  if (recent_detections_.size() > static_cast<size_t>(laser_confirm_frames_)) {
    recent_detections_.pop_front();
  }
  if (recent_detections_.size() < static_cast<size_t>(laser_confirm_frames_)) {
    return std::nullopt;   // 帧数不足, 不采信
  }

  // 位置标准差检查
  double mx = 0, my = 0;
  for (const auto & d : recent_detections_) { mx += d.x; my += d.y; }
  mx /= recent_detections_.size(); my /= recent_detections_.size();
  double var = 0;
  for (const auto & d : recent_detections_) {
    var += (d.x - mx) * (d.x - mx) + (d.y - my) * (d.y - my);
  }
  if (std::sqrt(var / recent_detections_.size()) > laser_confirm_std_m_) {
    return std::nullopt;   // 抖动过大, 疑似误检/运动模糊
  }

  // base_laser_link → base_link (静态外参, 不经 map)
  try {
    auto tf = tf_buffer_->lookupTransform(
      "base_link", "base_laser_link", tf2::TimePointZero);
    geometry_msgs::msg::PoseStamped in, out;
    in.header.frame_id = "base_laser_link";
    in.pose.position.x = mx;
    in.pose.position.y = my;
    in.pose.orientation.z = std::sin(recent_detections_.back().yaw / 2.0);
    in.pose.orientation.w = std::cos(recent_detections_.back().yaw / 2.0);
    tf2::doTransform(in, out, tf);

    Pose2D p;
    p.x = out.pose.position.x;
    p.y = out.pose.position.y;
    p.yaw = tf2::getYaw(out.pose.orientation);
    p.valid = true;
    return p;
  } catch (const tf2::TransformException &) {
    return std::nullopt;
  }
}
```

### 10.3 与本地 Python 版的对照表

| 本地 Python | 217 C++ 移植 | 说明 |
|-------------|--------------|------|
| `Reflaction` 类 | `ReflactionSegment` struct | 去掉 `push_forward()`（依赖被注释的 scipy，死代码） |
| `ReflactionDetector._encode` | `Params::code`（yaml 可配） | 硬编码 → 配置化 |
| `find_reflaction()` | `findSegments()` | 逻辑等价；C++ 版增加 NaN/inf 防御 |
| `find_charger()` | `detect()` | 逻辑等价；返回 optional 替代 None |
| `Reflaction.get_pose()` | `detect()` 内的位姿合成 | yaw_offset/y_offset 保留为 0，不调 |
| `/scan` 回调里实时检测 | 缓存最新帧 + 按需检测 | 降低 CPU，避免每帧匹配 |
| `base_laser_link → odom → 广播 TF` | `base_laser_link → base_link` 直接用 | 去掉 odom/map 环节 = 免疫 AMCL 卡死 |

---

## 变更记录

| 日期 | 版本 | 内容 |
|------|------|------|
| 2026-08-05 | v1.0 | 初版：基于本地 `auto_charger.py` 反光条检测 + 217 `auto_recharge_ros2` FSM 的融合方案 |
