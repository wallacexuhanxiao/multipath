# 多解接入新 source 的端到端流程

## 修正上一轮的交付范围

上一轮 `248`、`243` 各交付了一条完整 Oracle V0 轨迹。换源视频不是
同一任务的多解，不能把这两个结果当作“每个规划区间三解”。
此前 `253` 已有三条完整、同源、4.8 秒的连续 PhysX 轨迹；其证据见
[原多解报告](ORACLE_DIVERSITY_RESULT_20261007.md)。

## 本轮接入方式

默认端到端入口现在是：

```text
raw TACO + static oracle schedule
  → local Anchor synthesis
  → one complete physical V0
  → three cuRobo solutions per RELAXED interval
  → two alternative uninterrupted full PhysX rollouts
  → synchronized videos + measured wrist plots
```

基线计作第一解。后续解只排除同一区间已通过的实际 wrist 路径；不同
区间不共用障碍。障碍按区间距离和时间预算自动生成、小→大→小，
只约束 wrist；真实手部、持物与场景碰撞仍然检查。抓握桥中的手指驱动
保持冻结，时间不拉伸。

`scripts/run_oracle_diversity.py` 可直接接入已通过的 V0 结果，不重训
Anchor。`run_end_to_end.py` 默认请求每段三解；显式设
`--solutions-per-interval 1` 可以只运行 V0。
局部 bank 成功和完整 trajectory 成功分别报告；不足三条完整解时，
不会把三解任务标成成功。已存在的第二条可以先做完整验证。

## 实际发现与修复

1. **新流程漏接多解阶段。** 原端到端程序在 fresh V0 成功后退出。
   现在调用现有区间排除/绕行模块，再执行完整替代轨迹。
2. **初始物理准备不一致。** 旧多解脚本重新从 source 手部 pose 构造
   初态。现在复用已通过 V0 的初始驱动准备，并从任务初态实际执行前驱；
   不向任何中间边界赋值。
3. **最终执行证据与规划 artifact 分开。** fresh V0 有最终 trace/命令，
   原候选保存完整 bridge metadata。接入时检查两者命令完全一致、source
   provenance 一致；实际成功仍以 fresh V0 报告为准。
4. **障碍尺寸搜索预算分配错误。** r157 前两段只有两解。旧搜索在大尺寸
   上重复遍历方向，缩小后只剩两个方向额度。改为每个尺寸遍历所有方向，
   r158 将 `[65,74)` 补到三解；后两段原有三解继续保留。
5. **几何成功不足以证明后续 grasp 成功。** r158 第一段有多条真实绕行，
   但随后的同一 A1 core 失败。现在只在观察到这类 failure 后，增加该
   RELAXED 区间末尾保留的已通过移动驱动历史，同时减少可绕行时间对应
   的障碍尺寸；总时长、core、交互阈值、控制器增益保持不变。
6. **比较视频的字幕时间偏了一帧。** 完整 rollout 的 `N+1` 边界记录
   描述 `N` 个视频帧，不代表视频从 `1/fps` 开始。完整比较现在从 `t=0`
   标注，帧数与源时钟一致。
7. **共享 handoff 命令被旧路径障碍挡住。** 实测数据复查显示必须保留的
   39 个末端命令中有两个落入排除区域，最小间隙 −2.070 mm。障碍现在
   只沿可改变的 source-time 前缀生成，并保存时间采样路径以避免把弧长
   采样误当时间采样。同一数据检查得到 +39.876 mm 间隙；78 个非空障碍
   仍同时排除两条旧路径，没有撤销内部路径排除。
8. **把已知的实际出口速度问题接入 repair loop。** r159 从初态真实通过
   替代首段与 A1，但 `[65,74)` 报 `No fixed-time boundary trajectory
   in coordinate 1`。从这个物理前驱接入现有的 Anchor 末端速度 shooting，
   仍要求 padded interaction、fresh core 与后继桥物理验证；不通过就失败。
   该工具的初始化也改为复用实际前驱初始驱动准备。
9. **释放后的自由手终点被错当作任务条件。** r160 的前六段连续通过，
   但最后 0.2 s 要回到旧 wrist 终点，角速度边界使该终点不可达。
   在没有后继 Anchor、物体已释放的末段，改为从实际状态生成原时长内
   的制动目标，仍由 cuRobo 规划并进行实际碰撞、速度和任务检查。
   目标只是规划 proposal，不直接赋给物理状态。
10. **末段复用了下一 Anchor 的手指姿态 gate。** r161 已走完 126 帧，
    但开手驱动不变时，实测手指仍有 0.135 rad 收敛误差，被终点姿态
    gate 否决。释放后的末段没有下一交互需要这一姿态；现在只保留
    姿态误差作为诊断，要求物体持续释放、环境保持及完整实际任务成功。
    中间 Anchor 的交接检查保持原条件。r162 从初态重新执行后通过，
    没有把 r161 的旧失败报告事后改成成功。

上述代码没有 sequence 名称分支或手写样本控制参数；调度仍是允许的
静态 oracle 输入。是否有跨样本收益，以实际结果为准。

## 248 实验状态

| 实验 | Hopper 作业 | 结果 |
|---|---|---|
| r157 | 653514.hopper-m-02 | 局部解数 2 / 2 / 3 / 3，未取得新完整多解 |
| r158 | 653517.hopper-m-02 | 尺寸搜索修复后为 2 / 3 / 3 / 3；第一段后续 A1 失败 |
| r159 | 653523.hopper-m-02 | 替代首段和 A1 连续通过；0.3 s carry 桥因实际出口速度不可达而失败 |
| r160 | 653539.hopper-m-02 | 四段局部 bank 均为三解；替代轨迹前六段通过，末段旧自由手终点不可达 |
| r161 | 653552.hopper-m-02 | 126 帧实际完成；末段被无后继交互的手指终态要求拒绝 |
| r162 | 653554.hopper-m-02 | 两条替代完整 rollout 均通过，与原基线组成三条同源成功轨迹 |

## 最终实际结果

同一 source `20231024_248`，126 帧 / 30 fps，三条均为 **4.2 s**，
没有额外等待、源时钟拉伸或段间状态重置。每条均包含三个实际成功的
Anchor 和四个 cuRobo RELAXED 区间。替代路径在所有 RELAXED 区间
通过实际 wrist 排除、连续后继执行与最终任务条件。

| 完整轨迹 | 时长 | 段间重置 | 最终物体位置误差 | 旋转误差 | 实际任务 |
|---|---:|---:|---:|---:|---|
| 解 1（原 V0 基线） | 4.2 s | 0 | 10.62 mm | 13.10° | 通过、释放且稳定 |
| 解 2 | 4.2 s | 0 | 24.20 mm | 6.02° | 通过、释放且稳定 |
| 解 3 | 4.2 s | 0 | 7.13 mm | 9.47° | 通过、释放且稳定 |

物体最终合同保持原值：位置 50 mm、旋转 0.35 rad、速度 0.03 m/s，
且已经释放。没有为多解放宽这些阈值。

### 多解体现在哪里

下面是完整实际执行中，每条 wrist 曲线到另一条曲线的最大最近点距离。
这不是 command 路径、局部 trial 或不同 source 之间的比较。

| RELAXED 帧区间 | 源时间预算 | 解 2 → 解 1 | 解 3 → 解 1 | 解 3 → 解 2 |
|---|---:|---:|---:|---:|
| [0, 26) 接近抓握 | 0.867 s | 179.66 mm | 22.57 mm | 72.60 mm |
| [65, 74) 持物去程 | 0.300 s | 29.07 mm | 6.69 mm | 27.76 mm |
| [89, 96) 持物返回 | 0.233 s | 16.96 mm | 20.79 mm | 28.15 mm |
| [120, 126) 释放后离开 | 0.200 s | 16.23 mm | 14.24 mm | 4.22 mm |

第一段解 2 有明显大弧线。其余区间时间较短，部分解的空间差异较小，
尤其最后一段解 2 与解 3 只有毫米级；不声称每一对都是大幅绕行。
图中每个子图对三条曲线使用同一空间投影、毫米单位和等比例坐标，
曲线没有平移或放大差异。解 1 为蓝色、解 2 为粉色、解 3 为黄色。

### 视频和可检查证据

- [三条完整实际视频，同步上下排列](evidence/oracle_diversity_20261008/solutions_stacked.mp4)
- [四个区间的实测路径图](evidence/oracle_diversity_20261008/actual_wrist_paths.png)
  / [PDF](evidence/oracle_diversity_20261008/actual_wrist_paths.pdf)
- [解 1](evidence/oracle_diversity_20261008/solution_0/floating_xhand_actual.mp4)、
  [解 2](evidence/oracle_diversity_20261008/solution_1/floating_xhand_actual.mp4)、
  [解 3](evidence/oracle_diversity_20261008/solution_2/floating_xhand_actual.mp4)
- [机器可读结果](evidence/oracle_diversity_20261008/diversity_result.json)、
  [本地导出索引](evidence/oracle_diversity_20261008/collection.json)、
  [结果摘要](evidence/oracle_diversity_20261008/result_summary.json)

每条 `solution_*` 保存实际 `actual_trace.json`、`pipeline_result.json`、
四段命令与实测路径 `.npz`、实测 mesh transforms 和视频。视频是把
记录的 PhysX 几何状态渲染出来，不是 reference animation；视频帧数
126、fps 30、最后 trace 边界 4.2 s 已检查，堆叠视频全部帧可解码。

## 复用范围与证据边界

- 修复依据是否有后继 Anchor、接触模式、实际边界速度、source-time
  预算和已验证移动驱动历史；代码没有 `248` 名称分支。
- r162 复用已通过 V0、局部三解 bank 和 r160 物理修正后的 Anchor
  artifact，然后从真实任务初态 fresh 执行两条完整替代轨迹；共约
  165 s 是这轮复验 wall time，不是从原始输入冷启动的总训练时间。
- 原基线和替代解满足同一 core 交互、同一语义/时间条件。为使持物
  bridge 在原时长内可达，A1 出口速度经过物理 shooting 修正，因此
  不声称三条的 Anchor 命令逐位完全相同。没有跨段状态赋值。
- 实际验证三条完整组合，不是四段各三解的全部 `3^4 = 81` 种组合。
- `243` 仍只有一条完整 V0；本轮没有验证它的多解。通用代码已经接入
  默认端到端入口，但本轮证据来自已有 V0 的续跑，不是重新冷启动全流程。
- 当前为 floating XHand；没有验证 mounted 机械臂、自碰撞强化检查
  或流体内容物转移。实际加速度保留诊断，nominal 命令预算仍用于规划。
- 解 3 的 Isaac 进程在 `simulation_app.close()` 退出时发生 -11，
  发生在完整执行和结果写盘之后。Hopper 外层作业为 0；检查了完整
  trace、127 个时钟边界、全段成功和实际几何导出，未把缺失/部分输出
  当作成功。该退出问题保留在运行记录中，不影响已保存的物理证据。

`hpc/run_oracle_diversity.pbs` 使用 H200 interactive、20 min；可通过
`MULTIPATH_V0_RUN`、`MULTIPATH_SCHEDULE`、`MULTIPATH_RUN_NAME` 接入任意
已通过的同架构 V0。`MULTIPATH_RESUME_BANK`、`MULTIPATH_VALIDATED_ANCHORS`
和 `MULTIPATH_EXECUTED_PREFIX_RUN` 支持复用已有物理结果；prefix 只复用
命令，仍从任务初态执行。默认端到端和收集入口已经连上这一阶段。
