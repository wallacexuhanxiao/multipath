# 成功案例演示：同场景多解叠加与实测轨迹

日期：2026-10-11。只使用完整连续 PhysX 与 task 检查通过、段间零 reset 的 Put-In 结果。三个 source 各三条解；没有使用双手 Brush 的失败前缀。

## 汇报主视频

[同场景叠加合辑，29.4 s](/Users/apple/Desktop/semantic verifier/multipath/outputs/presentation_demos_20261011/showcase_overlay.mp4)

每个 source 的三条实测运动在同一相机、同一源时间下半透明叠加。青、粉、黄分别对应三条解，场景中显示逐步增长的实测 wrist 轨迹；右侧按各个 RELAXED 区间显示共同空间投影，单位 mm。曲线没有人工放大、重新拟合或换成规划目标。

运动规划区间播放 **0.25×**，Interaction Anchor 播放 **1×**。底部时间条标识 MP / A，顶部始终显示原 source time。慢放通过重复演示帧实现，不改变原执行的时间预算、控制指令或物理结果。

旧录制的 endpoint/startpoint 采样差异已统一：从对应实测网格重新渲染，各解从 source boundary 0 开始。移除的只是虚拟规划障碍与旧路径的显示，物理物体和手部几何保留。重新渲染所用 trace 与成功证据逐文件 SHA-256 相同；MuJoCo 不推进物理。

## 辅助材料

- [同步三栏合辑](/Users/apple/Desktop/semantic verifier/multipath/outputs/presentation_demos_20261011/showcase_stacked.mp4)
- [成功案例同图汇总](/Users/apple/Desktop/semantic verifier/multipath/outputs/presentation_demos_20261011/successful_cases.png)
- [253 单案例叠加，11.5 s](/Users/apple/Desktop/semantic verifier/multipath/outputs/presentation_demos_20261011/20231024_253/overlay_slow_paths.mp4)
- [248 单案例叠加，9.0 s](/Users/apple/Desktop/semantic verifier/multipath/outputs/presentation_demos_20261011/20231024_248/overlay_slow_paths.mp4)
- [243 单案例叠加，8.9 s](/Users/apple/Desktop/semantic verifier/multipath/outputs/presentation_demos_20261011/20231024_243/overlay_slow_paths.mp4)

原实际时长分别为 **4.8、4.2、4.3 s**。合辑为 1920×1080、30 fps、H.264。所有最终视频已完整解码，接近、持物桥段与结束画面已检查。叠加显示的是三个分别成功的物理 rollout，不是三只手在一个物理世界同时执行。

## 复用程序

程序：[render_diversity_demo.py](/Users/apple/Desktop/semantic verifier/multipath/scripts/render_diversity_demo.py)。输入：[inputs.json](/Users/apple/Desktop/semantic verifier/multipath/docs/evidence/presentation_demos_20261011/inputs.json)，机器摘要：[summary.json](/Users/apple/Desktop/semantic verifier/multipath/docs/evidence/presentation_demos_20261011/summary.json)。

程序读取各成功 run 的 schedule、actual trace 和录制视频。`clean_render_runs` 可指定同一实际 trace 对应的原始网格记录，由两路并行重新渲染干净画面；`recorded_video_boundary_offset` 表示视频帧对应的实测边界。区间划分、轨迹投影和慢放长度从输入计算，没有按 source 名称设计路径或控制参数。

```bash
MPLCONFIGDIR=/private/tmp/multipath_demo_matplotlib \
  /tmp/ego-render-env/bin/python scripts/render_diversity_demo.py \
  --cases docs/evidence/presentation_demos_20261011/inputs.json \
  --output-dir outputs/presentation_demos_20261011 \
  --slow-factor 4
```

需已有实际录制资源与支持 H.264 的 OpenCV；`clean_render_runs` 还需要本地 MuJoCo 渲染环境。此程序只制作演示，不训练或修改原 pipeline。
