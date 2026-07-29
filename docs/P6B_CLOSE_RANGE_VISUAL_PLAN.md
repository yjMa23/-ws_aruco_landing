# P6B 近距多尺度 ArUco 视觉子计划

## 1. 问题背景

P6B 首次静止甲板最终下降试验：

```text
TEST_HEIGHT_HOLD
→ FINAL_DESCENT
→ ArUco 长时丢失
→ RECOVER_TO_GNSS
```

离线结果：

```text
最终下降参考最低高度: 0.4266 m
实际最低相对高度: 0.3924 m
P6A 接触候选/确认: 0 / 0
FINAL_DESCENT 内 PX4 ground_contact: 0
```

`x500_mono_cam_down` 的相机位于机体参考点下方 `0.14 m`。当机体相对甲板高度约 `0.39 m` 时，相机距甲板仅约 `0.25 m`；当前 `0.50 m` Marker 已接近或超出相机视场，导致真实接触前视觉丢失。

安全规则禁止在视觉失效后继续最终下降，因此必须扩展近距视觉，不能采用盲降、延长视觉超时或绕过恢复逻辑。

---

## 2. 目标

建立远距与近距 Marker 自动切换：

```text
远距主 Marker：ID 0，边长 0.50 m，甲板中心
近距 Marker：ID 1，边长约 0.16 m，位于主 Marker 周边
```

要求：

- 远距阶段优先使用 ID 0；
- ID 0 超出视场后自动使用可见的近距 Marker；
- 控制器始终得到甲板中心位姿，而不是近距 Marker 的偏置位置；
- Marker 切换时甲板中心位置连续；
- 不使用 Ground Truth 选择 Marker；
- 不改变 P4.7 水平控制律、P5C 垂直估计或 P6A 触地判据；
- 视觉完全失效时仍执行原恢复流程。

---

## 3. Task 1：多 Marker 检测器

扩展 `aruco_detector`：

配置：

```text
marker_ids: [0, 1, 2, 3, 4]
marker_lengths_m: [0.50, 0.16, 0.16, 0.16, 0.16]
marker_priorities: [0, 1, 1, 1, 1]
```

规则：

1. 检测所有配置 ID；
2. 优先级数字越小越优先；
3. 同优先级选择图像角点面积最大的 Marker；
4. 使用对应物理边长进行 PnP；
5. 发布选中 Marker 的：
   - `/aruco/pose`
   - `/aruco/id`
   - `/aruco/visible`
6. 配置数组长度不一致、ID 重复、边长非法时拒绝启动；
7. 保留旧 `target_id + marker_length` 参数兼容入口，未配置数组时行为不变。

新增纯函数和测试，至少覆盖：

- 优先级选择；
- 同优先级面积选择；
- 未配置 ID 忽略；
- 不同 ID 使用正确物理边长；
- 非法配置拒绝。

---

## 4. Task 2：甲板 Marker 布局

保留现有中心主 Marker：

```text
ID 0，0.50 m，offset = [0, 0, 0]
```

新增四个近距 Marker，避免覆盖主 Marker：

```text
ID 1: offset [+0.18, 0.00, 0]
ID 2: offset [-0.18, 0.00, 0]
ID 3: offset [0.00, +0.18, 0]
ID 4: offset [0.00, -0.18, 0]
```

边长初始 `0.16 m`。

布局原则：

- 小 Marker 不覆盖主 Marker 的黑白码区；
- 四个 Marker 对称布置，近距相机视场内至少保留一个完整 Marker；
- Marker 平面 z 只比甲板表面高约 `1～2 mm`；
- 使用 DICT_4X4_50 对应 ID 的准确纹理；
- 纹理可使用 ASCII PGM，文件内容进入 Git；
- 仿真视觉布局属于环境，不向控制器暴露 Ground Truth。

---

## 5. Task 3：Marker 到甲板中心几何补偿

新增配置：

```text
visual_markers.ids: [0, 1, 2, 3, 4]
visual_markers.deck_center_offsets_marker_m:
  ID0: [0.00, 0.00, 0.00]
  ID1: [-0.18, 0.00, 0.00]
  ID2: [+0.18, 0.00, 0.00]
  ID3: [0.00, -0.18, 0.00]
  ID4: [0.00, +0.18, 0.00]
```

控制器处理：

```text
T_ned_deck = T_ned_marker * T_marker_deck
```

其中偏移是已知人工布置几何，不是仿真 Ground Truth。

要求：

- `/aruco/id` 与 `/aruco/pose` 必须使用相同图像时间戳对应；
- 未知 ID 拒绝该帧；
- Marker 切换后输出仍表示统一甲板中心；
- 甲板姿态继续使用 Marker 法向量；
- 目标状态估计器只接收补偿后的甲板中心位置。

新增纯 C++ 几何映射模块或测试，覆盖所有 ID、旋转后偏移和未知 ID 拒绝。

---

## 6. Task 4：可观测性与切换调试

新增调试话题：

```text
/landing/active_marker_id
/landing/active_marker_pose_ned
```

保留：

```text
/landing/marker_pose_ned
```

其语义更新为**补偿后的甲板中心位姿**。

rosbag 记录：

- `/aruco/id`
- `/landing/active_marker_id`
- `/landing/active_marker_pose_ned`
- `/landing/marker_pose_ned`

离线检查：

- ID 0 到近距 ID 的切换高度；
- 切换前后甲板中心位置跳变量；
- 每个 ID 的可见持续时间；
- 近距最低持续可见高度；
- Marker 切换对水平与垂直估计误差的影响。

---

## 7. Task 5：分级 SITL 验收

### 7.1 5 m 安全高度

- ID 0 应为主检测；
- P4.7 静止水平跟踪不回归；
- 甲板中心位置 RMSE 不恶化。

### 7.2 0.50 m 测试高度

- 允许切换到近距 Marker；
- 不进入最终下降；
- 视觉不得长时丢失；
- P5C 相对高度误差保持门槛内。

### 7.3 P6B 最终下降

- 显式启用最终下降；
- ID 0 丢失前或同时近距 Marker接管；
- 不允许 `FINAL_DESCENT → RECOVER_TO_GNSS` 由视场问题触发；
- 继续执行 P6A 真实接触候选与确认；
- 触地确认后保持，不 Land、不 Disarm。

---

## 8. 验收门槛

- 远距主 Marker 正常检测；
- 近距 Marker 至少一个在物理接触前持续可见；
- Marker 切换后的甲板中心位置跳变 `≤ 0.05 m`；
- 5 m 水平位置 RMSE 不明显恶化；
- 0.50 m 相对高度 RMSE `≤ 0.10 m`；
- 最终下降期间不因 Marker 超出视场触发长时视觉恢复；
- Ground Truth 不进入检测器或控制器；
- 视觉全部失效时仍暂停/恢复；
- 不发送 `NAV_LAND` 或 Disarm；
- 全工作区测试通过。

---

## 9. 执行顺序

```text
保存本计划
→ 生成 ID 1～4 纹理
→ 扩展多 Marker 检测器和测试
→ 扩展甲板视觉布局
→ 实现 Marker 到甲板中心补偿和测试
→ 调试话题与 rosbag
→ 5 m 回归
→ 0.50 m 近距视觉验收
→ P6B 最终下降复验
```

## 10. 当前状态

- 本计划已保存。
- 尚未修改多 Marker 检测器或甲板布局。
- 首次 P6B bag 保留用于对照：
  `bags/p6b_static_final_descent_zff1p0_20260725_151809`。
