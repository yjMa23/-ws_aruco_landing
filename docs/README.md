# 项目文档

本文档集只保留当前事实、理论、操作、论文结果和未完成计划。已完成开发过程由 Git 历史追溯，不在活跃文档中重复维护。

## 阅读入口

- [当前实现与安全边界](reference/SYSTEM_OVERVIEW.md)
- [移动甲板降落控制理论](reference/LANDING_CONTROL_THEORY.md)
- [坐标系与时间契约](reference/COORDINATE_FRAMES.md)
- [Marine vessel 刚体运动学](reference/MARINE_VESSEL_KINEMATICS.md)
- [VRX WAM-V 集成契约](reference/VRX_WAMV_INTEGRATION.md)
- [ArUco 检测公式](reference/aruco_detector_formulas.md)
- [安装、启动、实验和排障](guides/OPERATIONS.md)
- [Marine scene 构建与验证](guides/MARINE_SCENE_BUILD.md)
- [VRX WAM-V 构建与验证](guides/VRX_WAMV_BUILD.md)
- [论文结果与置信区间](results/PAPER_RESULTS.md)
- [数据来源与 SHA256](results/DATA_PROVENANCE.md)
- [唯一下一步计划](plans/NEXT_DEVELOPMENT_PLAN.md)

包级使用说明：

- [`aruco_detector`](../src/aruco_detector/README.md)
- [`aruco_precision_landing_cpp`](../src/aruco_precision_landing_cpp/README.md)
- [`moving_deck_sim`](../src/moving_deck_sim/README.md)

## 维护约定

- 当前实现只写已经落地并验证的能力。
- 下一步计划只写尚未完成的工作，完成项必须移出计划。
- 操作命令、参数和公开接口发生变化时，同一变更内同步相关文档。
- 文件和章节按能力命名，不使用项目生命周期阶段代号。
