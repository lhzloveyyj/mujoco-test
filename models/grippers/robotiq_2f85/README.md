# Robotiq 2F-85 两指自适应夹爪

**来源**:[MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie/tree/main/robotiq_2f85)
的 `robotiq_2f85` 模型(Google DeepMind 维护的开源模型库)。

**许可**:BSD-3-Clause,版权归 ROS-Industrial,见同目录 [`LICENSE`](./LICENSE)。
网格文件本身未被修改。

## 文件夹里有什么

| 文件 | 说明 |
| --- | --- |
| `robotiq_2f85.xml` | **官方原版模型**(未修改的参考件)。它自带 `meshdir="assets"`,所以在这个目录下可以直接单独加载:`python -m mujoco.viewer` 或任何 MJCF 工具都能打开,方便对照/复用 |
| `assets/*.stl` | 8 个零件网格(单位**毫米**,使用时按 0.001 缩放) |
| `LICENSE` | 上游许可证 |

## 它是怎么接进本项目的手臂的

**接法不在这里,而在** [`../../two_link_arm.xml`](../../two_link_arm.xml) —— 那个文件把
夹爪的 body 树**内联**进了小臂末端(而不是用 `<include>`,因为内联后改安装位置、
改类名更直观,也没法和主模型的 `<default>` 打配合)。

集成时做的三件事:

1. **改名**:上游的 `base` body 会和手臂底座 `base` 撞名,改成 `gripper_base`;
   `base_mount` → `gripper_mount`,上游的 `pinch` site → `ee`(抓取中心 TCP)。
2. **安装朝向**:上游工具轴是 `+z`,法兰在小臂末端 `x=0.40`,所以绕 y 轴转 90°
   (`quat="0.70710678 0 0 0.70710678 0"`)让它顺着小臂指向 `+x`;
   整个夹爪挂在腕回转关节 `wrist` 下面,可以绕小臂轴线转。
3. **接触排除**:`link2 ↔ wrist ↔ gripper_mount` 之间是刚性连接,加了 exclude;
   另外 6 条 exclude 是上游自带的。

## 控制接口

上游只有 1 个驱动器 `fingers_actuator`,挂在名为 `split` 的肌腱上,同时驱动左右两根
驱动杆;两套四杆机构靠 3 条 `equality` 约束闭合。所以 8 个关节实际只有 1 个自由度。

| 量 | 说明 |
| --- | --- |
| `ctrl` | **0 = 全开,255 = 闭紧**(和真机 Modbus 一致,注意别反着用) |
| 行程 | 指垫间距 98 mm → 15 mm,约 85 mm,与 2F-85 规格一致 |
| 闭紧状态 | 两片指垫会互相接触(`ncon=2`),这是上游模型自带的行为,相当于机械限位,不是 bug |
