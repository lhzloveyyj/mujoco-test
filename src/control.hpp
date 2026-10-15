// ===========================================================================
//  控制接口:你的算法在这里被调用
//
//  这个头文件只声明接口,实现放在 control.cpp —— 那是你唯一需要改的文件。
// ===========================================================================

#pragma once

#include <mujoco/mujoco.h>

inline constexpr double kPi = 3.14159265358979323846;

// 每个仿真步(默认 2 ms)调用一次:
//     m, d    MuJoCo 的模型和数据(所有状态都在 d 里)
//     t       当前仿真时间(秒)
//     dt      仿真步长(秒)
//
// 模型里有什么(对应 models/two_link_arm.xml)—— 5 自由度手臂 + Robotiq 2F-85 夹爪:
//     joint1_yaw          绕 z 轴回转(底座)      d->qpos[0] / d->ctrl[0]
//     joint2_shoulder     绕 y 轴俯仰(肩)        d->qpos[1] / d->ctrl[1]
//     joint3_elbow        绕 y 轴俯仰(肘)        d->qpos[2] / d->ctrl[2]
//     joint4_forearm_pitch 绕 y 轴俯仰(小臂中段)= 第二个肘,让小臂上下折
//                                                          d->qpos[3] / d->ctrl[3]
//     joint5_wrist_roll   绕小臂轴线回转(法兰处,腕)        d->qpos[4] / d->ctrl[4]
//                         —— 让夹爪绕杆转圈、改变张合方向,TCP 位置不动
//     夹爪(Robotiq 2F-85) 8 个关节的四杆机构,由 1 个驱动器 + equality 约束联动
//                         d->ctrl[5]     = 开合指令 0~255:**0 = 全开,255 = 闭紧**
//                         d->qpos[5..12] = 夹爪内部 8 个关节角(耦合的,一般不用直接读)
//     site "ee"           末端标记 = 两指之间的抓取中心(TCP)
//     body "target"       mocap 目标点,改 d->mocap_pos 就能挪它
//
// 俯仰关节的轴不平行于重力,所以重力会产生关节力矩 —— 位置伺服要一直出力撑着,
// 手臂水平伸出时会有几毫米到几厘米的"静差"(真实机械臂也这样)。
void my_algorithm(const mjModel* m, mjData* d, double t, double dt);
