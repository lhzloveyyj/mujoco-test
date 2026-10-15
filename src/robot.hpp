// ===========================================================================
//  模型封装:加载 MJCF、步进、读状态
//
//  Sim 只负责"把模型跑起来",不含任何控制逻辑(算法在 control.cpp)
//  和任何界面代码(窗口在 viewer.cpp)。换模型、换部件时改这里。
// ===========================================================================

#pragma once

#include <mujoco/mujoco.h>

#include <cmath>
#include <cstdio>
#include <string>

#include "control.hpp"   // Sim::step() 要调用 my_algorithm()

struct Sim {
  mjModel* m = nullptr;
  mjData* d = nullptr;
  int ee = -1;             // 末端 site 的 id
  int mocap = -1;          // 目标点(mocap body)的 mocap id
  double L1 = 0.0;         // 大臂长度
  double L2 = 0.0;         // 小臂长度
  double tool = 0.0;       // 夹爪从安装法兰到抓取中心(TCP)的长度
  int pad_l = -1;          // 两片指垫的几何体 id,用来算实际开合程度
  int pad_r = -1;
  double dt = 0.002;
  double t = 0.0;

  bool load(const std::string& xml) {
    char err[1024] = {0};
    m = mj_loadXML(xml.c_str(), nullptr, err, sizeof(err));
    if (!m) {
      std::fprintf(stderr, "加载模型失败:%s\n%s\n", xml.c_str(), err);
      return false;
    }
    d = mj_makeData(m);
    mj_resetData(m, d);
    ee = mj_name2id(m, mjOBJ_SITE, "ee");
    const int tid = mj_name2id(m, mjOBJ_BODY, "target");
    mocap = (tid >= 0) ? m->body_mocapid[tid] : -1;
    L1 = m->body_pos[3 * mj_name2id(m, mjOBJ_BODY, "link2") + 0];
    // 胶囊几何体用 fromto 定义:size[1] 是两球心距离的一半,也就是连杆长度的一半。
    // 小臂被"小臂回转关节"分成两段,长度加起来才是完整小臂。
    L2 = 0.0;
    for (const char* gname : {"link2_geom", "link2b_geom"}) {
      const int g = mj_name2id(m, mjOBJ_GEOM, gname);
      if (g >= 0) L2 += 2.0 * m->geom_size[3 * g + 1];
    }
    // 夹爪刚性固定在法兰上,所以"法兰→TCP"的距离是常数,随便摆个姿态量一下即可
    const int mount = mj_name2id(m, mjOBJ_BODY, "gripper_mount");
    pad_l = mj_name2id(m, mjOBJ_GEOM, "left_pad1");
    pad_r = mj_name2id(m, mjOBJ_GEOM, "right_pad1");
    mj_forward(m, d);
    tool = std::hypot(std::hypot(d->site_xpos[3 * ee] - d->xpos[3 * mount],
                                 d->site_xpos[3 * ee + 1] - d->xpos[3 * mount + 1]),
                      d->site_xpos[3 * ee + 2] - d->xpos[3 * mount + 2]);
    dt = m->opt.timestep;
    return true;
  }

  // 走一个仿真步:先调用你的算法,再让物理引擎前进
  void step() {
    my_algorithm(m, d, t, dt);
    mj_step(m, d);
    mj_kinematics(m, d);
    t += dt;
  }

  double qpos_deg(int i) const { return d->qpos[i] * 180.0 / kPi; }

  // 两片指垫之间的实际距离(毫米)= 夹爪开合程度
  double grip_mm() const {
    if (pad_l < 0 || pad_r < 0) return 0.0;
    const double* a = d->geom_xpos + 3 * pad_l;
    const double* b = d->geom_xpos + 3 * pad_r;
    return std::hypot(std::hypot(a[0] - b[0], a[1] - b[1]), a[2] - b[2]) * 1000.0;
  }

  ~Sim() {
    if (d) mj_deleteData(d);
    if (m) mj_deleteModel(m);
  }
};
