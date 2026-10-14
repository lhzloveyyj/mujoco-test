// ===========================================================================
//  入口:解析命令行、加载模型、打开窗口
//
//  make                       # 编译出 sim
//  ./sim                      # 打开原生 3D 窗口(默认)
//  ./sim models/two_link_arm.xml 10     # [模型路径] [跑多少秒后自动退出]
//
//  代码分工:
//      control.cpp  你的算法        ← 平时只改这个
//      robot.hpp    模型加载与步进
//      viewer.cpp   窗口、渲染、鼠标交互
//      main.cpp     入口(这个文件)
// ===========================================================================

#include <cstdio>
#include <cstdlib>
#include <string>

#include "robot.hpp"
#include "viewer.hpp"

#ifndef FKIK_MODEL_XML
#define FKIK_MODEL_XML "models/two_link_arm.xml"
#endif

int main(int argc, char** argv) {
  std::string xml = FKIK_MODEL_XML;
  double duration = 0.0;   // 0 = 一直跑,直到你关窗口
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (!a.empty() && a[0] == '-' && a != "-") {
      std::fprintf(stderr, "未知参数:%s\n", a.c_str());
      return 1;
    }
    if (a.find_first_not_of("0123456789.") == std::string::npos) duration = std::atof(a.c_str());
    else xml = a;
  }

  Sim sim;
  if (!sim.load(xml)) return 1;
  std::printf("模型:%s\n", xml.c_str());
  std::printf("  大臂 %.3f m,小臂 %.3f m;Robotiq 2F-85 夹爪从法兰再伸出 %.3f m;步长 %.4f s\n",
              sim.L1, sim.L2, sim.tool, sim.dt);
  std::printf("  模型共 %ld 个自由度:4 个臂关节(回转/肩/肘/腕)+ 夹爪内部 8 个\n",
              (long)sim.m->nq);
  std::printf("  控制量:d->ctrl[0..3] = 四个关节目标角,d->ctrl[4] = 夹爪开合(0~255)\n");
  std::printf("你的算法在 src/control.cpp 的 my_algorithm() 里\n\n");

  return run_viewer(sim, duration);
}
