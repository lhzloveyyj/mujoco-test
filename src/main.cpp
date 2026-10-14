// ===========================================================================
//  入口(main):解析命令行 → 加载模型 → 打开窗口跑起来
//
//  这个文件故意保持很短:它不碰物理、不碰算法、不碰渲染,只负责"把东西接起来"。
//  想加新的命令行参数(比如 --speed、--headless)就改这里。
//
//  用法:
//      make                                    # 编译出 sim
//      ./sim                                   # 打开原生 3D 窗口(默认)
//      ./sim models/two_link_arm.xml 10        # [模型路径] [跑多少秒后自动退出]
//
//  代码分工(依赖方向:main → viewer → robot → control,不会互相绕):
//      src/control.cpp  你的算法 my_algorithm()   ← 平时只改这个
//      src/robot.hpp    模型加载与步进(Sim)
//      src/viewer.cpp   窗口、渲染、鼠标交互
//      src/main.cpp     入口(这个文件)
// ===========================================================================

#include <cstdio>
#include <cstdlib>
#include <string>

#include "robot.hpp"    // Sim:加载 MJCF、步进
#include "viewer.hpp"   // run_viewer:开窗口跑仿真

// 编译时由 Makefile 用 -DFKIK_MODEL_XML=... 传进来(绝对路径),
// 所以从任何目录运行 ./sim 都能找到默认模型;这里只是没传时的兜底。
#ifndef FKIK_MODEL_XML
#define FKIK_MODEL_XML "models/two_link_arm.xml"
#endif

int main(int argc, char** argv) {
  // ---------------------------------------------------------------------
  //  1) 解析命令行:两个可选的位置参数,顺序无所谓,靠"像不像数字"区分
  // ---------------------------------------------------------------------
  std::string xml = FKIK_MODEL_XML;   // 模型路径
  double duration = 0.0;              // 跑多少秒后自动退出;0 = 一直跑,直到你关窗口

  for (int i = 1; i < argc; ++i) {    // argv[0] 是程序名,从 1 开始
    const std::string a = argv[i];

    // 说明:两个参数"谁在前谁在后"都可以,所以不能按下标判断,只能看内容:
    //   * 全是数字和点 → 当成时长(比如 10、2.5)
    //   * 其它         → 当成模型路径(比如 models/two_link_arm.xml)
    // find_first_not_of 返回"第一个不是数字/点"的位置;返回 npos 就是"全是数字/点"。
    const bool looks_like_number = (a.find_first_not_of("0123456789.") == std::string::npos);

    if (!a.empty() && a[0] == '-' && a != "-") {
      // 以 - 开头的都当成"选项"。目前除了裸的 "-" 没有别的选项,
      // 所以遇到就不猜了,直接报错退出,免得把打错的参数默默忽略掉。
      std::fprintf(stderr, "未知参数:%s\n", a.c_str());
      return 1;
    }
    if (looks_like_number) duration = std::atof(a.c_str());   // 字符串转 double
    else xml = a;
  }

  // ---------------------------------------------------------------------
  //  2) 加载模型(失败会自己打印 MuJoCo 的报错)
  // ---------------------------------------------------------------------
  Sim sim;
  if (!sim.load(xml)) return 1;   // 非 0 返回码,方便脚本判断

  // 启动横幅:把"这次跑的到底是什么"说清楚,免得改完模型不知道有没有生效
  std::printf("模型:%s\n", xml.c_str());
  std::printf("  大臂 %.3f m,小臂 %.3f m;Robotiq 2F-85 夹爪从法兰再伸出 %.3f m;步长 %.4f s\n",
              sim.L1, sim.L2, sim.tool, sim.dt);
  std::printf("  模型共 %ld 个自由度:4 个臂关节(回转/肩/肘/腕)+ 夹爪内部 8 个\n",
              (long)sim.m->nq);   // nq 是 mjtSize(long),所以要转一下才能配 %ld
  std::printf("  控制量:d->ctrl[0..3] = 四个关节目标角,d->ctrl[4] = 夹爪开合(0~255)\n");
  std::printf("你的算法在 src/control.cpp 的 my_algorithm() 里\n\n");

  // ---------------------------------------------------------------------
  //  3) 开窗口跑仿真(这一步会一直阻塞,直到关窗口或跑够 duration 秒)
  //     返回值直接当作进程退出码:0 = 正常退出
  // ---------------------------------------------------------------------
  return run_viewer(sim, duration);
}
