// ===========================================================================
//  界面实现:GLFW 窗口 + MuJoCo 自带渲染器(mjv/mjr),和官方 simulate 一个路子
//
//  这个文件只管"显示"和"交互",不含控制算法(control.cpp)也不含模型封装(robot.hpp)。
// ===========================================================================

#include "viewer.hpp"

#include <GLFW/glfw3.h>

#include <cstdio>
#include <string>
#include <vector>

namespace {

// ------------------------- 鼠标:改相机 -------------------------
const mjModel* g_m = nullptr;
mjvCamera* g_cam = nullptr;
double g_last_x = 0, g_last_y = 0, g_height = 600;
int g_drag = -1;   // 0 = 左键旋转,1 = 右键平移

void on_button(GLFWwindow*, int button, int action, int) {
  if (action == GLFW_PRESS) {
    g_drag = (button == GLFW_MOUSE_BUTTON_LEFT) ? 0
             : (button == GLFW_MOUSE_BUTTON_RIGHT) ? 1 : -1;
  } else {
    g_drag = -1;
  }
}
void on_cursor(GLFWwindow*, double x, double y) {
  const double dx = (x - g_last_x) / g_height, dy = (y - g_last_y) / g_height;
  g_last_x = x;
  g_last_y = y;
  if (g_drag == 0)
    mjv_moveCamera(g_m, mjMOUSE_ROTATE_V, dx, dy, g_cam);
  else if (g_drag == 1)
    mjv_moveCamera(g_m, mjMOUSE_MOVE_V, dx, dy, g_cam);
}
void on_scroll(GLFWwindow*, double, double dy) {
  mjv_moveCamera(g_m, mjMOUSE_ZOOM, 0, -0.05 * dy, g_cam);
}

// ------------------------- 左下角:电机信息面板 -------------------------
// 注意两点(都实测过):
//   1) 用 mjr_overlay,mjr_text 在这套构建里根本画不出来;
//   2) 窗口里的文字只支持 ASCII —— MuJoCo 自带字体没有中文字形,
//      写中文会变成方块,所以界面文字全用英文,中文说明放在终端和 README 里。
void draw_motor_panel(const Sim& sim, const mjrRect& viewport, const mjrContext& con) {
  std::string line = "Motors:";
  for (int a = 0; a < sim.m->nu; ++a) {
    // 执行器的传动对象可能是关节,也可能是肌腱(夹爪就是挂在 split 肌腱上的)
    const int type = sim.m->actuator_trntype[a];
    const int id = sim.m->actuator_trnid[2 * a];
    const char* what = (type == mjTRN_JOINT)
                           ? mj_id2name(sim.m, mjOBJ_JOINT, id)
                           : (type == mjTRN_TENDON ? mj_id2name(sim.m, mjOBJ_TENDON, id)
                                                   : "(other)");
    char buf[96];
    std::snprintf(buf, sizeof(buf), "  %s->%s (%+.1f)", mj_id2name(sim.m, mjOBJ_ACTUATOR, a),
                  what ? what : "?", sim.d->actuator_force[a]);
    line += buf;
  }
  line += "   N*m / N";
  // 只画一行:mjr_overlay 的两段文字是横向并排的,画两行很容易在底部撞在一起
  mjr_overlay(mjFONT_NORMAL, mjGRID_BOTTOMLEFT, viewport, line.c_str(), "", &con);
}

// ------------------------- 左上角:状态行 -------------------------
void draw_status(const Sim& sim, const mjrRect& viewport, const mjrContext& con,
                 bool paused) {
  const double* tp = sim.d->site_xpos + 3 * sim.ee;
  char line1[128], line2[192];
  std::snprintf(line1, sizeof(line1), "t = %.2f s%s", sim.t, paused ? "   [PAUSED]" : "");
  std::snprintf(line2, sizeof(line2),
                "yaw=%+.1f sh=%+.1f el=%+.1f wr=%+.1f deg | TCP=(%.3f,%.3f,%.3f) grip=%.0fmm",
                sim.qpos_deg(0), sim.qpos_deg(1), sim.qpos_deg(2), sim.qpos_deg(3), tp[0],
                tp[1], tp[2], sim.grip_mm());
  mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport, line1, line2, &con);
}

// 首帧渲染自检:确认 GL 真把画面画出来了(全黑说明 GL 上下文有问题)
void check_first_frame(const mjrRect& viewport, const mjrContext& con) {
  std::vector<unsigned char> rgb(3 * static_cast<size_t>(viewport.width) * viewport.height);
  mjr_readPixels(rgb.data(), nullptr, viewport, &con);
  long sum = 0;
  for (unsigned char v : rgb) sum += v;
  std::printf("首帧渲染:帧缓冲 %dx%d,平均亮度 %.1f(接近 0 说明没画出来)\n", viewport.width,
              viewport.height, sum / static_cast<double>(rgb.size()));
}

void print_usage_hint() {
  std::printf("窗口已打开:\n");
  std::printf("  鼠标左键拖动 = 转视角,右键拖动 = 平移,滚轮 = 缩放\n");
  std::printf("  空格 = 暂停/继续,M = 显示/隐藏底部电机信息,Esc = 退出\n");
  std::printf("电机在哪:\n");
  std::printf("  手臂四个关节处,深灰色圆柱壳体 + 橙色转子法兰就是电机;\n");
  std::printf("  壳体固定在不转的一侧(底座 / 近端连杆),转子跟着远端连杆转。\n");
  std::printf("  末端是 Robotiq 2F-85 夹爪(四杆机构,1 个驱动器带 8 个关节)。\n");
  std::printf("  左下角列出每个执行器驱动的关节/肌腱和它当前的出力(臂 N·m,夹爪 N·m)\n");
  std::printf("  (窗口里的文字只能用 ASCII —— MuJoCo 内置字体没有中文字形)\n");
}

}  // namespace

int run_viewer(Sim& sim, double duration) {
  if (!glfwInit()) {
    std::fprintf(stderr, "GLFW 初始化失败(没有可用显示?)\n");
    return 1;
  }
  // 注意:不要要求 core profile —— MuJoCo 渲染器在 core profile 下会报
  // "OpenGL ARB_framebuffer_object required",默认上下文(兼容模式)才正常。
  GLFWwindow* window = glfwCreateWindow(1100, 700, "MuJoCo: 4-DOF arm + 2F-85", nullptr,
                                        nullptr);
  if (!window) {
    glfwTerminate();
    std::fprintf(stderr, "创建窗口失败\n");
    return 1;
  }
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);   // 垂直同步,别把 CPU 跑满

  int width = 0, height = 0;
  glfwGetFramebufferSize(window, &width, &height);

  mjvScene scn;
  mjv_defaultScene(&scn);
  mjv_makeScene(sim.m, &scn, 4000);
  mjvCamera cam;
  mjv_defaultCamera(&cam);
  cam.azimuth = 135.0;
  cam.elevation = -18.0;
  cam.distance = 1.9;
  cam.lookat[0] = 0.3;
  cam.lookat[2] = 0.30;
  mjvOption opt;
  mjv_defaultOption(&opt);
  mjrContext con;
  mjr_defaultContext(&con);
  mjr_makeContext(sim.m, &con, mjFONTSCALE_150);

  g_m = sim.m;
  g_cam = &cam;
  g_height = static_cast<double>(height);
  g_last_x = g_last_y = 0.0;
  glfwSetMouseButtonCallback(window, on_button);
  glfwSetCursorPosCallback(window, on_cursor);
  glfwSetScrollCallback(window, on_scroll);

  print_usage_hint();

  bool paused = false;
  bool space_down = false;
  bool show_motors = true;    // M 键开关
  bool m_down = false;
  bool first_frame = true;
  double wall_last = glfwGetTime();
  double sim_target = 0.0;   // 让仿真跟上真实时间

  while (!glfwWindowShouldClose(window)) {
    const double now = glfwGetTime();
    const double elapsed = now - wall_last;
    wall_last = now;

    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) break;
    const bool space = glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS;
    if (space && !space_down) paused = !paused;
    space_down = space;
    const bool mkey = glfwGetKey(window, GLFW_KEY_M) == GLFW_PRESS;
    if (mkey && !m_down) show_motors = !show_motors;
    m_down = mkey;

    if (!paused) {
      sim_target += elapsed;
      int guard = 0;
      while (sim.t < sim_target && guard++ < 300) sim.step();
      if (duration > 0 && sim.t >= duration) break;
    }

    glfwGetFramebufferSize(window, &width, &height);
    g_height = height > 0 ? static_cast<double>(height) : 600.0;
    const mjrRect viewport{0, 0, width, height};

    mjv_updateScene(sim.m, sim.d, &opt, nullptr, &cam, mjCAT_ALL, &scn);
    mjr_render(viewport, &scn, &con);
    if (show_motors) draw_motor_panel(sim, viewport, con);
    draw_status(sim, viewport, con, paused);

    if (first_frame) {
      first_frame = false;
      check_first_frame(viewport, con);
    }

    glfwSwapBuffers(window);
    glfwPollEvents();
  }

  mjr_freeContext(&con);
  mjv_freeScene(&scn);
  glfwTerminate();
  return 0;
}
