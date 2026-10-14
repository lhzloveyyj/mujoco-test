// ===========================================================================
//  仿真程序:MuJoCo 原生窗口 + 你自己写的算法
//
//  这份文件是自包含的,你需要改的只有一处:
//
//      my_algorithm()      ← 你的算法写在里面(见下方)
//
//  编译运行(在项目根目录):
//      make                       # 编译出 sim
//      ./sim                      # 打开原生 3D 窗口(默认)
//      ./sim models/two_link_arm.xml 10     # [模型路径] [跑多少秒后自动退出]
//
//  窗口操作:
//      鼠标左键拖动 = 转视角    鼠标右键拖动 = 平移    滚轮 = 缩放
//      空格 = 暂停/继续         M = 显示/隐藏电机信息文字   Esc 或关闭窗口 = 退出
//
//  画面里怎么看出电机装在哪:
//      模型里每个关节处都有真实的电机几何体:深灰色圆柱壳体(装在不转的一侧)
//      + 橙色转子法兰(跟着远端连杆转)。夹爪后面那个深灰方块是它的驱动电机。
//      左下角会列出每个电机驱动的关节和它当前的出力,按 M 可以隐藏这行文字。
//
//  模型里有什么(对应 models/two_link_arm.xml)—— 4 自由度手臂 + Robotiq 2F-85 夹爪:
//      joint1_yaw          绕 z 轴回转(底座)      d->qpos[0] / d->ctrl[0]
//      joint2_shoulder     绕 y 轴俯仰(肩)        d->qpos[1] / d->ctrl[1]
//      joint3_elbow        绕 y 轴俯仰(肘)        d->qpos[2] / d->ctrl[2]
//      joint4_wrist        绕小臂轴线回转(腕)      d->qpos[3] / d->ctrl[3]
//                          —— 轴和小臂同轴,所以转它只是让夹爪"绕杆转圈",
//                             张合方向跟着转,TCP 位置不动
//      夹爪(Robotiq 2F-85) 8 个关节的四杆机构,由 1 个驱动器 + equality 约束联动
//                          d->ctrl[4]     = 开合指令 0~255:**0 = 全开,255 = 闭紧**
//                          d->qpos[4..11] = 夹爪内部 8 个关节角(耦合的,一般不用直接读)
//      site "ee"           末端标记 = 两指之间的抓取中心(TCP)
//      body "target"       mocap 目标点,改 d->mocap_pos 就能挪它
//
//  俯仰关节的轴不平行于重力,所以重力会产生关节力矩 —— 位置伺服要一直出力撑着,
//  手臂水平伸出时会有几毫米到几厘米的"静差"(真实机械臂也这样)。
// ===========================================================================

#include <mujoco/mujoco.h>

#include <GLFW/glfw3.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#ifndef FKIK_MODEL_XML
#define FKIK_MODEL_XML "models/two_link_arm.xml"
#endif

static constexpr double kPi = 3.14159265358979323846;

// ===========================================================================
//  ★★★ 你的算法写在这里 ★★★
//
//  每个仿真步会被调用一次,参数:
//      m, d    MuJoCo 的模型和数据(所有状态都在 d 里)
//      t       当前时间(秒)
//      dt      仿真步长(这个模型是 0.002 s)
//
//  读状态:
//      d->qpos[0..3]              回转 / 肩 / 肘 / 腕 的关节角(弧度)
//      d->qvel[0..3]              对应的关节角速度
//      d->qpos[4..11]             夹爪的 8 个关节角(想读开合程度看 ctrl[4] 就行)
//      d->site_xpos[3*ee]         末端抓取中心(TCP)的世界坐标 (x, y, z)
//
//  写输出(选一种,或混用):
//      d->ctrl[0..3]              四个臂关节的目标角(位置伺服去追,最常用)
//      d->ctrl[4]                 夹爪:0 = 全开,255 = 闭紧(想夹住东西给中间值)
//      d->qpos[0..3]              直接指定关节角,机械臂立刻到位(纯运动学)
//      d->qfrc_applied[0..3]      直接施加关节力矩(自己写力矩控制器时用)
// ===========================================================================
static void my_algorithm(const mjModel* m, mjData* d, double t, double dt) {
  (void)m;
  (void)dt;

  // 默认示例:回转慢慢转、肩和肘来回摆动(手臂上下动)、腕部转圈 —— 换成你的算法即可
  d->ctrl[0] = 0.5 * std::sin(2.0 * kPi * 0.10 * t);          // 回转
  d->ctrl[1] = -0.6 + 0.5 * std::sin(2.0 * kPi * 0.25 * t);    // 肩俯仰(上下)
  d->ctrl[2] = 1.2 + 0.6 * std::sin(2.0 * kPi * 0.25 * t);     // 肘俯仰
  d->ctrl[3] = kPi * std::sin(2.0 * kPi * 0.15 * t);           // 腕:绕小臂轴线来回转
  // 夹爪:每 2 秒开合一次(0 = 全开,255 = 闭紧)
  d->ctrl[4] = 127.5 * (1.0 - std::cos(2.0 * kPi * 0.5 * t));

  // 例(运动学模式):直接给关节角,不做动力学 —— 练 FK/IK 时最省事
  //   d->qpos[0] = 0.6 * std::sin(2.0 * kPi * 0.2 * t);
  //   d->qpos[1] = -0.8;
  //
  // 例(力矩模式):自己算力矩
  //   const double err = 0.5 - d->qpos[0];
  //   d->qfrc_applied[0] = 30.0 * err - 2.0 * d->qvel[0];
}

// ===========================================================================
//  下面都是固定套路(窗口、渲染、画动画帧),你都不用管
// ===========================================================================

struct Sim {
  mjModel* m = nullptr;
  mjData* d = nullptr;
  int ee = -1;
  int mocap = -1;
  double L1 = 0.0;         // 连杆 1 长度
  double L2 = 0.0;         // 连杆 2 长度
  double tool = 0.0;       // 夹爪从安装法兰到抓取中心(TCP)的长度
  int pad_l = -1;          // 两片指垫的几何体,用来算实际开合程度
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
    // 胶囊几何体用 fromto 定义:size[1] 是两球心距离的一半,也就是连杆长度的一半
    const int g2 = mj_name2id(m, mjOBJ_GEOM, "link2_geom");
    L2 = 2.0 * m->geom_size[3 * g2 + 1];
    // 夹爪是刚性固定在法兰上的,所以"法兰→TCP"的距离是常数,随便摆个姿态量一下即可
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

// ---------------------------- 原生窗口 ----------------------------
// 用 GLFW 开窗口 + MuJoCo 自带渲染器(mjv/mjr)画,和 MuJoCo 官方 simulate 一个路子。
namespace win {
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
}  // namespace win

// 左下角文字面板:列出所有执行器 -> 它驱动的关节 -> 当前出力
// 注意两点(都实测过):
//   1) 用 mjr_overlay,mjr_text 在这套构建里根本画不出来;
//   2) 窗口里的文字只支持 ASCII —— MuJoCo 自带字体没有中文字形,
//      写中文会变成方块,所以这里全用英文,中文说明放在终端和 README 里。
static void draw_motor_panel(const Sim& sim, const mjrRect& viewport, const mjrContext& con) {
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

static int run_window(Sim& sim, double duration) {
  if (!glfwInit()) {
    std::fprintf(stderr, "GLFW 初始化失败(没有可用显示?)\n");
    return 1;
  }
  // 注意:不要要求 core profile —— MuJoCo 渲染器在 core profile 下会报
  // "OpenGL ARB_framebuffer_object required",默认上下文(兼容模式)才正常。
  GLFWwindow* window = glfwCreateWindow(1100, 700, "MuJoCo: 3-DOF arm", nullptr, nullptr);
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

  win::g_m = sim.m;
  win::g_cam = &cam;
  win::g_height = static_cast<double>(height);
  win::g_last_x = win::g_last_y = 0.0;
  glfwSetMouseButtonCallback(window, win::on_button);
  glfwSetCursorPosCallback(window, win::on_cursor);
  glfwSetScrollCallback(window, win::on_scroll);

  std::printf("窗口已打开:\n");
  std::printf("  鼠标左键拖动 = 转视角,右键拖动 = 平移,滚轮 = 缩放\n");
  std::printf("  空格 = 暂停/继续,M = 显示/隐藏底部电机信息,Esc = 退出\n");
  std::printf("电机在哪:\n");
  std::printf("  手臂三个关节处,深灰色圆柱壳体 + 橙色转子法兰就是电机;\n");
  std::printf("  壳体固定在不转的一侧(底座 / 近端连杆),转子跟着远端连杆转。\n");
  std::printf("  末端是 Robotiq 2F-85 夹爪(四杆机构,1 个驱动器带 8 个关节)。\n");
  std::printf("  左下角列出每个执行器驱动的关节/肌腱和它当前的出力(臂 N·m,夹爪 N·m)\n");
  std::printf("  (窗口里的文字只能用 ASCII —— MuJoCo 内置字体没有中文字形,写中文会变方块)\n");

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
    win::g_height = height > 0 ? static_cast<double>(height) : 600.0;
    const mjrRect viewport{0, 0, width, height};

    mjv_updateScene(sim.m, sim.d, &opt, nullptr, &cam, mjCAT_ALL, &scn);
    mjr_render(viewport, &scn, &con);
    if (show_motors) draw_motor_panel(sim, viewport, con);

    if (first_frame) {   // 首帧自检:确认 GL 真把画面画出来了
      first_frame = false;
      std::vector<unsigned char> rgb(3 * static_cast<size_t>(width) * height);
      mjr_readPixels(rgb.data(), nullptr, viewport, &con);
      long sum = 0;
      for (unsigned char v : rgb) sum += v;
      std::printf("首帧渲染:帧缓冲 %dx%d,平均亮度 %.1f(接近 0 说明没画出来)\n", width,
                  height, sum / static_cast<double>(rgb.size()));
    }

    const double* tp = sim.d->site_xpos + 3 * sim.ee;
    char line1[128], line2[128];
    // 窗口文字只能是 ASCII(中文字形不在 MuJoCo 内置字体里)
    std::snprintf(line1, sizeof(line1), "t = %.2f s%s", sim.t, paused ? "   [PAUSED]" : "");
    std::snprintf(line2, sizeof(line2),
                  "yaw=%+.1f sh=%+.1f el=%+.1f wr=%+.1f deg | TCP=(%.3f,%.3f,%.3f) grip=%.0fmm",
                  sim.qpos_deg(0), sim.qpos_deg(1), sim.qpos_deg(2), sim.qpos_deg(3), tp[0],
                  tp[1], tp[2], sim.grip_mm());
    mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport, line1, line2, &con);

    glfwSwapBuffers(window);
    glfwPollEvents();
  }

  mjr_freeContext(&con);
  mjv_freeScene(&scn);
  glfwTerminate();
  return 0;
}

int main(int argc, char** argv) {
  std::string xml = FKIK_MODEL_XML;
  double duration = 0.0;   // 0 = 一直跑,直到你关窗口
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (!a.empty() && a[0] == '-' && a != "-") { std::fprintf(stderr, "未知参数:%s\n", a.c_str()); return 1; }
    else if (a.find_first_not_of("0123456789.") == std::string::npos) duration = std::atof(a.c_str());
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
  std::printf("你的算法在 sim.cpp 的 my_algorithm() 里\n\n");

  return run_window(sim, duration);
}
