# models/ 目录约定

每个模型/部件一个文件夹,网格放在各自部件文件夹的 `assets/` 里,不要混在一起。

```
models/
├── two_link_arm.xml              # 主模型(整机):手臂 + 夹爪 + 地面 + 目标点,平时跑这个
├── grippers/
│   └── robotiq_2f85/             # 一个部件 = 一个文件夹
│       ├── README.md             # 来源、许可、怎么接上、控制接口
│       ├── LICENSE
│       ├── robotiq_2f85.xml      # 上游原版模型(参考用,可单独加载)
│       └── assets/*.stl          # 这个部件自己的网格
├── arms/                         # (以后)机械臂本体,同样一个部件一个文件夹
└── objects/                      # (以后)要抓的物体,比如方块、杯子、圆柱
```

## 加新模型 / 新部件时

1. 在 `models/` 下按类别建文件夹:`grippers/<名字>/`、`arms/<名字>/`、`objects/<名字>/`;
2. 它自己的网格一律放进该文件夹的 `assets/`;
3. 主模型 `two_link_arm.xml` 里引用网格时写**相对 `models/` 的完整路径**,例如:

   ```xml
   <mesh class="gripper" file="grippers/robotiq_2f85/assets/base.stl"/>
   ```

   (不设 `meshdir`,默认就是 XML 所在目录 `models/`,这样不同部件的网格可以各放各的。)
4. 如果这个部件来自开源模型库(比如 [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie)),
   把它的 `LICENSE` 和来源说明一起放进这个文件夹。

## 命名提醒

MJCF 里 body / joint / site / geom 的**名字必须全局唯一**。把第三方部件接进主模型时,
先检查有没有重名(例如夹爪的 `base` 和手臂底座的 `base`),有就加前缀改名,
本项目用的是 `gripper_` 前缀。
