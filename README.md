# 7 轴机械臂工程

本仓库同时保存机械臂嵌入式固件和 MATLAB 可视化/模型工具。克隆一次即可获得固件、
串口解析程序、URDF 以及运行所需的全部 STL 网格，不依赖原开发电脑上的绝对路径。

## 目录结构

```text
.
├─ arm_small_machine/       ThreadX 嵌入式固件
└─ matlab_arm_visualizer/   MATLAB 串口可视化、URDF 和 meshes
```

- [固件说明](arm_small_machine/README.md)
- [MATLAB 工具说明](matlab_arm_visualizer/README.md)

MATLAB 模型资源来自独立仓库提交 `8bc9839 Bundle portable arm model assets`。运行仓库
不包含约 138 MB 的 STEP 源档；程序不依赖该文件，需要归档时应使用 Git LFS 或 Release
附件。

## 固件构建

依赖 CMake、Ninja 和 ARM GNU Toolchain。以下命令从总仓库根目录执行：

```bash
cmake -S arm_small_machine/board/dji_c \
  -B arm_small_machine/build/dji_c/Debug \
  --preset Debug -DROBOT=arm -DBOARD=single
cmake --build arm_small_machine/build/dji_c/Debug -j
```

输出文件为 `arm_small_machine/build/dji_c/Debug/base.elf`。烧录和 Ozone 调试前必须先支撑
机械臂；下载、复位或调试器停核期间，单片机无法持续发送重力补偿力矩。

## MATLAB 快速验证

安装 MATLAB、Robotics System Toolbox 和 Instrument Control Toolbox，然后在 MATLAB 中
进入 `matlab_arm_visualizer`：

```matlab
test_portable_package
test_board_link_parser
show_placeholder_arm
```

连接串口时：

```matlab
session = arm_serial_visualizer("COM15");
```

程序会根据自身位置寻找 `model/robot_dynamics.urdf` 和 `model/meshes`，仓库移动到其他
盘符或电脑后无需修改路径。

## 板间串口协议

当前固件通过 USART6（115200 baud，约 10 ms 一帧）发送固定 36 字节二进制帧：

```text
0xAA | LEN=32 | SEQ | J1..J7, TOOL（8 个 little-endian float32）| 0xFF
```

J1~J7 均为单位 `rad` 的绝对关节角，不是增量。发送坐标为：

```text
J1 = r1 + 1.836
J2 = r2 + 0.952
J3 = r3 - 3.148
J4 = total_angle4 - 1.01242732
J5 = 0.040 - r5
J6 = r6 - 1.04290104
J7 = r7 - 0.066
```

其中 J1/J2/J3/J5/J6/J7 使用 DM 单圈绝对角反馈，J4 使用 GM6020 软件累计多圈角。
MATLAB 端直接用最新一帧覆盖显示值，不得再次执行 `deg2rad` 或做增量累加。

第 8 个 float 是夹爪两状态命令：遥控器通道数组下标 4 为 `240` 时发送 `+1.0f`，
为 `1807` 时发送 `-1.0f`；遥控器离线或值无效时也发送释放值 `-1.0f`。这两个发送值
集中定义在固件 `apps/arm/arm_def.h`，接收端最终数值确定后可直接修改宏。

## 两部分的职责

- 固件负责电机反馈解码、零点和方向换算、重力补偿、安全门以及串口发送。
- MATLAB 负责协议解析、原始关节值显示及固件坐标到 URDF/CAD 坐标的可视化映射。
- MATLAB 中的 URDF 惯性参数可用于离线分析，但不会自动改写固件中的重力补偿参数。
- 修改角度协议、零点或方向时，应同时验证固件发送向量和 MATLAB 映射，避免把模型修正
  混入对外通信协议。
