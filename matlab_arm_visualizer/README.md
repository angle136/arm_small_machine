# MATLAB 7轴机械臂串口实时可视化

该目录是总仓库内独立、可迁移的 MATLAB 工具，不修改同级 `arm_small_machine`
嵌入式工程。代码、URDF 和完整 STL 网格都在同一个仓库中，不依赖原电脑的用户名、桌面路径
或仓库外部文件。

## 仓库内容和运行环境

```text
matlab_arm_visualizer/
├─ *.m                         MATLAB 程序和离线测试
└─ model/
   ├─ robot_dynamics.urdf      默认使用的惯性修正版
   ├─ robot.urdf               保留的原始导出文件
   └─ meshes/                  完整 STL 网格
```

另一台电脑只需安装 MATLAB、Robotics System Toolbox 和 Instrument Control
Toolbox，然后克隆整个仓库。在 MATLAB“当前文件夹”中进入仓库根目录即可运行，
不需要修改模型路径。可先执行 `test_portable_package` 检查模型文件是否完整。

## 当前固件协议

当前工程不是 `#角度,...\r\n` 文本协议，而是固定 36 字节二进制帧：

```text
0xAA | LEN=32 | SEQ | J1...J7, TOOL（8个little-endian float32）| 0xFF
```

- 波特率：115200，8N1（当前 `apps/arm/robot.cmake` 的 `BOARD_LINK_BAUD`）。
- 固件约每 10 ms 发送一帧。
- J1~J7 已经是机械关节角，单位为弧度。
- 每帧的 J1~J7 是当前位置的绝对关节角，不是相对上一帧的增量；MATLAB 直接用最新一帧覆盖模型配置。
- TOOL 是夹爪两状态命令：遥控器通道数组下标 4 为 `240` 时发送 `+1.0f`，
  为 `1807` 时发送 `-1.0f`；遥控器离线或通道值无效时发送 `-1.0f`。
- MATLAB 端不得再次执行 `deg2rad`，也不做增量累加。固件零位到 CAD/URDF
  零位的固定转换集中写在 `map_arm_configuration.m`，不会改变右侧显示的原始值。

## 建议验证顺序

在 MATLAB 中将当前文件夹切换到本目录，然后依次执行：

```matlab
% 1. 验证仓库内 URDF、网格引用和模型导入
test_portable_package

% 2. 不连接硬件，验证二进制解析器
test_board_link_parser

% 3. 不连接硬件，查看机械臂模型
show_placeholder_arm

% 也可以在不接串口时手动更新 7 个关节（单位 rad）
viewer = show_placeholder_arm;
viewer.setConfiguration([0.2, -0.4, 0.8, 0.1, -0.6, 0.3, 0.5]);
viewer.getConfiguration()
viewer.setInversion([1 1 1 1 1 0 0]); % 当前标定：J1~J5 输入取反
viewer.getInversion()
viewer.close()

% 指定启动视图编号（01~10）
viewer = show_placeholder_arm("", [], 'DefaultView', 3);

% 4. 只验证实机串口，打印每一帧
session = arm_serial_visualizer("COM5", ...
    'Visualize', false, 'PrintEveryNFrames', 1);

% 若实际固件工程配置为 921600，可显式指定：
session = arm_serial_visualizer("COM5", 'BaudRate', 921600);

% 5. 串口与模型联合运行
session.stop();
session = arm_serial_visualizer(); % 弹窗选择可用串口，默认主视图 05

% 串口模式也可以指定启动视图编号
session = arm_serial_visualizer("COM15", 'DefaultView', 3);

% 运行中应用当前方向标定：J1~J5 取反
session.setInversion([1 1 1 1 1 0 0]);
session.getInversion()

% 6. 安全停止并释放串口
session.stop();
```

也可以编辑并运行 `run_arm_visualizer.m`。

`show_placeholder_arm` 只是离线模型预览，因此不会要求选择端口。运行
`arm_serial_visualizer()` 或 `run_arm_visualizer.m` 时才会弹出串口选择窗口。

程序启动时根据整条关节链、URDF visual origin、mesh scale 和 STL 网格边界，一次
计算覆盖连续全部关节角的固定工作空间。实时刷新只更新机械臂姿态，不再重算范围
或格子；三维窗口默认隐藏坐标网格、刻度和黑色坐标背景，原始值仍显示在右侧。
按 `V` 键循环编号视图，数字键 `1`~`9`
直接选择 01~09，数字键 `0` 选择 10；左上角会持续显示当前编号和名称。滚轮可
缩放。缩放后若模型超出画面，按 `F` 保持当前观察
方向并重新完整取景；按 `H` 或 `Home` 随时回到固定主视图 05。切换任一编号
视图时也会自动完整取景。鼠标左键拖动为自由旋转；鼠标中键拖动只绕基座竖直轴
作水平旋转，不会改变俯仰角。

模型使用两个互为相反侧、位置固定的方向光源和少量固定环境光。相机停留在同一
水平半球时光源完全不动；只有跨入相反半球时才切换到另一侧固定光源。因此转到
机械臂背面不会全黑，也不会为了让光源逐帧跟随相机而持续增加渲染开销。

视图编号如下，默认主视图使用 05：

```text
01 正视          02 后视
03 左视          04 右视
05 主视图（Y上、Z右、轻微斜视）  06 仰视
07 等轴 前右     08 等轴 前左
09 等轴 后左     10 等轴 后右
```

以后确定默认方向后，只需修改 `run_arm_visualizer.m` 的 `DEFAULT_VIEW`，或启动时
传入 `'DefaultView', 编号`。

实时窗口右侧会显示累计接收字节、有效帧数量、最新原始 HEX 和 J1~J7。
默认不向命令行周期打印串口数据，以免拖慢界面。需要专项诊断时，可把
`ConsoleDiagnostics` 设为 `true`；其中 `[WAIT]` 表示端口已打开但没有新字节，
`[RAW]` 表示已经收到物理串口数据，`RX HEX` 和 `SEQ` 表示成功解析。

## 实物 URDF

当前使用仓库内置的 `model/robot_dynamics.urdf` 和 `model/meshes`。默认路径由
`default_arm_urdf_path.m` 根据代码自身位置计算，因此移动或克隆整个仓库后无需
修改。该模型包含 8 个 link（`base_link` + `link_002` 至
`link_008`）和 7 个 revolute joint（`link_002_joint` 至 `link_008_joint`）。
`build_placeholder_arm.m` 默认直接通过 `importrobot` 导入。不要单独移动 URDF
或 meshes，否则 URDF 中的相对网格路径会失效。

原始 `robot.urdf` 保持不动。修正版只替换原文件中不可能成立的 `link_005` 和
`link_006` 惯性矩阵，质量、质心、关节和网格均未修改。替换值由各连杆 STL
封闭网格按均匀密度积分，再按照 URDF 原质量缩放得到；两个矩阵均通过半正定和
主惯量三角不等式检查。这组惯性适合修复导入并作为动力学初值，静态重力项仍由
原质量和原质心决定。若以后取得 CAD 质量属性，可再替换这两组估计值。

如果以后导入其他仍含非法惯性矩阵的 URDF，程序仍会仅针对该错误生成临时的
无惯性副本用于可视化；这条回退路径不能用于 `gravityTorque` 动力学计算。

串口样本的 7 个关节值按 URDF 链顺序使用，每帧都是绝对角度覆盖，不做
“相对上一帧的增量累加”：
`link_002_joint`、`link_003_joint`、`link_004_joint`、`link_005_joint`、
`link_006_joint`、`link_007_joint`、`link_008_joint`。URDF 关节轴和安装偏置
由 URDF 自带的 axis/origin 定义；可视化启用 URDF 的 visual meshes，不再使用
简化碰撞圆柱。实时运行使用 `rigidBodyTree.show(...,'FastUpdate',true)` 更新已创建
的图形对象，并仅在关节值变化时按配置频率刷新，避免每帧重新创建网格阻塞相机操作。

固件发送的零点是整臂竖直标定零点，这里的“竖直”严格定义为主连杆方向垂直于
基座平面。URDF 的零配置则是 CAD 导出时的弯曲装配姿态。程序通过
`map_arm_configuration.m` 显式叠加 7 个 URDF 零位偏置，使固件
`[0 0 0 0 0 0 0]` 的主连杆沿 CAD 基座法向排列，即垂直于基座平面。程序保留
CAD 基座与连杆之间的真实相对方向，不使用 `show` 的 `Position` 参数旋转机器人；
该参数在当前 MATLAB 版本中是四元素绘图区位置，并不是六自由度基座位姿。右侧
状态面板和标题仍显示固件原始绝对角度，不显示加入偏置后的内部 URDF 角度。

若某个电机的实际正方向与 URDF 相反，通过 7 位 `InversionFlags` 调整：`0` 保持
输入，`1` 对该关节输入取反。可在 `run_arm_visualizer.m` 顶部修改
`JOINT_INVERSION`，也可在运行中调用
`session.setInversion([1 1 1 1 1 0 0])`。当前默认值就是这组标定，即 J1~J5
取反，J6、J7 保持。该处理严格发生在 URDF 零位偏置之前，
所以无论方向标志如何设置，单片机全零输入对应的已标定零姿态都不会改变；右侧
状态面板仍显示未经取反的串口原始值，并另外显示当前 7 位方向标志。

当前零位还包含一次实物网格手动标定修正：
`[0, -0.15, -0.19, 2.70, -0.24, 0.90, -1.10] rad`。这是旧映射下确认正确
零姿态时输入的绝对角度；它已合并到 `ZERO_POSE_TRIM_RAD`，所以现在向查看器或
串口模型输入全 0，显示的就是该姿态。后续微调只需修改这组修正，不要改串口原值。

URDF 中各关节限位目前都是约 `[-1.57, 1.57] rad`，是导出模型里的值，未必等于
固件的实际安全限位。串口实时显示以固件发送的弧度为准，不更改其角度正负号或
量程；模型接口仅使用上述明确记录的 CAD 零位偏置。

若 URDF 的关节顺序或坐标约定与固件不同，应在 MATLAB 模型接口处显式映射，
不要修改固件现有的 J1~J7 发送坐标契约。

## 接线提示

当前板端发送接口为 USART6：PG14/TX、PG9/RX，3.3 V TTL。仅接收机械臂数据时，
至少连接板端 TX 到 USB 转串口模块 RX，并连接双方 GND。不要在未确认供电方案时连接 VCC。

重新加载并启动串口模式：

```matlab
close all
clear functions
session = arm_serial_visualizer("COM15");
```
