# App 层外修改审计记录

## 规则

本工程的机械臂业务实现原则上只修改 `apps/` 层。任何确实需要修改 `apps/` 之外文件的操作，都必须在本文件追加记录，并在交付说明中明确告知：

- 修改了哪些文件；
- 修改的目的和影响范围；
- 是否经过构建或其他验证；
- 该修改是本轮主动实施，还是工作区既有改动被保留并同步。

本记录只审计 `apps/` 之外的文件；`apps/` 内的业务改动不在这里展开。

## 2026-10-01：机械臂第一阶段基线

对应提交：`c5296db feat(arm): add dji_c motor bring-up baseline`

### 本轮主动修改

| 文件 | 修改内容 | 目的 | 验证 |
| --- | --- | --- | --- |
| `.vscode/tasks.json` | 将 `Build dji_c` 设为默认构建任务；构建命令显式传入 `-DROBOT=arm -DBOARD=single`。保留工作区原有的 F103/F105 构建任务。 | 让快捷键构建机械臂时固定指向 DJI C/F407 板，而不是 H7；避免复用错误的 CMake 机器人缓存。 | `cmake --build build/dji_c/Debug --config Debug -j 4` 通过。 |
| `README.md` | 将项目说明改为 ARM 机械臂第一阶段目标，补充电机注册表、构建命令和分阶段验收顺序。 | 让仓库入口文档与当前“先注册电机、确认在线，再做重力补偿”的目标一致。 | 与 `dji_c/arm` 构建结果一致。 |
| `modules/MOTOR/motor_def.h` | 增加 `DM4340` 电机类型枚举。 | 允许 J2 按实际型号注册，不再落回错误的默认电机类型。 | 工程编译通过。 |
| `modules/MOTOR/DAMIAO/motor_damiao.c` | 将达妙参数表改为按枚举索引，并补充 DM6220、DM4340、DM3507 的位置、速度、力矩量程。 | 让七个电机的反馈解码和 MIT 报文编码使用对应型号量程。参数仍需结合实物手册和反馈实测复核。 | 工程编译通过；未接实物验证量程。 |

### 工作区既有改动：未由本轮主动设计，但随远端覆盖同步

以下改动在本轮开始前已存在于工作树。为完成用户要求的整目录覆盖，它们被包含在 `c5296db` 中；这里记录来源，后续不应把它们误认为机械臂 App 逻辑修改：

| 范围 | 现状 | 备注 |
| --- | --- | --- |
| `board/105_rc/**` | 删除/调整 F1 外设、HAL CAN/TIM 文件及 CubeMX/CMake 配置。 | 属于工作区原有的 105 RC 板裁剪改动；本阶段构建目标是 `board/dji_c`，未依赖它。 |
| `board/bsp/bsp_init.c` | 移除蜂鸣器初始化和 BSP 层 `BSP_CAN_TaskInit()` 调用。 | 属于工作区原有初始化调整；机械臂应用在注册电机前显式启动 CAN 任务。 |
| `.vscode/tasks.json` 中 F103/F105 条目 | 新增两个板卡构建任务。 | 属于工作区原有任务配置；本轮只改变 DJI C 默认任务和其 ARM 参数。 |

## 2026-10-01：状态日志标签修正

对应提交：`2b1b893 fix(arm): label motor status logs by joint`

本次只修改 `apps/arm/single_board/robot_control.c`，将达妙状态日志从按数组下标显示改为正确显示 `J1/J2/J3/J5/J6/J7`。没有发生 App 层外修改，因此没有新增审计项。

## 当前验证状态

- 本地分支 `arm` 与 `origin/main` 同步。
- `build/dji_c/Debug/base.elf` 已成功生成。
- 尚未接入真实机械臂硬件；在线/掉线、反馈 ID 和量程仍需烧录后通过 `arm_status` 日志确认。

## 2026-10-01：达妙电机 CAN 发送节奏调整

### 本轮主动修改

| 文件 | 修改内容 | 目的 | 验证 |
| --- | --- | --- | --- |
| `modules/MOTOR/DAMIAO/motor_damiao.c` | 达妙清错和启动命令均重复发送两次；普通 DM 控制帧发送后增加 200us 间隔，命令帧发送后增加 400us 间隔。 | 复用旧机械臂工程对达妙 CAN 抢占/响应时序的处理方式，避免 6 个 DM 电机在同一周期内无间隔排队发送导致反馈停止、离线检测超时。 | `cmake --build build/dji_c/Debug --parallel 4` 通过；需烧录后观察 `arm_status`/`g_arm_feedback_snapshot` 是否持续更新。 |
| `modules/MOTOR/DJI/motor_dji.c` | DJI/GM6020 共享控制帧发送后增加 200us 间隔。 | 复用旧机械臂工程中 GM6020 对前后 CAN 空窗的要求，避免 6020 控制帧与 DM 帧紧贴发送导致反馈异常。 | `cmake --build build/dji_c/Debug --parallel 4` 通过；需烧录后观察 J4 反馈和 DM 反馈是否持续更新。 |

## 2026-10-01：电机驱动注释补全

### 本轮主动修改

| 文件 | 修改内容 | 目的 | 验证 |
| --- | --- | --- | --- |
| `modules/MOTOR/DAMIAO/motor_damiao.c` | 为本阶段改动过或直接受影响的达妙函数补充标准中文嵌入式注释，包括 `mit_ctrl()`、`pos_speed_ctrl()`、`speed_ctrl()`、`psi_ctrl()`、`dm_ControlAndSend()`、`Motor_DM_Cmd()` 和 `Motor_DM_Init()`。 | 明确函数输入、输出/返回、调用关系和 CAN 帧间隔设计原因，便于后续审查和接手调试；不改变运行逻辑。 | `cmake --build build/dji_c/Debug --parallel 4` 通过。 |
| `modules/MOTOR/DJI/motor_dji.c` | 为 `Motor_DJI_Flush()` 补充标准中文嵌入式注释。 | 明确 GM6020 控制帧发送流程和帧间隔原因，便于后续审查；不改变运行逻辑。 | `cmake --build build/dji_c/Debug --parallel 4` 通过。 |

## 2026-10-01：电机 CAN 槽位调度调整

### 本轮主动修改

| 文件 | 修改内容 | 目的 | 验证 |
| --- | --- | --- | --- |
| `modules/MOTOR/motor_base.c` | `Motor_UpdateAll()` 从单次遍历全部电机，调整为 CAN 槽位轮询：每个槽位内每条 CAN 总线最多调用一台电机的 `ControlAndSend()`，槽位之间插入 200us 等待；新增内部辅助函数 `motor_is_can_bus()` 和 `motor_find_next_can()`。 | 复刻旧机械臂工程 `Core/Src/freertos.c` 的节奏：CAN1 和 CAN2 互不抢占，可在同一时间片各发一帧；真正需要间隔的是同一条 CAN 内部相邻报文。 | `cmake --build build/dji_c/Debug --parallel 4` 通过。 |
| `modules/MOTOR/motor_base.h` | 更新 `Motor_UpdateAll()` 接口注释，说明 CAN 槽位轮询调度和调用关系。 | 让调度行为在公共头文件中可审查。 | `cmake --build build/dji_c/Debug --parallel 4` 通过。 |
| `modules/MOTOR/DAMIAO/motor_damiao.c` | 删除达妙运行控制帧发送函数内部的 200us 延时，并删除 `dm_ControlAndSend()` 尾部已失效的注释掉延时代码；达妙特殊命令帧 400us 间隔保持不变。 | 避免把 CAN1/CAN2 跨总线发送也人为串行拉开；运行期同一 CAN 内部间隔统一由 `Motor_UpdateAll()` 的槽位调度保证。 | `cmake --build build/dji_c/Debug --parallel 4` 通过。 |

## 2026-10-01：升级到主仓库底座并迁移机械臂必要补丁

### 本轮主动修改

| 文件 | 修改内容 | 目的 | 验证 |
| --- | --- | --- | --- |
| 全工程底座 | 将当前工作区底座切换到 `upstream/main`（`HebutMas/mas_embedded_threadx` 的 `2d39c0f`），再叠加机械臂 APP 和必要模块兼容补丁。 | 复用主仓库最新两阶段电机框架、CAN TX FIFO、模块结构等改动，减少继续维护旧底座的风险。 | 独立 worktree 试迁移构建通过；主工作区 `cmake -S board/dji_c -B build/dji_c/Debug --preset Debug -G Ninja -DROBOT=arm -DBOARD=single` 配置通过，`cmake --build build/dji_c/Debug --parallel 4` 构建通过。 |
| `apps/arm/**` | 保留机械臂 APP 逻辑和中文标准注释；将旧的 `Motor_DM_Stop()` / `Motor_DJI_Stop()` 适配为主仓库通用 `Motor_Stop(&base)`。 | 适配主仓库电机 API，同时保持当前“只注册电机、输出失能、读取反馈快照和状态日志”的 APP 行为。 | 主工作区构建通过。 |
| `modules/MOTOR/motor_def.h` | 增加 `DM4340` 电机类型枚举。 | 允许 J2 按实际达妙 DM4340 型号注册。 | 主工作区构建通过。 |
| `modules/MOTOR/DAMIAO/motor_damiao.c` | 将达妙参数表扩展为按电机枚举索引，补充 DM4310、DM4340、DM6220、DM3507 参数；特殊命令帧间隔调整为 400us；初始化阶段清错和启动命令各重复发送一次；补充中文标准注释。 | 防止主仓库默认全部落回 DM4310 参数导致 J1/J2/J6/J7 反馈和控制量程错误；保留旧工程上电命令节奏。 | 主工作区构建通过。 |
| `modules/MOTOR/motor_base.c` / `modules/MOTOR/motor_base.h` | 在主仓库两阶段框架的 `Motor_ApplyAll()` 中加入 CAN 槽位轮询：每个槽位每条 CAN 总线最多应用一台电机，槽位之间等待 200us；补充中文标准注释。 | 主仓库已有 CAN TX FIFO，但未保证同一 CAN 内部相邻运行控制帧间隔；该补丁复刻旧工程 CAN1/CAN2 可同片发送、同 CAN 内部要隔开的节奏。 | 主工作区构建通过。 |

## 2026-10-01：DM4340 参数复核与默认构建任务修复

### 本轮主动修改

| 文件 | 修改内容 | 目的 | 验证 |
| --- | --- | --- | --- |
| `.vscode/tasks.json` | 将默认 `Build selected board`、`Build dji_c`、`Build damiao_h7` 构建任务从 `type: shell` 改为 `type: process`；Windows 下直接启动 `powershell`，避免 VSCode 外层 PowerShell 再套一层内部 PowerShell；三个构建任务均显式传入 `-DROBOT=arm -DBOARD=single`。 | 修复快捷键构建时 `$board='dji_c'` 被外层引号吃掉、变成 `$board= dji_c` 的错误，同时避免 `cmake` 去错误读取 `board/CMakePresets.json`；保证手动选择任意板子时仍编译机械臂 APP。 | `tasks.json` 通过 `ConvertFrom-Json` 校验；按任务等价命令执行 `dji_c` 构建通过，输出 `Robot: arm`、`Board: single`，`ninja: no work to do`。 |
| `modules/MOTOR/DAMIAO/motor_damiao.c` | 对 DM4340 参数来源补充说明：位置 ±12.5 rad、速度 ±10 rad/s、力矩 ±28 N·m 参照旧机械臂工程；Kp/Kd 下限保留 0。 | 新建 4340 型号时匹配旧工程实际量程；同时保留旧工程 `DM_4340_cmd()` 只发力矩、位置/速度/Kp/Kd 原始字节为 0 的行为，避免 Kp=0/Kd=0 被负下限量程错误编码成非零原始值。 | 主工作区 `dji_c/arm` 构建通过。 |

## 后续记录格式

后续每次发生 App 层外修改时，在本文件末尾新增日期、提交、文件、目的、验证和来源；如果只修改 `apps/`，在交付说明中注明“无 App 层外修改”。
