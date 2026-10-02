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

## 2026-10-01：电机底层安全限位和致命故障蜂鸣

### 本轮主动修改

| 文件 | 修改内容 | 目的 | 验证 |
| --- | --- | --- | --- |
| `modules/MOTOR/motor_def.h` | 新增 `Motor_Safety_Limit_s`，并在 `Motor_Init_Config_s` 中加入 `safety_limit_config`。 | 让各 APP 在注册电机时传入 raw/角度限位参数，但限位检查逻辑留在电机底层。 | `cmake --build build/dji_c/Debug --parallel 4` 通过。 |
| `modules/MOTOR/motor_base.h` / `modules/MOTOR/motor_base.c` | 在 `Motor_Base` 中保存安全限位、名称和原始位置读取回调；`Motor_ApplyAll()` 在调用驱动 `Apply()` 前执行底层限位检查；超限后锁存全局安全故障、失能全部电机、清零输出/PID 积分，并拒绝后续 `Motor_Start()` 重新使能。 | 保证不管上层如何调用 start/set/output，真正下发前都必须经过底层安全闸门；任一电机超限即全局急停。 | `cmake --build build/dji_c/Debug --parallel 4` 通过。 |
| `modules/MOTOR/DAMIAO/motor_damiao.h` / `modules/MOTOR/DAMIAO/motor_damiao.c` | 达妙反馈结构保存反馈包 16bit 原始位置值；初始化时接入安全限位配置和 raw 读取回调。 | 让达妙关节可直接使用实测 raw 值做底层限位，避免用角度反推造成调试偏差。 | `cmake --build build/dji_c/Debug --parallel 4` 通过。 |
| `modules/MOTOR/DJI/motor_dji.c` | GM6020/DJI 初始化时接入安全限位配置和 raw 读取回调，raw 值使用 `ecd`。 | 让 J4 GM6020 可使用 0~8191 编码器原始值做底层限位。 | `cmake --build build/dji_c/Debug --parallel 4` 通过。 |
| `modules/OFFLINE/module_offline.h` / `modules/OFFLINE/module_offline.c` | 新增 `Module_Offline_SetFatalFault()` 和 `Module_Offline_HasFatalFault()`；Offline 线程在 fatal fault 锁存后统一持续红灯和蜂鸣。 | 使用统一系统报警模块处理蜂鸣，避免电机底层直接操作蜂鸣器；本次不改变看门狗喂狗/复位逻辑。 | `cmake --build build/dji_c/Debug --parallel 4` 通过。 |

## 2026-10-02：底层限位预扫描和 raw 跨零处理

### 本轮主动修改

| 文件 | 修改内容 | 目的 | 验证 |
| --- | --- | --- | --- |
| `modules/MOTOR/motor_base.h` / `modules/MOTOR/motor_base.c` | 在 `Motor_Base` 中新增 `feedback_valid`；`Motor_ApplyAll()` 改为先对全部电机做安全限位预扫描，再开始任何 CAN/PWM/UART Apply；raw 限位支持跨编码器 0 点区间；限位在收到首帧有效反馈前不参与判定。 | 修复“前面电机已下发、后面电机才发现超限”的半周期风险；避免上电 raw 初始 0 在首帧反馈前误触发；支持 GM6020/达妙 raw 区间跨 0 点。 | `cmake --build build/dji_c/Debug --parallel 4` 通过。 |
| `modules/MOTOR/DAMIAO/motor_damiao.c` | 达妙接收回调在完成反馈解码后置 `base.feedback_valid=1`。 | 明确 raw/角度限位必须基于真实反馈帧，避免未收到反馈时误判。 | `cmake --build build/dji_c/Debug --parallel 4` 通过。 |
| `modules/MOTOR/DJI/motor_dji.c` | DJI/GM6020 接收回调在完成反馈解码后置 `base.feedback_valid=1`。 | 明确 raw/角度限位必须基于真实反馈帧，避免未收到反馈时误判。 | `cmake --build build/dji_c/Debug --parallel 4` 通过。 |

## 2026-10-02：具体驱动 Apply 入口增加单电机最终安全校验

### 本轮主动修改

| 文件 | 修改内容 | 目的 | 验证 |
| --- | --- | --- | --- |
| `modules/MOTOR/motor_base.h` / `modules/MOTOR/motor_base.c` | 新增 `Motor_SafetyCheckBeforeApply()`；在具体驱动真正编码 MIT/位置/速度/电流报文前，重复检查当前电机的反馈限位。若刚刚越限，只失能当前电机、清零当前输出，并返回失败状态。 | 在 `Motor_ApplyAll()` 全局预扫描之外增加单电机最后一道拦截，缩短“预扫描通过到具体电机发送”之间的窗口；保持实现简单，不重复重写各协议编码函数。 | `cmake --build build/dji_c/Debug --parallel 4` 通过。 |
| `modules/MOTOR/DAMIAO/motor_damiao.c` | `dm_apply()` 入口调用 `Motor_SafetyCheckBeforeApply()`；失败后进入已有零 MIT/位置/速度/PSI 输出分支。 | 保证达妙具体模式发送前，当前电机越限时不再发正常力矩/速度/位置指令。 | `cmake --build build/dji_c/Debug --parallel 4` 通过。 |
| `modules/MOTOR/DJI/motor_dji.c` | `dji_apply()` 入口调用 `Motor_SafetyCheckBeforeApply()`；失败后进入已有零电流帧分支。 | 保证 GM6020 具体控制帧发送前，当前电机越限时只发零电流，不继续输出正常电流。 | `cmake --build build/dji_c/Debug --parallel 4` 通过。 |

## 2026-10-02：机械臂角度限位来源整理

### 本轮主动修改

| 文件 | 修改内容 | 目的 | 验证 |
| --- | --- | --- | --- |
| `modules/MOTOR/motor_def.h` / `modules/MOTOR/motor_base.c` | 新增 `Motor_Safety_Angle_Source_e`，角度限位可明确选择单圈/协议角度或软件累计总角度；安全检查统一通过选择器读取角度。 | 让不跨圈关节使用 `single_round_angle`，J4/J6 跨圈关节使用 `total_angle`，不改变任何电机控制模式。 | `cmake --build build/dji_c/Debug --parallel 4` 通过。 |
| `apps/arm/arm_def.h` / `apps/arm/single_board/robot_control.c` | 写入 J2~J7 角度限位（单位 rad）：J2 `[-1.9,1.8]`、J3 `[0.2,5.53]`、J4 `[-4.5,9.0]`、J5 `[-1.6,1.6]`、J6 `[-5.2,7.0]`、J7 `[-2.0,2.0]`；J2/J3/J5/J7 使用单圈/协议角度，J4/J6 使用累计角度；J1 限位保持关闭。 | 将实测机械限位直接配置到安全层，避免把角度值误当成 raw 编码值。 | `cmake --build build/dji_c/Debug --parallel 4` 通过。 |
| `apps/arm/single_board/robot_control.c` | 将 J1 电机型号由 DM6220 改为 DM4310。 | 匹配底盘一号电机更换后的实物型号，避免速度/力矩编解码量程复用错误参数。 | `cmake --build build/dji_c/Debug --parallel 4` 通过。 |
| `modules/MOTOR/DAMIAO/motor_damiao.c` / `modules/MOTOR/DJI/motor_dji.c` | 首次收到有效反馈时只建立累计角度基线，不根据默认 0 值判断跨圈。 | 防止 J4/J6 上电第一帧因与默认值差异过大而误加/减一圈，导致累计角度限位误触发。 | `cmake --build build/dji_c/Debug --parallel 4` 通过。 |

## 后续记录格式

后续每次发生 App 层外修改时，在本文件末尾新增日期、提交、文件、目的、验证和来源；如果只修改 `apps/`，在交付说明中注明“无 App 层外修改”。
