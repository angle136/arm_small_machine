%% 7轴机械臂串口实时可视化启动脚本
% 本目录独立于嵌入式工程，不会修改或控制下位机。

%% 1. 用户配置
PORT_NAME = "";                 % 留空时弹窗选择；也可直接填写 "COM5"
ENABLE_VISUALIZATION = true;    % false：只接收串口，不创建 3D 图窗
CONSOLE_DIAGNOSTICS = false;    % false：原始值只显示在右侧面板，不刷命令行
PRINT_EVERY_N_FRAMES = 10;      % 仅 CONSOLE_DIAGNOSTICS=true 时生效
PRINT_RAW_CHUNKS = false;        % 仅 CONSOLE_DIAGNOSTICS=true 时生效
TIMER_PERIOD_S = 0.02;          % 50 Hz 轮询；固件约 100 Hz 发送
RENDER_PERIOD_S = 0.20;         % 3D 以 5 Hz 刷新；串口读取仍为 50 Hz
BAUD_RATE = 115200;             % 与 apps/arm/robot.cmake 的 BOARD_LINK_BAUD 一致
LIVE_VISUALS = true;            % false 可切换为简单碰撞几何，界面更流畅
URDF_PATH = "";                % 留空使用仓库内置 model/robot_dynamics.urdf
DEFAULT_VIEW = 5;               % 01~10；05 主视图为 Y 向上、Z 向右
JOINT_INVERSION = [1 1 1 1 1 0 0]; % 已标定：J1~J5 输入取反

%% 2. 启动
% 重复运行前，先执行 armSession.stop() 释放旧串口。
if strlength(PORT_NAME) == 0
    PORT_NAME = select_serial_port();
end

if strlength(PORT_NAME) == 0
    fprintf('[CANCEL] 未选择串口，没有启动实时接收。\n');
    return;
end

armSession = arm_serial_visualizer(PORT_NAME, ...
    'Visualize', ENABLE_VISUALIZATION, ...
    'ConsoleDiagnostics', CONSOLE_DIAGNOSTICS, ...
    'PrintEveryNFrames', PRINT_EVERY_N_FRAMES, ...
    'PrintRawChunks', PRINT_RAW_CHUNKS, ...
    'TimerPeriod', TIMER_PERIOD_S, ...
    'RenderPeriod', RENDER_PERIOD_S, ...
    'BaudRate', BAUD_RATE, ...
    'LiveVisuals', LIVE_VISUALS, ...
    'UrdfPath', URDF_PATH, ...
    'DefaultView', DEFAULT_VIEW, ...
    'InversionFlags', JOINT_INVERSION);

%% 3. 常用命令
% 查看运行统计：
%   armSession.status()
%
% 运行中修改方向：
%   armSession.setInversion([1 1 1 1 1 0 0])
%
% 安全停止并释放串口：
%   armSession.stop()
