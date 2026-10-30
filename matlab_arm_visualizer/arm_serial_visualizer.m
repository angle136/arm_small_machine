function session = arm_serial_visualizer(portName, varargin)
%ARM_SERIAL_VISUALIZER 从串口接收 7 轴关节角并刷新 rigidBodyTree。
%
% 当前 arm 固件使用二进制 board_link 协议，而不是以 # 开头的文本行：
%   0xAA | 36 | SEQ | J1~J7 + TOOL（8个float32）| STATUS（uint32）| 0xFF
% 关节角已经是 rad，因此本程序不会再次调用 deg2rad。
%
% 基本使用：
%   session = arm_serial_visualizer("COM5");
%   session = arm_serial_visualizer();       % 弹窗选择可用串口
%
% 只测试串口接收和解析，不显示模型：
%   session = arm_serial_visualizer("COM5", ...
%       'Visualize', false, 'PrintEveryNFrames', 1);
%
% 安全停止并释放串口：
%   session.stop();
%
% 可选名称-值参数：
%   Visualize          - 是否显示机械臂，默认 true。
%   ConsoleDiagnostics - 是否向命令行周期打印接收数据，默认 false。
%   PrintEveryNFrames  - 开启命令行诊断时的有效帧打印间隔，默认 10。
%   PrintRawChunks     - 开启命令行诊断时是否打印原始 HEX，默认 false。
%   TimerPeriod        - timer 周期，默认 0.02 s。
%   RenderPeriod       - 3D 模型刷新周期，默认 0.20 s；不影响串口读取频率。
%   BaudRate           - 串口波特率，默认 115200（当前 apps/arm/robot.cmake）。
%   LiveVisuals        - 实时绘制 URDF 网格；false 时使用碰撞几何，响应更快。
%   UrdfPath           - URDF 路径；留空时使用 build_placeholder_arm 的默认路径。
%   DefaultView        - 启动时的编号视图，1~10，默认 5（Y向上、Z向右）。
%   InversionFlags     - J1~J7 输入方向开关；0 保持，1 取反。
%                        默认 [1 1 1 1 1 0 0]。

% -------------------------------------------------------------------------
% 1. 参数检查
% -------------------------------------------------------------------------
if nargin < 1 || isempty(portName)
    portName = select_serial_port();
    if strlength(portName) == 0
        error('arm_serial_visualizer:PortSelectionCancelled', ...
              '未选择串口，启动已取消。');
    end
end

parser = inputParser;
addRequired(parser, 'portName', @(x) ischar(x) || (isstring(x) && isscalar(x)));
addParameter(parser, 'Visualize', true, @(x) islogical(x) && isscalar(x));
addParameter(parser, 'ConsoleDiagnostics', false, @(x) islogical(x) && isscalar(x));
addParameter(parser, 'PrintEveryNFrames', 10, ...
    @(x) isnumeric(x) && isscalar(x) && isfinite(x) && x >= 1 && fix(x) == x);
addParameter(parser, 'PrintRawChunks', false, @(x) islogical(x) && isscalar(x));
addParameter(parser, 'TimerPeriod', 0.02, ...
    @(x) isnumeric(x) && isscalar(x) && isfinite(x) && x >= 0.01);
addParameter(parser, 'RenderPeriod', 0.20, ...
    @(x) isnumeric(x) && isscalar(x) && isfinite(x) && x >= 0.05);
addParameter(parser, 'BaudRate', 115200, ...
    @(x) isnumeric(x) && isscalar(x) && isfinite(x) && x > 0 && fix(x) == x);
addParameter(parser, 'LiveVisuals', true, @(x) islogical(x) && isscalar(x));
addParameter(parser, 'UrdfPath', "", ...
    @(x) ischar(x) || (isstring(x) && isscalar(x)));
addParameter(parser, 'DefaultView', 5, ...
    @(x) isnumeric(x) && isscalar(x) && isfinite(x) && ...
         x >= 1 && x <= 10 && fix(x) == x);
addParameter(parser, 'InversionFlags', [1 1 1 1 1 0 0], ...
    @(x) (isnumeric(x) || islogical(x)) && isvector(x) && numel(x) == 7);
parse(parser, portName, varargin{:});

portName = char(string(parser.Results.portName));
visualize = parser.Results.Visualize;
consoleDiagnostics = parser.Results.ConsoleDiagnostics;
printEveryNFrames = double(parser.Results.PrintEveryNFrames);
printRawChunks = parser.Results.PrintRawChunks;
timerPeriod = double(parser.Results.TimerPeriod);
renderPeriod = double(parser.Results.RenderPeriod);
baudRate = double(parser.Results.BaudRate);
liveVisuals = parser.Results.LiveVisuals;
urdfPath = string(parser.Results.UrdfPath);
defaultView = double(parser.Results.DefaultView);
inversionFlags = validateInversionFlags(parser.Results.InversionFlags);

% 这些变量由嵌套回调共享，避免 global 和阻塞式 while 循环。
serialDevice = [];
timerObject = [];
robot = [];
fig = [];
ax = [];
statusText = [];
rxBuffer = zeros(1, 0, 'uint8');
lastSequence = [];
latestJointAngles = nan(1, 7);
latestToolValue = nan;
latestRawPreview = '尚未收到任何字节';
validFrameCount = 0;
droppedFrameCount = 0;
invalidFrameCount = 0;
callbackErrorCount = 0;
totalByteCount = 0;
readChunkCount = 0;
isStopping = false;
cameraInitialized = false;
startTime = datetime('now');
sessionClock = tic;
lastReceiveElapsed = nan;
lastValidFrameElapsed = nan;
lastConsoleReportElapsed = -inf;
lastRawPrintElapsed = -inf;
lastRenderElapsed = -inf;
lastStatusUpdateElapsed = -inf;
lastRenderedAngles = nan(1, 7);
modelBoundsInitialized = false;
workspaceRadius = [];

% -------------------------------------------------------------------------
% 2. 打开串口、创建模型和 timer
% -------------------------------------------------------------------------
try
    serialDevice = serialport(portName, baudRate, ...
        'DataBits', 8, ...
        'StopBits', 1, ...
        'Parity', 'none', ...
        'Timeout', 0.05);
    flush(serialDevice);

    if visualize
        robot = build_placeholder_arm(urdfPath);
        % 启动时一次计算覆盖连续全部关节角的固定空间，timer 内不再重算。
        workspaceRadius = calculate_arm_workspace_radius(robot, urdfPath);
        fig = figure('Name', sprintf('7轴机械臂实时姿态 - %s', portName), ...
                     'NumberTitle', 'off', ...
                     'Color', [0.97, 0.97, 0.97], ...
                     'WindowState', 'maximized');
        ax = axes('Parent', fig, ...
                  'Units', 'normalized', ...
                  'Position', [0.04, 0.08, 0.70, 0.86]);
        statusPanel = uipanel('Parent', fig, ...
                              'Title', '数据接收状态', ...
                              'Units', 'normalized', ...
                              'Position', [0.77, 0.08, 0.22, 0.86], ...
                              'BackgroundColor', [0.97, 0.97, 0.97]);
        statusText = uicontrol('Parent', statusPanel, ...
                               'Style', 'text', ...
                               'Units', 'normalized', ...
                               'Position', [0.04, 0.03, 0.92, 0.87], ...
                               'HorizontalAlignment', 'left', ...
                               'FontName', 'Microsoft YaHei UI', ...
                               'FontSize', 10, ...
                               'BackgroundColor', [0.97, 0.97, 0.97], ...
                               'String', '正在初始化...');
        uicontrol('Parent', statusPanel, ...
                  'Style', 'pushbutton', ...
                  'Units', 'normalized', ...
                  'Position', [0.58, 0.92, 0.38, 0.055], ...
                  'FontName', 'Microsoft YaHei UI', ...
                  'FontSize', 10, ...
                  'String', '刷新姿态 (R)', ...
                  'Callback', @onRefreshButton);
        drawRobot(zeros(1, 7), uint8(0));
        updateStatusPanel();
        setappdata(fig, 'ArmForceRefresh', @forceRefreshView);
        enable_arm_camera_controls(fig, ax, defaultView);
        set(fig, 'CloseRequestFcn', @onFigureClose);
    end

    timerObject = timer( ...
        'Name', 'ArmBoardLinkVisualizerTimer', ...
        'ExecutionMode', 'fixedSpacing', ...
        'Period', timerPeriod, ...
        'BusyMode', 'drop', ...
        'TimerFcn', @onTimerTick);

    start(timerObject);
catch startupError
    % 初始化中途失败也要释放已经创建的资源。
    cleanupResources();
    rethrow(startupError);
end

fprintf('[START] %s, %d baud, 8N1, timer=%.3f s, visualize=%d\n', ...
    portName, baudRate, timerPeriod, visualize);
fprintf('[INFO] 原始值和关节角显示在图窗右侧；按 V 键循环编号视图。\n');

% 返回轻量控制句柄。串口对象保留在闭包内，stop() 后会解除引用并释放 COM 口。
session.stop = @stopSession;
session.status = @getStatus;
session.setInversion = @setInversion;
session.getInversion = @getInversion;
session.refresh = @forceRefreshView;

% -------------------------------------------------------------------------
% 3. timer 回调：非阻塞读取、拆帧、调试打印和绘图
% -------------------------------------------------------------------------
    function onTimerTick(~, ~)
        if isStopping || isempty(serialDevice)
            return;
        end

        try
            currentElapsed = toc(sessionClock);
            availableByteCount = serialDevice.NumBytesAvailable;
            if availableByteCount > 0
                newBytes = read(serialDevice, availableByteCount, 'uint8');
            else
                newBytes = zeros(1, 0, 'uint8');
            end
            if availableByteCount <= 0
                if consoleDiagnostics && currentElapsed - lastConsoleReportElapsed >= 2.0
                    fprintf('[WAIT] %s 暂无新字节，累计=%d byte，有效帧=%d。\n', ...
                        portName, totalByteCount, validFrameCount);
                    lastConsoleReportElapsed = currentElapsed;
                end
                if currentElapsed - lastStatusUpdateElapsed >= 0.25
                    updateStatusPanel();
                    lastStatusUpdateElapsed = currentElapsed;
                end
                return;
            end

            totalByteCount = totalByteCount + numel(newBytes);
            readChunkCount = readChunkCount + 1;
            lastReceiveElapsed = currentElapsed;
            latestRawPreview = makeHexPreview(newBytes, 16);

            if consoleDiagnostics && printRawChunks && (readChunkCount == 1 || ...
                    currentElapsed - lastRawPrintElapsed >= 0.5)
                fprintf('[RAW] %s received %d byte(s), total=%d: %s\n', ...
                    portName, numel(newBytes), totalByteCount, ...
                    makeHexPreview(newBytes, 96));
                lastRawPrintElapsed = currentElapsed;
                lastConsoleReportElapsed = currentElapsed;
            end

            [samples, sequences, rawFrames, rxBuffer, parseStats] = ...
                parse_board_link_bytes(rxBuffer, newBytes);

            invalidFrameCount = invalidFrameCount + parseStats.invalidLength + ...
                parseStats.invalidTail + parseStats.nonFinitePayload;

            if consoleDiagnostics && (parseStats.invalidLength > 0 || ...
                    parseStats.invalidTail > 0 || parseStats.nonFinitePayload > 0)
                fprintf('[SKIP] len=%d, tail=%d, nonfinite=%d, discarded=%d byte(s)\n', ...
                    parseStats.invalidLength, parseStats.invalidTail, ...
                    parseStats.nonFinitePayload, parseStats.discardedBytes);
            end

            for frameIndex = 1:size(samples, 1)
                currentSequence = sequences(frameIndex);
                updateSequenceStatistics(currentSequence);
                validFrameCount = validFrameCount + 1;
                lastValidFrameElapsed = currentElapsed;
                latestJointAngles = samples(frameIndex, 1:7);
                latestToolValue = samples(frameIndex, 8);

                if consoleDiagnostics && ...
                        mod(validFrameCount - 1, printEveryNFrames) == 0
                    printFrame(rawFrames{frameIndex}, currentSequence, samples(frameIndex, :));
                end
            end

            % 一次回调可能收到多帧。只显示最新帧，避免绘图速度低于串口时累积延迟。
            if visualize && ~isempty(samples) && isgraphics(fig) && ...
                    currentElapsed - lastRenderElapsed >= renderPeriod && ...
                    (any(isnan(lastRenderedAngles)) || ...
                     any(abs(samples(end, 1:7) - lastRenderedAngles) > 1e-5))
                if drawRobot(samples(end, 1:7), sequences(end))
                    lastRenderElapsed = currentElapsed;
                    lastRenderedAngles = samples(end, 1:7);
                end
            end
            if currentElapsed - lastStatusUpdateElapsed >= 0.25
                updateStatusPanel();
                lastStatusUpdateElapsed = currentElapsed;
            end
        catch callbackError
            % 单次读取、解析或绘图异常只记录，不让 timer 因异常数据停止。
            callbackErrorCount = callbackErrorCount + 1;
            warning('arm_serial_visualizer:CallbackError', ...
                '本次 timer 回调已跳过：%s', callbackError.message);
        end
    end

% -------------------------------------------------------------------------
% 4. 调试输出和序号统计
% -------------------------------------------------------------------------
    function hexText = makeHexPreview(bytes, maximumByteCount)
        bytes = reshape(uint8(bytes), 1, []);
        shownByteCount = min(numel(bytes), maximumByteCount);
        hexText = strtrim(sprintf('%02X ', bytes(1:shownByteCount)));
        if numel(bytes) > shownByteCount
            hexText = sprintf('%s ... (+%d byte)', ...
                hexText, numel(bytes) - shownByteCount);
        end
    end

    function updateSequenceStatistics(currentSequence)
        if ~isempty(lastSequence)
            expectedSequence = mod(double(lastSequence) + 1, 256);
            gap = mod(double(currentSequence) - expectedSequence, 256);
            if gap > 0
                droppedFrameCount = droppedFrameCount + gap;
                if consoleDiagnostics
                    fprintf('[SEQ] expected=%d, received=%d, estimated lost=%d\n', ...
                        expectedSequence, double(currentSequence), gap);
                end
            end
        end
        lastSequence = currentSequence;
    end

    function updateStatusPanel()
        if ~visualize || isempty(statusText) || ~isgraphics(statusText)
            return;
        end

        currentElapsed = toc(sessionClock);
        if totalByteCount == 0
            receiveState = '未收到任何数据字节';
        elseif validFrameCount == 0
            receiveState = '已收到字节，但尚未解析出有效帧';
        else
            receiveState = '正在接收有效关节帧';
        end

        if isnan(lastReceiveElapsed)
            receiveAgeText = '--';
        else
            receiveAgeText = sprintf('%.2f s', currentElapsed - lastReceiveElapsed);
        end

        if isnan(lastValidFrameElapsed)
            validAgeText = '--';
        else
            validAgeText = sprintf('%.2f s', currentElapsed - lastValidFrameElapsed);
        end

        if isempty(lastSequence)
            sequenceText = '--';
        else
            sequenceText = sprintf('%d', double(lastSequence));
        end

        if any(isnan(latestJointAngles))
            jointRadText = '--';
            jointDegText = '--';
            toolText = '--';
        else
            jointRadText = strtrim(sprintf('% .3f ', latestJointAngles));
            jointDegText = strtrim(sprintf('% .1f ', rad2deg(latestJointAngles)));
            toolText = sprintf('%.4f', latestToolValue);
        end

        statusText.String = sprintf([ ...
            '数据源：%s\n\n' ...
            'V键：循环 01~10 编号视图\n' ...
            '数字键：直接选择对应视图\n' ...
            'H/Home键：回到固定主视图 05\n' ...
            'F键：当前方向重新完整取景\n' ...
            'R键：使用最新关节角强制刷新\n' ...
            '左键拖动：自由旋转\n' ...
            '中键拖动：仅绕基座水平旋转 | 滚轮：缩放\n\n' ...
            '状态：%s\n' ...
            '累计接收：%d byte\n' ...
            '读取批次：%d\n' ...
            '有效帧：%d\n' ...
            '无效帧：%d\n' ...
            '估计丢帧：%d\n' ...
            '缓存字节：%d\n' ...
            '最新 SEQ：%s\n' ...
            '距最近字节：%s\n' ...
            '距最近有效帧：%s\n\n' ...
            '最新绝对关节角 J1~J7 / rad\n%s\n\n' ...
            '最新绝对关节角 J1~J7 / deg\n%s\n\n' ...
            '方向取反标志 J1~J7（0保持，1取反）\n%s\n\n' ...
            'TOOL：%s\n\n' ...
            '最新原始字节（最多16B）\n%s'], ...
            portName, receiveState, totalByteCount, readChunkCount, ...
            validFrameCount, invalidFrameCount, droppedFrameCount, ...
            numel(rxBuffer), sequenceText, receiveAgeText, validAgeText, ...
            jointRadText, jointDegText, sprintf('%d ', inversionFlags), ...
            toolText, latestRawPreview);
    end

    function printFrame(rawFrame, sequence, values)
        hexText = strtrim(sprintf('%02X ', rawFrame));
        fprintf('RX HEX: %s\n', hexText);
        fprintf('SEQ=%3d  J(rad)=[% .4f % .4f % .4f % .4f % .4f % .4f % .4f]\n', ...
            double(sequence), values(1:7));
        fprintf('         J(deg)=[% .1f % .1f % .1f % .1f % .1f % .1f % .1f]  TOOL=% .4f\n', ...
            rad2deg(values(1:7)), values(8));
    end

% -------------------------------------------------------------------------
% 5. rigidBodyTree 实时刷新
% -------------------------------------------------------------------------
    function rendered = drawRobot(jointAnglesRad, ~)
        rendered = false;
        if ~isgraphics(fig) || getCameraBusy(fig)
            return;
        end

        % 保存用户当前相机状态，避免实时刷新把旋转和缩放操作重置掉。
        if cameraInitialized
            savedCameraPosition = ax.CameraPosition;
            savedCameraTarget = ax.CameraTarget;
            savedCameraUpVector = ax.CameraUpVector;
            savedCameraViewAngle = ax.CameraViewAngle;
        end

        % 单片机的全零定义是“主连杆垂直于基座平面”。CAD 导出的 URDF
        % 零位是弯曲装配姿态，因此只在模型接口处叠加显式零位偏置。
        modelConfiguration = map_arm_configuration(jointAnglesRad, inversionFlags);
        if modelBoundsInitialized
            % 首次 show 已创建所有网格；之后只更新刚体变换矩阵。
            show(robot, modelConfiguration, ...
                'Parent', ax, ...
                'PreservePlot', false, ...
                'FastUpdate', true, ...
                'Frames', 'off', ...
                'Visuals', onOff(liveVisuals), ...
                'Collisions', onOff(~liveVisuals));
        else
            % MATLAB 推荐首次完整绘制时关闭 FastUpdate，用于建立图形缓存。
            show(robot, modelConfiguration, ...
                'Parent', ax, ...
                'PreservePlot', false, ...
                'FastUpdate', false, ...
                'Frames', 'off', ...
                'Visuals', onOff(liveVisuals), ...
                'Collisions', onOff(~liveVisuals));
        end

        if ~modelBoundsInitialized
            % 只在第一次绘图后应用一次；随后 X/Y/ZLim 保持 manual。
            set_arm_workspace_limits(ax, workspaceRadius);
            modelBoundsInitialized = true;
        end

        if cameraInitialized
            ax.CameraPosition = savedCameraPosition;
            ax.CameraTarget = savedCameraTarget;
            ax.CameraUpVector = savedCameraUpVector;
            ax.CameraViewAngle = savedCameraViewAngle;
        else
            camtarget(ax, mean([ax.XLim; ax.YLim; ax.ZLim], 2)');
            cameraInitialized = true;
        end

        % 光源通常会被 FastUpdate 保留；若 show 重建图元，只在此处恢复固定光源。
        if isappdata(fig, 'ArmRefreshLighting')
            refreshLighting = getappdata(fig, 'ArmRefreshLighting');
            if isa(refreshLighting, 'function_handle')
                refreshLighting();
            end
        end

        drawnow limitrate;
        rendered = true;
    end

    function value = getCameraBusy(figureHandle)
        value = isappdata(figureHandle, 'ArmCameraInteracting') && ...
            logical(getappdata(figureHandle, 'ArmCameraInteracting'));
    end

    function textValue = onOff(value)
        textValue = ternary(value, 'on', 'off');
    end

    function result = ternary(condition, trueValue, falseValue)
        if condition
            result = trueValue;
        else
            result = falseValue;
        end
    end

% -------------------------------------------------------------------------
% 6. 安全关闭：关闭图窗或手动 stop() 都进入同一路径
% -------------------------------------------------------------------------
    function onFigureClose(~, ~)
        stopSession();
    end

    function onRefreshButton(~, ~)
        forceRefreshView();
    end

    function forceRefreshView()
        if isStopping || ~visualize || isempty(fig) || ~isgraphics(fig)
            return;
        end

        currentElapsed = toc(sessionClock);
        setappdata(fig, 'ArmCameraInteracting', false);
        fig.Pointer = 'arrow';
        lastRenderedAngles = nan(1, 7);

        if ~isempty(timerObject) && isvalid(timerObject) && ...
                strcmp(timerObject.Running, 'off')
            start(timerObject);
        end

        if ~any(isnan(latestJointAngles))
            if drawRobot(latestJointAngles, lastSequence)
                lastRenderElapsed = currentElapsed;
                lastRenderedAngles = latestJointAngles;
            end
        end
        updateStatusPanel();
        drawnow;
    end

    function stopSession()
        if isStopping
            return;
        end
        isStopping = true;
        cleanupResources();
        fprintf('[STOP] timer 已停止，串口 %s 已释放。\n', portName);
    end

    function cleanupResources()
        if ~isempty(timerObject) && isvalid(timerObject)
            if strcmp(timerObject.Running, 'on')
                stop(timerObject);
            end
            delete(timerObject);
        end
        timerObject = [];

        if ~isempty(serialDevice)
            try
                flush(serialDevice);
            catch
                % 串口可能已经被系统移除；清理阶段无需再次抛错。
            end
            serialDevice = []; % 删除最后一个引用后，MATLAB 会释放 COM 口。
        end

        if ~isempty(fig) && isgraphics(fig)
            if isappdata(fig, 'ArmForceRefresh')
                rmappdata(fig, 'ArmForceRefresh');
            end
            set(fig, 'CloseRequestFcn', '');
            delete(fig);
        end
        fig = [];
    end

    function status = getStatus()
        status.port = string(portName);
        status.running = ~isStopping && ~isempty(timerObject) && ...
            isvalid(timerObject) && strcmp(timerObject.Running, 'on');
        status.validFrames = validFrameCount;
        status.estimatedDroppedFrames = droppedFrameCount;
        status.invalidFrames = invalidFrameCount;
        status.callbackErrors = callbackErrorCount;
        status.totalBytes = totalByteCount;
        status.readChunks = readChunkCount;
        status.bufferedBytes = numel(rxBuffer);
        status.lastSequence = lastSequence;
        status.latestJointAnglesRad = latestJointAngles;
        status.latestToolValue = latestToolValue;
        status.latestRawHex = string(latestRawPreview);
        status.inversionFlags = inversionFlags;
        status.startTime = startTime;
    end

    function setInversion(newFlags)
        % 标志只影响显示映射，不修改右侧面板保存的单片机原始值。
        inversionFlags = validateInversionFlags(newFlags);
        lastRenderedAngles = nan(1, 7);

        % 已收到有效帧时立即应用；若此刻正在拖动相机，下一帧会自动重试。
        if visualize && isgraphics(fig) && ~any(isnan(latestJointAngles)) && ...
                ~getCameraBusy(fig)
            drawRobot(latestJointAngles, lastSequence);
            lastRenderElapsed = toc(sessionClock);
            lastRenderedAngles = latestJointAngles;
        end
        updateStatusPanel();
    end

    function flags = getInversion()
        flags = inversionFlags;
    end
end

function flags = validateInversionFlags(flags)
% 复用方向修正函数完成有限值、长度及 0/1 范围检查。
apply_arm_input_inversion(zeros(1, 7), flags);
flags = double(reshape(flags, 1, 7));
end
