function viewer = show_placeholder_arm(urdfPath, initialConfiguration, varargin)
%SHOW_PLACEHOLDER_ARM 离线显示并手动更新 7 轴 URDF 模型，不打开串口。
%
% 运行方式：
%   show_placeholder_arm
%   viewer = show_placeholder_arm("C:\\path\\robot.urdf", q0)
%   viewer = show_placeholder_arm("", [], 'DefaultView', 5)
%   viewer.setConfiguration([q1 q2 q3 q4 q5 q6 q7])
%   viewer.setInversion([1 1 1 1 1 0 0])
%
% 返回的 viewer 句柄用于不接串口时测试关节角；实时串口模式由
% arm_serial_visualizer 内部的 timer 自动调用同样的 show(robot, q) 更新。

if nargin < 1
    urdfPath = "";
end
if nargin < 2 || isempty(initialConfiguration)
    initialConfiguration = zeros(1, 7);
end
initialConfiguration = validateConfiguration(initialConfiguration);

parser = inputParser;
addParameter(parser, 'DefaultView', 5, ...
    @(x) isnumeric(x) && isscalar(x) && isfinite(x) && ...
         x >= 1 && x <= 10 && fix(x) == x);
addParameter(parser, 'InversionFlags', [1 1 1 1 1 0 0], ...
    @(x) (isnumeric(x) || islogical(x)) && isvector(x) && numel(x) == 7);
parse(parser, varargin{:});
defaultView = double(parser.Results.DefaultView);
inversionFlags = validateInversionFlags(parser.Results.InversionFlags);
robot = build_placeholder_arm(urdfPath);
workspaceRadius = calculate_arm_workspace_radius(robot, urdfPath);

fig = figure('Name', '7轴机械臂 URDF 实物模型', ...
             'NumberTitle', 'off', ...
             'Color', [0.97, 0.97, 0.97], ...
             'WindowState', 'maximized');
ax = axes('Parent', fig);

modelConfiguration = map_arm_configuration(initialConfiguration, inversionFlags);
show(robot, modelConfiguration, ...
     'Parent', ax, ...
     'PreservePlot', false, ...
     'FastUpdate', false, ...
     'Frames', 'off', ...
     'Visuals', 'on', ...
     'Collisions', 'off');

set_arm_workspace_limits(ax, workspaceRadius);
camtarget(ax, mean([ax.XLim; ax.YLim; ax.ZLim], 2)');

% 左键自由旋转，中键仅绕基座竖直轴水平旋转，滚轮缩放。
enable_arm_camera_controls(fig, ax, defaultView);
currentConfiguration = initialConfiguration;
viewer.robot = robot;
viewer.figure = fig;
viewer.axes = ax;
viewer.setConfiguration = @setConfiguration;
viewer.getConfiguration = @getConfiguration;
viewer.setInversion = @setInversion;
viewer.getInversion = @getInversion;
viewer.close = @closeViewer;

    function setConfiguration(configuration)
        if ~isgraphics(fig)
            error('show_placeholder_arm:FigureClosed', '模型图窗已经关闭。');
        end
        configuration = validateConfiguration(configuration);

        % 更新姿态时保留用户已经旋转/缩放后的相机状态。
        savedCameraPosition = ax.CameraPosition;
        savedCameraTarget = ax.CameraTarget;
        savedCameraUpVector = ax.CameraUpVector;
        savedCameraViewAngle = ax.CameraViewAngle;

        modelConfiguration = map_arm_configuration(configuration, inversionFlags);
        show(robot, modelConfiguration, ...
            'Parent', ax, ...
            'PreservePlot', false, ...
            'FastUpdate', true, ...
            'Frames', 'off', ...
            'Visuals', 'on', ...
            'Collisions', 'off');
        ax.CameraPosition = savedCameraPosition;
        ax.CameraTarget = savedCameraTarget;
        ax.CameraUpVector = savedCameraUpVector;
        ax.CameraViewAngle = savedCameraViewAngle;
        if isappdata(fig, 'ArmRefreshLighting')
            refreshLighting = getappdata(fig, 'ArmRefreshLighting');
            if isa(refreshLighting, 'function_handle')
                refreshLighting();
            end
        end
        currentConfiguration = configuration;
        drawnow limitrate nocallbacks;
    end

    function configuration = getConfiguration()
        configuration = currentConfiguration;
    end

    function setInversion(newFlags)
        % 运行时修改后立即用当前原始关节值重绘，便于逐轴校准方向。
        inversionFlags = validateInversionFlags(newFlags);
        setConfiguration(currentConfiguration);
    end

    function flags = getInversion()
        flags = inversionFlags;
    end

    function closeViewer()
        if isgraphics(fig)
            delete(fig);
        end
    end
end

function flags = validateInversionFlags(flags)
% 复用实际方向修正函数的完整检查，保证入口参数规则一致。
apply_arm_input_inversion(zeros(1, 7), flags);
flags = double(reshape(flags, 1, 7));
end

function configuration = validateConfiguration(configuration)
configuration = double(reshape(configuration, 1, []));
if numel(configuration) ~= 7 || any(~isfinite(configuration))
    error('show_placeholder_arm:InvalidConfiguration', ...
        '关节配置必须是包含 7 个有限弧度值的向量。');
end
end
