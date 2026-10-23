function robot = build_placeholder_arm(urdfPath)
%BUILD_PLACEHOLDER_ARM 导入实物 URDF；找不到时才退回简化占位模型。
%
% 默认实物模型随仓库保存在 model/robot_dynamics.urdf；路径根据本文件位置
% 自动解析，不依赖用户名、当前工作目录或仓库克隆位置。
% 该 URDF 包含 8 个 link（基座 + 7 个旋转关节）和完整 meshes 目录。
% importrobot 会按照 URDF 中的 parent/child、origin 和 axis 建立真实树，
% 因此这里不再根据照片或重力补偿力臂猜测几何结构。
%
% 可传入其他 URDF：
%   robot = build_placeholder_arm("D:\\model\\my_arm.urdf");
%
% 【后续替换为自己的 URDF 时，确保 meshes 目录仍位于 URDF 相对路径下】

if nargin < 1 || strlength(string(urdfPath)) == 0
    urdfPath = default_arm_urdf_path();
end
urdfPath = string(urdfPath);

if isfile(urdfPath)
    % importrobot 按 URDF 文件中的相对路径寻找 meshes；不要把网格文件复制到
    % MATLAB 工程，保留原始模型目录可避免路径和单位被悄悄改变。
    try
        robot = importrobot(urdfPath);
    catch importError
        % 部分 CAD 导出的惯性张量存在微小数值误差，MATLAB 会因其不是
        % 半正定矩阵而拒绝整个 URDF。当前用途是实时姿态显示，不依赖惯性；
        % 只对这个明确错误移除 inertial 节点，其他导入错误继续抛出。
        inertiaError = contains(importError.message, ...
            'positive semi-definite inertia matrix', 'IgnoreCase', true) || ...
            contains(importError.identifier, 'InertiaMatrixNotPSD', 'IgnoreCase', true);
        if ~inertiaError
            rethrow(importError);
        end
        warning('build_placeholder_arm:InvalidUrdfInertia', ...
            ['URDF 惯性矩阵不是半正定，已临时忽略惯性数据并保留关节/网格。' ...
             '原始 URDF 未修改。']);
        robot = importUrdfWithoutInertial(urdfPath);
    end
    robot.DataFormat = 'row';
    robot.Gravity = [0, 0, -9.81];

    jointCount = numel(homeConfiguration(robot));
    % rigidBodyTree 的 NumBodies 不计 Base；8 个 URDF link 会对应 7 个 Body。
    if robot.NumBodies ~= 7 || jointCount ~= 7
        warning('build_placeholder_arm:UnexpectedModelSize', ...
            'URDF 已导入，但非基座 Body 数=%d、关节数=%d；请核对是否为基座+7轴模型。', ...
            robot.NumBodies, jointCount);
    end
    return;
end

warning('build_placeholder_arm:UrdfNotFound', ...
    '未找到 URDF：%s，将使用下面的简化占位模型。', urdfPath);

% -------------------------------------------------------------------------
% 以下是没有 URDF 时的保底模型。它只用于检查串口程序和 Robotics Toolbox
% 是否可运行，不能代表实物尺寸；一旦内置 URDF 可用，上面的 importrobot
% 分支会直接返回，不会执行这里的猜测模型。
% -------------------------------------------------------------------------

robot = rigidBodyTree('DataFormat', 'row', 'MaxNumBodies', 9);

% -------------------------------------------------------------------------
% 1. 静态基座
% -------------------------------------------------------------------------
pedestal = rigidBody('pedestal');
pedestalJoint = rigidBodyJoint('pedestal_fixed', 'fixed');
setFixedTransform(pedestalJoint, eye(4));
pedestal.Joint = pedestalJoint;

pedestalShape = collisionCylinder(0.075, 0.08);
addCollision(pedestal, pedestalShape, trvec2tform([0, 0, 0.04]));
addBody(robot, pedestal, robot.BaseName);

% -------------------------------------------------------------------------
% 2. 七个旋转关节
% -------------------------------------------------------------------------
% 用户描述的是“能够运动的方向/平面”，不是电机转轴方向：
%   J1  水平运动 -> 竖直 Z 轴
%   J2  垂直运动 -> 水平 Y 轴
%   J3  垂直运动 -> 水平 Y 轴
%   J4  水平运动 -> 竖直 Z 轴
%   J5  垂直运动 -> 水平 Y 轴
%   J6  水平运动 -> 竖直 Z 轴
%   J7  垂直运动 -> 水平 Y 轴
% 对于“垂直运动”的关节，仅凭水平/垂直还不能区分 X 轴或 Y 轴；本占位
% 模型暂统一取 Y 轴，后续根据实机朝向或 URDF 再确定其水平面内的方位。
jointAxes = [0, 0, 1; ...
             0, 1, 0; ...
             0, 1, 0; ...
             0, 0, 1; ...
             0, 1, 0; ...
             0, 0, 1; ...
             0, 1, 0];

% 当前关节原点相对上一关节的 Z 向距离，仅作占位。
jointOffsets = [0.08, 0.12, 0.20, 0.12, 0.04, 0.08, 0.06];

% 每个关节之后显示的连杆长度。前三个主要尺度来自固件重力模型。
segmentLengths = [0.12, 0.20, 0.12, 0.04, 0.08, 0.06, 0.10];
segmentRadii = [0.045, 0.040, 0.036, 0.032, 0.030, 0.026, 0.022];

% 不把固件限位写入 rigidBodyJoint.PositionLimits：当前固件全局关闭了软件
% 限位，而且 J3 的试验限位不包含机械零位 0。强行写入会让 MATLAB 自动改写
% HomePosition 并产生警告，反而破坏“整臂竖直为 q=0”的显示约定。

parentName = pedestal.Name;
for jointIndex = 1:7
    bodyName = sprintf('link%d', jointIndex);
    jointName = sprintf('J%d', jointIndex);

    body = rigidBody(bodyName);
    joint = rigidBodyJoint(jointName, 'revolute');
    joint.JointAxis = jointAxes(jointIndex, :);
    setFixedTransform(joint, trvec2tform([0, 0, jointOffsets(jointIndex)]));
    body.Joint = joint;

    % 用碰撞几何体画出简单连杆，保证没有 STL/DAE 文件时模型仍清晰可见。
    linkShape = collisionCylinder(segmentRadii(jointIndex), ...
                                  segmentLengths(jointIndex));
    linkPose = trvec2tform([0, 0, segmentLengths(jointIndex) / 2]);
    addCollision(body, linkShape, linkPose);

    jointShape = collisionSphere(segmentRadii(jointIndex) * 1.25);
    addCollision(body, jointShape, eye(4));

    addBody(robot, body, parentName);
    parentName = bodyName;
end

% -------------------------------------------------------------------------
% 3. 固定工具端，仅用于更直观地观察 J7 姿态
% -------------------------------------------------------------------------
tool = rigidBody('tool');
toolJoint = rigidBodyJoint('tool_fixed', 'fixed');
setFixedTransform(toolJoint, trvec2tform([0, 0, segmentLengths(end)]));
tool.Joint = toolJoint;

toolShape = collisionBox(0.10, 0.035, 0.025);
addCollision(tool, toolShape, trvec2tform([0.05, 0, 0]));
addBody(robot, tool, parentName);
end

function robot = importUrdfWithoutInertial(urdfPath)
%IMPORTURDFWITHOUTINERTIAL 用同目录临时 XML 保留相对 meshes 路径。
% 临时文件只在 importrobot 调用期间存在，原始 URDF 不会被覆盖。

sourcePath = char(urdfPath);
sourceFolder = fileparts(sourcePath);
temporaryPath = [tempname(sourceFolder), '.urdf'];
cleanupObject = onCleanup(@() deleteTemporaryFile(temporaryPath));

document = xmlread(sourcePath);
inertialNodes = document.getElementsByTagName('inertial');
for nodeIndex = inertialNodes.getLength - 1:-1:0
    node = inertialNodes.item(nodeIndex);
    node.getParentNode.removeChild(node);
end
xmlwrite(temporaryPath, document);

% 临时 URDF 与原始文件同目录，因此 meshes/ 相对引用保持有效。
robot = importrobot(temporaryPath);
clear cleanupObject;
end

function deleteTemporaryFile(filePath)
if isfile(filePath)
    try
        delete(filePath);
    catch
        % 清理失败不应遮盖已经完成的模型导入。
    end
end
end
