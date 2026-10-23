function workspaceRadius = calculate_arm_workspace_radius(robot, urdfPath)
%CALCULATE_ARM_WORKSPACE_RADIUS 一次计算覆盖所有关节角的固定工作空间半径。
%
% 对任意串联关节角，某个网格点到基座的距离不会超过：
%   各级关节固定平移长度之和 + 该网格点到所属连杆坐标原点的距离。
% 因此无需枚举或采样关节角。程序读取 URDF visual origin、mesh scale 和 STL
% 顶点包围盒，求出所有视觉网格在连杆局部坐标中的最大半径，再加固定留白。

if nargin < 2 || strlength(string(urdfPath)) == 0
    urdfPath = default_arm_urdf_path();
end
urdfPath = string(urdfPath);

chainReach = 0;
for bodyIndex = 1:numel(robot.Bodies)
    transform = robot.Bodies{bodyIndex}.Joint.JointToParentTransform;
    chainReach = chainReach + norm(transform(1:3, 4));
end

% 若使用简化占位模型或某个 STL 无法读取，0.25 m 仍可覆盖电机、基座和末端。
visualRadius = 0.25;
if isfile(urdfPath)
    try
        visualRadius = max(visualRadius, readUrdfVisualRadius(urdfPath));
    catch visualBoundsError
        warning('calculate_arm_workspace_radius:VisualBoundsFallback', ...
            'URDF 网格边界读取失败，改用 0.25 m 保守余量：%s', ...
            visualBoundsError.message);
    end
end

% 8%% 显示留白加 0.03 m 边缘；最小半径避免占位模型贴边。
workspaceRadius = max(0.75, 1.08 * (chainReach + visualRadius + 0.03));
end

function maximumRadius = readUrdfVisualRadius(urdfPath)
document = xmlread(char(urdfPath));
visualNodes = document.getElementsByTagName('visual');
urdfFolder = fileparts(char(urdfPath));
meshBoundsCache = containers.Map('KeyType', 'char', 'ValueType', 'any');
maximumRadius = 0;

for visualIndex = 0:visualNodes.getLength - 1
    visualNode = visualNodes.item(visualIndex);
    meshNodes = visualNode.getElementsByTagName('mesh');
    if meshNodes.getLength == 0
        continue;
    end

    meshNode = meshNodes.item(0);
    meshFile = strtrim(char(meshNode.getAttribute('filename')));
    if isempty(meshFile) || startsWith(meshFile, 'package://')
        continue;
    end
    meshPath = fullfile(urdfFolder, strrep(meshFile, '/', filesep));
    if ~isfile(meshPath)
        continue;
    end

    scale = parseVectorAttribute(meshNode, 'scale', [1, 1, 1]);
    originNodes = visualNode.getElementsByTagName('origin');
    if originNodes.getLength > 0
        originNode = originNodes.item(0);
        xyz = parseVectorAttribute(originNode, 'xyz', [0, 0, 0]);
        rpy = parseVectorAttribute(originNode, 'rpy', [0, 0, 0]);
    else
        xyz = [0, 0, 0];
        rpy = [0, 0, 0];
    end

    cacheKey = char(meshPath);
    if isKey(meshBoundsCache, cacheKey)
        meshBounds = meshBoundsCache(cacheKey);
    else
        [minimumVertex, maximumVertex] = readBinaryStlBounds(meshPath);
        meshBounds = [minimumVertex; maximumVertex];
        meshBoundsCache(cacheKey) = meshBounds;
    end

    scaledEndpoints = meshBounds .* scale;
    minimumVertex = min(scaledEndpoints, [], 1);
    maximumVertex = max(scaledEndpoints, [], 1);
    corners = makeBoxCorners(minimumVertex, maximumVertex);
    rotation = rpyRotation(rpy);
    localCorners = (rotation * corners')' + xyz;
    maximumRadius = max(maximumRadius, ...
        max(sqrt(sum(localCorners .^ 2, 2))));
end
end

function value = parseVectorAttribute(node, attributeName, defaultValue)
attributeText = strtrim(char(node.getAttribute(attributeName)));
if isempty(attributeText)
    value = defaultValue;
    return;
end
value = sscanf(attributeText, '%f').';
if numel(value) ~= 3 || any(~isfinite(value))
    value = defaultValue;
end
end

function [minimumVertex, maximumVertex] = readBinaryStlBounds(meshPath)
fileId = fopen(meshPath, 'rb', 'ieee-le');
if fileId < 0
    error('calculate_arm_workspace_radius:MeshOpenFailed', ...
        '无法打开 STL：%s', meshPath);
end
cleanupObject = onCleanup(@() fclose(fileId));

fseek(fileId, 80, 'bof');
triangleCount = fread(fileId, 1, 'uint32=>double');
if isempty(triangleCount) || triangleCount < 1
    error('calculate_arm_workspace_radius:InvalidStl', ...
        'STL 不包含有效三角形：%s', meshPath);
end
fseek(fileId, 84, 'bof');
triangleData = fread(fileId, [12, triangleCount], ...
    '12*single=>double', 2, 'ieee-le');
if size(triangleData, 2) ~= triangleCount
    error('calculate_arm_workspace_radius:InvalidStl', ...
        'STL 数据长度不完整：%s', meshPath);
end

vertices = reshape(triangleData(4:12, :), 3, []).';
minimumVertex = min(vertices, [], 1);
maximumVertex = max(vertices, [], 1);
clear cleanupObject;
end

function corners = makeBoxCorners(minimumVertex, maximumVertex)
[x, y, z] = ndgrid( ...
    [minimumVertex(1), maximumVertex(1)], ...
    [minimumVertex(2), maximumVertex(2)], ...
    [minimumVertex(3), maximumVertex(3)]);
corners = [x(:), y(:), z(:)];
end

function rotation = rpyRotation(rpy)
roll = rpy(1);
pitch = rpy(2);
yaw = rpy(3);
rotationX = [1, 0, 0; 0, cos(roll), -sin(roll); 0, sin(roll), cos(roll)];
rotationY = [cos(pitch), 0, sin(pitch); 0, 1, 0; -sin(pitch), 0, cos(pitch)];
rotationZ = [cos(yaw), -sin(yaw), 0; sin(yaw), cos(yaw), 0; 0, 0, 1];
rotation = rotationZ * rotationY * rotationX;
end
