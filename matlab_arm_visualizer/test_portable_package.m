function test_portable_package()
%TEST_PORTABLE_PACKAGE 验证仓库内置 URDF 和网格文件是否完整、可迁移。
%
% 本测试不打开串口、不创建图窗。将仓库克隆到任意位置后可直接运行：
%   test_portable_package

urdfPath = default_arm_urdf_path();
modelFolder = fileparts(char(urdfPath));
document = xmlread(char(urdfPath));
meshNodes = document.getElementsByTagName('mesh');

assert(meshNodes.getLength > 0, '内置 URDF 没有任何 mesh 引用。');

missingFiles = strings(0, 1);
absoluteReferences = strings(0, 1);
for meshIndex = 0:meshNodes.getLength - 1
    meshReference = string(strtrim(char( ...
        meshNodes.item(meshIndex).getAttribute('filename'))));
    if strlength(meshReference) == 0
        continue;
    end

    % 内置模型只能使用相对路径，不能携带原电脑盘符或 package:// 环境依赖。
    isWindowsAbsolute = ~isempty(regexp(meshReference, '^[A-Za-z]:[\\/]', 'once'));
    isUnixAbsolute = startsWith(meshReference, '/');
    isPackageReference = startsWith(meshReference, 'package://');
    if isWindowsAbsolute || isUnixAbsolute || isPackageReference
        absoluteReferences(end + 1, 1) = meshReference; %#ok<AGROW>
        continue;
    end

    relativePath = strrep(char(meshReference), '/', filesep);
    if ~isfile(fullfile(modelFolder, relativePath))
        missingFiles(end + 1, 1) = meshReference; %#ok<AGROW>
    end
end

assert(isempty(absoluteReferences), ...
    'URDF 含不可迁移的绝对/package mesh 路径：%s', ...
    strjoin(unique(absoluteReferences), ', '));
assert(isempty(missingFiles), ...
    '仓库缺少 URDF 引用的网格：%s', strjoin(unique(missingFiles), ', '));

fprintf('[PASS] 便携包完整：URDF 和 %d 个 mesh 引用均可从仓库相对路径解析。\n', ...
    meshNodes.getLength);
end
