function urdfPath = default_arm_urdf_path()
%DEFAULT_ARM_URDF_PATH 返回仓库内置机械臂模型的绝对路径。
%
% 路径始终根据本函数文件所在目录计算，与 Windows 用户名、仓库克隆位置、
% MATLAB 当前工作目录均无关。这样整个 matlab_arm_visualizer 目录可以直接
% 提交到 Git，并在另一台电脑克隆后使用。

repositoryFolder = fileparts(mfilename('fullpath'));
urdfPath = string(fullfile(repositoryFolder, 'model', 'robot_dynamics.urdf'));

if ~isfile(urdfPath)
    error('default_arm_urdf_path:ModelMissing', ...
        ['未找到仓库内置模型：%s\n' ...
         '请确认 model/robot_dynamics.urdf 和 model/meshes 已完整提交。'], ...
        urdfPath);
end
end
