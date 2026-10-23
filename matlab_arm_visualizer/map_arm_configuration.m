function modelConfiguration = map_arm_configuration(jointAnglesRad, inversionFlags)
%MAP_ARM_CONFIGURATION 将固件绝对关节角映射到 CAD/URDF 关节坐标。
%
% 固件约定：整臂主连杆垂直于基座平面时，J1~J7 全部为 0 rad。
% URDF 则把 CAD 导出时的弯曲装配姿态定义成 q=0，因此两者不能直接相等。
% CAD_ALIGNMENT_RAD 由 URDF 逐级变换求得；ZERO_POSE_TRIM_RAD 是根据实物
% 网格显示手动标定的零位修正。两者相加后，外部输入全 0 就对应确认过的
% 目标零姿态。
%
% inversionFlags 按 J1~J7 排列：0 保持输入方向，1 将输入取反。方向修正
% 先于 URDF 零位偏置执行，所以修改方向标志不会改变全零输入对应的姿态。
% 映射只改变 URDF 的关节配置，不旋转整个机器人。这样会保留 CAD 基座平面
% 与连杆之间的真实相对方向；“竖直”以连杆垂直于基座平面为准。

if nargin < 2 || isempty(inversionFlags)
    inversionFlags = [1 1 1 1 1 0 0];
end

validateattributes(jointAnglesRad, {'numeric'}, ...
    {'real', 'finite', 'vector', 'numel', 7}, mfilename, 'jointAnglesRad');
jointAnglesRad = double(reshape(jointAnglesRad, 1, 7));
jointAnglesRad = apply_arm_input_inversion(jointAnglesRad, inversionFlags);

CAD_ALIGNMENT_RAD = [ ...
     0.000000, ... % J1：绕基座法向旋转，不影响竖直零位
    -1.128566, ... % J2
    -2.728680, ... % J3（等价于 3.554505 rad）
     0.410406, ... % J4
     0.208264, ... % J5
    -0.900452, ... % J6
     0.000000];    % J7：不改变关节中心位置，保持 CAD 末端零姿态

% 这组数值来自离线查看器中确认正确的姿态：
% viewer.setConfiguration([0, -0.15, -0.19, 2.7, -0.24, 0.9, -1.1])
% 将其作为零位修正后，viewer.setConfiguration(zeros(1, 7)) 显示同一姿态。
ZERO_POSE_TRIM_RAD = [0.00, -0.15, -0.19, 2.70, -0.24, 0.90, -1.10];
URDF_ZERO_OFFSET_RAD = CAD_ALIGNMENT_RAD + ZERO_POSE_TRIM_RAD;

% 方向开关只作用于外部输入；URDF 固定零位偏置不能随方向开关一起取反。
modelConfiguration = jointAnglesRad + URDF_ZERO_OFFSET_RAD;
end
