function correctedAnglesRad = apply_arm_input_inversion(jointAnglesRad, inversionFlags)
%APPLY_ARM_INPUT_INVERSION 按 7 个 0/1 标志修正关节输入方向。
%
%   corrected = apply_arm_input_inversion(angles, flags)
%
% angles 和 corrected 均为 J1~J7 的弧度值。flags 的对应关系为：
%   0：保持该关节输入的正负方向；
%   1：将该关节输入乘以 -1。
%
% 本函数只处理单片机/手动输入，不处理 URDF 零位偏置。因此输入为全零时，
% 无论 flags 如何设置，模型的已标定零姿态都不会改变。

validateattributes(jointAnglesRad, {'numeric'}, ...
    {'real', 'finite', 'vector', 'numel', 7}, mfilename, 'jointAnglesRad');
validateattributes(inversionFlags, {'numeric', 'logical'}, ...
    {'real', 'finite', 'vector', 'numel', 7}, mfilename, 'inversionFlags');

jointAnglesRad = double(reshape(jointAnglesRad, 1, 7));
inversionFlags = double(reshape(inversionFlags, 1, 7));
if any(inversionFlags ~= 0 & inversionFlags ~= 1)
    error('apply_arm_input_inversion:InvalidFlags', ...
        '方向标志必须是包含 7 个 0 或 1 的向量。');
end

% 0 -> +1，1 -> -1。
directionSigns = 1 - 2 * inversionFlags;
correctedAnglesRad = jointAnglesRad .* directionSigns;
end
