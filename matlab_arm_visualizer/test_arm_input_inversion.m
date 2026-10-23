function test_arm_input_inversion()
%TEST_ARM_INPUT_INVERSION 离线验证 7 轴方向开关和零姿态不变性。
%
% 本测试不打开串口、不创建图窗，可直接运行：
%   test_arm_input_inversion

angles = [0.10, -0.20, 0.30, -0.40, 0.50, -0.60, 0.70];
flags =  [0,     1,    0,     1,    0,     1,    0];
expected = [0.10, 0.20, 0.30, 0.40, 0.50, 0.60, 0.70];

corrected = apply_arm_input_inversion(angles, flags);
assert(max(abs(corrected - expected)) < 1e-12, ...
    '0/1 方向标志没有按关节正确应用。');

% 方向开关必须在固定零位偏置之前执行，因此全零输入映射结果应完全相同。
zeroWithoutInversion = map_arm_configuration(zeros(1, 7), zeros(1, 7));
zeroWithInversion = map_arm_configuration(zeros(1, 7), ones(1, 7));
assert(isequal(zeroWithoutInversion, zeroWithInversion), ...
    '方向取反不应改变已标定的全零姿态。');

didRejectInvalidFlag = false;
try
    apply_arm_input_inversion(angles, [0 0 0 2 0 0 0]);
catch inversionError
    didRejectInvalidFlag = strcmp(inversionError.identifier, ...
        'apply_arm_input_inversion:InvalidFlags');
end
assert(didRejectInvalidFlag, '非 0/1 标志应被明确拒绝。');

fprintf('[PASS] 关节输入方向测试通过：逐轴取反、零姿态和错误标志均已覆盖。\n');
end
