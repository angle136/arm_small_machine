function test_board_link_parser()
%TEST_BOARD_LINK_PARSER 离线验证二进制拆帧、粘包、残帧和错误恢复。
%
% 本测试不打开串口、不创建图窗，可直接运行：
%   test_board_link_parser

expectedValues = single([0.0, 0.1, -0.2, 0.3, -0.4, 0.5, 0.0, 0.0]);
expectedStatus = uint32(2);
payload = [typecast(expectedValues, 'uint8'), typecast(expectedStatus, 'uint8')];
validFrame = [uint8(hex2dec('AA')), uint8(36), uint8(7), payload, ...
              uint8(hex2dec('FF'))];
expectedFrame = uint8(hex2dec([ ...
    'AA'; '24'; '07'; ...
    '00'; '00'; '00'; '00'; ...
    'CD'; 'CC'; 'CC'; '3D'; ...
    'CD'; 'CC'; '4C'; 'BE'; ...
    '9A'; '99'; '99'; '3E'; ...
    'CD'; 'CC'; 'CC'; 'BE'; ...
    '00'; '00'; '00'; '3F'; ...
    '00'; '00'; '00'; '00'; ...
    '00'; '00'; '00'; '00'; ...
    '02'; '00'; '00'; '00'; ...
    'FF']))';
assert(isequal(validFrame, expectedFrame), ...
       '标准测试帧与 40 字节协议定义不一致。');

% -------------------------------------------------------------------------
% 测试 1：一帧被拆成两次接收，第一次不能误判，第二次应正确拼接。
% -------------------------------------------------------------------------
[samples1, seq1, ~, remain1, ~] = parse_board_link_bytes( ...
    uint8([]), validFrame(1:11));
assert(isempty(samples1), '残帧不应提前输出数据。');
assert(isempty(seq1), '残帧不应提前输出序号。');
assert(numel(remain1) == 11, '残帧应完整保留。');

[samples2, seq2, ~, remain2, stats2, status2] = parse_board_link_bytes( ...
    remain1, validFrame(12:end));
assert(size(samples2, 1) == 1, '完整帧应解析出一条记录。');
assert(seq2 == uint8(7), '帧序号解析错误。');
assert(max(abs(samples2(1, :) - double(expectedValues))) < 1e-6, ...
       'float32 载荷解析错误。');
assert(isempty(remain2), '完整帧解析后不应残留字节。');
assert(stats2.acceptedFrames == 1, '有效帧计数错误。');
assert(status2 == expectedStatus, 'uint32 状态位解析错误。');

% -------------------------------------------------------------------------
% 测试 2：乱码、错误长度、错误帧尾之后仍能重新同步到有效帧。
% -------------------------------------------------------------------------
badLengthFrame = [uint8(hex2dec('AA')), uint8(35), uint8(1), uint8(1:12)];
badTailFrame = validFrame;
badTailFrame(end) = uint8(0);
mixedBytes = [uint8([1, 2, 3]), badLengthFrame, badTailFrame, validFrame];

[samples3, seq3, raw3, remain3, stats3] = parse_board_link_bytes( ...
    uint8([]), mixedBytes);
assert(size(samples3, 1) == 1, '错误数据后应恢复并解析最后一帧。');
assert(seq3 == uint8(7), '重新同步后的序号错误。');
assert(numel(raw3{1}) == 40, 'arm 固件有效帧长度应为 40 字节。');
assert(isempty(remain3), '测试数据解析后不应残留字节。');
assert(stats3.invalidLength >= 1, '未检测到错误长度。');
assert(stats3.invalidTail >= 1, '未检测到错误帧尾。');

% -------------------------------------------------------------------------
% 测试 3：旧版 LEN=32 帧继续供 arm_imu/J-Link 使用，缺省状态为 0。
% -------------------------------------------------------------------------
legacyPayload = typecast(expectedValues, 'uint8');
legacyFrame = [uint8(hex2dec('AA')), uint8(32), uint8(8), legacyPayload, ...
               uint8(hex2dec('FF'))];
[samples4, seq4, raw4, remain4, stats4, status4] = parse_board_link_bytes( ...
    uint8([]), legacyFrame);
assert(size(samples4, 1) == 1 && seq4 == uint8(8), ...
       '旧版 LEN=32 帧兼容解析失败。');
assert(numel(raw4{1}) == 36 && isempty(remain4), ...
       '旧版完整帧长度或残留缓存错误。');
assert(stats4.acceptedFrames == 1 && status4 == uint32(0), ...
       '旧版帧应被接受并返回状态 0。');

fprintf('[PASS] board_link 解析器测试通过：新旧载荷、状态位及错误恢复均已覆盖。\n');
end
