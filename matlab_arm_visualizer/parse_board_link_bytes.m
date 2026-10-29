function [samples, sequences, rawFrames, remainingBuffer, stats, statuses] = parse_board_link_bytes(oldBuffer, newBytes)
%PARSE_BOARD_LINK_BYTES 解析机械臂下位机的 board_link 二进制数据流。
%
% arm 固件协议（总长度 40 字节）：
%   0xAA | LEN=36 | SEQ | 8 个 little-endian float32 | uint32 状态位 | 0xFF
%
% 8 个 float32 中，前 7 个依次映射 J1~J7，单位已经是 rad，第 8 个是工具量。
% uint32 状态位中 bit0=保持当前姿态，bit1=完全跟随。
% 该函数只负责拆帧，不修改角度。
%
% 输入：
%   oldBuffer - 上一次未解析完的 uint8 字节。
%   newBytes  - 本次新收到的 uint8 字节。
%
% 输出：
%   samples         - N×8 double，每行是一帧的数据。
%   sequences       - 1×N uint8，对应帧序号。
%   rawFrames       - 1×N cell，每项保存一帧原始字节，便于调试打印。
%   remainingBuffer - 不完整尾帧，留给下一次调用继续解析。
%   stats           - 本次解析过程中的丢弃和错误统计。
%   statuses        - 1×N uint32，arm 状态位。

FRAME_HEADER = uint8(hex2dec('AA'));
FRAME_TAIL = uint8(hex2dec('FF'));
EXPECTED_PAYLOAD_LENGTH = 36;
FLOAT_PAYLOAD_LENGTH = 32;
VALUE_COUNT = 8;

% serialport.read() 的返回形状可能随调用方式变化，统一整理为行向量。
remainingBuffer = [reshape(uint8(oldBuffer), 1, []), ...
                   reshape(uint8(newBytes), 1, [])];

samples = zeros(0, VALUE_COUNT);
sequences = zeros(1, 0, 'uint8');
rawFrames = cell(1, 0);
statuses = zeros(1, 0, 'uint32');

stats.acceptedFrames = 0;
stats.discardedBytes = 0;
stats.invalidLength = 0;
stats.invalidTail = 0;
stats.nonFinitePayload = 0;

while true
    % 搜索下一个帧头。帧头之前的乱码直接丢弃，实现自动重新同步。
    headerIndex = find(remainingBuffer == FRAME_HEADER, 1, 'first');
    if isempty(headerIndex)
        stats.discardedBytes = stats.discardedBytes + numel(remainingBuffer);
        remainingBuffer = zeros(1, 0, 'uint8');
        break;
    end

    if headerIndex > 1
        stats.discardedBytes = stats.discardedBytes + headerIndex - 1;
        remainingBuffer(1:headerIndex - 1) = [];
    end

    % 至少要收到帧头、长度和序号，才能继续判断。
    if numel(remainingBuffer) < 3
        break;
    end

    payloadLength = double(remainingBuffer(2));
    if payloadLength ~= EXPECTED_PAYLOAD_LENGTH
        % 长度字段不正确时仅丢弃当前 0xAA，随后继续寻找下一个帧头。
        stats.invalidLength = stats.invalidLength + 1;
        stats.discardedBytes = stats.discardedBytes + 1;
        remainingBuffer(1) = [];
        continue;
    end

    frameLength = payloadLength + 4; % 帧头、长度、序号、载荷、帧尾
    if numel(remainingBuffer) < frameLength
        % 数据尚未收全，不判错，保留到下一次 timer 回调。
        break;
    end

    if remainingBuffer(frameLength) ~= FRAME_TAIL
        stats.invalidTail = stats.invalidTail + 1;
        stats.discardedBytes = stats.discardedBytes + 1;
        remainingBuffer(1) = [];
        continue;
    end

    rawFrame = remainingBuffer(1:frameLength);
    floatPayload = rawFrame(4:3 + FLOAT_PAYLOAD_LENGTH);
    values = typecast(floatPayload, 'single');
    status = typecast(rawFrame(4 + FLOAT_PAYLOAD_LENGTH:3 + payloadLength), 'uint32');

    % STM32 和常见 Windows PC 都是小端。保留大端主机兼容处理，避免隐式假设。
    [~, ~, endian] = computer;
    if endian == 'B'
        values = swapbytes(values);
        status = swapbytes(status);
    end
    values = double(values);

    if all(isfinite(values))
        samples(end + 1, :) = values; %#ok<AGROW>
        sequences(end + 1) = rawFrame(3); %#ok<AGROW>
        rawFrames{end + 1} = rawFrame; %#ok<AGROW>
        statuses(end + 1) = status; %#ok<AGROW>
        stats.acceptedFrames = stats.acceptedFrames + 1;
    else
        % NaN/Inf 对可视化没有意义，但整帧边界仍然可信，因此整帧跳过。
        stats.nonFinitePayload = stats.nonFinitePayload + 1;
    end

    remainingBuffer(1:frameLength) = [];
end
end
