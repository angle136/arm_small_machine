function portName = select_serial_port()
%SELECT_SERIAL_PORT 列出当前可用串口，并让用户选择机械臂连接端口。
%
% 返回：
%   portName - 例如 "COM5"；取消选择时返回空字符串 ""。

availablePorts = serialportlist('available');
portName = "";

if isempty(availablePorts)
    warning('select_serial_port:NoAvailablePort', ...
        ['没有检测到可用串口。请检查 USB 转串口模块、驱动和接线，' ...
         '并确认该 COM 口未被其他程序占用。']);
    return;
end

availablePorts = string(availablePorts);

% COM15 是用户常用端口，优先放在列表第一项并作为默认选择；如果当前
% 不存在 COM15，则保留系统返回顺序，并默认选中第一项。
preferredPort = "COM15";
preferredIndex = find(strcmpi(availablePorts, preferredPort), 1, 'first');
if ~isempty(preferredIndex) && preferredIndex ~= 1
    availablePorts([1, preferredIndex]) = availablePorts([preferredIndex, 1]);
end

if usejava('desktop')
    [selectedIndex, confirmed] = listdlg( ...
        'Name', '选择机械臂串口', ...
        'PromptString', '请选择连接 USART6 的 USB 转串口端口：', ...
        'SelectionMode', 'single', ...
        'ListString', cellstr(availablePorts), ...
        'InitialValue', 1, ...
        'ListSize', [360, 180]);

    if confirmed && ~isempty(selectedIndex)
        portName = availablePorts(selectedIndex);
    end
else
    % 无桌面环境时退化为命令行选择。
    fprintf('可用串口：\n');
    for index = 1:numel(availablePorts)
        fprintf('  %d. %s\n', index, availablePorts(index));
    end

    selectedIndex = input('请输入串口序号，直接回车取消： ');
    if isnumeric(selectedIndex) && isscalar(selectedIndex) && ...
            selectedIndex >= 1 && selectedIndex <= numel(availablePorts) && ...
            fix(selectedIndex) == selectedIndex
        portName = availablePorts(selectedIndex);
    end
end
end
