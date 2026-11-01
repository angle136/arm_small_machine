function source = arm_jlink_byte_source(varargin)
%ARM_JLINK_BYTE_SOURCE 通过 JLinkRTTLogger 非阻塞读取二进制 RTT 通道。
%
% 本函数只建立调试探针读取连接，不复位、暂停或烧写目标芯片。返回值可作为
% arm_serial_visualizer 的 ByteSource。关闭 source 时会结束本次 Logger 并删除临时文件。

if ~ispc
    error('arm_jlink_byte_source:WindowsOnly', ...
        '当前 J-Link RTT 自动入口仅支持 Windows。');
end

parser = inputParser;
addParameter(parser, 'Device', 'STM32F407IG', ...
    @(x) ischar(x) || (isstring(x) && isscalar(x)));
addParameter(parser, 'Interface', 'SWD', ...
    @(x) ischar(x) || (isstring(x) && isscalar(x)));
addParameter(parser, 'SpeedKHz', 4000, ...
    @(x) isnumeric(x) && isscalar(x) && isfinite(x) && x > 0);
addParameter(parser, 'Channel', 1, ...
    @(x) isnumeric(x) && isscalar(x) && isfinite(x) && x >= 0 && fix(x) == x);
addParameter(parser, 'StartupTimeout', 10, ...
    @(x) isnumeric(x) && isscalar(x) && isfinite(x) && x > 0);
addParameter(parser, 'MaximumReadBytes', 65536, ...
    @(x) isnumeric(x) && isscalar(x) && isfinite(x) && x >= 40 && fix(x) == x);
addParameter(parser, 'AutoReleaseProbe', true, ...
    @(x) islogical(x) && isscalar(x));
parse(parser, varargin{:});

settings = parser.Results;
settings.Device = char(string(settings.Device));
settings.Interface = char(string(settings.Interface));
settings.SpeedKHz = double(settings.SpeedKHz);
settings.Channel = double(settings.Channel);
settings.StartupTimeout = double(settings.StartupTimeout);
settings.MaximumReadBytes = double(settings.MaximumReadBytes);

if settings.AutoReleaseProbe
    releaseJLinkClients();
else
    assertJLinkProbeIsFree();
end

loggerExe = findJLinkRttLogger();
logPath = [tempname, '.bin'];
logStream = -1;
loggerProcess = [];
closed = false;

try
    arguments = sprintf('-Device %s -If %s -Speed %d -RTTChannel %d "%s"', ...
        settings.Device, settings.Interface, settings.SpeedKHz, ...
        settings.Channel, logPath);
    startInfo = System.Diagnostics.ProcessStartInfo();
    startInfo.FileName = loggerExe;
    startInfo.Arguments = arguments;
    startInfo.UseShellExecute = false;
    startInfo.CreateNoWindow = true;
    % 不重定向 stdout/stderr，避免未消费的管道填满后 Logger 停止读取 RTT。
    startInfo.RedirectStandardOutput = false;
    startInfo.RedirectStandardError = false;
    loggerProcess = System.Diagnostics.Process.Start(startInfo);

    waitClock = tic;
    openMessage = 'RTT 输出文件尚未创建。';
    while toc(waitClock) < settings.StartupTimeout
        if loggerProcess.HasExited
            error('arm_jlink_byte_source:LoggerExited', ...
                'JLinkRTTLogger 在连接前退出，请检查 J-Link、板子供电和 RTT Channel %d。', ...
                settings.Channel);
        end
        if isfile(logPath)
            candidateStream = fopen(logPath, 'rb');
            if candidateStream >= 0
                logStream = candidateStream;
                break;
            end
            openMessage = 'MATLAB 无法打开 RTT 二进制输出文件。';
        end
        pause(0.05);
    end
    if logStream < 0
        error('arm_jlink_byte_source:OpenTimeout', ...
            '%.0f 秒内未能打开 RTT Channel %d：%s', ...
            settings.StartupTimeout, settings.Channel, openMessage);
    end
catch startupError
    closeSource();
    rethrow(startupError);
end

source.description = sprintf('J-Link RTT Channel %d', settings.Channel);
source.readAvailable = @readAvailableBytes;
source.close = @closeSource;
source.status = @getSourceStatus;

    function bytes = readAvailableBytes()
        bytes = zeros(1, 0, 'uint8');
        if closed || logStream < 0
            return;
        end
        if ~isempty(loggerProcess) && loggerProcess.HasExited
            error('arm_jlink_byte_source:LoggerStopped', ...
                'JLinkRTTLogger 已停止，请检查 J-Link 连接和板子供电。');
        end

        % 使用 MATLAB 原生文件 API。不要改回 .NET FileStream.Read；R2026a 下曾
        % 出现访问冲突。只读取 Logger 已经落盘的新增字节，不阻塞 timer。
        currentPosition = ftell(logStream);
        fseek(logStream, 0, 'eof');
        fileLength = ftell(logStream);
        if currentPosition < 0 || fileLength < 0
            return;
        end
        if currentPosition > fileLength
            currentPosition = 0;
        end
        availableByteCount = fileLength - currentPosition;
        if availableByteCount <= 0
            return;
        end

        fseek(logStream, currentPosition, 'bof');
        readLength = min(settings.MaximumReadBytes, floor(availableByteCount));
        bytes = reshape(fread(logStream, readLength, '*uint8'), 1, []);
    end

    function status = getSourceStatus()
        status.closed = closed;
        status.channel = settings.Channel;
        status.logPath = string(logPath);
        status.loggerRunning = ~closed && ~isempty(loggerProcess) && ...
            ~loggerProcess.HasExited;
    end

    function closeSource()
        if closed
            return;
        end
        closed = true;

        if logStream >= 0
            try
                fclose(logStream);
            catch
            end
            logStream = -1;
        end
        if ~isempty(loggerProcess)
            try
                if ~loggerProcess.HasExited
                    loggerProcess.Kill();
                    loggerProcess.WaitForExit(2000);
                end
                loggerProcess.Dispose();
            catch
            end
            loggerProcess = [];
        end
        if isfile(logPath)
            try
                delete(logPath);
            catch
            end
        end
    end
end

function loggerExe = findJLinkRttLogger()
candidates = {
    getenv('JLINK_RTT_LOGGER'), ...
    'C:\Code for environment\JLink_V798d\JLinkRTTLogger.exe'
};
for index = 1:numel(candidates)
    if ~isempty(candidates{index}) && isfile(candidates{index})
        loggerExe = candidates{index};
        return;
    end
end

[status, output] = system('where JLinkRTTLogger.exe');
if status == 0
    paths = splitlines(strtrim(output));
    if ~isempty(paths) && isfile(paths{1})
        loggerExe = char(paths{1});
        return;
    end
end
error('arm_jlink_byte_source:LoggerNotFound', ...
    ['找不到 JLinkRTTLogger.exe。请安装 SEGGER J-Link Software Pack，' ...
     '或设置环境变量 JLINK_RTT_LOGGER。']);
end

function releaseJLinkClients()
[busyPids, busyNames] = findJLinkClientProcesses();
if isempty(busyPids)
    return;
end

fprintf(1, '[J-LINK] 正在释放占用：%s\n', ...
    strjoin(unique(busyNames, 'stable'), ', '));
for index = 1:numel(busyPids)
    command = sprintf('taskkill /PID %d /T /F >NUL 2>&1', busyPids(index));
    system(command);
end
pause(0.5);
assertJLinkProbeIsFree();
end

function assertJLinkProbeIsFree()
[~, busyNames] = findJLinkClientProcesses();
if ~isempty(busyNames)
    error('arm_jlink_byte_source:ProbeBusy', ...
        '无法自动结束 J-Link 占用进程：%s。请以管理员权限关闭后重试。', ...
        strjoin(unique(busyNames, 'stable'), ', '));
end
end

function [busyPids, busyNames] = findJLinkClientProcesses()
% 直接枚举 PID，不依赖 tasklist 的语言、列宽或窗口可见性。
processNames = {'Ozone', 'JLink', 'JLinkRTTViewer', 'JLinkGUIServer', ...
    'JLinkGDBServer', 'JLinkGDBServerCL', 'JLinkRTTLogger', ...
    'JLinkRemoteServer', 'JLinkRemoteServerCL', 'openocd'};
busyPids = [];
busyNames = {};
for index = 1:numel(processNames)
    processes = System.Diagnostics.Process.GetProcessesByName(processNames{index});
    for processIndex = 1:numel(processes)
        try
            if ~processes(processIndex).HasExited
                busyPids(end + 1) = processes(processIndex).Id; %#ok<AGROW>
                busyNames{end + 1} = processNames{index}; %#ok<AGROW>
            end
        catch
            % 进程可能在枚举期间自行退出。
        end
    end
end
end
