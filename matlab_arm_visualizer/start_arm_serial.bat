@echo off
setlocal
set "SCRIPT_DIR=%~dp0"
set "MATLAB_BIN="

if defined MATLAB_EXE if exist "%MATLAB_EXE%" set "MATLAB_BIN=%MATLAB_EXE%"
if not defined MATLAB_BIN if exist "D:\Matlab_itself\bin\matlab.exe" set "MATLAB_BIN=D:\Matlab_itself\bin\matlab.exe"
if not defined MATLAB_BIN for /f "delims=" %%I in ('where matlab.exe 2^>nul') do if not defined MATLAB_BIN set "MATLAB_BIN=%%I"

if not defined MATLAB_BIN (
    echo MATLAB was not found. Set MATLAB_EXE to the full path of matlab.exe.
    pause
    exit /b 1
)

start "" "%MATLAB_BIN%" -sd "%SCRIPT_DIR%" -r "run_arm_serial_visualizer"
