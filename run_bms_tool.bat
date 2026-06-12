@echo off
setlocal
set "PYTHON_EXE=%LOCALAPPDATA%\Programs\Python\Python313\python.exe"

cd /d "%~dp0"

set "LOG_DIR=%~dp0logs"
if not exist "%LOG_DIR%" mkdir "%LOG_DIR%"
for /f "delims=" %%I in ('powershell -NoProfile -Command "Get-Date -Format yyyyMMdd_HHmmss"') do set "RUN_ID=%%I"
if not defined RUN_ID set "RUN_ID=%RANDOM%"
set "BMS_TOOL_LOG_FILE=%LOG_DIR%\bms_tool_%RUN_ID%.log"

echo [%DATE% %TIME%] Launcher started. > "%BMS_TOOL_LOG_FILE%"
echo Working directory: %CD% >> "%BMS_TOOL_LOG_FILE%"
echo Python: %PYTHON_EXE% >> "%BMS_TOOL_LOG_FILE%"
echo Log file: %BMS_TOOL_LOG_FILE%

if not exist "%PYTHON_EXE%" (
    echo ERROR: Python was not found at %PYTHON_EXE% >> "%BMS_TOOL_LOG_FILE%"
    echo Python was not found at:
    echo %PYTHON_EXE%
    echo.
    echo Install Python 3.13 or update this launcher with your python.exe path.
    echo.
    echo See log file:
    echo %BMS_TOOL_LOG_FILE%
    pause
    exit /b 1
)

"%PYTHON_EXE%" run_bms_tool.py >> "%BMS_TOOL_LOG_FILE%" 2>&1
set "EXIT_CODE=%ERRORLEVEL%"
echo [%DATE% %TIME%] Launcher exited with code %EXIT_CODE%. >> "%BMS_TOOL_LOG_FILE%"

if not "%EXIT_CODE%"=="0" (
    echo.
    echo BMS Tool exited with code %EXIT_CODE%.
    echo See log file:
    echo %BMS_TOOL_LOG_FILE%
    pause
)

exit /b %EXIT_CODE%
