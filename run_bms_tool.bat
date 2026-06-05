@echo off
set "PYTHON_EXE=%LOCALAPPDATA%\Programs\Python\Python313\python.exe"

if not exist "%PYTHON_EXE%" (
    echo Python was not found at:
    echo %PYTHON_EXE%
    echo.
    echo Install Python 3.13 or update this launcher with your python.exe path.
    pause
    exit /b 1
)

cd /d "%~dp0"
"%PYTHON_EXE%" run_bms_tool.py
