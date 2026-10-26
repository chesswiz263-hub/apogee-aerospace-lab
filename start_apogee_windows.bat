@echo off
setlocal
cd /d "%~dp0"
echo.
echo APOGEE v2.4 - Reference Regression and Orbital Validation
echo Installing Python dependencies (if needed)...
where py >nul 2>&1
if %errorlevel% EQU 0 (
    set "PYTHON=py -3"
) else (
    set "PYTHON=python"
)
%PYTHON% -m pip install -r requirements.txt
if errorlevel 1 (
    echo Python dependency installation failed.
    pause
    exit /b 1
)
echo.
echo Open http://127.0.0.1:8000 in your browser.
echo Press Ctrl+C to stop APOGEE.
%PYTHON% -m uvicorn apogee.api:app --host 127.0.0.1 --port 8000
pause
