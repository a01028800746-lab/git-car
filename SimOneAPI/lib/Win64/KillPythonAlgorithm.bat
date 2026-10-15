@echo off
set "PID_FILE=D:/Sim-One/SimOneAPI/lib/Win64/.simone_python_algorithm.pid"
if not exist "%PID_FILE%" exit /b 0
set /p ALGORITHM_PID=<"%PID_FILE%"
taskkill /f /t /pid %ALGORITHM_PID% >nul 2>&1
del "%PID_FILE%" >nul 2>&1
