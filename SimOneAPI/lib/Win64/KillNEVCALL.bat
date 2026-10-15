@echo off
setlocal
cd /d "%~dp0"
"C:\Python311\python.exe" -X utf8 "%~dp0AutoDrive.py" --stop
exit /b %errorlevel%
