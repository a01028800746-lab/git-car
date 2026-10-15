@echo off
setlocal EnableExtensions DisableDelayedExpansion
rem Normal platform launch remains LIVE. This explicit flag is offline only.
set "AUTODRIVE_LAUNCH_MODE=LIVE"
set "AUTODRIVE_LAUNCH_ARGS="
set "AUTODRIVE_LAUNCH_PYFLAGS="
if /i "%~1"=="--self-test" (
    set "AUTODRIVE_LAUNCH_MODE=SELF_TEST"
    set "AUTODRIVE_LAUNCH_ARGS=--self-test"
    set "AUTODRIVE_LAUNCH_PYFLAGS=-B"
)
set "AUTODRIVE_LAUNCH_PYTHON=C:\Python311\python.exe"
set "AUTODRIVE_LAUNCH_SCRIPT=%~dp0AutoDrive.py"
set "AUTODRIVE_LAUNCH_LOG=%~dp0AutoDrive-launcher.log"
rem Append instead of overwriting previous startup failures.
type nul >>"%AUTODRIVE_LAUNCH_LOG%" 2>nul
if not errorlevel 1 goto :log_ready
if not defined TEMP goto :log_unavailable
set "AUTODRIVE_LAUNCH_LOG=%TEMP%\AutoDrive-launcher-fallback.log"
type nul >>"%AUTODRIVE_LAUNCH_LOG%" 2>nul
if errorlevel 1 goto :log_unavailable

:log_ready
>>"%AUTODRIVE_LAUNCH_LOG%" echo [BEGIN] %DATE% %TIME%
>>"%AUTODRIVE_LAUNCH_LOG%" echo MODE=%AUTODRIVE_LAUNCH_MODE%
>>"%AUTODRIVE_LAUNCH_LOG%" echo LAUNCHER="%~f0"
>>"%AUTODRIVE_LAUNCH_LOG%" echo ENTRY_CWD="%CD%"
>>"%AUTODRIVE_LAUNCH_LOG%" echo PYTHON="%AUTODRIVE_LAUNCH_PYTHON%"
>>"%AUTODRIVE_LAUNCH_LOG%" echo CONTROLLER="%AUTODRIVE_LAUNCH_SCRIPT%"
echo AutoDrive launcher mode: %AUTODRIVE_LAUNCH_MODE%
echo Startup diagnostics: "%AUTODRIVE_LAUNCH_LOG%"
cd /d "%~dp0"
if errorlevel 1 goto :bad_directory
>>"%AUTODRIVE_LAUNCH_LOG%" echo WORK_CWD="%CD%"
if not exist "%AUTODRIVE_LAUNCH_PYTHON%" goto :missing_python
if not exist "%AUTODRIVE_LAUNCH_SCRIPT%" goto :missing_controller
>>"%AUTODRIVE_LAUNCH_LOG%" echo [PYTHON_BEGIN] %DATE% %TIME%
"%AUTODRIVE_LAUNCH_PYTHON%" -X utf8 -u %AUTODRIVE_LAUNCH_PYFLAGS% "%AUTODRIVE_LAUNCH_SCRIPT%" %AUTODRIVE_LAUNCH_ARGS% >>"%AUTODRIVE_LAUNCH_LOG%" 2>&1
set "AUTODRIVE_LAUNCH_RC=%ERRORLEVEL%"
goto :finish

:missing_python
>>"%AUTODRIVE_LAUNCH_LOG%" echo ERROR: Python interpreter not found: "%AUTODRIVE_LAUNCH_PYTHON%"
set "AUTODRIVE_LAUNCH_RC=1"
goto :finish

:missing_controller
>>"%AUTODRIVE_LAUNCH_LOG%" echo ERROR: Controller script not found: "%AUTODRIVE_LAUNCH_SCRIPT%"
set "AUTODRIVE_LAUNCH_RC=1"
goto :finish

:bad_directory
>>"%AUTODRIVE_LAUNCH_LOG%" echo ERROR: Cannot enter SDK directory: "%~dp0"
set "AUTODRIVE_LAUNCH_RC=1"
goto :finish

:finish
>>"%AUTODRIVE_LAUNCH_LOG%" echo EXIT_CODE=%AUTODRIVE_LAUNCH_RC%
>>"%AUTODRIVE_LAUNCH_LOG%" echo [END] %DATE% %TIME%
echo AutoDrive launcher exit code: %AUTODRIVE_LAUNCH_RC%
echo Startup diagnostics: "%AUTODRIVE_LAUNCH_LOG%"
exit /b %AUTODRIVE_LAUNCH_RC%

:log_unavailable
echo ERROR: Cannot write startup diagnostics in the SDK directory or TEMP.
echo AutoDrive was not started. Check directory permissions.
exit /b 1
