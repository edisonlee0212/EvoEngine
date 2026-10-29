@echo off
setlocal
set "ZACH_SCRIPT_DIR=%~dp0"
if not defined ZACH_PYTHON set "ZACH_PYTHON=python"
if not exist "%ZACH_SCRIPT_DIR%.venv\Scripts\python.exe" (
  "%ZACH_PYTHON%" -m venv "%ZACH_SCRIPT_DIR%.venv"
  if errorlevel 1 exit /b 1
)
"%ZACH_SCRIPT_DIR%.venv\Scripts\python.exe" -m pip install -r "%ZACH_SCRIPT_DIR%requirements.txt" --disable-pip-version-check
if errorlevel 1 exit /b 1
"%ZACH_SCRIPT_DIR%.venv\Scripts\python.exe" "%ZACH_SCRIPT_DIR%generate.py" %*
exit /b %errorlevel%
