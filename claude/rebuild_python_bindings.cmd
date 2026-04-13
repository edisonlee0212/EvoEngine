@echo off
REM Quick rebuild script for PyDigitalAgriculture Python bindings
REM Run this from "x64 Native Tools Command Prompt for VS 2022"

echo ============================================================
echo EvoEngine Python Bindings Rebuild
echo ============================================================
echo.
echo This will rebuild only the PyDigitalAgriculture module
echo to fix the pybind11/stl.h include issue.
echo.
echo IMPORTANT: You must run this from VS Developer Command Prompt!
echo.
pause

cd /d %~dp0..
echo Current directory: %CD%
echo.

if not exist "out\build\x64-Release\build.ninja" (
    echo ERROR: Build directory not found!
    echo Expected: out\build\x64-Release\
    echo.
    echo Please ensure you've built EvoEngine at least once.
    pause
    exit /b 1
)

echo [1/2] Changing to build directory...
cd out\build\x64-Release
echo.

echo [2/2] Rebuilding PyDigitalAgriculture...
cmake --build . --config Release --target PyDigitalAgriculture -j 8

if %ERRORLEVEL% EQU 0 (
    echo.
    echo ============================================================
    echo SUCCESS! Python bindings rebuilt.
    echo ============================================================
    echo.
    echo Updated file:
    dir /b PythonBinding\PyDigitalAgriculture*.pyd
    echo.
    echo Next step: Run the PAR calculation script
    echo   cd C:\Users\Brenda\code\EvoEngine\claude
    echo   py -3.9 sorghum_single_leaf_daily_par_evoengine.py
    echo.
) else (
    echo.
    echo ============================================================
    echo BUILD FAILED!
    echo ============================================================
    echo.
    echo Make sure you're running this from:
    echo "x64 Native Tools Command Prompt for VS 2022"
    echo.
    echo If the error persists, try a full rebuild:
    echo   cd C:\Users\Brenda\code\EvoEngine
    echo   build.cmd --no-test
    echo.
)

pause
