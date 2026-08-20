@echo off
setlocal
set "ROOT=%~dp0"
set "APP="
for %%P in (sorghum-editor vrt vs2026-x64-nocuda) do if not defined APP if exist "%ROOT%out\install\%%P\bin\DigitalAgricultureApp.exe" set "APP=%ROOT%out\install\%%P\bin\DigitalAgricultureApp.exe"

if not defined APP (
  echo Sorghum Editor is not installed yet.
  echo Expected under: %ROOT%out\install
  echo Build and install the DigitalAgricultureApp, then run this file again.
  pause
  exit /b 1
)

pushd "%ROOT%"
start "Sorghum Genotype Lab" "%APP%" --editor
popd
