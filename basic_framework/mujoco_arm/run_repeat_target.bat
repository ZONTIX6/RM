@echo off
setlocal
cd /d "%~dp0"

set "PYEXE="
if exist ".venv\Scripts\python.exe" set "PYEXE=%~dp0.venv\Scripts\python.exe"
if not defined PYEXE (
  for /f "delims=" %%I in ('where py 2^>nul') do (
    if not defined PYEXE set "PYEXE=%%~I"
  )
)
if not defined PYEXE (
  for /f "delims=" %%I in ('where python 2^>nul') do (
    if not defined PYEXE set "PYEXE=%%~I"
  )
)
if not defined PYEXE (
  echo Python not found. Install Python 3.12 and the packages in requirements.txt.
  pause
  exit /b 1
)

"%PYEXE%" run_ntu_arm.py --repeat-target --print-joints --print-period 0.5
pause
endlocal
