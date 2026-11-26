@echo off
rem Builds RoverX1Sim.exe on the Desktop (needs: python -m pip install pyinstaller)
cd /d "%~dp0"
set "OUT=%~dp0.."
set "WORK=%TEMP%\RoverX1Sim_build"

python -m PyInstaller --noconfirm --clean --onefile --name RoverX1Sim ^
  --icon "%~dp0assets\icon.ico" ^
  --add-data "%~dp0static;static" ^
  --hidden-import sdk_compat --hidden-import random --hidden-import statistics ^
  --hidden-import datetime --hidden-import itertools --hidden-import collections ^
  --hidden-import functools --hidden-import string ^
  --distpath "%OUT%" --workpath "%WORK%" --specpath "%WORK%" ^
  "%~dp0server.py"

echo.
if exist "%OUT%\RoverX1Sim.exe" (echo Done: %OUT%\RoverX1Sim.exe) else (echo Build failed - see the messages above)
pause
