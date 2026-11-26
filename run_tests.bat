@echo off
rem Runs all automated tests of the simulator
cd /d "%~dp0"
python -m unittest discover -s tests -v
pause
