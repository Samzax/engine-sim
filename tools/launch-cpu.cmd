@echo off
setlocal
set ENGINE_SIM_GPU=0
cd /d "%~dp0"
start "" "%~dp0engine-sim-app.exe"
