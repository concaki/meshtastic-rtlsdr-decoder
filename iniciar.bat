@echo off
chcp 65001 >nul
cd /d "%~dp0"
set PYTHONUTF8=1
REM Ajuste as opções abaixo se precisar (ex.: --porta COM5 --ganho 10 --ppm 1)
python mesh_sdr.py --freq 919.875 --preset LongFast --ganho 0 %*
pause
