@echo off
chcp 65001 >nul
cd /d "%~dp0"
set PYTHONUTF8=1
echo Gerando gravacao sintetica com 13 pacotes Meshtastic...
python tools\gerar_sinal_teste.py teste.cu8 > teste_comando.txt
type teste_comando.txt
echo.
echo Decodificando...
for /f "usebackq delims=" %%L in (`findstr /b /c:"python mesh_sdr.py" teste_comando.txt`) do %%L
pause
