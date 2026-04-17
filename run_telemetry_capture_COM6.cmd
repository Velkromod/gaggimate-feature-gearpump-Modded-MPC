@echo off
setlocal
cd /d "%~dp0"

if not exist ".\.venv\Scripts\activate.bat" (
  echo [ERROR] No se encontro .\.venv\Scripts\activate.bat
  echo Asegurate de estar en la raiz de la repo y de haber creado el entorno virtual.
  pause
  exit /b 1
)

call ".\.venv\Scripts\activate.bat"

echo [INFO] Iniciando captura de telemetria en COM6...
echo [INFO] Presiona Ctrl+C para detener.
echo.

python tools\telemetry\capture_shots.py --port COM6 --baud 115200 --outdir .\shots --profile "Cremina_Lever"

echo.
echo [INFO] Captura finalizada.
pause
exit /b 0
