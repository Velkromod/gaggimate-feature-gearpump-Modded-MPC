@echo off
setlocal
cd /d "%~dp0"

color 07
title GaggiMate Dev Toolkit Launcher (pwsh 7.6+)

echo ==========================================
echo   GaggiMate Dev Toolkit Launcher
echo ==========================================
echo.
echo [INFO] Toolkit dir: %CD%
echo.

if not exist ".\gaggimate_dev_toolkit.ps1" (
  color 0C
  echo [ERROR] No se encontro gaggimate_dev_toolkit.ps1 en esta carpeta.
  echo [TIP] Copia este .cmd y el .ps1 dentro de la carpeta dev_toolkit.
  echo.
  pause
  exit /b 1
)

where pwsh >nul 2>nul
if errorlevel 1 (
  color 0C
  echo [ERROR] No se encontro pwsh.exe en PATH.
  echo [TIP] Instala PowerShell 7.6+ o agrega pwsh al PATH.
  echo.
  pause
  exit /b 1
)

echo [INFO] Iniciando toolkit con pwsh...
echo.

pwsh -NoLogo -NoProfile -ExecutionPolicy Bypass -File ".\gaggimate_dev_toolkit.ps1"
set "EXITCODE=%ERRORLEVEL%"

echo.
if not "%EXITCODE%"=="0" (
  color 0C
  echo [ERROR] El toolkit termino con codigo %EXITCODE%.
  echo [TIP] Si pwsh mostro un error, copialo o sacale una foto.
) else (
  color 0A
  echo [OK] Toolkit finalizado correctamente.
)

echo.
pause
exit /b %EXITCODE%
