@echo off
setlocal
cd /d "%~dp0"

set "PATCH=ff_shape_tune_midrise_repo_exact.patch"

if not exist "%PATCH%" (
  echo [ERROR] No se encontro %PATCH% en la carpeta actual.
  pause
  exit /b 1
)

echo [INFO] Verificando que el parche aplique limpio...
git apply --check --ignore-space-change --ignore-whitespace "%PATCH%"
if errorlevel 1 (
  echo [ERROR] El parche no aplica limpio. Revisa si tu repo coincide con la base esperada.
  pause
  exit /b 1
)

echo [INFO] Aplicando parche...
git apply --ignore-space-change --ignore-whitespace "%PATCH%"
if errorlevel 1 (
  echo [ERROR] Fallo al aplicar el parche.
  pause
  exit /b 1
)

echo [INFO] Verificando cambios esperados...
powershell -NoProfile -ExecutionPolicy Bypass -Command ^
  "$patterns=@(" ^
  "'_feedforwardDynamicMaxPct = 8.5f'," ^
  "'_ffPressureGateStartBar = 1.3f'," ^
  "'_ffPressureGateFullBar = 3.7f'," ^
  "'_ffPressureTaperStartBar = 7.2f'," ^
  "'_ffAboveGateFullBar = 0.22f'," ^
  "'_ffGammaBoostMax = 0.16f'" ^
  ");" ^
  "$path='.\lib\NayrodPID\src\PressureController\PressureController.h';" ^
  "$ok=$true;" ^
  "foreach($p in $patterns){ if(-not (Select-String -Path $path -Pattern $p -Quiet)){ $ok=$false; Write-Host ('[MISSING] ' + $p) } };" ^
  "if($ok){ Write-Host '[OK] Parche FF aplicado correctamente.'; exit 0 } else { exit 2 }"
if errorlevel 1 (
  echo [ERROR] La verificacion final no encontro todos los cambios esperados.
  pause
  exit /b 1
)

echo [OK] Parche FF aplicado correctamente.
pause
exit /b 0
