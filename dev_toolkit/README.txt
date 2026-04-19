Bundle orientado a PowerShell 7.6+

Cambios:
- El lanzador .cmd usa pwsh.exe, no powershell.exe.
- El script valida PSEdition=Core y PSVersion>=7.6.0.
- Se usa ConvertFrom-Json -AsHashtable, que esta documentado en PowerShell 7.
- Se usan $PSScriptRoot / $PSCommandPath para localizar el script actual.
- Se mantiene la estructura:
  dev_toolkit/
    patches/
    config/
    logs/

Incluye:
- ff_shape_tune_midrise_repo_exact.patch
- pump_tacho_feature_v7.patch

Nota:
- PowerShell 7 no reemplaza Windows PowerShell 5.1; viven lado a lado.
- Por eso el lanzador fuerza pwsh, para no volver a caer en 5.1 por accidente.
- Este toolkit aplica parches sobre la repo local; la captura de telemetria sigue usando tools/telemetry/capture_shots.py desde la repo.


Bugfix v4:
- Corregido Resolve-PythonCommand/Start-TelemetryCapture: ahora se fuerza array con @(...), evitando que una ruta unica como C:\\...\python.exe se indexe como string y termine intentando ejecutar solo la letra C.


Nota v5: la captura ahora espera a que Windows vuelva a exponer y abrir el COM despues del flasheo antes de lanzar capture_shots.py.
