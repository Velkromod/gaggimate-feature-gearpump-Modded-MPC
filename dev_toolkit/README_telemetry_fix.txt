Arreglo incluido

Problema corregido:
- La opcion de telemetria podia intentar ejecutar solo la letra "C"
  cuando Resolve-PythonCommand devolvia una ruta unica como string.
- Se corrigio forzando contexto de array en Start-TelemetryCapture:

  $py = @(Resolve-PythonCommand -RepoRoot $RepoRoot)

Eso evita que PowerShell interprete la ruta "C:\...\python.exe" como una
cadena indexable y termine usando solo $py[0] = "C".

Instalacion:
1) Reemplaza en tu repo:
   dev_toolkit\gaggimate_dev_toolkit.ps1
   dev_toolkit\gaggimate_dev_toolkit.cmd
2) Vuelve a abrir:
   dev_toolkit\gaggimate_dev_toolkit.cmd
3) Usa la opcion [4] o [5] para telemetria.

