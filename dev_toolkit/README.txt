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

Nota:
- PowerShell 7 no reemplaza Windows PowerShell 5.1; viven lado a lado.
- Por eso el lanzador fuerza pwsh, para no volver a caer en 5.1 por accidente.
