param()

$ErrorActionPreference = "Stop"
Set-StrictMode -Version Latest

# Require PowerShell 7.6+ explicitly.
if ($PSVersionTable.PSEdition -ne 'Core') {
    throw "Este toolkit requiere PowerShell 7+ (pwsh). Edicion actual: $($PSVersionTable.PSEdition)."
}
if ($PSVersionTable.PSVersion -lt [version]'7.6.0') {
    throw "Este toolkit requiere PowerShell 7.6.0 o superior. Version actual: $($PSVersionTable.PSVersion)."
}

function Write-Section {
    param([string]$Text)
    Write-Host ""
    Write-Host ("=== {0} ===" -f $Text) -ForegroundColor Cyan
}

function Write-Info {
    param([string]$Text)
    Write-Host ("[INFO] {0}" -f $Text) -ForegroundColor Yellow
}

function Write-Ok {
    param([string]$Text)
    Write-Host ("[OK] {0}" -f $Text) -ForegroundColor Green
}

function Write-Fail {
    param([string]$Text)
    Write-Host ("[ERROR] {0}" -f $Text) -ForegroundColor Red
}

function Pause-Return {
    Write-Host ""
    Read-Host "Presiona Enter para volver al menu" | Out-Null
}

function Get-ToolkitRoot {
    if ($PSScriptRoot -and -not [string]::IsNullOrWhiteSpace($PSScriptRoot)) {
        return $PSScriptRoot
    }
    if ($PSCommandPath -and -not [string]::IsNullOrWhiteSpace($PSCommandPath)) {
        return (Split-Path -Parent $PSCommandPath)
    }
    return (Get-Location).Path
}

function Get-RepoRoot {
    param([string]$ToolkitRoot)

    $parent = Split-Path -Parent $ToolkitRoot
    if (-not [string]::IsNullOrWhiteSpace($parent)) {
        $capture = Join-Path $parent "tools\telemetry\capture_shots.py"
        if (Test-Path $capture) {
            return $parent
        }
    }

    $captureLocal = Join-Path $ToolkitRoot "tools\telemetry\capture_shots.py"
    if (Test-Path $captureLocal) {
        return $ToolkitRoot
    }

    return $parent
}

function Ensure-ToolkitDirectories {
    param([string]$ToolkitRoot)
    foreach ($dir in @("patches","config","logs")) {
        $path = Join-Path $ToolkitRoot $dir
        if (-not (Test-Path $path)) {
            New-Item -ItemType Directory -Path $path | Out-Null
        }
    }
}

function Get-LogPath {
    param([string]$ToolkitRoot)
    return (Join-Path $ToolkitRoot "logs\gaggimate_dev_toolkit.log")
}

function Write-ToolkitLog {
    param(
        [string]$ToolkitRoot,
        [string]$Level,
        [string]$Message
    )
    try {
        $logPath = Get-LogPath -ToolkitRoot $ToolkitRoot
        $stamp = Get-Date -Format "yyyy-MM-dd HH:mm:ss"
        Add-Content -Path $logPath -Value ("[{0}] [{1}] {2}" -f $stamp, $Level, $Message)
    }
    catch {
    }
}

function Assert-RepoShape {
    param([string]$Root)

    $expected = @(
        "tools\telemetry\capture_shots.py",
        "lib"
    )

    $missing = @()
    foreach ($item in $expected) {
        $path = Join-Path $Root $item
        if (-not (Test-Path $path)) {
            $missing += $item
        }
    }

    if ($missing.Count -gt 0) {
        throw ("La carpeta no parece ser la raiz de la repo esperada. Faltan: {0}" -f ($missing -join ", "))
    }
}

function Get-ConfigPath {
    param([string]$ToolkitRoot)
    return (Join-Path $ToolkitRoot "config\gaggimate_dev_toolkit.config.json")
}

function Get-DefaultConfig {
    return @{
        telemetry = @{
            com_port = "COM6"
            baud = "115200"
            outdir = ".\shots"
            profile = "Cremina_Lever"
        }
        ui = @{
            last_patch = ""
        }
    }
}

function Save-ToolkitConfig {
    param(
        [string]$ToolkitRoot,
        [hashtable]$Config
    )
    $configPath = Get-ConfigPath -ToolkitRoot $ToolkitRoot
    $json = $Config | ConvertTo-Json -Depth 8
    Set-Content -Path $configPath -Value $json -Encoding UTF8
    return $configPath
}

function Load-ToolkitConfig {
    param([string]$ToolkitRoot)

    $default = Get-DefaultConfig
    $configPath = Get-ConfigPath -ToolkitRoot $ToolkitRoot

    if (-not (Test-Path $configPath)) {
        Save-ToolkitConfig -ToolkitRoot $ToolkitRoot -Config $default | Out-Null
        return $default
    }

    $raw = Get-Content $configPath -Raw
    if ([string]::IsNullOrWhiteSpace($raw)) {
        Save-ToolkitConfig -ToolkitRoot $ToolkitRoot -Config $default | Out-Null
        return $default
    }

    $obj = $raw | ConvertFrom-Json -AsHashtable

    foreach ($section in @("telemetry","ui")) {
        if (-not $obj.ContainsKey($section)) {
            $obj[$section] = $default[$section]
        }
    }

    foreach ($k in @("com_port","baud","outdir","profile")) {
        if (-not $obj.telemetry.ContainsKey($k) -or [string]::IsNullOrWhiteSpace([string]$obj.telemetry[$k])) {
            $obj.telemetry[$k] = $default.telemetry[$k]
        }
    }

    if (-not $obj.ui.ContainsKey("last_patch")) {
        $obj.ui["last_patch"] = ""
    }

    return $obj
}

function Get-AvailableComPorts {
    try {
        return @([System.IO.Ports.SerialPort]::GetPortNames() | Sort-Object)
    }
    catch {
        return @()
    }
}

function Show-CurrentTelemetryConfig {
    param(
        [hashtable]$Config,
        [string]$RepoRoot,
        [string]$ToolkitRoot
    )

    Write-Section "Configuracion actual"
    Write-Host ("  Repo root    : {0}" -f $RepoRoot)
    Write-Host ("  Toolkit dir  : {0}" -f $ToolkitRoot)
    Write-Host ("  Patches dir  : {0}" -f (Join-Path $ToolkitRoot "patches"))
    Write-Host ("  Config file  : {0}" -f (Get-ConfigPath -ToolkitRoot $ToolkitRoot))
    Write-Host ("  Toolkit log  : {0}" -f (Get-LogPath -ToolkitRoot $ToolkitRoot))
    Write-Host ("  PS Edition   : {0}" -f $PSVersionTable.PSEdition)
    Write-Host ("  PS Version   : {0}" -f $PSVersionTable.PSVersion)
    Write-Host ("  COM          : {0}" -f $Config.telemetry.com_port)
    Write-Host ("  Baud         : {0}" -f $Config.telemetry.baud)
    Write-Host ("  OutDir repo  : {0}" -f $Config.telemetry.outdir)
    Write-Host ("  Profile      : {0}" -f $Config.telemetry.profile)
    if (-not [string]::IsNullOrWhiteSpace([string]$Config.ui.last_patch)) {
        Write-Host ("  Ultimo patch : {0}" -f $Config.ui.last_patch)
    }
}

function Configure-TelemetryDefaults {
    param(
        [string]$ToolkitRoot,
        [string]$RepoRoot,
        [hashtable]$Config
    )

    Show-CurrentTelemetryConfig -Config $Config -RepoRoot $RepoRoot -ToolkitRoot $ToolkitRoot
    Write-Host ""
    Write-Host "Deja vacio para mantener el valor actual."

    $port = Read-Host ("Puerto COM [{0}]" -f $Config.telemetry.com_port)
    if (-not [string]::IsNullOrWhiteSpace($port)) { $Config.telemetry.com_port = $port.Trim() }

    $baud = Read-Host ("Baud [{0}]" -f $Config.telemetry.baud)
    if (-not [string]::IsNullOrWhiteSpace($baud)) { $Config.telemetry.baud = $baud.Trim() }

    $outdir = Read-Host ("OutDir relativo a repo [{0}]" -f $Config.telemetry.outdir)
    if (-not [string]::IsNullOrWhiteSpace($outdir)) { $Config.telemetry.outdir = $outdir.Trim() }

    $profile = Read-Host ("Profile [{0}]" -f $Config.telemetry.profile)
    if (-not [string]::IsNullOrWhiteSpace($profile)) { $Config.telemetry.profile = $profile.Trim() }

    $path = Save-ToolkitConfig -ToolkitRoot $ToolkitRoot -Config $Config
    Write-ToolkitLog -ToolkitRoot $ToolkitRoot -Level "INFO" -Message ("Configuracion actualizada: COM={0}, Baud={1}, OutDir={2}, Profile={3}" -f $Config.telemetry.com_port, $Config.telemetry.baud, $Config.telemetry.outdir, $Config.telemetry.profile)
    Write-Ok ("Configuracion guardada en: {0}" -f $path)
}

function Choose-DefaultComPort {
    param(
        [string]$ToolkitRoot,
        [hashtable]$Config
    )

    $ports = @(Get-AvailableComPorts)
    Write-Section "Puertos COM detectados"

    if ($ports.Length -eq 0) {
        Write-Fail "No se detectaron puertos COM. Puedes escribir uno manualmente."
        $manual = Read-Host ("Puerto COM actual [{0}]" -f $Config.telemetry.com_port)
        if (-not [string]::IsNullOrWhiteSpace($manual)) {
            $Config.telemetry.com_port = $manual.Trim()
            Save-ToolkitConfig -ToolkitRoot $ToolkitRoot -Config $Config | Out-Null
            Write-ToolkitLog -ToolkitRoot $ToolkitRoot -Level "INFO" -Message ("COM por defecto cambiado manualmente a {0}" -f $Config.telemetry.com_port)
            Write-Ok ("COM por defecto actualizado a: {0}" -f $Config.telemetry.com_port)
        }
        return
    }

    for ($i = 0; $i -lt $ports.Length; $i++) {
        $mark = ""
        if ($ports[$i] -eq $Config.telemetry.com_port) { $mark = " (actual)" }
        Write-Host ("  [{0}] {1}{2}" -f ($i + 1), $ports[$i], $mark)
    }
    Write-Host "  [M] Ingresar manualmente"
    Write-Host "  [0] Cancelar"
    Write-Host ""

    $selection = Read-Host "Elige el puerto"
    if ([string]::IsNullOrWhiteSpace($selection)) { return }

    if ($selection.Trim().ToUpperInvariant() -eq "M") {
        $manual = Read-Host "Escribe el COM (ej: COM6)"
        if (-not [string]::IsNullOrWhiteSpace($manual)) {
            $Config.telemetry.com_port = $manual.Trim()
            Save-ToolkitConfig -ToolkitRoot $ToolkitRoot -Config $Config | Out-Null
            Write-ToolkitLog -ToolkitRoot $ToolkitRoot -Level "INFO" -Message ("COM por defecto cambiado manualmente a {0}" -f $Config.telemetry.com_port)
            Write-Ok ("COM por defecto actualizado a: {0}" -f $Config.telemetry.com_port)
        }
        return
    }

    $parsed = 0
    if (-not [int]::TryParse($selection, [ref]$parsed)) {
        Write-Fail "Debes ingresar un numero, M o 0."
        return
    }
    if ($parsed -eq 0) { return }

    $index = $parsed - 1
    if ($index -lt 0 -or $index -ge $ports.Length) {
        Write-Fail "Seleccion fuera de rango."
        return
    }

    $Config.telemetry.com_port = $ports[$index]
    Save-ToolkitConfig -ToolkitRoot $ToolkitRoot -Config $Config | Out-Null
    Write-ToolkitLog -ToolkitRoot $ToolkitRoot -Level "INFO" -Message ("COM por defecto cambiado a {0}" -f $Config.telemetry.com_port)
    Write-Ok ("COM por defecto actualizado a: {0}" -f $Config.telemetry.com_port)
}

function Get-PatchFiles {
    param([string]$ToolkitRoot)
    $patchDir = Join-Path $ToolkitRoot "patches"
    if (-not (Test-Path $patchDir)) {
        return @()
    }
    return @(Get-ChildItem -Path $patchDir -Filter *.patch -File | Sort-Object Name)
}

function Select-Patch {
    param(
        [string]$ToolkitRoot,
        [string]$LastPatch
    )

    $patches = @(Get-PatchFiles -ToolkitRoot $ToolkitRoot)
    if ($patches.Length -eq 0) {
        Write-Fail ("No se encontraron archivos .patch en {0}" -f (Join-Path $ToolkitRoot "patches"))
        return $null
    }

    Write-Section "Parches disponibles"
    for ($i = 0; $i -lt $patches.Length; $i++) {
        $mark = ""
        if ($patches[$i].Name -eq $LastPatch) { $mark = " (ultimo usado)" }
        Write-Host ("  [{0}] {1}{2}" -f ($i + 1), $patches[$i].Name, $mark)
    }
    Write-Host "  [0] Cancelar"
    Write-Host ""

    $selection = Read-Host "Elige el numero del patch"
    $parsed = 0
    if (-not [int]::TryParse($selection, [ref]$parsed)) {
        Write-Fail "Debes ingresar un numero."
        return $null
    }
    if ($parsed -eq 0) {
        return $null
    }

    $index = $parsed - 1
    if ($index -lt 0 -or $index -ge $patches.Length) {
        Write-Fail "Seleccion fuera de rango."
        return $null
    }

    return $patches[$index]
}

function Confirm-Action {
    param([string]$Message)

    $resp = Read-Host ("{0} [y/N]" -f $Message)
    if ([string]::IsNullOrWhiteSpace($resp)) { return $false }
    return ($resp.Trim().ToUpperInvariant() -eq "Y")
}

function Invoke-GitApplyAction {
    param(
        [ValidateSet("apply","revert","check")]
        [string]$Action,
        [System.IO.FileInfo]$Patch,
        [string]$RepoRoot,
        [string]$ToolkitRoot,
        [hashtable]$Config
    )

    $baseArgs = @("--ignore-space-change","--ignore-whitespace")
    $patchPath = $Patch.FullName

    Push-Location $RepoRoot
    try {
        switch ($Action) {
            "apply" {
                if (-not (Confirm-Action -Message ("Aplicar patch '{0}'?" -f $Patch.Name))) {
                    Write-Info "Operacion cancelada."
                    return
                }

                Write-Info "Verificando que el patch aplique limpio..."
                & git apply --check @baseArgs $patchPath
                if ($LASTEXITCODE -ne 0) {
                    throw "El patch no aplica limpio."
                }

                Write-Info "Aplicando patch..."
                & git apply @baseArgs $patchPath
                if ($LASTEXITCODE -ne 0) {
                    throw "Fallo al aplicar el patch."
                }

                Write-Info "Verificando que quedo aplicado..."
                & git apply -R --check @baseArgs $patchPath
                if ($LASTEXITCODE -ne 0) {
                    throw "El patch parece haberse aplicado de forma incompleta."
                }

                $Config.ui.last_patch = $Patch.Name
                Save-ToolkitConfig -ToolkitRoot $ToolkitRoot -Config $Config | Out-Null
                Write-ToolkitLog -ToolkitRoot $ToolkitRoot -Level "INFO" -Message ("Patch aplicado: {0}" -f $Patch.Name)
                Write-Ok ("Parche aplicado correctamente: {0}" -f $Patch.Name)
            }
            "revert" {
                if (-not (Confirm-Action -Message ("Revertir patch '{0}'?" -f $Patch.Name))) {
                    Write-Info "Operacion cancelada."
                    return
                }

                Write-Info "Verificando que el patch se pueda revertir..."
                & git apply -R --check @baseArgs $patchPath
                if ($LASTEXITCODE -ne 0) {
                    throw "El patch no se puede revertir limpio. Puede no estar aplicado o el arbol puede haber cambiado."
                }

                Write-Info "Revirtiendo patch..."
                & git apply -R @baseArgs $patchPath
                if ($LASTEXITCODE -ne 0) {
                    throw "Fallo al revertir el patch."
                }

                Write-ToolkitLog -ToolkitRoot $ToolkitRoot -Level "INFO" -Message ("Patch revertido: {0}" -f $Patch.Name)
                Write-Ok ("Parche revertido correctamente: {0}" -f $Patch.Name)
            }
            "check" {
                Write-Info "Ejecutando check del patch..."
                & git apply --check @baseArgs $patchPath
                if ($LASTEXITCODE -ne 0) {
                    throw "El patch no aplica limpio."
                }

                Write-ToolkitLog -ToolkitRoot $ToolkitRoot -Level "INFO" -Message ("Patch check OK: {0}" -f $Patch.Name)
                Write-Ok ("El patch aplica limpio: {0}" -f $Patch.Name)
            }
        }
    }
    finally {
        Pop-Location
    }
}

function Resolve-PythonCommand {
    param([string]$RepoRoot)

    $venvPython = Join-Path $RepoRoot ".venv\Scripts\python.exe"
    if (Test-Path $venvPython) {
        return @($venvPython)
    }

    $pyCmd = Get-Command py -ErrorAction SilentlyContinue
    if ($pyCmd) {
        return @("py","-3")
    }

    $pythonCmd = Get-Command python -ErrorAction SilentlyContinue
    if ($pythonCmd) {
        return @("python")
    }

    throw "No encontre .venv\Scripts\python.exe, ni py, ni python."
}

function Start-TelemetryCapture {
    param(
        [string]$RepoRoot,
        [string]$ToolkitRoot,
        [string]$Port,
        [string]$Baud,
        [string]$OutDir,
        [string]$Profile
    )

    $captureScript = Join-Path $RepoRoot "tools\telemetry\capture_shots.py"
    if (-not (Test-Path $captureScript)) {
        throw "No se encontro tools\telemetry\capture_shots.py"
    }

    $py = Resolve-PythonCommand -RepoRoot $RepoRoot
    Write-Section "Captura de telemetria"
    Write-Info ("Puerto: {0} | Baud: {1} | Perfil: {2} | OutDir repo: {3}" -f $Port, $Baud, $Profile, $OutDir)
    Write-Info "Usa Ctrl+C para detener."

    Write-ToolkitLog -ToolkitRoot $ToolkitRoot -Level "INFO" -Message ("Inicio telemetria: COM={0}, Baud={1}, OutDir={2}, Profile={3}" -f $Port, $Baud, $OutDir, $Profile)

    Push-Location $RepoRoot
    try {
        if ($py.Length -gt 1) {
            & $py[0] @($py[1..($py.Length-1)]) $captureScript --port $Port --baud $Baud --outdir $OutDir --profile $Profile
        } else {
            & $py[0] $captureScript --port $Port --baud $Baud --outdir $OutDir --profile $Profile
        }
    }
    finally {
        Write-ToolkitLog -ToolkitRoot $ToolkitRoot -Level "INFO" -Message "Fin telemetria"
        Pop-Location
    }
}

function Show-Menu {
    param(
        [hashtable]$Config,
        [string]$RepoRoot,
        [string]$ToolkitRoot
    )

    Write-Host ""
    Write-Host "==========================================" -ForegroundColor DarkCyan
    Write-Host "   GaggiMate Dev Toolkit (pwsh 7.6+)      " -ForegroundColor Cyan
    Write-Host "==========================================" -ForegroundColor DarkCyan
    Write-Host ("  Repo    : {0}" -f $RepoRoot)
    Write-Host ("  Toolkit : {0}" -f $ToolkitRoot)
    Write-Host ("  Patches : {0}" -f (Join-Path $ToolkitRoot "patches"))
    Write-Host ("  COM     : {0}" -f $Config.telemetry.com_port)
    Write-Host ""
    Write-Host "  [1] Aplicar patch"
    Write-Host "  [2] Revertir patch"
    Write-Host "  [3] Check de patch"
    Write-Host "  [4] Tomar telemetria con configuracion guardada"
    Write-Host "  [5] Tomar telemetria personalizada"
    Write-Host "  [6] Elegir COM por defecto"
    Write-Host "  [7] Configurar defaults de telemetria"
    Write-Host "  [8] Ver configuracion actual"
    Write-Host "  [9] Abrir carpeta shots"
    Write-Host "  [0] Salir"
    Write-Host ""
}

$toolkitRoot = Get-ToolkitRoot
Ensure-ToolkitDirectories -ToolkitRoot $toolkitRoot
$repoRoot = Get-RepoRoot -ToolkitRoot $toolkitRoot
Set-Location $repoRoot
Assert-RepoShape -Root $repoRoot

while ($true) {
    try {
        $config = Load-ToolkitConfig -ToolkitRoot $toolkitRoot
        Show-Menu -Config $config -RepoRoot $repoRoot -ToolkitRoot $toolkitRoot
        $choice = Read-Host "Elige una opcion"

        switch ($choice) {
            "1" {
                $patch = Select-Patch -ToolkitRoot $toolkitRoot -LastPatch $config.ui.last_patch
                if ($patch) {
                    Invoke-GitApplyAction -Action "apply" -Patch $patch -RepoRoot $repoRoot -ToolkitRoot $toolkitRoot -Config $config
                }
                Pause-Return
            }
            "2" {
                $patch = Select-Patch -ToolkitRoot $toolkitRoot -LastPatch $config.ui.last_patch
                if ($patch) {
                    Invoke-GitApplyAction -Action "revert" -Patch $patch -RepoRoot $repoRoot -ToolkitRoot $toolkitRoot -Config $config
                }
                Pause-Return
            }
            "3" {
                $patch = Select-Patch -ToolkitRoot $toolkitRoot -LastPatch $config.ui.last_patch
                if ($patch) {
                    Invoke-GitApplyAction -Action "check" -Patch $patch -RepoRoot $repoRoot -ToolkitRoot $toolkitRoot -Config $config
                }
                Pause-Return
            }
            "4" {
                Start-TelemetryCapture -RepoRoot $repoRoot `
                    -ToolkitRoot $toolkitRoot `
                    -Port $config.telemetry.com_port `
                    -Baud $config.telemetry.baud `
                    -OutDir $config.telemetry.outdir `
                    -Profile $config.telemetry.profile
                Pause-Return
            }
            "5" {
                $port = Read-Host ("Puerto COM [{0}]" -f $config.telemetry.com_port)
                if ([string]::IsNullOrWhiteSpace($port)) { $port = $config.telemetry.com_port }

                $baud = Read-Host ("Baud [{0}]" -f $config.telemetry.baud)
                if ([string]::IsNullOrWhiteSpace($baud)) { $baud = $config.telemetry.baud }

                $outdir = Read-Host ("OutDir relativo a repo [{0}]" -f $config.telemetry.outdir)
                if ([string]::IsNullOrWhiteSpace($outdir)) { $outdir = $config.telemetry.outdir }

                $profile = Read-Host ("Profile [{0}]" -f $config.telemetry.profile)
                if ([string]::IsNullOrWhiteSpace($profile)) { $profile = $config.telemetry.profile }

                Start-TelemetryCapture -RepoRoot $repoRoot -ToolkitRoot $toolkitRoot -Port $port -Baud $baud -OutDir $outdir -Profile $profile
                Pause-Return
            }
            "6" {
                Choose-DefaultComPort -ToolkitRoot $toolkitRoot -Config $config
                Pause-Return
            }
            "7" {
                Configure-TelemetryDefaults -ToolkitRoot $toolkitRoot -RepoRoot $repoRoot -Config $config
                Pause-Return
            }
            "8" {
                Show-CurrentTelemetryConfig -Config $config -RepoRoot $repoRoot -ToolkitRoot $toolkitRoot
                Pause-Return
            }
            "9" {
                $shots = Join-Path $repoRoot "shots"
                if (-not (Test-Path $shots)) {
                    New-Item -ItemType Directory -Path $shots | Out-Null
                }
                Start-Process explorer.exe $shots
                Write-Ok ("Carpeta abierta: {0}" -f $shots)
                Pause-Return
            }
            "0" {
                exit 0
            }
            default {
                Write-Fail "Opcion invalida."
                Pause-Return
            }
        }
    }
    catch {
        Write-ToolkitLog -ToolkitRoot $toolkitRoot -Level "ERROR" -Message $_.Exception.Message
        Write-Host ""
        Write-Fail ($_.Exception.Message)
        Pause-Return
    }
}
