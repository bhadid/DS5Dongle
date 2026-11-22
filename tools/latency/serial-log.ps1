# Log the diagnostic firmware's USB serial output to a file, with PC timestamps.
# Waits for the port to appear and reopens it if the Pico is unplugged or reboots.
# Lines written to <logfile>.cmd are sent to the Pico (diag console commands) and the file removed.
# Usage: powershell -ExecutionPolicy Bypass -File serial-log.ps1 <COMx> <logfile> [seconds]
param([Parameter(Mandatory)][string]$Port, [string]$Log = 'pico-serial.log', [int]$Seconds = 600)
$ErrorActionPreference = 'Stop'
$Log = [IO.Path]::GetFullPath((Join-Path $PWD $Log))
$CmdFile = "$Log.cmd"
New-Item -ItemType Directory -Force (Split-Path $Log) | Out-Null
$w = [System.IO.StreamWriter]::new($Log, $true)
$w.AutoFlush = $true
function Note($m) { $w.WriteLine((Get-Date).ToString('HH:mm:ss.fff') + ' ' + $m) }
$end = (Get-Date).AddSeconds($Seconds)
try {
    while ((Get-Date) -lt $end) {
        if ([System.IO.Ports.SerialPort]::GetPortNames() -notcontains $Port) { Start-Sleep -Milliseconds 200; continue }
        $sp = New-Object System.IO.Ports.SerialPort $Port, 115200
        $sp.DtrEnable = $true   # Pico stdio_usb only sends while the host has DTR set
        $sp.ReadTimeout = 200
        try { $sp.Open() } catch { Start-Sleep -Milliseconds 200; continue }
        Note "[LOGGER] $Port opened"
        try {
            while ((Get-Date) -lt $end) {
                if (Test-Path $CmdFile) {
                    foreach ($c in Get-Content $CmdFile) {
                        if ($c.Trim()) { $sp.Write($c.Trim() + "`n"); Note "[LOGGER] sent '$($c.Trim())'" }
                    }
                    Remove-Item $CmdFile
                }
                try { $line = $sp.ReadLine() } catch [System.TimeoutException] { continue }
                Note $line.TrimEnd()
            }
        } catch {
            Note "[LOGGER] $Port lost: $($_.Exception.Message.Trim())"
        } finally { try { $sp.Close() } catch {} }
    }
} finally { $w.Close() }
