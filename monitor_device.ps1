# Monitor script for ESP32S3 with reset capability and timeout
# Usage: .\monitor_device.ps1 [timeout_seconds] [-NoReset]

param(
    [Parameter(Mandatory=$false)]
    [int]$TimeoutSeconds = 30,
    
    [Parameter(Mandatory=$false)]
    [switch]$NoReset
)

$PlatformIO = "C:\Users\phrop\.platformio\penv\Scripts\platformio.exe"
$Environment = "mci_fenrir_esp32s3"
$Baud = 115200

Write-Host "=========================================" -ForegroundColor Cyan
Write-Host "ESP32S3 Serial Monitor" -ForegroundColor Cyan
Write-Host "=========================================" -ForegroundColor Cyan

# Set UTF-8 encoding to avoid Unicode errors
chcp 65001 | Out-Null
$env:PYTHONIOENCODING = 'utf-8'

# Function to ensure COM port is free
function Clear-ComPort {
    param([string]$Port)
    Write-Host "Ensuring COM port is free..." -ForegroundColor Yellow
    # Kill any processes that might be holding the port
    Get-Process | Where-Object {$_.ProcessName -like "*python*" -or $_.ProcessName -like "*pio*" -or $_.ProcessName -like "*platformio*"} | Stop-Process -Force -ErrorAction SilentlyContinue
    Start-Sleep -Milliseconds 500
}

# Function to auto-detect COM port
function Get-DevicePort {
    Write-Host "Auto-detecting COM port..." -ForegroundColor Yellow
    try {
        $deviceList = & $PlatformIO device list --json 2>&1 | ConvertFrom-Json
        if ($deviceList) {
            # Look for ESP32-S3 or any serial port
            foreach ($device in $deviceList) {
                if ($device.port -and $device.port -match '^COM\d+$') {
                    Write-Host "Found device on: $($device.port)" -ForegroundColor Green
                    return $device.port
                }
            }
        }
    } catch {
        # Fallback: try to get port from device list text output
        try {
            $deviceOutput = & $PlatformIO device list 2>&1
            $portMatch = $deviceOutput | Select-String -Pattern 'COM\d+' | Select-Object -First 1
            if ($portMatch) {
                $port = $portMatch.Matches[0].Value
                Write-Host "Found device on: $port" -ForegroundColor Green
                return $port
            }
        } catch {}
    }
    
    Write-Host "Warning: Could not auto-detect port, PlatformIO will try to find it" -ForegroundColor Yellow
    return $null
}

# Function to reset device via PlatformIO
function Reset-Device {
    param([string]$Port)
    Write-Host "Resetting device..." -ForegroundColor Yellow
    try {
        if ($Port) {
            & $PlatformIO run --target reset --environment $Environment --upload-port $Port 2>&1 | Out-Null
        } else {
            & $PlatformIO run --target reset --environment $Environment 2>&1 | Out-Null
        }
        Start-Sleep -Milliseconds 500
        Write-Host "Device reset complete" -ForegroundColor Green
    } catch {
        Write-Host "Note: Reset attempted" -ForegroundColor Gray
    }
}

# Auto-detect port
$Port = Get-DevicePort
if ($Port) {
    Write-Host "Port: $Port | Baud: $Baud" -ForegroundColor Yellow
    # Ensure port is free before proceeding
    Clear-ComPort -Port $Port
} else {
    Write-Host "Port: AUTO | Baud: $Baud" -ForegroundColor Yellow
}
Write-Host "Timeout: $TimeoutSeconds seconds (0 = no timeout)" -ForegroundColor Yellow
Write-Host ""

# Reset device first to trigger boot sequence (unless NoReset is specified)
if (-not $NoReset) {
    Reset-Device -Port $Port
    Start-Sleep -Seconds 1
} else {
    Write-Host "Skipping reset (device already restarted)" -ForegroundColor Gray
    Write-Host ""
}

Write-Host "Starting monitor..." -ForegroundColor Cyan
if ($TimeoutSeconds -gt 0) {
    Write-Host "Will timeout after $TimeoutSeconds seconds if no output" -ForegroundColor Gray
}
Write-Host "Press Ctrl+C to stop" -ForegroundColor Gray
Write-Host ""

# Start monitoring with timeout wrapper
if ($TimeoutSeconds -gt 0) {
    $job = Start-Job -ScriptBlock {
        param($pio, $env, $port, $baud)
        Set-Location $using:PWD
        chcp 65001 | Out-Null
        $env:PYTHONIOENCODING = 'utf-8'
        if ($port) {
            & $pio device monitor --environment $env --port $port --baud $baud 2>&1
        } else {
            & $pio device monitor --environment $env --baud $baud 2>&1
        }
    } -ArgumentList $PlatformIO, $Environment, $Port, $Baud
    
    try {
        $null = Wait-Job -Job $job -Timeout $TimeoutSeconds
        if ($job.State -eq 'Running') {
            Write-Host ""
            Write-Host "Timeout reached - no output detected" -ForegroundColor Yellow
            Write-Host "Stopping monitor..." -ForegroundColor Yellow
            Stop-Job -Job $job -ErrorAction SilentlyContinue
        } else {
            Receive-Job -Job $job | Write-Host
        }
    } catch {
        Write-Host "Monitor interrupted" -ForegroundColor Yellow
    } finally {
        Remove-Job -Job $job -Force -ErrorAction SilentlyContinue
        # Clean up any remaining processes
        Clear-ComPort -Port $Port
    }
} else {
    # No timeout - run directly
    try {
        if ($Port) {
            & $PlatformIO device monitor --environment $Environment --port $Port --baud $Baud
        } else {
            & $PlatformIO device monitor --environment $Environment --baud $Baud
        }
    } finally {
        # Clean up on exit
        Clear-ComPort -Port $Port
    }
}

