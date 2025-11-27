# Flash script for ESP32S3 with new partition table
# Usage: .\flash_device.ps1 COM3
# Replace COM3 with your actual COM port

param(
    [Parameter(Mandatory=$true)]
    [string]$Port
)

$PlatformIO = "C:\Users\phrop\.platformio\penv\Scripts\platformio.exe"
$Environment = "mci_fenrir_esp32s3"

Write-Host "=========================================" -ForegroundColor Cyan
Write-Host "ESP32S3 Flash Procedure" -ForegroundColor Cyan
Write-Host "=========================================" -ForegroundColor Cyan
Write-Host ""

Write-Host "Step 1: Erasing flash completely..." -ForegroundColor Yellow
& $PlatformIO run --target erase --environment $Environment --upload-port $Port
if ($LASTEXITCODE -ne 0) {
    Write-Host "ERROR: Flash erase failed!" -ForegroundColor Red
    exit 1
}

Write-Host ""
Write-Host "Step 2: Building firmware..." -ForegroundColor Yellow
& $PlatformIO run --environment $Environment
if ($LASTEXITCODE -ne 0) {
    Write-Host "ERROR: Build failed!" -ForegroundColor Red
    exit 1
}

Write-Host ""
Write-Host "Step 3: Uploading firmware with new partition table..." -ForegroundColor Yellow
& $PlatformIO run --target upload --environment $Environment --upload-port $Port
if ($LASTEXITCODE -ne 0) {
    Write-Host "ERROR: Upload failed!" -ForegroundColor Red
    exit 1
}

Write-Host ""
Write-Host "=========================================" -ForegroundColor Green
Write-Host "SUCCESS! Device flashed successfully." -ForegroundColor Green
Write-Host "=========================================" -ForegroundColor Green
Write-Host ""
Write-Host "The device should now boot with:" -ForegroundColor Cyan
Write-Host "  - New partition table (1536K OTA partitions)" -ForegroundColor Cyan
Write-Host "  - Coredump partition (128K)" -ForegroundColor Cyan
Write-Host "  - SimpleFS partition (832K) at 'simplefs'" -ForegroundColor Cyan

