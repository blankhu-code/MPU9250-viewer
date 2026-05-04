# ESP-IDF Submodule Fix and Build Script
# Usage: Run .\fix_submodules.ps1 in PowerShell

$ErrorActionPreference = "Continue"
$ESP_IDF_PATH = "D:\32\.espressif\v6.0.1\esp-idf"
$PROJECT_PATH = "D:\TAI\ESP32\hello_world"

Write-Host "========================================" -ForegroundColor Cyan
Write-Host "ESP-IDF Submodule Fix and Build" -ForegroundColor Cyan
Write-Host "========================================" -ForegroundColor Cyan
Write-Host ""

# Step 1: Enter ESP-IDF directory
Write-Host "[Step 1/4] Entering ESP-IDF directory..." -ForegroundColor Yellow
Set-Location $ESP_IDF_PATH

# Step 2: Initialize key submodules for ESP32-S3
Write-Host ""
Write-Host "[Step 2/4] Initializing key submodules..." -ForegroundColor Yellow

$submodules = @(
    "components/bootloader/subproject/components/micro-ecc/micro-ecc",
    "components/mbedtls/mbedtls",
    "components/lwip/lwip",
    "components/esp_wifi/lib",
    "components/esp_phy/lib",
    "components/heap/tlsf",
    "components/unity/unity",
    "components/cmock/CMock",
    "components/protobuf-c/protobuf-c",
    "components/openthread/openthread",
    "components/openthread/lib",
    "components/spiffs/spiffs",
    "components/bt/controller/lib_esp32c3_family",
    "components/bt/controller/lib_esp32c5/esp32c5-bt-lib",
    "components/bt/controller/lib_esp32c6/esp32c6-bt-lib",
    "components/bt/controller/lib_esp32h2/esp32h2-bt-lib",
    "components/bt/esp_ble_mesh/lib/lib",
    "components/bt/host/nimble/nimble",
    "components/esp_coex/lib"
)

foreach ($submodule in $submodules) {
    Write-Host "  Initializing: $submodule" -ForegroundColor Gray
    git submodule update --init $submodule
    Write-Host "  Done: $submodule" -ForegroundColor Green
}

Write-Host "  Key submodules initialized!" -ForegroundColor Green

# Step 3: Enter project directory and load environment
Write-Host ""
Write-Host "[Step 3/4] Loading ESP-IDF environment..." -ForegroundColor Yellow
Set-Location $PROJECT_PATH
. D:\TAI\ESP32\setup_env.ps1

# Step 4: Build project
Write-Host ""
Write-Host "[Step 4/4] Building project..." -ForegroundColor Yellow
python D:\32\.espressif\v6.0.1\esp-idf\tools\idf.py set-target esp32s3
if ($LASTEXITCODE -ne 0) {
    Write-Host ""
    Write-Host "Set target failed, trying to clean and rebuild..." -ForegroundColor Yellow
    Remove-Item -Path "build" -Recurse -Force -ErrorAction SilentlyContinue
    python D:\32\.espressif\v6.0.1\esp-idf\tools\idf.py set-target esp32s3
}

Write-Host ""
Write-Host "Starting build..." -ForegroundColor Yellow
python D:\32\.espressif\v6.0.1\esp-idf\tools\idf.py build

if ($LASTEXITCODE -eq 0) {
    Write-Host ""
    Write-Host "========================================" -ForegroundColor Green
    Write-Host "BUILD SUCCESS!" -ForegroundColor Green
    Write-Host "========================================" -ForegroundColor Green
    Write-Host ""
    Write-Host "Firmware location: build\hello_world.bin" -ForegroundColor Cyan
    Write-Host ""
    Write-Host "Next step: Flash to ESP32-S3" -ForegroundColor Yellow
    Write-Host "Command: python D:\32\.espressif\v6.0.1\esp-idf\tools\idf.py -p COM3 flash" -ForegroundColor Cyan
} else {
    Write-Host ""
    Write-Host "========================================" -ForegroundColor Red
    Write-Host "BUILD FAILED - Check error messages above" -ForegroundColor Red
    Write-Host "========================================" -ForegroundColor Red
}
