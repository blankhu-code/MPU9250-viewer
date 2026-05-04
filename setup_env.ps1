# ESP-IDF Environment Setup Script
$env:IDF_PATH = "D:\32\.espressif\v6.0.1\esp-idf"
$env:ESP_IDF_VERSION = "6.0.1"
$env:IDF_PYTHON_ENV_PATH = "C:\Users\86131\.espressif\python_env\idf6.0_py3.13_env"
$env:IDF_TOOLS_PATH = "C:\Users\86131\.espressif"
$env:PATH = "D:\TAI\ESP32\tools\cmake\4.0.3\cmake-4.0.3-windows-x86_64\bin;D:\TAI\ESP32\tools\ninja\1.13.2;C:\Users\86131\.espressif\python_env\idf6.0_py3.13_env\Scripts;C:\Users\86131\.espressif\tools\xtensa-esp-elf-gdb\16.3_20250913\xtensa-esp-elf-gdb\bin;C:\Users\86131\.espressif\tools\xtensa-esp-elf\esp-15.2.0_20251204\xtensa-esp-elf\bin;C:\Users\86131\.espressif\tools\riscv32-esp-elf\esp-15.2.0_20251204\riscv32-esp-elf\bin;C:\Users\86131\.espressif\tools\esp32ulp-elf\2.38_20240113\esp32ulp-elf\bin;C:\Users\86131\AppData\Local\Programs\Python\Python313;C:\Users\86131\AppData\Local\Programs\Python\Python313\Scripts;$env:PATH"

Write-Host "ESP-IDF environment loaded successfully"
Write-Host "IDF_PATH: $env:IDF_PATH"
Write-Host "ESP_IDF_VERSION: $env:ESP_IDF_VERSION"
