@echo off
cd /d D:\TAI\ESP32\hello_world
set IDF_PATH=D:\32\.espressif\v6.0.1\esp-idf
set IDF_PYTHON_ENV_PATH=C:\Users\86131\.espressif\python_env\idf6.0_py3.13_env
set ESP_IDF_VERSION=6.0.1
set ESP_ROM_ELF_DIR=C:\Users\86131\.espressif\dist\esp-rom-elf
set PATH=D:\TAI\ESP32\tools\cmake\4.0.3\cmake-4.0.3-windows-x86_64\bin;D:\TAI\ESP32\tools\ninja\1.13.2;C:\Users\86131\.espressif\tools\xtensa-esp-elf\esp-15.2.0_20251204\xtensa-esp-elf\bin;C:\Users\86131\.espressif\tools\riscv32-esp-elf\esp-15.2.0_20251204\riscv32-esp-elf\bin;C:\Users\86131\.espressif\tools\esp32ulp-elf\2.38_20240113\esp32ulp-elf\bin;C:\Users\86131\.espressif\tools\idf-exe\1.0.3;C:\Users\86131\.espressif\python_env\idf6.0_py3.13_env\Scripts;%PATH%
C:\Users\86131\.espressif\python_env\idf6.0_py3.13_env\Scripts\python.exe D:\32\.espressif\v6.0.1\esp-idf\tools\idf.py build
