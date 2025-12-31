@echo off
setlocal enabledelayedexpansion

:: 설정
set MATTER_PATH=c:\ncs\v3.2.1\modules\lib\matter
::set OUTPUT_PATH=c:\matter_factory_data
set VENDOR_ID=65521
set PRODUCT_ID=32774
set VENDOR_NAME=JYP
set PRODUCT_NAME=TempHumi Sensor
set HW_VER=1
set HW_VER_STR=v1.0
set DATE=2025-12-31
set SPAKE2_IT=1000
set SPAKE2_SALT=U1BBS0UyUCBLZXkgU2FsdA==

:: Enable Key (16바이트 = 32자리 hex)
:: 테스트용 고정 키 또는 디바이스별 고유 키 사용 가능
:: set ENABLE_KEY=00112233445566778899aabbccddeeff
::     --enable_key %ENABLE_KEY% ^

:: 시작 번호와 생성 개수
set START_NUM=1
set COUNT=10

cd /d %MATTER_PATH%

for /L %%i in (%START_NUM%, 1, %COUNT%) do (
  set /a DISC=3840+%%i
  set /a PASS=20202020+%%i
  set SN=TempHumiSensorA-%%i
  
  echo Generating factory data for device %%i...
  
  python scripts/tools/nrfconnect/generate_nrfconnect_chip_factory_data.py ^
    --sn "!SN!" ^
    --vendor_id %VENDOR_ID% ^
    --product_id %PRODUCT_ID% ^
    --vendor_name "%VENDOR_NAME%" ^
    --product_name "%PRODUCT_NAME%" ^
    --date "%DATE%" ^
    --hw_ver %HW_VER% ^
    --hw_ver_str "%HW_VER_STR%" ^
    --dac_cert "credentials/development/attestation/Matter-Development-DAC-FFF1-8006-Cert.der" ^
    --dac_key "credentials/development/attestation/Matter-Development-DAC-FFF1-8006-Key.der" ^
    --pai_cert "credentials/development/attestation/Matter-Development-PAI-FFF1-noPID-Cert.der" ^
    --spake2_it %SPAKE2_IT% ^
    --spake2_salt "%SPAKE2_SALT%" ^
    --discriminator !DISC! ^
    --passcode !PASS! ^
    --generate_rd_uid ^
    --include_passcode ^
    --generate_onboarding ^
    --schema "scripts/tools/nrfconnect/nrfconnect_factory_data.schema" ^
    --out "build/device_%%i" ^
    --offset 0xf7000 ^
    --size 0x1000 ^
    --overwrite
  
  echo Device %%i completed.
  echo.
)

echo All factory data generated!

REM ### 실행
REM PS F:\NRF\NRF_Matter_SHT3x> ./factory_gen.bat
REM Generating factory data for device 1...
REM [WARNING] Cannot find rotating device UID in provided arguments list. A new one will be generated.
REM [INFO] 

REM The new rotate device UID: 60ef3b97a8a28e3825dc0e3eb6c94b85

REM [INFO] Generating SPAKE2+ Verifier...
REM [WARNING] KEY password has not been provided. It means that DAC key is not encrypted.
REM [INFO] Validating JSON with schema...
REM [INFO] Validate OK
REM Device 1 completed.


REM ### 6.2 생성된 디바이스별 폴더 구조
REM device_1.bin
REM device_1.hex
REM device_1.png
REM device_1.txt
REM device_1.json

