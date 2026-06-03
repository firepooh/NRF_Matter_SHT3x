@echo off
setlocal

:: 인자 확인
if "%1"=="" (
  echo 사용법: factory_write.bat [번호]
  echo 예시: factory_write.bat 1
  exit /b 1
)

set DEVICE_NUM=%1
set FACTORY_HEX=factorydata\device_%DEVICE_NUM%.hex

:: 파일 존재 확인
if not exist "%FACTORY_HEX%" (
  echo 에러: %FACTORY_HEX% 파일이 존재하지 않습니다.
  exit /b 1
)

echo ========================================
echo Factory Data Write - Device %DEVICE_NUM%
echo Factory HEX: %FACTORY_HEX%
echo ========================================

:: 1. 전체 삭제
echo [1/4] Erasing chip...
nrfjprog --eraseall
if errorlevel 1 (
  echo 에러: Erase 실패
  exit /b 1
)

:: 2. 펌웨어 플래시
echo [2/4] Flashing firmware...
::west flash --runner jlink
west flash --runner jlink -d build_1
if errorlevel 1 (
  echo 에러: Firmware flash 실패
  exit /b 1
)

:: 3. Factory Data 플래시
echo [3/4] Flashing factory data...
nrfjprog --program "%FACTORY_HEX%" --sectorerase --verify
if errorlevel 1 (
  echo 에러: Factory data flash 실패
  exit /b 1
)

:: 4. 리셋
echo [4/4] Resetting device...
nrfjprog --reset

echo ========================================
echo 완료! Device %DEVICE_NUM% 프로그래밍 성공
echo ========================================

endlocal