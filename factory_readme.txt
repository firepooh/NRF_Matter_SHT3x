
1. NCS 환경 설정이 필요하므로 윈도우 파워쉘이 아닌 VSCODE NCS "Open terminal"로 동작 시킴.

2. 터미널에서 아래 명령어로 원하는 factory data 갯수만큼 생성.
명령어 : ./factory_gen.bat 
factory data 생성되는 위치 : c:\ncs\v3.2.1\modules\lib\matter\build

3. 생성된 factory data 중 *.hex, *.png만 프로젝트 폴더 하위 "factorydata" 폴더에 복사
factory data 생성되는 위치 : c:\ncs\v3.2.1\modules\lib\matter\build 
factory data 가져오는 위치 : 프로젝트폴더\factorydata

4. 원하는 factory data을 골라서 실행
./factory_write.bat 1
./factory_write.bat 2
.....
./factory_write.bat 10

5. 컴파일 설정
prj.conf/prj_release.conf
CONFIG_CHIP_FACTORY_DATA=y
CONFIG_CHIP_FACTORY_DATA_BUILD=n
CONFIG_NCS_SAMPLE_MATTER_TEST_EVENT_TRIGGERS_REGISTER_DEFAULTS=n
CONFIG_NCS_SAMPLE_MATTER_TEST_EVENT_TRIGGERS=n

Kconfig.sysbuild
config MATTER_FACTORY_DATA_GENERATE
	default n

sysbuild.conf
# Factory Data 생성 비활성화
SB_CONFIG_MATTER_FACTORY_DATA_GENERATE=n
SB_CONFIG_MATTER_FACTORY_DATA_MERGE_WITH_FIRMWARE=n



기타
- 기본 테스트는 아래 폴더위치에서 파이썬 실행 시켜서 동작 확인 함.
c:\ncs\v3.2.1\modules\lib\matter

python "scripts/tools/nrfconnect/generate_nrfconnect_chip_factory_data.py" `
  --sn "TEMP-SENSOR-001" `
  --vendor_id 65521 `
  --product_id 32774 `
  --vendor_name "My Company" `
  --product_name "Temperature Sensor" `
  --date "2025-01-01" `
  --hw_ver 1 `
  --hw_ver_str "v1.0" `
  --dac_cert "credentials/development/attestation/Matter-Development-DAC-FFF1-8006-Cert.der" `
  --dac_key "credentials/development/attestation/Matter-Development-DAC-FFF1-8006-Key.der" `
  --pai_cert "credentials/development/attestation/Matter-Development-PAI-FFF1-noPID-Cert.der" `
  --spake2_it 1000 `
  --spake2_salt "U1BBS0UyUCBLZXkgU2FsdA==" `
  --discriminator 0xF05 `
  --passcode 20202025 `
  --generate_rd_uid `
  --include_passcode `
  --generate_onboarding `
  --schema "scripts/tools/nrfconnect/nrfconnect_factory_data.schema" `
  --out build/factory5 `
  --offset 0xf7000 `
  --size 0x1000 `
  --overwrite
