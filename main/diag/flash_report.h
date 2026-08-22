#pragma once

/* 플래시 용량/사용량 리포트 (진단 전용, branch flash_info_check)

   왜 필요한가:
     esptool의 flash_id는 "칩이 몇 MB인지"만 알려준다. 정작 알고 싶은
     "이 펌웨어가 그중 얼마를 쓰고 있는가"(app 이미지 실크기, 파티션 여유,
     NVS 사용률, 그리고 sdkconfig 설정값이 물리 크기와 맞는가)는
     런타임에 칩 위에서만 확정할 수 있다.

   ⚠ 출력은 ESP_LOGx가 아니라 printf를 쓴다 — app_main이
     esp_log_level_set("*", ESP_LOG_NONE)으로 전역 묵음을 걸기 때문에
     ESP_LOGx로 찍으면 아무것도 보이지 않는다.

   ⚠ 전류 실측 빌드에서는 반드시 INGPS_FLASH_REPORT=0으로 되돌릴 것.
     부팅 1회 출력이지만 UART 송신이 CPU를 깨우고 PM 락을 잡는다. */
void flash_report_print(void);
