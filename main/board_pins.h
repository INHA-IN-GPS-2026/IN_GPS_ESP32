// board_pins.h — NEW SMART MODULE (rev 4.0) 핀맵 단일 출처.
// 근거: Docs/Schemetic/I2C_init_ver.pdf (SMART SENSOR 2026.04.22, rev 4.0)
//
// 이 파일이 스키매틱 네트명 ↔ GPIO 대응의 유일한 정의다. 핀 번호를 다른 파일에
// 직접 써넣지 말 것 — 아날로그 버전에서 NTC 핀이 README/코드/주석에 각각 다르게
// 적혀 있던 것이 이 파일을 만든 이유다.
#pragma once

/* === 스키매틱 네트 ↔ GPIO ==============================================
   MCU: ESP32-S3-WROOM-1-N16R8 (U5)

   | 네트명     | GPIO | 모듈핀 | 용도                                   |
   |------------|------|--------|----------------------------------------|
   | X_OUT      |  5   |   5    | ADXL335 X  (ADC1_CH4)                  |
   | Y_OUT      |  6   |   6    | ADXL335 Y  (ADC1_CH5)                  |
   | Z_OUT      |  7   |   7    | ADXL335 Z  (ADC1_CH6)                  |
   | SDA_OUT    |  8   |  12    | I2C #1 SDA — BQ35100 배터리 게이지     |
   | SCL_OUT    |  9   |  17    | I2C #1 SCL — BQ35100                   |
   | LBO_OUT    | 10   |  18    | 저전압 표시 입력                       |
   | PGOOD_OUT  | 11   |  19    | power good 입력                        |
   | ALERT_OUT  | 12   |  20    | BQ35100 ALERT 입력                     |
   | SDA_TH     | 13   |  21    | I2C #2 SDA — AS6221 ×2 (J1 터미널)     |
   | SCL_TH     | 14   |  22    | I2C #2 SCL — AS6221 ×2 (J1 터미널)     |
   | ALERT_TH1  | 15   |   8    | AS6221 #1 ALERT (이 버전 미사용)       |
   | ALERT_TH2  | 16   |   9    | AS6221 #2 ALERT (이 버전 미사용)       |
   | CHIP_PU    | EN   |   3    |                                        |
   | U0RXD/TXD  |  —   | 36/37  | 콘솔                                   |
   | GPIO0      |  0   |  27    | 부트 스트랩                            |

   GPIO3 / GPIO4 는 아날로그 버전에서 NTC 분압(TH1/TH2)이 물려 있던 핀이다.
   이 리비전에서는 아무 네트도 붙지 않는다 — ADC로 읽지 말 것.
   === =================================================================== */

/* --- 진동 (ULP SARADC) --- */
#define PIN_ADXL_X        5     /* ADC1_CH4 */
#define PIN_ADXL_Y        6     /* ADC1_CH5 */
#define PIN_ADXL_Z        7     /* ADC1_CH6 */

/* --- 온도 I2C 버스 (AS6221 ×2, J1 CON6) ---
   ⚠ 하드웨어 주의: 스키매틱의 THERMISTOR TERMINAL 블록에는 J1과 3.3V/GND만 있고
     SDA_TH/SCL_TH에 풀업 저항이 없다. (10k 풀업 R9/R10은 BQ35100 버스 전용.)
     AS6221 데이터시트는 SDA/SCL 외부 풀업을 필수로 요구하므로,
     풀업은 센서 모듈 쪽에 있어야 한다. 없다면 ESP32 내부 풀업(~45kΩ)에
     의존하게 되며 — as6221.c에서 켜 두었다 — 배선이 길면 파형이 무너진다.
     그래서 버스 속도를 400kHz가 아니라 100kHz로 잡았다. */
#define PIN_TH_SDA        13
#define PIN_TH_SCL        14
#define PIN_TH_ALERT1     15    /* 이 버전에서는 설정하지 않는다(입력 미사용) */
#define PIN_TH_ALERT2     16

/* --- 배터리 게이지 I2C 버스 (BQ35100) --- 이 버전 미구현 --- */
#define PIN_BAT_SDA       8
#define PIN_BAT_SCL       9
#define PIN_BAT_ALERT     12
#define PIN_BAT_LBO       10
#define PIN_BAT_PGOOD     11

/* --- 기타 --- */
#define PIN_RGB_LED       48    /* WS2812B (개발보드). 부팅 시 소등만 한다. */
