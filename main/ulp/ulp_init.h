#pragma once

/**
 * @brief ULP RISC-V ADC 초기화 및 실행
 *        HW 핀맵: GPIO3(CH2)=TH1, GPIO4(CH3)=TH2, GPIO5/6/7(CH4/5/6)=ADXL X/Y/Z.
 *        (GPIO8/CH7은 SDA_OUT이라 ADC 미사용.) 설정 후 ULP 바이너리 로드.
 */
void start_ulp_adc_gpio4(void);
