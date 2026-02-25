// ulp_shared.h
#pragma once
#include <stdint.h>

#define ULP_MAGIC   0x554C5053u
#define ULP_VERSION 1

#define MAX_SENSORS 8
#define MAX_AXES    3
#define RAW_BUF_LEN 16

typedef enum { SENS_NONE=0, SENS_ADC=1, SENS_GPIO=2 } sens_bus_t;
typedef enum { FEAT_NONE=0, FEAT_DELTA=1, FEAT_ENERGY=2 } feat_type_t;
typedef enum { EVT_NONE=0, EVT_SENSOR_TRIG=1 } event_type_t;

typedef struct {
    uint8_t  enabled;
    uint8_t  always_on;
    uint8_t  bus;
    uint8_t  feat;

    uint16_t sample_period_ms;
    uint16_t th1;
    uint16_t window_len;
    uint16_t hit_to_wake;

    uint8_t  ch[MAX_AXES];
    uint8_t  axes;
    uint16_t reserved;
} sens_cfg_t;

typedef struct {
    int16_t  last_raw[MAX_AXES];
    uint32_t acc;
    uint16_t win_cnt;
    uint16_t hit_cnt;
    int16_t  raw_buf[RAW_BUF_LEN][MAX_AXES];
    uint16_t widx;
} sens_rt_t;

typedef struct {
    uint32_t last_event_counter;
    uint8_t  last_event_sensor_id;
    uint8_t  last_event_type;
    uint16_t last_score;
    int16_t  last_raw[MAX_AXES];
    uint32_t sample_counter;
} ulp_report_t;

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint8_t  raw_buf_enable;
    uint8_t  reserved0;

    sens_cfg_t    sensors[MAX_SENSORS];
    sens_rt_t     rt[MAX_SENSORS];
    ulp_report_t  rpt;
} ulp_shared_t;
extern volatile ulp_shared_t ulp_shared;