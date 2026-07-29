# ESP32-S3 듀얼코어 태스크 아키텍처 (2026-07-18)

앱 태스크를 Core 1로 분리해 BLE 스택(Core 0)과의 경합을 제거하고,
워치독 모니터를 Core 1에 두어 크로스코어 감시를 구성했다.

- `app_main.c:60` — `adv_cycle` → `xTaskCreatePinnedToCore(..., 1)`
- `wdt_guard.c:217` — `wdt_guard` → `xTaskCreatePinnedToCore(..., 1)`
- `CONFIG_FREERTOS_UNICORE` 미설정(듀얼코어), TWDT `idle_core_mask`는 양코어 감시

```mermaid
flowchart TB
    subgraph ULP["ULP (RTC 코프로세서)"]
        U1["ADC 200Hz 샘플링<br/>sum_ntc / ntc_count 누적"]
    end

    subgraph C0["Core 0 (PRO_CPU)"]
        BT["BT Controller<br/>(CONFIG_BT_CTRL_PINNED_TO_CORE_0)"]
        NH["NimBLE Host task<br/>(CONFIG_BT_NIMBLE_PINNED_TO_CORE=0)"]
        MAIN["main task (부팅)<br/>초기화 후 종료"]
    end

    subgraph C1["Core 1 (APP_CPU)"]
        ADV["adv_cycle (prio 5)<br/>온도변환(Ratiometric+S-H)<br/>광고 페이로드 갱신"]
        WDT["wdt_guard (prio 6)<br/>헬스모니터 / heartbeat 감시"]
    end

    U1 -->|"raw 누적값 (RTC 공유메모리)"| ADV
    MAIN -->|"on_sync → 태스크 생성"| ADV
    MAIN -->|"wdt_guard_init"| WDT
    ADV -->|"adv 데이터 갱신 API"| NH
    NH <--> BT
    BT -->|"BLE 광고 1s 주기<br/>(주기 변경 예정)"| GW["STM32 게이트웨이"]

    ADV -.->|"WDT_HB_ADV heartbeat"| WDT
    WDT -.->|"Core 0 정지 시에도 생존<br/>heartbeat 미갱신 → 재부팅"| C0

    subgraph TWDT["Task WDT (5s, panic→reboot)"]
        T1["idle_core_mask = Core0 | Core1<br/>양코어 idle 기아 감시"]
    end
    TWDT -.-> C0
    TWDT -.-> C1
```

## 감시 계층 요약

| 계층 | 대상 | 동작 |
|---|---|---|
| wdt_guard (Core 1, prio 6) | ADV heartbeat 15s | 미갱신 시 재부팅 |
| Task WDT 5s | 양코어 idle 기아 | panic → 재부팅 |
| 크로스코어 | Core 0(BLE 스택) 전체 정지 | Core 1 모니터 생존 → 감지 |

우선순위: Core 1 안에서 wdt_guard(6) > adv_cycle(5) — adv가 spin해도 감시 유지.
