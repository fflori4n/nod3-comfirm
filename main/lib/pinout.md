MAGPIE SENSOR BOARD:

CHIP: ESP32 C3 - 
┏━━━━━━━━━━━━━━━━━━━━━┳━━━━━━━━━━━━━━┳━━━━━━━━━━┳━━━━━━━━━━━━━━━━┳━━━━━━━━━━━━━━━┓
┃ Memory Type/Section ┃ Used [bytes] ┃ Used [%] ┃ Remain [bytes] ┃ Total [bytes] ┃
┡━━━━━━━━━━━━━━━━━━━━━╇━━━━━━━━━━━━━━╇━━━━━━━━━━╇━━━━━━━━━━━━━━━━╇━━━━━━━━━━━━━━━┩
│ Flash Code          │       748314 │          │                │               │
│    .text            │       748314 │          │                │               │
│ Flash Data          │       173744 │          │                │               │
│    .rodata          │       173488 │          │                │               │
│    .appdesc         │          256 │          │                │               │
│ DRAM                │       166800 │    51.91 │         154496 │        321296 │
│    .text            │        90284 │     28.1 │                │               │
│    .data            │        53852 │    16.76 │                │               │
│    .bss             │        22664 │     7.05 │                │               │
│ RTC SLOW            │         5752 │    70.21 │           2440 │          8192 │
│    .force_fast      │         5696 │    69.53 │                │               │
│    .force_slow      │           32 │     0.39 │                │               │
│    .rtc_reserved    │           24 │     0.29 │                │               │
└─────────────────────┴──────────────┴──────────┴────────────────┴───────────────┘

## MCU PINS:

- GPIO_NUM_0 - Battery voltage 
    - ADC INPUT - ADC_UNIT_1, ADC_CHANNEL_0
    - Measure voltage of 18650 cell, via voltage divider r1 = 470K, r2= 470k
    - Conv factor: raw/(v * 0.5) = 40960/2.5
    - (raw * 2.5/40960) / 0.5 = v
    - raw * ((2.5/40960) * 2) = v

- GPIO_NUM_1 - Supply voltage 
    - ADC INPUT - ADC_UNIT_1, ADC_CHANNEL_1
    - Measure incoming supply voltage, depending on the populated resistor either incoming unregulated solar voltage or USB 5V charge voltage
    - via voltage divider r1 = 10K, r2= 10k
    - Conv factor: raw/(v * 0.5) = 40960/2.5
    - (raw * 2.5/40960) / 0.5 = v
    - raw * ((2.5/40960) * 2) = v

- GPIO_NUM_2 - Supply voltage 
    - ADC INPUT - ADC_UNIT_1, ADC_CHANNEL_2

- GPIO_NUM_3 - Supply voltage 
    - ADC INPUT - ADC_UNIT_1, ADC_CHANNEL_3

- GPIO_NUM_4 - Supply voltage 
    - ADC INPUT - ADC_UNIT_1, ADC_CHANNEL_4

- GPIO_NUM_5 - Supply voltage 
    - ADC INPUT - ADC_UNIT_1, ADC_CHANNEL_5
    - Measure temperature using 10K NTC resistor
    - ADC to VOLTAGE: (2.5/40960) -> calculate NTC val -> calculate temperature

- GPIO_NUM_6
    - DIGITAL IN - mmWave motion sensor
    - DIGITAL OUT - GREEN LED

- GPIO_NUM_7
    - DIGITAL OUT - power key for board low power mode disable
    - HIGH - turn on 3V3 and VBAT sensor supply
    - LOW - turn on 3V3 and VBAT sensor supply

- GPIO_NUM_8 - I2C BUS SDA
- GPIO_NUM_9 - I2C BUS SCL / USER PUSHBUTTON
- GPIO_NUM_10 - YELLOW LED
    - DIGITAL OUT / reserved for 1Wire
- GPIO_NUM_20 - UART0 RX
- GPIO_NUM_21 - UART0 TX