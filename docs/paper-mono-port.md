# Paper Mono port

This tree adds an `paper_mono` PlatformIO target for the M5Stack Paper Mono
hardware used by PaperToDo. The port keeps the CrossPoint Reader application
and adapts FreeInk at the board, input, storage, RTC, battery, and display
boundaries.

## Build and flash

```sh
pio run -e paper_mono
pio run -e paper_mono -t upload
pio device monitor -b 115200
```

The target is selected as `default_envs`, uses the ESP32-S3 N16R8 profile,
16 MB QIO flash, and 8 MB octal PSRAM.

## Hardware mapping

| Function | Paper Mono connection |
| --- | --- |
| EPD | SSD1683, 800 x 480 |
| EPD SPI | SCK 15, MOSI 14, CS 16, DC 17, BUSY 18 |
| I2C | SDA 47, SCL 48, 100 kHz |
| EPD power/reset | M5IOE1 at `0x6f`, IO3 / IO5 |
| Power monitor | M5PM1 at `0x6e` |
| RTC | RX8130CE at `0x32` |
| Previous / up | GPIO 2, active low |
| Next / down | GPIO 3, active low |
| SDMMC | CLK 13, CMD 12, D0..D3 11,10,9,8 |
| SD power | M5IOE1 IO14 |

The Paper Mono profile applies the panel's 180-degree mount transform while
streaming each SSD1683 RAM plane. The controller itself remains in PaperToDo's
validated native X-/Y+ scan mode, avoiding the clipped half-screen writes seen
when both SSD1683 scan-direction registers are reversed. The transform is shared
by full, partial, cleanup, and four-gray refreshes.

Button translation preserves the actions expected by CrossPoint Reader:

- short GPIO 2: up / previous
- short GPIO 3: down / next
- hold GPIO 2: back
- hold GPIO 3: confirm
- hold both: power / sleep

The two on-screen hints are rendered as a compact rail at the upper-left edge,
next to the physical keys. Each hint shows the short-press action first and the
hold action second; Paper Mono does not reserve an unused four-button footer.

Either button wakes the device from deep sleep. EPD and SD power are disabled
before sleep. The first display after every boot or deep-sleep wake performs one
a direct non-flashing differential update to establish both controller RAM planes.
All later updates in that wake session use non-flashing differential paths.

## Non-flashing four-gray refresh

`Ssd1683Driver` ports PaperToDo's validated g4ff pipeline. CrossPoint's two
one-bit grayscale planes are converted into four physical levels, then updated
with:

1. a differential black/white base pass, forcing old gray pixels back to a
   known endpoint;
2. a custom gray adjustment pass;
3. zero to three optional cleanup passes.

The default is scheme B with one cleanup pass, one base pass, and three dark
and light adjustment frames. The custom update trigger is `0xcf`; bits `0x10`
and `0x20` remain clear so OTP waveforms cannot replace the injected LUT.

Runtime tuning is exposed by `Ssd1683GrayParams` and these FreeInk helpers:

```cpp
freeink::ssd1683SetGrayParams(params);
freeink::ssd1683SetFastGrayBase(true);
freeink::ssd1683AbortGray();
freeink::ssd1683ResetGray();
```

Grayscale page changes use the injected partial waveform and do not perform a
full-screen black/white flash. A normal black-and-white display path invalidates
the gray state so the next grayscale page rebuilds it safely. CrossPoint's
periodic `HALF_REFRESH` request is mapped to a differential partial followed by
the cleanup LUT; it removes accumulated ghosting without an inversion flash.
Normal UI partial refreshes also apply the cleanup LUT only to pixels changing
from black to white, preventing stale labels from overlapping returned screens
without re-driving the unchanged background. Four-gray pages skip that extra
pass because their g4ff pipeline already ends with its own cleanup.
