# Open Kindle Click (ESP32-C6)

Firmware that turns a Seeed Studio XIAO ESP32C6 into a remote that a Kindle accepts as a Kindle Click: page forward and back, light up and down, and sleep. The Kindle also shows the remote's battery level and firmware version.

Hardware
--------

A Seeed Studio XIAO ESP32C6, and four push buttons, each between a pin and GND:

| Pin | GPIO | Button | Key |
|---|---|---|---|
| D0 | GPIO0 | top face | page forward |
| D1 | GPIO1 | bottom face | page back |
| D2 | GPIO2 | upper side | light up (repeats while held) |
| D3 | GPIO21 | lower side | light down (repeats while held) |

- Hold both side buttons for 1 s to put the Kindle to sleep.
- Hold both face buttons for 10 s to forget the Kindle and go back to pairing mode.
- A LiPo cell on the BAT pads is optional (charged via USB Type-C; the board's red charge LED shows charge status). On battery, the remote stops advertising after 5 minutes without its Kindle; any button wakes it.

LED (orange user LED on GPIO15): fast blinks in pairing mode, a short blink every 2 s while waiting for its Kindle, off when connected, and a short flash on each key sent.

Build and flash
---------------

In the Arduino IDE, install the **esp32** boards package by Espressif Systems (version 3.0.0 or later; board manager URL `https://espressif.github.io/arduino-esp32/package_esp32_index.json`). Choose **XIAO_ESP32C6**, open `OpenKindleClick/OpenKindleClick.ino` and upload.

Pairing
-------

On the Kindle, open Settings › Wi-Fi & Bluetooth › Bluetooth devices and choose the remote; there is no code to confirm. Settings › Accessories then shows its battery level and firmware version.

To pair it again, forget it on the Kindle (Bluetooth devices › the remote › Forget), then hold both face buttons for 10 s.

USB serial (115200 baud) takes test commands: `n`/`p` page forward/back, `+`/`-` light, `s` sleep, `i` status, `c` forget the Kindle; `h` lists them all.

Known limits
------------

- The remote can't wake the Kindle; no remote can. After you wake it, a page turn pressed while the remote reconnects is delivered once the Kindle is ready, a few seconds later.
- Kindle firmware 5.20.1 doesn't count remote page turns as activity, so the Kindle still sleeps when its sleep timer runs out after the last screen touch. A longer timer helps (Settings › Device options › Sleep timer).
- Only Kindle models with Kindle Click support accept it.

[PROTOCOL.md](https://github.com/zuyan9/Open-Kindle-Click-nRF52840/blob/main/PROTOCOL.md) describes what the Kindle checks and why the firmware does what it does.

Not affiliated with Amazon. Kindle is a trademark of Amazon.com, Inc. or its affiliates.

## Reference
- [Open-Kindle-Click-nRF52840](https://github.com/zuyan9/Open-Kindle-Click-nRF52840): the same remote for XIAO nRF52840
- [Flipper-Zero-Kindle-Click](https://github.com/zuyan9/Flipper-Zero-Kindle-Click): Flipper Zero app
- [Open-Kindle-Click-Android](https://github.com/zuyan9/Open-Kindle-Click-Android): Android phone as the remote
