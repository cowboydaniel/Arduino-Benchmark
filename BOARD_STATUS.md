# Board Testing Status

Tracks which boards have been benchmarked. Pick a board, run both sketches, save the Serial Monitor output to `reports/<board>-<version>.txt`, and update this file.

Report path convention: `reports/<board_slug>-<core_version>.txt` (e.g. `reports/unor3-1.8.7.txt`). For boards with no versioned core, omit the version suffix (e.g. `reports/multiduino.txt`).

The **Benchmark Version** column records which version of the benchmark suite was used. When the benchmark version changes (defined in `BenchmarkHelpers.h`), boards may need to be re-tested to have comparable results under the new version. Results from different benchmark versions should not be compared directly.

| Board | Core/IDE Version | Benchmark Version | Status | Report |
|-------|-----------------|-------------------|--------|--------|
| Arduino Uno (R3) | Core: 1.8.7/IDE: 2.3.7 | 1.0.0 | passed | reports/unor3-1.8.7.txt |
| Arduino Uno R4 WiFi | Core: 1.5.2/IDE: 2.3.7 | 1.0.0 | passed | reports/unor4wifi-1.5.2.txt |
| Arduino Uno R4 Minima | Core: 1.5.2/IDE: 2.3.7 | 1.0.0 | passed | reports/unor4minima-1.5.2.txt |
| Arduino Uno Mini LE | Core: 1.8.7/IDE: 2.3.7 | 1.0.0 | passed | reports/unominile-1.8.7.txt |
| Arduino Uno WiFi | - | - | - | - |
| Arduino Uno WiFi Rev2 | - | - | - | - |
| Arduino Uno Q (MCU) | Core: 0.53.1/IDE: 2.3.7 | 1.0.0 | passed | reports/unoq-0.53.1.txt |
| Arduino Nano | - | - | - | - |
| Arduino Nano Every | - | - | - | - |
| Arduino Nano 33 IoT | - | - | - | - |
| Arduino Nano 33 BLE | - | - | - | - |
| Arduino Nano 33 BLE Sense | - | - | - | - |
| Arduino Nano 33 BLE Sense Rev2 | - | - | - | - |
| Arduino Nano ESP32 | Core: 3.3.7/IDE: 2.3.7 | 1.0.0 | passed | reports/nanoesp32-3.3.7.txt |
| Arduino Nano RP2040 Connect | - | - | - | - |
| Arduino Nano Matter | - | - | - | - |
| Arduino Mega 2560 | - | - | - | - |
| Arduino Mega 2560 Rev3 | Core: 1.5.7/IDE: 2.3.7 | 1.0.0 | passed | reports/mega2560r3-1.5.7.txt |
| Arduino Mega ADK | - | - | - | - |
| Arduino Mega (original) | - | - | - | - |
| Arduino Leonardo | - | - | - | - |
| Arduino Leonardo ETH | - | - | - | - |
| Arduino Micro | - | - | - | - |
| Arduino Due | - | - | - | - |
| Arduino Zero | - | - | - | - |
| Arduino Pro Mini 3.3V | - | - | - | - |
| Arduino Pro Mini 5V | - | - | - | - |
| Arduino Mini | - | - | - | - |
| Arduino Fio | - | - | - | - |
| Arduino BT | - | - | - | - |
| Arduino Ethernet | - | - | - | - |
| Arduino Esplora | - | - | - | - |
| Arduino Robot Control | - | - | - | - |
| Arduino Robot Motor | - | - | - | - |
| Arduino Yun | Core: 1.8.7/IDE: 2.3.7 | 1.0.0 | passed | reports/yun-1.8.7.txt |
| Arduino Yun Rev 2 | - | - | - | - |
| Arduino Tian | - | - | - | - |
| Arduino Industrial 101 | - | - | - | - |
| Arduino 101 (Intel Curie) | Core: 2.0.6/IDE: 2.3.7 | 1.0.0 | passed | reports/arduino101-2.0.6.txt |
| Arduino Diecimila | - | - | - | - |
| Arduino Duemilanove | - | - | - | - |
| Arduino NG | - | - | - | - |
| Arduino MKR1000 | - | - | - | - |
| Arduino MKR Zero | - | - | - | - |
| Arduino MKR WiFi 1010 | - | - | - | - |
| Arduino MKR FOX 1200 | - | - | - | - |
| Arduino MKR WAN 1300 | - | - | - | - |
| Arduino MKR WAN 1310 | - | - | - | - |
| Arduino MKR GSM 1400 | - | - | - | - |
| Arduino MKR NB 1500 | - | - | - | - |
| Arduino MKR Vidor 4000 | - | - | - | - |
| Arduino MKR Motor Carrier | - | - | - | - |
| Arduino Portenta H7 | - | - | - | - |
| Arduino Portenta H7 Lite | - | - | - | - |
| Arduino Portenta H7 Lite Connected | - | - | - | - |
| Arduino Portenta C33 | - | - | - | - |
| Arduino Portenta X8 | - | - | - | - |
| Arduino Giga R1 WiFi | - | - | - | - |
| Arduino Nicla Sense ME | - | - | - | - |
| Arduino Nicla Vision | - | - | - | - |
| Arduino Nicla Voice | - | - | - | - |
| Arduino Edge Control | - | - | - | - |
| Arduino Opta | - | - | - | - |
| Arduino Opta Lite | - | - | - | - |
| Arduino Opta RS485 | - | - | - | - |
| Arduino Alvik | - | - | - | - |
| Arduino Primo | - | - | - | - |
| Arduino Star Otto | - | - | - | - |
| LilyPad Arduino | - | - | - | - |
| LilyPad Arduino USB | - | - | - | - |
| LilyPad Arduino SimpleSnap | - | - | - | - |
| ESP32-Dev Module | Core: 3.3.7/IDE: 2.3.7 | 1.0.0 | passed | reports/esp32devmodule-3.3.7.txt |
| ESP32-DevKitC V4 (WROOM-32) | - | - | - | - |
| ESP32-WROVER DevKit | - | - | - | - |
| ESP32-PICO-D4 | - | - | - | - |
| ESP32-S2-DevKitM-1 | - | - | - | - |
| ESP32-S2-Saola-1 | - | - | - | - |
| ESP32-S3-DevKitC-1 | - | - | - | - |
| ESP32-S3-DevKitM-1 | - | - | - | - |
| ESP32-C3-Dev-Module | Core: 3.3.7/IDE: 2.3.7 | 1.0.0 | passed | reports/esp32c3devmodule-3.3.7.txt |
| ESP32-C3-DevKitM-1 | - | - | - | - |
| ESP32-C3-DevKitC-02 | - | - | - | - |
| ESP32-C6-DevKitC-1 | - | - | - | - |
| ESP32-H2-DevKitM-1 | - | - | - | - |
| ESP32-C5-DevKitM-1 | - | - | - | - |
| AI Thinker ESP32-CAM | - | - | - | - |
| AI Thinker NodeMCU-32S | - | - | - | - |
| NodeMCU ESP-32S | - | - | - | - |
| ESP8266 NodeMCU v2 | - | - | - | - |
| ESP8266 NodeMCU v3 | - | - | - | - |
| ESP8266 ESP-01 | - | - | - | - |
| ESP8266 ESP-01S | - | - | - | - |
| ESP8266 ESP-07 | - | - | - | - |
| ESP8266 ESP-12E | - | - | - | - |
| ESP8266 ESP-12F | - | - | - | - |
| Wemos D1 | - | - | - | - |
| Wemos D1 Mini | - | - | - | - |
| Wemos D1 Mini Pro | - | - | - | - |
| Wemos D1 R32 | - | - | - | - |
| Lolin D32 | - | - | - | - |
| Lolin D32 Pro | - | - | - | - |
| Lolin S2 Mini | - | - | - | - |
| Lolin S2 Pico | - | - | - | - |
| Lolin S3 | - | - | - | - |
| Lolin S3 Mini | - | - | - | - |
| Lolin S3 Pro | - | - | - | - |
| Lolin C3 Mini | - | - | - | - |
| Lolin C3 Pico | - | - | - | - |
| LILYGO T-Display | - | - | - | - |
| LILYGO T-Display S3 | - | - | - | - |
| LILYGO T-Display S3 AMOLED | - | - | - | - |
| LILYGO T-Beam | - | - | - | - |
| LILYGO T-Beam Supreme | - | - | - | - |
| LILYGO T-Deck | - | - | - | - |
| LILYGO T-Watch | - | - | - | - |
| LILYGO T-Watch S3 | - | - | - | - |
| LILYGO T-Dongle S3 | - | - | - | - |
| LILYGO T-Camera | - | - | - | - |
| LILYGO T-ETH-Lite | - | - | - | - |
| LILYGO T-QT | - | - | - | - |
| LILYGO T-PicoC3 | - | - | - | - |
| Heltec WiFi LoRa 32 V2 | - | - | - | - |
| Heltec WiFi LoRa 32 V3 | - | - | - | - |
| Heltec WiFi Kit 32 | - | - | - | - |
| Heltec WiFi Kit 32 V3 | - | - | - | - |
| Heltec Wireless Stick | - | - | - | - |
| Heltec Wireless Stick Lite | - | - | - | - |
| Heltec Wireless Stick Lite V3 | - | - | - | - |
| Heltec Wireless Tracker | - | - | - | - |
| Heltec Wireless Paper | - | - | - | - |
| Heltec CubeCell HTCC-AB01 | - | - | - | - |
| Heltec CubeCell HTCC-AB02 | - | - | - | - |
| Heltec Vision Master E213 | - | - | - | - |
| Heltec Vision Master E290 | - | - | - | - |
| M5Stack Core (Basic) | - | - | - | - |
| M5Stack Core2 | - | - | - | - |
| M5Stack Core2 AWS | - | - | - | - |
| M5Stack CoreS3 | - | - | - | - |
| M5Stack CoreS3 SE | - | - | - | - |
| M5Stack Fire | - | - | - | - |
| M5StickC | - | - | - | - |
| M5StickC Plus | - | - | - | - |
| M5StickC Plus2 | - | - | - | - |
| M5Atom Lite | - | - | - | - |
| M5Atom Matrix | - | - | - | - |
| M5Atom S3 | - | - | - | - |
| M5Atom S3 Lite | - | - | - | - |
| M5Atom S3R | - | - | - | - |
| M5Stamp Pico | - | - | - | - |
| M5Stamp S3 | - | - | - | - |
| M5Stamp C3 | - | - | - | - |
| M5Paper | - | - | - | - |
| M5NanoC6 | - | - | - | - |
| M5Cardputer | - | - | - | - |
| M5Dial | - | - | - | - |
| M5DinMeter | - | - | - | - |
| M5Capsule | - | - | - | - |
| M5Station | - | - | - | - |
| DFRobot Beetle | - | - | - | - |
| DFRobot Beetle ESP32 | - | - | - | - |
| DFRobot Beetle ESP32-C3 | - | - | - | - |
| DFRobot Beetle ESP32-C6 | - | - | - | - |
| DFRobot FireBeetle 2 ESP32-E | - | - | - | - |
| DFRobot FireBeetle 2 ESP32-S3 | - | - | - | - |
| DFRobot Romeo | - | - | - | - |
| DFRobot Romeo BLE | - | - | - | - |
| DFRobot Bluno | - | - | - | - |
| DFRobot Bluno Mega 2560 | - | - | - | - |
| DFRobot Bluno Nano | - | - | - | - |
| Raspberry Pi Pico | - | - | - | - |
| Raspberry Pi Pico W | - | - | - | - |
| Raspberry Pi Pico 2 (RP2350) | - | - | - | - |
| Raspberry Pi Pico 2 W | - | - | - | - |
| Waveshare RP2040-Zero | - | - | - | - |
| Waveshare RP2040-Plus | - | - | - | - |
| Waveshare RP2040-One | - | - | - | - |
| Waveshare RP2040-LCD-0.96 | - | - | - | - |
| Waveshare RP2040-Tiny | - | - | - | - |
| Waveshare ESP32-S3-Zero | - | - | - | - |
| Waveshare ESP32-S3-Pico | - | - | - | - |
| Pimoroni Tiny 2040 | - | - | - | - |
| Pimoroni Pico LiPo | - | - | - | - |
| Pimoroni Badger 2040 | - | - | - | - |
| Pimoroni Badger 2040 W | - | - | - | - |
| Pimoroni Plasma 2040 | - | - | - | - |
| Pimoroni Plasma 2040 W | - | - | - | - |
| Teensy 4.1 | - | - | - | - |
| Teensy 4.0 | - | - | - | - |
| Teensy MicroMod | - | - | - | - |
| Teensy 3.6 | - | - | - | - |
| Teensy 3.5 | - | - | - | - |
| Teensy 3.2 | - | - | - | - |
| Teensy 3.1 | - | - | - | - |
| Teensy LC | - | - | - | - |
| Teensy 2.0 | - | - | - | - |
| Teensy++ 2.0 | - | - | - | - |
| STM32 Blue Pill (F103C8) | - | - | - | - |
| STM32 Black Pill (F401CC) | - | - | - | - |
| STM32 Black Pill (F411CE) | - | - | - | - |
| STM32 Nucleo-F030R8 | - | - | - | - |
| STM32 Nucleo-F072RB | - | - | - | - |
| STM32 Nucleo-F091RC | - | - | - | - |
| STM32 Nucleo-F103RB | - | - | - | - |
| STM32 Nucleo-F302R8 | - | - | - | - |
| STM32 Nucleo-F303RE | - | - | - | - |
| STM32 Nucleo-F334R8 | - | - | - | - |
| STM32 Nucleo-F401RE | - | - | - | - |
| STM32 Nucleo-F411RE | - | - | - | - |
| STM32 Nucleo-F446RE | - | - | - | - |
| STM32 Nucleo-F746ZG | - | - | - | - |
| STM32 Nucleo-F767ZI | - | - | - | - |
| STM32 Nucleo-H743ZI | - | - | - | - |
| STM32 Nucleo-H7A3ZI-Q | - | - | - | - |
| STM32 Nucleo-L053R8 | - | - | - | - |
| STM32 Nucleo-L152RE | - | - | - | - |
| STM32 Nucleo-L432KC | - | - | - | - |
| STM32 Nucleo-L476RG | - | - | - | - |
| STM32 Nucleo-L496ZG | - | - | - | - |
| STM32 Nucleo-G071RB | - | - | - | - |
| STM32 Nucleo-G431RB | - | - | - | - |
| STM32 Nucleo-G474RE | - | - | - | - |
| STM32 Nucleo-U575ZI-Q | - | - | - | - |
| STM32 Nucleo-WB55RG | - | - | - | - |
| STM32 Nucleo-WL55JC | - | - | - | - |
| STM32 Discovery F407VG | - | - | - | - |
| STM32 Discovery F429ZI | - | - | - | - |
| STM32 Discovery F469NI | - | - | - | - |
| STM32 Discovery F746NG | - | - | - | - |
| STM32 Discovery F769NI | - | - | - | - |
| STM32 Discovery L475VG-IOT01A | - | - | - | - |
| LeafLabs Maple | - | - | - | - |
| LeafLabs Maple Mini | - | - | - | - |
| Adafruit Feather 32u4 Basic | - | - | - | - |
| Adafruit Feather 32u4 Bluefruit LE | - | - | - | - |
| Adafruit Feather 32u4 Adalogger | - | - | - | - |
| Adafruit Feather 328P | - | - | - | - |
| Adafruit Feather M0 Basic | - | - | - | - |
| Adafruit Feather M0 Express | - | - | - | - |
| Adafruit Feather M0 WiFi | - | - | - | - |
| Adafruit Feather M0 Bluefruit LE | - | - | - | - |
| Adafruit Feather M0 Adalogger | - | - | - | - |
| Adafruit Feather M0 RFM69 | - | - | - | - |
| Adafruit Feather M0 RFM96 LoRa | - | - | - | - |
| Adafruit Feather M4 Express | - | - | - | - |
| Adafruit Feather M4 CAN | - | - | - | - |
| Adafruit Feather nRF52832 | - | - | - | - |
| Adafruit Feather nRF52840 Express | - | - | - | - |
| Adafruit Feather nRF52840 Sense | - | - | - | - |
| Adafruit Feather RP2040 | - | - | - | - |
| Adafruit Feather RP2040 Scorpio | - | - | - | - |
| Adafruit Feather RP2040 DVI | - | - | - | - |
| Adafruit Feather RP2040 RFM | - | - | - | - |
| Adafruit Feather ESP32 V2 | - | - | - | - |
| Adafruit Feather ESP32-S2 | - | - | - | - |
| Adafruit Feather ESP32-S2 TFT | - | - | - | - |
| Adafruit Feather ESP32-S3 | - | - | - | - |
| Adafruit Feather ESP32-S3 TFT | - | - | - | - |
| Adafruit Feather ESP32-S3 Reverse TFT | - | - | - | - |
| Adafruit Metro M0 Express | - | - | - | - |
| Adafruit Metro M4 Express | - | - | - | - |
| Adafruit Metro M4 Airlift Lite | - | - | - | - |
| Adafruit Metro ESP32-S2 | - | - | - | - |
| Adafruit Metro ESP32-S3 | - | - | - | - |
| Adafruit Metro RP2040 | - | - | - | - |
| Adafruit Grand Central M4 | - | - | - | - |
| Adafruit ItsyBitsy 32u4 (3V) | - | - | - | - |
| Adafruit ItsyBitsy 32u4 (5V) | - | - | - | - |
| Adafruit ItsyBitsy M0 Express | - | - | - | - |
| Adafruit ItsyBitsy M4 Express | - | - | - | - |
| Adafruit ItsyBitsy nRF52840 | - | - | - | - |
| Adafruit ItsyBitsy RP2040 | - | - | - | - |
| Adafruit QT Py (SAMD21) | - | - | - | - |
| Adafruit QT Py RP2040 | - | - | - | - |
| Adafruit QT Py ESP32-S2 | - | - | - | - |
| Adafruit QT Py ESP32-C3 | - | - | - | - |
| Adafruit QT Py ESP32-S3 | - | - | - | - |
| Adafruit Trinket M0 | - | - | - | - |
| Adafruit Gemma M0 | - | - | - | - |
| Adafruit Flora | - | - | - | - |
| Adafruit Circuit Playground Express | - | - | - | - |
| Adafruit Circuit Playground Bluefruit | - | - | - | - |
| Adafruit CLUE nRF52840 | - | - | - | - |
| Adafruit FunHouse | - | - | - | - |
| Adafruit MagTag | - | - | - | - |
| Adafruit PyPortal | - | - | - | - |
| Adafruit PyPortal Titano | - | - | - | - |
| Adafruit HalloWing M0 | - | - | - | - |
| Adafruit HalloWing M4 | - | - | - | - |
| Adafruit MatrixPortal M4 | - | - | - | - |
| Adafruit MatrixPortal S3 | - | - | - | - |
| Adafruit MacroPad RP2040 | - | - | - | - |
| Adafruit KB2040 | - | - | - | - |
| Adafruit Trinkey QT2040 | - | - | - | - |
| Seeeduino XIAO (SAMD21) | - | - | - | - |
| Seeed XIAO RP2040 | - | - | - | - |
| Seeed XIAO nRF52840 | - | - | - | - |
| Seeed XIAO nRF52840 Sense | - | - | - | - |
| Seeed XIAO ESP32-C3 | - | - | - | - |
| Seeed XIAO ESP32-C6 | - | - | - | - |
| Seeed XIAO ESP32-S3 | - | - | - | - |
| Seeed XIAO ESP32-S3 Sense | - | - | - | - |
| Seeed XIAO RA4M1 | - | - | - | - |
| Seeed Wio Terminal | - | - | - | - |
| Seeed Wio Lite | - | - | - | - |
| Seeed Wio-E5 mini | - | - | - | - |
| Seeed Wio Tracker 1110 | - | - | - | - |
| Seeeduino V4.2 | - | - | - | - |
| Seeeduino Mega | - | - | - | - |
| Seeeduino Lotus | - | - | - | - |
| SparkFun Pro Micro 3.3V | - | - | - | - |
| SparkFun Pro Micro 5V | - | - | - | - |
| SparkFun RedBoard | - | - | - | - |
| SparkFun RedBoard Plus | - | - | - | - |
| SparkFun RedBoard Turbo | - | - | - | - |
| SparkFun RedBoard Artemis | - | - | - | - |
| SparkFun RedBoard Artemis Nano | - | - | - | - |
| SparkFun RedBoard Artemis ATP | - | - | - | - |
| SparkFun Thing Plus (ESP32) | - | - | - | - |
| SparkFun Thing Plus (ESP32-S2) | - | - | - | - |
| SparkFun Thing Plus (ESP32-C6) | - | - | - | - |
| SparkFun Thing Plus RP2040 | - | - | - | - |
| SparkFun Thing Plus SAMD51 | - | - | - | - |
| SparkFun Thing Plus nRF9160 | - | - | - | - |
| SparkFun Thing Plus STM32 | - | - | - | - |
| SparkFun MicroMod ESP32 | - | - | - | - |
| SparkFun MicroMod Artemis | - | - | - | - |
| SparkFun MicroMod SAMD51 | - | - | - | - |
| SparkFun MicroMod RP2040 | - | - | - | - |
| SparkFun MicroMod nRF52840 | - | - | - | - |
| SparkFun MicroMod STM32 | - | - | - | - |
| SparkFun SAMD21 Mini | - | - | - | - |
| SparkFun SAMD21 Dev | - | - | - | - |
| SparkFun ESP8266 Thing | - | - | - | - |
| SparkFun ESP8266 Thing Dev | - | - | - | - |
| SparkFun RP2040 MicroMod | - | - | - | - |
| SparkFun Artemis Module | - | - | - | - |
| SparkFun Artemis Nano | - | - | - | - |
| SparkFun Qwiic Pocket Dev Board (ESP32-C6) | - | - | - | - |
| Particle Photon | - | - | - | - |
| Particle Photon 2 | - | - | - | - |
| Particle Electron | - | - | - | - |
| Particle Argon | - | - | - | - |
| Particle Boron | - | - | - | - |
| Particle Xenon | - | - | - | - |
| Particle P2 | - | - | - | - |
| Particle Tracker | - | - | - | - |
| Particle M-SoM | - | - | - | - |
| RAK4631 (nRF52840 + LoRa) | - | - | - | - |
| RAK11200 (ESP32) | - | - | - | - |
| RAK11310 (RP2040 + LoRa) | - | - | - | - |
| RAK3172 (STM32WLE5) | - | - | - | - |
| nRF52840 DK | - | - | - | - |
| nRF52832 DK | - | - | - | - |
| nRF5340 DK | - | - | - | - |
| Olimex ESP32-DevKit-LiPo | - | - | - | - |
| Olimex ESP32-EVB | - | - | - | - |
| Olimex ESP32-POE | - | - | - | - |
| Olimex ESP32-S2-DevKit-LiPo | - | - | - | - |
| Olimex STM32-H103 | - | - | - | - |
| Olimex ESP32-C3-DevKit-LiPo | - | - | - | - |
| Olimex RP2040-PICO-PC | - | - | - | - |
| DOIT ESP32 DevKit V1 | - | - | - | - |
| DOIT ESP32S2 DevKit V1 | - | - | - | - |
| AZ-Delivery ESP32 Dev Kit C V4 | - | - | - | - |
| AZ-Delivery D1 Mini ESP32 | - | - | - | - |
| AZ-Delivery NodeMCU V3 ESP8266 | - | - | - | - |
| Unexpected Maker TinyS2 | - | - | - | - |
| Unexpected Maker TinyS3 | - | - | - | - |
| Unexpected Maker TinyC6 | - | - | - | - |
| Unexpected Maker FeatherS2 | - | - | - | - |
| Unexpected Maker FeatherS3 | - | - | - | - |
| Unexpected Maker ProS3 | - | - | - | - |
| Unexpected Maker NanoS3 | - | - | - | - |
| WeAct Studio Black Pill V2.0 (F411CE) | - | - | - | - |
| WeAct Studio Black Pill V3.1 (H743VI) | - | - | - | - |
| WeAct Studio RP2040 | - | - | - | - |
| WeAct Studio ESP32-C3 | - | - | - | - |
| WeMos LOLIN32 | - | - | - | - |
| WeMos LOLIN32 Lite | - | - | - | - |
| Intel Galileo | - | - | - | - |
| Intel Galileo Gen 2 | - | - | - | - |
| Intel Edison | - | - | - | - |
| chipKIT Uno32 | - | - | - | - |
| chipKIT Max32 | - | - | - | - |
| chipKIT Wi-FIRE | - | - | - | - |
| Industruino D21G | - | - | - | - |
| RobotDyn SAMD21 M0-Mini | - | - | - | - |
| Sanguino (ATmega644P) | - | - | - | - |
| Sanguino (ATmega1284P) | - | - | - | - |
| Moteino (ATmega328P + RFM69) | - | - | - | - |
| Moteino MEGA (ATmega1284P + RFM69) | - | - | - | - |
| Moteino M0 (SAMD21 + RFM69) | - | - | - | - |
| ATtiny85 (Digispark) | - | - | - | - |
| ATtiny84 | - | - | - | - |
| ATtiny88 (MH-Tiny) | - | - | - | - |
| ATtiny1614 | - | - | - | - |
| ATtiny3216 | - | - | - | - |
| ATtiny402 | - | - | - | - |
| ATtiny412 | - | - | - | - |
| ATtiny814 | - | - | - | - |
| ATmega328P (standalone) | - | - | - | - |
| ATmega328PB (standalone) | - | - | - | - |
| ATmega2560 (standalone) | - | - | - | - |
| ATmega32U4 (standalone) | - | - | - | - |
| ATmega1284P (standalone) | - | - | - | - |
| ATmega4809 (standalone) | - | - | - | - |
| AVR128DA28 | - | - | - | - |
| AVR128DA48 | - | - | - | - |
| AVR128DB28 | - | - | - | - |
| AVR128DB48 | - | - | - | - |
| AVR64DD32 | - | - | - | - |
| MegaCoreX (ATmega x09) | - | - | - | - |
| Multiduino | Core: n/a/IDE: 2.3.7 | 1.0.0 | passed | reports/multiduino.txt |
| INK4u Board (AVR128DA28) | Core: 1.5.11/IDE: 2.3.7 | 1.0.0 | passed | reports/ink4u-1.5.11.txt |
| SODAQ Explorer | - | - | - | - |
| SODAQ ONE | - | - | - | - |
| SODAQ SARA | - | - | - | - |
| SODAQ Autonomo | - | - | - | - |
| WIZnet W5100S-EVB-Pico | - | - | - | - |
| WIZnet W5500-EVB-Pico | - | - | - | - |
| Arduino Nano Clone (CH340) | Core: 1.8.7/IDE: 2.3.7 | 1.0.0 | passed | reports/nanoclone-1.8.7.txt |
| Arduino Uno Clone (CH340) | Core: 1.8.7/IDE: 2.3.7 | 1.0.0 | passed | reports/unor3clone-1.8.7.txt |
| Arduino Pro Mini Clone 3.3V | - | - | - | - |
| Arduino Pro Mini Clone 5V | - | - | - | - |
| Arduino Mega Clone (CH340) | - | - | - | - |
