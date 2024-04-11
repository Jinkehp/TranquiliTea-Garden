# TranquiliTea Garden
This is a product created by Tea(m) Pot. This code is used to support our product "The TranquiliTea Garden". This is a product that is created for the course "Interaction Technology Innovation" at Utrecht University, as part of the Human-Computer Interaction Master's programme.

The product is an interactive Zen garden around the tea drinking experience. It aims to stimulate the user to create a conscious moment of relaxation and refreshment.

Simply turn it on, scan your chosen tea flavour, and put the teabag, hot water and temperature sensor in your cup or teapot. Afterwards, start interacting with the garden whilst waiting for the tea to be ready. Immerse yourself in the ambient lights and music, matching the tea flavour, and experience a relaxing cup of tea.

#### Note
The mist module is optional. By default it is not used and its code is commented out.


## Hardware
  - (OPTIONAL) Mist module: 2x;
    https://www.tinytronics.nl/en/mechanics-and-actuators/others/ultrasonic-mist-module
  - Relay: 1x;
    https://www.tinytronics.nl/en/switches/relays/5v-relay-1-channel-high-active-or-low-active
  - Water temperature sensor: 1x;
    https://www.tinytronics.nl/en/sensors/temperature/ds18b20-to-92-thermometer-temperature-sensor-with-cable-waterproof-high-temperature-1m
  - 4.7K Ohm Resistor: 1x;
  - RFID/NFC reader/scanner: 1x;
    https://www.tinytronics.nl/en/communication-and-signals/wireless/rfid/rfid-nfc-kit-pn532-with-s50-card-and-s50-key-tag
  - RFID/NFC Sticker Tags: 4x;
    https://hackerstore.nl/Artikel/1243
  - RGB LED strip: ~1m (60 LEDs);
    https://www.tinytronics.nl/en/lighting/led-strips/led-strips/ws2812b-digital-5050-rgb-led-strip-60-leds-1m
  - 1000uF capacitor: 1x;
    https://www.tinytronics.nl/en/components/capacitors/1000uf-16v-elektrolytic-capacitor
  - 390 Ohm Resistor: 1x;
  - MP3 player module: 1x;
    https://www.tinytronics.nl/en/audio/audio-sources/mini-mp3-module-yx5200
  - Mirco-SD (2GB): 1x;
  - Speakers: 1x (2 speakers);
    https://www.tinytronics.nl/en/audio/speakers/speakers/speaker-set-8%CF%89-2w-with-jst-ph-connector
  - 4700uF capacitor: 1x;
    https://www.tinytronics.nl/en/components/capacitors/4700uf-16v-elektrolytic-capacitor
  - Switch: 1x;
    https://www.tinytronics.nl/en/switches/manual-switches/rocker-switches/standaard-built-in-rocker-switch-normal
  - RGB LEDs common anode: 2x;
    https://hackerstore.nl/Artikel/697
  - 270 Ohm Resistor: 6x;
  - Esp32-S3-DevKitC-1: 1x;
    https://www.tinytronics.nl/en/development-boards/microcontroller-boards/with-wi-fi/espressif-esp32-c6-devkitc-1
  - Jumper wires
  - USB-A to Micro-USB cable: 1x;


## Libraries
  - ezButton
  - Wire
  - PN532_I2C
  - PN532
  - NfcAdapter
  - DallasTemperature
  - OneWire.h>
  - DFRobotDFPlayerMini
  - HardwareSerial
  - Adafruit_NeoPixel


## Used tutorials and guides
  - https://arduinogetstarted.com/tutorials/arduino-switch?utm_content=cmp-true
  - https://warlord0blog.wordpress.com/2021/10/09/esp32-and-nfc-over-i2c/
  - https://www.elechouse.com/elechouse/images/product/PN532_module_V3/PN532_%20Manual_V3.pdf
  - https://randomnerdtutorials.com/esp32-ds18b20-temperature-arduino-ide/
  - https://github.com/DKARDU/automatic-humidity/blob/master/auto_humid.ino
  - https://wiki.dfrobot.com/DFPlayer_Mini_SKU_DFR0299
  - https://www.hackster.io/munir03125344286/df-player-mini-interface-with-esp32-f1efca
  - https://github.com/pcbreflux/espressif/blob/master/esp32/arduino/sketchbook/ESP32_DFPlayer_full/ESP32_DFPlayer_full.ino
  - https://learn.adafruit.com/adafruit-neopixel-uberguide/basic-connections
  - https://learn.adafruit.com/adafruit-neopixel-uberguide/arduino-library-installation
  - https://www.tweaking4all.com/hardware/arduino/adruino-led-strip-effects/
      - Fade in/out
      - Twinkle
      - Snow sparkle
