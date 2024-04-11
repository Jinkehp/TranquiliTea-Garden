/*
 * ZenGarden.ino
 * Switch test:
 * - https://arduinogetstarted.com/tutorials/arduino-switch?utm_content=cmp-true
 * - The switch is connected to pin 21 with its pin under the 0.
 * - The switch is connected to the ground with its pin under the 1.
 * - Turn OFF: Reset the ESP32-S3-DevKitC-1.

 * PN532 NFC test:
 * - https://warlord0blog.wordpress.com/2021/10/09/esp32-and-nfc-over-i2c/
 * - https://www.elechouse.com/elechouse/images/product/PN532_module_V3/PN532_%20Manual_V3.pdf
 * - I2C mode
 *     - Make sure to configure the ON and KE switches on the NFC PN532 board.
 *     - Switch channel 1 to ON, instead of to 1.
 *     - Switch channel 2 to 2, instead of to KE.
 * - Any GPIO pin can be used for the I2C (SDA and SCL) pins.
 * - The NFC reader is connected to pin 8 with its SDA.
 * - The NFC reader is connected to pin 9 with its SCL.
 * - The NFC reader is connected to the 3.3V pin with its VCC.

 * Water temperature sensor test:
 * - https://randomnerdtutorials.com/esp32-ds18b20-temperature-arduino-ide/
 * - The temp sensor is connected to pin 5 with its yellow wire.
 * - The temp sensor is connected to the ground with its white wire.
 * - The temp sensor is connected to the power source with its red wire,
 *   but in parasite mode this wire is also connected to the ground.
 * - The temp sensor is connected to the power source with a 4.7K resistor,
 *   that is linked to the data/signal pin, which is connected to the power source.

//  * Mist module & Relay test:
//  * - https://github.com/DKARDU/automatic-humidity/blob/master/auto_humid.ino
//  * - Relay DC+ connects to the ESP32-S3-DevKitC-1 3.3V.
//  * - Relay DC- connects to the ESP32-S3-DevKitC-1 GND.
//  * - Relay IN connects to the ESP32-S3-DevKitC-1 GPIO 11.
//  * - Relay COM connects to ESP32-S3-DevKitC-1 5V.
//  * - Mist module 5V connects to Relay NO (normally Open).
//  * - Mist module GND connects to ESP32-S3-DevKitC-1 GND.

 * RGB indicator LEDs:
 * - Common anode (+), so 255 is OFF and 0 is ON.
 * - R, + (long leg), B, G

 * DFPlayerMini & Speakers test:
 * - https://wiki.dfrobot.com/DFPlayer_Mini_SKU_DFR0299
 * - https://www.hackster.io/munir03125344286/df-player-mini-interface-with-esp32-f1efca
 * - https://github.com/pcbreflux/espressif/blob/master/esp32/arduino/sketchbook/ESP32_DFPlayer_full/ESP32_DFPlayer_full.ino
 * - Example: DFRobotDFPlayerMini - GetStarted
 * - The speakers are both connected to the same + (left of GND) and - (right of GND) pins on the DFPlayer Mini MP3 Module.
 * - The red wire is the + (5V needed, since to much already on 3.3V) and the black wire is the -.
 * - RX van DFPlayer Mini needs to be connected to pin 17 (TX) of the ESP32-S3-DevKitC-1.
 * - TX van DFPlayer Mini needs to be connected to pin 18 (RX) of the ESP32-S3-DevKitC-1.

  * Adafruit RGB NeoPixel Strip test:
 * - https://learn.adafruit.com/adafruit-neopixel-uberguide/basic-connections
 * - https://learn.adafruit.com/adafruit-neopixel-uberguide/arduino-library-installation
 * - Example: Adafruit_NeoPixel - strandtest
 * - 1000uF Capacitor: long leg is +, short leg is -.
 * - Capacitor between NeoPixel strip's + and - connections on the ESP32-S3-DevKitC-1.
 *     - + leg between 5V+ of the NeoPixel strip and 5V of the ESP32-S3-DevKitC-1.
 *     - - leg between GND of the NeoPixel strip and GND of the ESP32-S3-DevKitC-1.
 * - 390 Ohm Resistor between NeoPixel strip's DATA-IN and the ESP32-S3-DevKitC-1.
 * - NeoPixel strip Din is connected to pin 48 of the ESP32-S3-DevKitC-1.
 * - NeoPixel strip 5V+ is connected to the 5V pin of the ESP32-S3-DevKitC-1.
 * - NeoPixel strip GND is connected to the GND pin of the ESP32-S3-DevKitC-1.
 * - BEST PRACTICES for most reliable operation:
 *     - MINIMIZE WIRING LENGTH between microcontroller board and first pixel.
 *     - AVOID connecting NeoPixels on a LIVE CIRCUIT.
 *         - If you must, ALWAYS connect GROUND (-) first, then +, then data (Din).
 */


// DEFINITIONS
#define DEBUG


// LIBRARIES
#include <Arduino.h>

// Switch.
#include <ezButton.h>

// NFC PN532, I2C Mode.
#include <Wire.h>
#include <PN532_I2C.h>
#include <PN532.h>
#include <NfcAdapter.h>

// Water temperature sensor.
#include <OneWire.h>
#include <DallasTemperature.h>

// DFPlayer Mini MP3 Module & Speakers.
#include <HardwareSerial.h>
#include <DFRobotDFPlayerMini.h>

// NeoPixel RGB LED strip lights.
#include <Adafruit_NeoPixel.h>


// CONSTANT/GLOBAL VARIABLES
// Switch.
ezButton toggleSwitch(21);
volatile bool isSwitchOn = false;

// NFC RGB indicator LED.
const int ledRedPinNFC = 12;
const int ledGreenPinNFC = 14;
const int ledBluePinNFC = 13;
bool isNFCDone = false;

// NFC PN532, I2C Mode.
PN532_I2C pn532i2c(Wire);
PN532 nfc(pn532i2c);
volatile bool connectedNFC = false;
// Tags in order: Autumn, Winter, Spring, Summer
const String tags[] = {"04 42 f0 22 7d 72 80", "04 46 f0 22 7d 72 80",
                       "04 50 f0 22 7d 72 80", "04 53 f1 22 7d 72 80"};
String finalTag = "";

// Water temperature sensor RGB indicator LED.
const int ledRedPinTemp = 4;
const int ledGreenPinTemp = 5;
const int ledBluePinTemp = 6;
bool isLedIndicationOFF = false;

// Water temperature sensor.
bool isTempIn = false;
bool isTempGood = false;
const int hotTemp = 60;
const int goodTemp = 53;
const int tempWireBus = 1;
OneWire tempWire(tempWireBus);
DallasTemperature tempSensor(&tempWire);

// // Relay & Mist module.
// const int relay = 2;

// DFPlayer Mini MP3 Module & Speakers.
const byte TX_Pin = 17;
const byte RX_Pin = 18;
DFRobotDFPlayerMini player;
const int mp3Volume1 = 30;  // Set volume to maximum (0 to 30).
bool isMusicOn = false;
bool isGongPlayed = false;
static unsigned long gongTimer;
bool isMusicChanged = false;

// Tree and mist RGB LED strip lights.
const int ledStripPin = 47;
const int ledStripCount = 14; //39
Adafruit_NeoPixel strip(ledStripCount, ledStripPin, NEO_GRB + NEO_KHZ800);
// Argument 1 = Number of pixels in NeoPixel strip
// Argument 2 = Arduino pin number (most are valid)
// Argument 3 = Pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)
bool isTreeLightsOn = false;
bool isTreeLightsChanged = false;

#define Brightness 10  //Set brightness to 1/10th
#define Full (255/Brightness)


// FUNCTIONS
void setup() {
    // Debug info: Initialize the serial connection.
    #ifdef DEBUG
        Serial.begin(115200);
    #endif

    toggleSwitch.setDebounceTime(50);  // set debounce time to 50 milliseconds

    // Set the LED pins as output for the NFC.
    pinMode(ledRedPinNFC, OUTPUT);
    pinMode(ledGreenPinNFC, OUTPUT);
    pinMode(ledBluePinNFC, OUTPUT);
    setLedColor(ledRedPinNFC, ledGreenPinNFC, ledBluePinNFC, 255, 255, 255);

    Serial.println("*** PN532 NFC RFID Enabled ***");

    // Set the LED pins as output for the water temp sensor.
    pinMode(ledRedPinTemp, OUTPUT);
    pinMode(ledGreenPinTemp, OUTPUT);
    pinMode(ledBluePinTemp, OUTPUT);
    setLedColor(ledRedPinTemp, ledGreenPinTemp, ledBluePinTemp, 255, 255, 255);

    // Start the DS18B20 sensor
    tempSensor.begin();

    // // Set the relay pin as output.
    // pinMode(relay, OUTPUT);

    // Set up the DFPlayer Mini MP3 Module.
    mp3Setup();

    // Set up the NeoPixel RGB LED strip lights.
    strip.begin();            // INITIALIZE NeoPixel strip object (REQUIRED)
    strip.show();             // Turn OFF all pixels ASAP
    strip.setBrightness(50);  // Set BRIGHTNESS to about 1/5 (max = 255)
}

/*
 * Declares the reset function at address 0.
 */
void(* resetFunc) (void) = 0;

void loop() {
    toggleSwitch.loop();  // MUST call the loop() function first

    if (toggleSwitch.isPressed()) {
        Serial.println("The switch: OFF -> ON");
    }

    if (toggleSwitch.isReleased()) {
        Serial.println("The switch: ON -> OFF");
        resetFunc(); // Reset the ESP32-S3-DevKitC-1
    }

    // Debug info: Print the state of the switch.
    int state = toggleSwitch.getState();

    if (state == LOW) {
        if (!isNFCDone) {
            setLedColor(ledRedPinNFC, ledGreenPinNFC, ledBluePinNFC, 250, 255, 255);
            readNFC();
        }

        if (!isTempIn && isNFCDone) {
            readTempSensor();
        }

        if (isTempIn && isNFCDone) {
            if (!isLedIndicationOFF) {
                // Turn OFF the NFC and Temp RGB LEDs.
                setLedColor(ledRedPinNFC, ledGreenPinNFC, ledBluePinNFC, 255, 255, 255);
                setLedColor(ledRedPinTemp, ledGreenPinTemp, ledBluePinTemp, 255, 255, 255);
                isLedIndicationOFF = true;
            }

            if (!isTempGood) {
                // turnRelayMistOn();
                readTempSensor();

                if (finalTag == "Autumn") {
                    LeavesSparkle(0x7f, 0x14, 0x00, 40, random(100, 850));  // Orange

                    if (!isMusicOn) {
                        mp3Autumn(1);
                        isMusicOn = true;
                    }
                } else if (finalTag == "Winter") {
                    SnowSparkle(0x00, 0x6d, 0x7f, 40, random(100, 950));  // Blue

                    if (!isMusicOn) {
                        mp3Winter(1);
                        isMusicOn = true;
                    }
                } else if (finalTag == "Spring") {
                    TwinkleSpring(0xfc, 0xc0, 0xf8, 10, 100, false);  // Pink

                    if (!isMusicOn) {
                        mp3Spring(1);
                        isMusicOn = true;
                    }
                } else if (finalTag == "Summer") {
                    FadeInOut(0x00, 0xe6, 0x00);  // Green
                    FadeInOut(0xff, 0xeb, 0x00);  // Yellow

                    if (!isMusicOn) {
                        mp3Summer(1);
                        isMusicOn = true;
                    }
                }
            } else {
                if (!isGongPlayed) {
                    mp3Gong();
                    gongTimer = millis();
                    isGongPlayed = true;
                }

                // turnRelayMistOff();

                if (millis() - gongTimer > 3000) {
                    if (finalTag == "Autumn") {
                        LeavesSparkle(0x7f, 0x14, 0x00, 40, random(100, 850));  // Orange

                        if (!isMusicChanged) {
                            mp3Autumn(2);
                            isMusicChanged = true;
                        }
                    } else if (finalTag == "Winter") {
                        SnowSparkle(0x00, 0x6d, 0x7f, 40, random(100, 950));  // Blue

                        if (!isMusicChanged) {
                            mp3Winter(2);
                            isMusicChanged = true;
                        }
                    } else if (finalTag == "Spring") {
                        TwinkleSpring(0xfc, 0xc0, 0xf8, 10, 100, false);  // Pink

                        if (!isMusicChanged) {
                            mp3Spring(2);
                            isMusicChanged = true;
                        }
                    } else if (finalTag == "Summer") {
                        FadeInOut(0x00, 0xe6, 0x00);  // Green
                        FadeInOut(0xff, 0xeb, 0x00);  // Yellow

                        if (!isMusicChanged) {
                            mp3Summer(2);
                            isMusicChanged = true;
                        }
                    }
                }
            }

            tempUpdate();
            mp3Update();
        }

        tempUpdate();
    }
}

void readNFC() {
    /*
     * Read the scanned NFC tag.
     */
    boolean success;
    uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };  // Buffer to store the returned UID
    uint8_t uidLength;  // Length of the UID (4 or 7 bytes depending on ISO14443A card type)
    String currentTag = "";

    while (!connectedNFC) {
        connectedNFC = connectNFC();
    }

    // Wait for an ISO14443A type cards (Mifare, etc.).  When one is found
    // 'uid' will be populated with the UID, and uidLength will indicate
    // if the uid is 4 bytes (Mifare Classic) or 7 bytes (Mifare Ultralight)
    success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, &uid[0], &uidLength);

    // If the tag os detected, print the UID.
    if (success) {
        Serial.println("Found a tag!");

        for (uint8_t i=0; i < uidLength; i++) {
            if (i != 0) {
                currentTag += " ";
            }

            if (uid[i] < 0x10) {
                currentTag += "0";
            }

            currentTag += String(uid[i], HEX);
        }

        Serial.println("Current Tag: " + currentTag);
        Serial.println("");
        Serial.println("");

        if (currentTag == tags[0]) {
            finalTag = "Autumn";
            // Set the LED color of the NFC RGB LED.
            setLedColor(ledRedPinNFC, ledGreenPinNFC, ledBluePinNFC, 255, 250, 255);
            setLedColor(ledRedPinTemp, ledGreenPinTemp, ledBluePinTemp, 250, 255, 255);
            isNFCDone = true;
            Serial.println("Autumn");
            Serial.println("NFC is DONE.");
        } else if (currentTag == tags[1]) {
            finalTag = "Winter";
            // Set the LED color of the NFC RGB LED.
            setLedColor(ledRedPinNFC, ledGreenPinNFC, ledBluePinNFC, 255, 250, 255);
            setLedColor(ledRedPinTemp, ledGreenPinTemp, ledBluePinTemp, 250, 255, 255);
            isNFCDone = true;
            Serial.println("Winter");
            Serial.println("NFC is DONE.");
        } else if (currentTag == tags[2]) {
            finalTag = "Spring";
            // Set the LED color of the NFC RGB LED.
            setLedColor(ledRedPinNFC, ledGreenPinNFC, ledBluePinNFC, 255, 250, 255);
            setLedColor(ledRedPinTemp, ledGreenPinTemp, ledBluePinTemp, 250, 255, 255);
            isNFCDone = true;
            Serial.println("Spring");
            Serial.println("NFC is DONE.");
        } else if (currentTag == tags[3]) {
            finalTag = "Summer";
            // Set the LED color of the NFC RGB LED.
            setLedColor(ledRedPinNFC, ledGreenPinNFC, ledBluePinNFC, 255, 250, 255);
            setLedColor(ledRedPinTemp, ledGreenPinTemp, ledBluePinTemp, 250, 255, 255);
            isNFCDone = true;
            Serial.println("Summer");
            Serial.println("NFC is DONE.");
        } else {
            finalTag = "";
            Serial.println("NFC is NOT DONE.");
            connectedNFC = false;
        }

        // Wait 0.5 second before continuing
        delay(500);
    } else {
        // PN532 probably timed out waiting for a tag
        Serial.println("Timed out waiting for a tag...");
        connectedNFC = false;
    }
}

bool connectNFC() {
    /*
     * Try connecting to the PN53x board.
     * If the connection fails, try again.
     * If the connection is successful, print its version info.
     */
    nfc.begin();

    // Try connecting to a PN53x board and getting its version info.
    uint32_t versiondata = nfc.getFirmwareVersion();
    if (! versiondata) {
        Serial.println("PN53x board not found!");
        return false;  // Try to connect to the PN532 board again.
    }

    // Connected to a PN53x board, print its version info.
    Serial.print("Found chip PN5");
    Serial.println((versiondata>>24) & 0xFF, HEX);

    // Set the max number of retry attempts to read from a card
    // This prevents us from waiting forever for a card, which is
    // the default behaviour of the PN532 (default = 0xFF).
    nfc.setPassiveActivationRetries(0x0A);

    // configure board to read RFID tags
    nfc.SAMConfig();

    Serial.println("Waiting for an tag...");
    Serial.println("");

    return true;
}

void setLedColor(int ledRedPin, int ledGreenPin, int ledBluePin, int red, int green, int blue) {
    /*
     * Set the RGB LED color of the LED.
     */
    // Serial.println("Set LED color");
    analogWrite(ledRedPin, red);
    analogWrite(ledGreenPin, green);
    analogWrite(ledBluePin, blue);
}

void readTempSensor() {
    /*
     * Read the temperature from the water temperature sensor.
     */
    tempSensor.requestTemperatures();
    float temperatureC = tempSensor.getTempCByIndex(0);
    float temperatureF = tempSensor.getTempFByIndex(0);

    Serial.print(temperatureC);
    Serial.println("°C");
    Serial.println("--------------------");

    if (!isTempIn && !isTempGood && temperatureC >= hotTemp) {
        setLedColor(ledRedPinTemp, ledGreenPinTemp, ledBluePinTemp, 255, 255, 250);
        isTempIn = true;
    }

    if (isTempIn && !isTempGood && temperatureC <= goodTemp) {
        isTempGood = true;
    }

    delay(500);
}

void tempUpdate() {
    /*
     * Update the hotTemp and goodTemp states manually with the serial monitor.
     */
    if (Serial.available()) {
        String inData = "";
        inData = Serial.readStringUntil('\n');
        if (inData.startsWith("i")) {
            Serial.println(F("isTempIn --> True ----------"));
            setLedColor(ledRedPinTemp, ledGreenPinTemp, ledBluePinTemp, 255, 255, 250);
            isTempIn = true;
        } else if (inData.startsWith("g")) {
            Serial.println(F("isTempGood --> True --------"));
            isTempGood = true;
        }
    }
}

// void turnRelayMistOn() {
//     /*
//      * Turn the relay ON, and therefore the mist modules.
//      */
//     digitalWrite(relay, HIGH);
//     Serial.println("Relay ON");
// }

// void turnRelayMistOff() {
//     /*
//      * Turn the relay OFF, and therefore the mist modules.
//      */
//     digitalWrite(relay, LOW);
//     Serial.println("Relay OFF");
// }

void mp3Setup() {
    /*
     * Setup the DFPlayer Mini MP3 Module.
     */
    Serial2.begin(9600, SERIAL_8N1, RX_Pin, TX_Pin);

    Serial.println();
    Serial.println(F("DFPlayer Mini Test"));
    Serial.println(F("Initializing DFPlayer ... (May take 3~5 seconds)"));

    // Start communication with DFPlayer Mini
    if (!player.begin(Serial2)) {
        Serial.println("Connecting to DFPlayer Mini failed!");
        while(true);  // Wait here forever.
    }

    Serial.println("DFPlayer Mini online.");

    player.setTimeOut(500);  //Set serial communictaion time out 500ms
    player.volume(mp3Volume1);
    player.EQ(DFPLAYER_EQ_NORMAL);
    player.outputDevice(DFPLAYER_DEVICE_SD);
}

void mp3Autumn(int trackNumber) {
    /*
     * Play the Autumn music.
     */
    int delayms=100;
    Serial.print(F("Play \"SD:/1/00"));  // Autumn folder, .wav or .mp3 file
    Serial.print(trackNumber);
    Serial.println(F("\""));
    player.playFolder(1, trackNumber);
    delay(delayms);
}

void mp3Winter(int trackNumber) {
    /*
     * Play the Winter music.
     */
    int delayms=100;
    Serial.print(F("Play \"SD:/2/00"));  // Winter folder, .wav or .mp3 file
    Serial.print(trackNumber);
    Serial.println(F("\""));
    player.playFolder(2, trackNumber);
    delay(delayms);
}

void mp3Spring(int trackNumber) {
    /*
     * Play the Spring music.
     */
    int delayms=100;
    Serial.print(F("Play \"SD:/3/00"));  // Spring folder, .wav or .mp3 file
    Serial.print(trackNumber);
    Serial.println(F("\""));
    player.playFolder(3, trackNumber);
    delay(delayms);
}

void mp3Summer(int trackNumber) {
    /*
     * Play the Summer music.
     */
    int delayms=100;
    Serial.print(F("Play \"SD:/4/00"));  // Summer folder, .wav or .mp3 file
    Serial.print(trackNumber);
    Serial.println(F("\""));
    player.playFolder(4, trackNumber);
    delay(delayms);
}

void mp3Gong() {
    /*
     * Play the sfx for "the tea is ready" indication.
     */
    int delayms=100;
    Serial.println(F("Play \"SD:/5/001\""));  // Sfx folder, .wav or .mp3 file
    player.playFolder(5, 1);
    delay(delayms);
}

void mp3Update() {
    /*
     * Update the DFPlayer Mini MP3 Module.
     */
    // Use the serial monitor to manually control the DFPlayer Mini MP3 Module.
    if (Serial.available()) {
        String inData = "";
        inData = Serial.readStringUntil('\n');
        if (inData.startsWith("n")) {
            Serial.println(F("next--------------------"));
            player.next();
            Serial.println(player.readCurrentFileNumber());  //read current play file number
        } else if (inData.startsWith("p")) {
            Serial.println(F("previous--------------------"));
            player.previous();
            Serial.println(player.readCurrentFileNumber());  //read current play file number
        } else if (inData.startsWith("+")) {
            Serial.println(F("up--------------------"));
            player.volumeUp();
            Serial.println(player.readVolume());  //read current volume
        } else if (inData.startsWith("-")) {
            Serial.println(F("down--------------------"));
            player.volumeDown();
            Serial.println(player.readVolume());  //read current volume
        } else if (inData.startsWith("*")) {
            Serial.println(F("pause--------------------"));
            player.pause();
        } else if (inData.startsWith(">")) {
            Serial.println(F("start--------------------"));
            player.start();
        }
    }

    if (player.available()) {
        printMP3Detail(player.readType(), player.read());  //Print the detail message from DFPlayer to handle different errors and states.
    }
}

void printMP3Detail(uint8_t type, int value){
    /*
     * Print the detail message from DFPlayer to handle different errors and states.
     */
    switch (type) {
        case TimeOut:
            Serial.println(F("Time Out!"));
            break;
        case WrongStack:
            Serial.println(F("Stack Wrong!"));
            break;
        case DFPlayerCardInserted:
            Serial.println(F("Card Inserted!"));
            break;
        case DFPlayerCardRemoved:
            Serial.println(F("Card Removed!"));
            break;
        case DFPlayerCardOnline:
            Serial.println(F("Card Online!"));
            break;
        case DFPlayerUSBInserted:
            Serial.println("USB Inserted!");
            break;
        case DFPlayerUSBRemoved:
            Serial.println("USB Removed!");
            break;
        case DFPlayerPlayFinished:
            Serial.print(F("Number:"));
            Serial.print(value);
            Serial.println(F(" Play Finished!"));
            break;
        case DFPlayerError:
            Serial.print(F("DFPlayerError:"));
            switch (value) {
                case Busy:
                    Serial.println(F("Card not found"));
                    break;
                case Sleeping:
                    Serial.println(F("Sleeping"));
                    break;
                case SerialWrongStack:
                    Serial.println(F("Get Wrong Stack"));
                    break;
                case CheckSumNotMatch:
                    Serial.println(F("Check Sum Not Match"));
                    break;
                case FileIndexOut:
                    Serial.println(F("File Index Out of Bound"));
                    break;
                case FileMismatch:
                    Serial.println(F("Cannot Find File"));
                    break;
                case Advertise:
                    Serial.println(F("In Advertise"));
                    break;
                default:
                    break;
            }

            break;
        default:
            break;
    }
}

void setPixel(int Pixel, byte red, byte green, byte blue) {
   strip.setPixelColor(Pixel, strip.Color(red, green, blue));
}

void setAll(byte red, byte green, byte blue) {
    for(int i = 0; i < ledStripCount; i++ ) {
        setPixel(i, red, green, blue);
    }

    strip.show();
}

void FadeInOut(byte red, byte green, byte blue){
    /*
     * https://www.tweaking4all.com/hardware/arduino/adruino-led-strip-effects/#LEDStripEffectFadeInandFadeOutYourownColors
     * Fade in/out.
     * For Summer between green and yellow.
     */
    float r, g, b;

    for(int k = 25; k < 256; k = k + 1) {
        r = (k / 256.0) * red;
        g = (k / 256.0) * green;
        b = (k / 256.0) * blue;

        setAll(r, g, b);
        strip.show();
    }

    delay(1000);

    for(int k = 255; k >= 30; k = k - 1) {
        r = (k / 256.0) * red;
        g = (k / 256.0) * green;
        b = (k / 256.0) * blue;

        setAll(r,g,b);
        strip.show();
    }
}

void TwinkleSpring(byte red, byte green, byte blue, int Count, int SpeedDelay, boolean OnlyOne) {
    /*
     * https://www.tweaking4all.com/hardware/arduino/adruino-led-strip-effects/#LEDStripEffectTwinkle
     * Twinkle.
     * For Spring between white and pink.
     */
    setAll(0xff,0x4b,0xf1);  // Pink

    for (int i = 0; i < Count; i++) {
        setPixel(random(ledStripCount), red, green, blue);
        strip.show();
        delay(SpeedDelay);

        if(OnlyOne) {
            setAll(0xff, 0x4b, 0xf1);
        }
    }

    delay(SpeedDelay);
}

void SnowSparkle(byte red, byte green, byte blue, int SparkleDelay, int SpeedDelay) {
    /*
     * https://www.tweaking4all.com/hardware/arduino/adruino-led-strip-effects/#LEDStripEffectSnowSparkle
     * Snow Sparkle.
     * For Winter between white and light blue.
     */
    setAll(red, green, blue);

    int Pixel1 = random(ledStripCount/2);
    int Pixel2 = random(ledStripCount/2, ledStripCount);
    setPixel(Pixel1, 0xff, 0xfe, 0xfa);
    setPixel(Pixel2, 0xff, 0xfe, 0xfa);
    strip.show();
    delay(SparkleDelay);
    setPixel(Pixel1, red, green, blue);
    setPixel(Pixel2, red, green, blue);
    strip.show();
    delay(SpeedDelay);
}

void LeavesSparkle(byte red, byte green, byte blue, int SparkleDelay, int SpeedDelay) {
    /*
     * https://www.tweaking4all.com/hardware/arduino/adruino-led-strip-effects/#LEDStripEffectSnowSparkle
     * Snow Sparkle --> Leaves Sparkle.
     * For Autumn between orange, red, yellow.
     */
    setAll(red, green, blue);

    int Pixel1 = random(ledStripCount/2);
    int Pixel2 = random(ledStripCount/2, ledStripCount);
    int c1 = random(2);
    int c2 = random(2);

    if (c1 == 0) {
        setPixel(Pixel1, 0xff, 0x00, 0x00);  // Red
    } else if (c1 == 1) {
        setPixel(Pixel1, 0xff, 0x80, 0x00);  // Yellow
    } else {
        setPixel(Pixel1, red, green, blue);
    }

    if (c2 == 0) {
        setPixel(Pixel2, 0xff, 0x00, 0x00);  // Red
    } else if (c2 == 1) {
        setPixel(Pixel2, 0xff, 0x80, 0x00);  // Yellow
    } else {
        setPixel(Pixel2, red, green, blue);
    }

    strip.show();
    delay(SparkleDelay);
    setPixel(Pixel1, red, green, blue);
    setPixel(Pixel2, red, green, blue);
    strip.show();
    delay(SpeedDelay);
}
