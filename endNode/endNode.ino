// Arduino Code for End Node of Nano Farm
// Based on examples from https://github.com/matthijskooijman/arduino-lmic
// and https://github.com/gonzalocasas/arduino-uno-dragino-lorawan/

#include <lmic.h>
#include <hal/hal.h>
#include "LowPower.h"

/*************************************
 * NwkSKey: network session key, AppSKey: application session key, and DevAddr: end-device address
 *************************************/
static const u1_t NWKSKEY[16] = { 0x06, 0x23, 0x25, 0xEF, 0x59, 0xED, 0xD9, 0xBA, 0xAD, 0xC9, 0xF8, 0x0B, 0x0F, 0x10, 0x68, 0x17 };
static const u1_t APPSKEY[16] = { 0x3A, 0x98, 0x2D, 0x70, 0xED, 0xB3, 0xDE, 0x24, 0xC7, 0xA9, 0x62, 0x49, 0x23, 0x87, 0x94, 0x8F };
static const u4_t DEVADDR = 0x26011A21;

#define MOISTURE_SENSOR_PIN A0
#define PUMP_PIN 4

#define AIR_VALUE 590
#define WATER_VALUE 74
#define WATERING_THRESHOLD 50 // Just guessed this to have a test
#define NUM_READINGS 3

volatile bool waitToSend;

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 9,
    .dio = {2, 6, 7},
};

void onEvent (ev_t ev)
{
    if (ev == EV_TXCOMPLETE)
    {
        Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
        if (LMIC.dataLen) {
          uint8_t downlink[LMIC.dataLen];
          memcpy(&downlink,&(LMIC.frame+LMIC.dataBeg)[0],LMIC.dataLen);
          Serial.println(F("Got Message"));
          Serial.println(downlink[0], HEX);
        }
        // Tell main program to continue
        waitToSend = false;
    }
}

void setup()
{
    Serial.begin(115200);
    Serial.println(F("Setting Up Nano Farm End Node"));

    pinMode(PUMP_PIN, OUTPUT);

    //Allow for AVR timing error
    LMIC_setClockError(MAX_CLOCK_ERROR * 5 / 100);

    // LMIC init
    os_init();

    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    // Set static session parameters.
    LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);

    // Only enable channel 0 so always picked up by single-channel gateway
    Serial.println("European Channels");
    for (int i = 1; i <= 8; i++) LMIC_disableChannel(i);

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF7,14);

}

byte readFromMoistureSensor()
{
    int moistureReading = analogRead(MOISTURE_SENSOR_PIN);
    // map readings to percentage scale
    byte val = map(moistureReading, WATER_VALUE, AIR_VALUE, 100, 0);

    return val;
}

void waterPlants(byte reading)
{
    if (reading <= WATERING_THRESHOLD)
    {
        Serial.println("Water Pump on");
        digitalWrite(PUMP_PIN, LOW);
        delay(2000);
        Serial.println("Water Pump off");
        digitalWrite(PUMP_PIN, HIGH);
    }
}

bool sendToServer(byte * data, int dataLength)
{
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
        return false;
    } else {
        // Prepare upstream data transmission at the next possible time.
        Serial.print(F("Sending packet: " ));
        LMIC_setTxData2(1, (uint8_t*) data, dataLength, 0);
        Serial.println(F(" Packet queued"));
        return true;
    }
}

void sleepFor10Mins ()
{
    for (int i=0; i<75; i++)
    {
        LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_ON);
    }
}

void loop()
{
    byte moistureReadings[NUM_READINGS] = {};

    sleepFor10Mins(); // 10 mins
    moistureReadings[0] = readFromMoistureSensor();
    Serial.print("0 : ");
    Serial.println(moistureReadings[0]);
    sleepFor10Mins(); // 20 mins
    moistureReadings[1] = readFromMoistureSensor();
    Serial.print("1 : ");
    Serial.println(moistureReadings[1]);
    sleepFor10Mins(); // 30 mins
    waterPlants(moistureReadings[1]);
    sleepFor10Mins(); // 40 mins
    moistureReadings[2] = readFromMoistureSensor();
    Serial.print("2 : ");
    Serial.println(moistureReadings[2]);
    sleepFor10Mins(); // 50 mins
    sleepFor10Mins(); // 60 mins
    waitToSend = sendToServer(moistureReadings, sizeof(moistureReadings));
    while (waitToSend)
    {
        os_runloop_once();
    }
}

