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
#define WATER_LEVEL_SENSOR_PIN A1
#define FEED_LEVEL_SENSOR_PIN A2
#define WATER_PUMP_PIN 4
#define FEED_PUMP_PIN 5

#define AIR_VALUE 575
#define WATER_VALUE 93

#define WATER_FLAG_MASK 0x80
#define FEED_FLAG_MASK 0x40
#define WATER_REFILL_FLAG_MASK 0x20
#define FEED_REFILL_FLAG_MASK 0x10

#define WATERING_THRESHOLD 400 // Just guessed this to have a test
#define TANK_REFILL_THRESHOLD 400
#define WATER_TIME 10000
#define FEED_TIME 2000
#define NUM_READINGS 3

volatile bool waitToSend;
int hourCounter;
byte lastFlags = 0x00;

struct Data
{
    int moistureReadings[NUM_READINGS];
    byte flags;
};

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

    pinMode(WATER_PUMP_PIN, OUTPUT);
    pinMode(FEED_PUMP_PIN, OUTPUT);
    digitalWrite(FEED_PUMP_PIN, HIGH);
    digitalWrite(WATER_PUMP_PIN, HIGH);
    hourCounter = 0;

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

    Data tmpdata;
    checkTankRefillRequired(WATER_LEVEL_SENSOR_PIN, &tmpdata);
    checkTankRefillRequired(FEED_LEVEL_SENSOR_PIN, &tmpdata);
    lastFlags = tmpdata.flags;

}

int readFromMoistureSensor()
{
    int sensorReading = analogRead(MOISTURE_SENSOR_PIN);
    // map readings to percentage scale
    int moistureReading = (-0.2*sensorReading + 106) * 10;

    return moistureReading;
}

bool checkTankRefillRequired(int tankPin, Data *data)
{
    int sensorReading = analogRead(tankPin);
    if (sensorReading > TANK_REFILL_THRESHOLD)
    {
        if (tankPin == WATER_LEVEL_SENSOR_PIN)
        {
            data->flags |= WATER_REFILL_FLAG_MASK;
        }
        else
        {
            data->flags |= FEED_REFILL_FLAG_MASK;
        }
        return true;
    }
    else
    {
        if (tankPin == WATER_LEVEL_SENSOR_PIN)
        {
            data->flags &= ~WATER_REFILL_FLAG_MASK;
        }
        else
        {
            data->flags &= ~FEED_REFILL_FLAG_MASK;
        }
        return false;
    }
}

bool feedPlants(Data *data)
{
    // Feed plants once a week
    if (hourCounter == 168) // 168 hours = 7 days
    {
        Serial.println("Feed Pump on");
        digitalWrite(FEED_PUMP_PIN, LOW);
        delay(FEED_TIME);
        Serial.println("Feed Pump off");
        digitalWrite(FEED_PUMP_PIN, HIGH);
        hourCounter = 0;
        data->flags |= FEED_FLAG_MASK;
        return true;
    }
    return false;
}

bool waterPlants(int reading, Data *data)
{
    if (reading <= WATERING_THRESHOLD)
    {
        Serial.println("Water Pump on");
        digitalWrite(WATER_PUMP_PIN, LOW);
        delay(WATER_TIME);
        Serial.println("Water Pump off");
        digitalWrite(WATER_PUMP_PIN, HIGH);
        data->flags |= WATER_FLAG_MASK;
        return true;
    }
    return false;
}

bool sendToServer(void * sensorData, int dataLength)
{
    Data * data = (Data *) sensorData;
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
        LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
    }
}

void loop()
{
    struct Data data;
    data.flags = lastFlags;
    if (lastFlags & WATER_REFILL_FLAG_MASK == WATER_REFILL_FLAG_MASK)
    {
        checkTankRefillRequired(WATER_LEVEL_SENSOR_PIN, &data);
    }
    if(lastFlags & FEED_REFILL_FLAG_MASK == FEED_REFILL_FLAG_MASK )
    {
        checkTankRefillRequired(FEED_LEVEL_SENSOR_PIN, &data);
    }

    sleepFor10Mins(); // 10 mins
    data.moistureReadings[0] = readFromMoistureSensor();
    Serial.print("0 : ");
    Serial.println(data.moistureReadings[0]);
    sleepFor10Mins(); // 20 mins
    data.moistureReadings[1] = readFromMoistureSensor();
    Serial.print("1 : ");
    Serial.println(data.moistureReadings[1]);
    sleepFor10Mins(); // 30 mins
    if (waterPlants(data.moistureReadings[1], &data))
    {
        checkTankRefillRequired(WATER_LEVEL_SENSOR_PIN, &data);
    }
    sleepFor10Mins(); // 40 mins
    data.moistureReadings[2] = readFromMoistureSensor();
    Serial.print("2 : ");
    Serial.println(data.moistureReadings[2]);
    sleepFor10Mins(); // 50 mins
    if (feedPlants(&data))
    {
        checkTankRefillRequired(FEED_LEVEL_SENSOR_PIN, &data);
    }
    sleepFor10Mins(); // 60 mins
    waitToSend = sendToServer(&data, sizeof(data));
    while (waitToSend)
    {
        os_runloop_once();
    }
    hourCounter++;
    //RESET WATER FEED FLAGS
    Serial.println(data.flags, HEX);
    data.flags &= 0x30;
    lastFlags = data.flags;
}

