/*******************************************************************************
  Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
  Permission is hereby granted, free of charge, to anyone
  obtaining a copy of this document and accompanying files,
  to do whatever they want with them without any restriction,
  including, but not limited to, copying, modification and redistribution.
  NO WARRANTY OF ANY KIND IS PROVIDED.
  This example sends a valid LoRaWAN packet with payload "Hello,
  world!", using frequency and encryption settings matching those of
  the The Things Network.
  This uses ABP (Activation-by-personalisation), where a DevAddr and
  Session keys are preconfigured (unlike OTAA, where a DevEUI and
  application key is configured, while the DevAddr and session keys are
  assigned/generated in the over-the-air-activation procedure).
Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
violated by this sketch when left running for longer)!
To use this sketch, first register your application and device with
the things network, to set or generate a DevAddr, NwkSKey and
AppSKey. Each device should have their own unique values for these
fields.
Do not forget to define the radio type correctly in config.h.
                                                               *******************************************************************************/

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

// LoRaWAN NwkSKey, network session key
// This is the default Semtech key, which is used by the early prototype TTN network.

static const PROGMEM u1_t NWKSKEY[16] = {0x2A, 0x89, 0x02, 0x50, 0x84, 0x2F, 0xDB, 0xEE, 0x13, 0x6E, 0x82, 0x8E, 0xD7, 0xAD, 0x5C, 0x53 };
static const PROGMEM u1_t APPSKEY[16] = {0x6D, 0x2E, 0x8E, 0xE7, 0x02, 0xB9, 0x2B, 0xF5, 0x7E, 0x69, 0x06, 0xD6, 0x59, 0x45, 0xE5, 0x70 };
static const PROGMEM u4_t DEVADDR = 0x2601122C;

#define LED 13
// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
#define TX_INTERVAL 3600
#define MOISTURE_SENSOR_PIN A3

// Just set value globally until sensor code added
int moistureReading = 0;
static osjob_t sendjob;

// Pin mapping
const lmic_pinmap lmic_pins = {
   .nss = 10,
   .rxtx = LMIC_UNUSED_PIN,
   .rst = 9,
   .dio = {2, 6, 7},
};

void onEvent (ev_t ev)
{
   Serial.print(os_getTime());
   Serial.print(": ");
   switch (ev)
   {
      case EV_TXCOMPLETE:
         Serial.println(F("EV_TXCOMPLETE"));
         Serial.println(LMIC.dataLen);
         // Schedule next transmission
         os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
         digitalWrite(LED, LOW);
         break;
      case EV_RESET:
         Serial.println(F("EV_RESET"));
         break;
      case EV_RXCOMPLETE:
         // data received in ping slot
         Serial.println(F("EV_RXCOMPLETE"));
         break;
      case EV_LINK_DEAD:
         Serial.println(F("EV_LINK_DEAD"));
         break;
      case EV_LINK_ALIVE:
         Serial.println(F("EV_LINK_ALIVE"));
         break;
      default:
         Serial.println(F("Unknown event"));
         break;
   }
}

void do_send(osjob_t* j)
{
   byte payload[2];
   int moistureReading = analogRead(MOISTURE_SENSOR_PIN);
   payload[0] = highByte(moistureReading);
   payload[1] = lowByte(moistureReading);
   // Check if there is not a current TX/RX job running
   if (LMIC.opmode & OP_TXRXPEND)
   {
      Serial.println(F("OP_TXRXPEND, not sending"));
   } else {
      // Prepare upstream data transmission at the next possible time.
      Serial.print(F("Sending: " ));
      Serial.println(moistureReading, HEX);
      LMIC_setTxData2(1, (uint8_t*) payload, 2 , 0);
      Serial.println(F(" Packet queued"));
      digitalWrite(LED, HIGH);
   }
   // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
   Serial.begin(115200);
   Serial.println(F("Setting Up Nano Farm End Node"));
   Serial.print(F("\nSending data to Gateway every: "));
   Serial.print(TX_INTERVAL);
   Serial.println(F(" seconds"));

   pinMode(LED, OUTPUT);
   digitalWrite(LED, LOW);

   // LMIC init
   os_init();
   // Reset the MAC state. Session and pending data transfers will be discarded.
   LMIC_reset();

   // Set static session parameters. Instead of dynamically establishing a session
   // by joining the network, precomputed session parameters are be provided.
   // On AVR, these values are stored in flash and only copied to RAM
   // once. Copy them to a temporary buffer here, LMIC_setSession will
   // copy them into a buffer of its own again.
   uint8_t appskey[sizeof(APPSKEY)];
   uint8_t nwkskey[sizeof(NWKSKEY)];
   memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
   memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
   LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);

   Serial.println("European Channels");
   for (int i = 1; i <= 8; i++) LMIC_disableChannel(i);

   // Disable link check validation
   LMIC_setLinkCheckMode(0);

   // TTN uses SF9 for its RX2 window.
   LMIC.dn2Dr = DR_SF9;

   // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
   Serial.println("SF7");
   LMIC_setDrTxpow(DR_SF7, 14);

   // Start job
   do_send(&sendjob);
}

void loop() {
   os_runloop_once();

}

