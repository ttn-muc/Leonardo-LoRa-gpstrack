/*
 * GPS Tracker with Arduino Leonardo
 *
 * Original code from:  https://github.com/OpenIotNetwork/LoRa-gpstrack
 *
 * Modified for Arduino Leonardo by Karin Willers 10-Jul-2018
 *
 * Arduino Leonardo has two serial ports:
 * 'Serial'  uses USB CDC communication
 * 'Serial1' is the 32u4 UART on pins 0 and 1
 *
 */

#include <lmic.h>
#include <hal/hal.h>
#include <TinyGPS.h>

/*
 * enable/disable debug on SoftSerial //#define DEBUG_SERIAL -> disabled
 * NOTE: on UNO debug and LorRa Tx isn't possible at the same time due to memory limitation. 
 *       If you enable debug only output on SoftSerial is generated but no LoRa Tx will take place
 */

#define DEBUG_SERIAL

// init TinyGPS
TinyGPS gps;

// Buffer data for LoRa packet transmission
uint8_t txBuffer[9];
// variables for GPS data we want to transmit
uint8_t hdo_gps;
uint16_t alt_gps;
uint32_t lat_gps_b, lon_gps_b;

/*
 * Credentials for ttnmap-kawillers
 */
static const u1_t NWKSKEY[16] = { 0x49, 0x0F, 0x23, 0x21, 0x41, 0x9F, 0x11, 0x17, 0x2A, 0xF9, 0x5C, 0xA2, 0x0B, 0xD1, 0x6D, 0xA7 };
static const u1_t APPSKEY[16] = { 0x65, 0x80, 0xE2, 0x61, 0xA4, 0xC9, 0xAC, 0xEA, 0xA7, 0xD0, 0x6F, 0x9C, 0xAC, 0x0C, 0x6A, 0x5F };
static const u4_t DEVADDR = 0x12345678; // <-- Change this address for every node!

/*
 * These callbacks are only used in over-the-air activation, so they are
 * left empty here (we cannot leave them out completely unless DISABLE_JOIN is set
 * in config.h, otherwise the linker will complain).
 */
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static osjob_t sendjob;

/*
 * TX_INTERVAL  = seconds to wait after LoRa TX for new transmission
 * GPS_INTERVAL = poll interval of GPS DATA in case of NO lock to satelite
 */
const unsigned TX_INTERVAL = 30;
const unsigned GPS_INTERVAL = 2;

/*
 * Pin mapping Dragino Shield
 */
const lmic_pinmap lmic_pins = {
  .nss = 10,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 9,
  .dio = {2, 6, 7},
};

/*
 * catch LoRa TXCOMPLETE and restart GPS position sending (do_send) after TX_INTERVAL
 */
void onEvent (ev_t ev) {
  if (ev == EV_TXCOMPLETE) {
#ifdef DEBUG_SERIAL 
    Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)")); 
#endif
    os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
  }
}

void do_send(osjob_t* j) {
  
  unsigned long age = 0;
  unsigned long fix_age;
  float flat, flon;
  
  gpsdelay(1000);
  gps.f_get_position(&flat, &flon, &fix_age);
  
  if (fix_age == TinyGPS::GPS_INVALID_AGE) {
#ifdef DEBUG_SERIAL
      Serial.print("No fix detected @ ");
      Serial.println(os_getTime());
#endif
    os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(GPS_INTERVAL), do_send);
   } 
  else if (fix_age > 1000) {
#ifdef DEBUG_SERIAL
      Serial.print("Warning: possible stale data! @ ");
      Serial.println(os_getTime());
      Serial.print("Age: ");
      Serial.println(fix_age);
#endif
    os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(GPS_INTERVAL), do_send);
   } 
  else if (fix_age < 1000) {
#ifdef DEBUG_SERIAL
      Serial.print("Data is fresh. @ "); Serial.println(os_getTime());
      Serial.print("Age: ");Serial.println(fix_age);
#endif
    lat_gps_b = ((flat + 90) / 180) * 16777215;
    lon_gps_b = ((flon + 180) / 360) * 16777215;
    txBuffer[0] = ( lat_gps_b >> 16 ) & 0xFF;
    txBuffer[1] = ( lat_gps_b >> 8 ) & 0xFF;
    txBuffer[2] = lat_gps_b & 0xFF;

    txBuffer[3] = ( lon_gps_b >> 16 ) & 0xFF;
    txBuffer[4] = ( lon_gps_b >> 8 ) & 0xFF;
    txBuffer[5] = lon_gps_b & 0xFF;
  
    alt_gps = gps.f_altitude();
    txBuffer[6] = ( alt_gps >> 8 ) & 0xFF;
    txBuffer[7] = alt_gps & 0xFF;
  
    hdo_gps = gps.hdop()/10;
    txBuffer[8] = hdo_gps & 0xFF;

#ifdef DEBUG_SERIAL
      Serial.print("[INFO] HEX:  ");

      for(size_t i = 0; i < sizeof(txBuffer); i++) {
        char buffer[3];
        sprintf(buffer, "%02x", txBuffer[i]);
        // toLog = toLog + String(buffer);
        Serial.print( buffer);
      }
      Serial.println("");
 
      Serial.print("[INFO] Lat:  "); Serial.println( lat_gps_b );
      Serial.print("[INFO] Lng:  "); Serial.println( lon_gps_b );
      Serial.print("[INFO] Alt:  "); Serial.println( alt_gps );
      Serial.print("[INFO] HDOP: "); Serial.println( hdo_gps );
 #endif
      if (LMIC.opmode & OP_TXRXPEND) {
 #ifdef DEBUG_SERIAL
            Serial.println(F("OP_TXRXPEND, not sending"));
 #endif
      } else {
 #ifdef DEBUG_SERIALX
          Serial.println("SIMULATING sending of uplink packet...waiting for TX_COMPLETE");
          os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
 #endif
          // Prepare upstream data transmission at the next possible time.
          LMIC_setTxData2(1, txBuffer, sizeof(txBuffer), 0);
          // Next TX is scheduled after TX_COMPLETE event.

      }
   }
}

static void gpsdelay(unsigned long ms) {
  unsigned long start = millis();
  do {
    while (Serial1.available()) {
      if (gps.encode(Serial1.read())) {
#ifdef DEBUG_SERIAL
          Serial.println("NEW GPS Data...");
#endif
      }
    }
  } while (millis() - start < ms);
}

void setup() {
#ifdef DEBUG_SERIAL
    Serial.begin(19200); // port logging
    Serial.print("STARTING ...");
#endif

  // This serial port communicates with the GPS
  Serial1.begin(9600);

  os_init();
  LMIC_reset();
  LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
  // Set up the channels used by the Things Network, which corresponds
  // to the defaults of most gateways. Without this, only three base
  // channels from the LoRaWAN specification are used
  // LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  // LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
  // LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  // LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  // LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  // LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  // LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  // LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  // LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK, DR_FSK), BAND_MILLI);        // g2-band
  // Disable link check validation
  LMIC_setLinkCheckMode(0);
  // TTN uses SF9 for its RX2 window.
  LMIC.dn2Dr = DR_SF9;
  // disable channels not known to single channel gateways
  // LMIC_disableChannel(0);  // uncomment to disable channel 0
  // LMIC_disableChannel(1);  // uncomment to disable channel 1
  // LMIC_disableChannel(2);  // uncomment to disable channel 2
  // LMIC_disableChannel(3);  // uncomment to disable channel 3
  // LMIC_disableChannel(4);  // uncomment to disable channel 4
  // LMIC_disableChannel(5);  // uncomment to disable channel 5
  // LMIC_disableChannel(6);  // uncomment to disable channel 6
  // LMIC_disableChannel(7);  // uncomment to disable channel 7
  // LMIC_disableChannel(8);  // uncomment to disable channel 8
  // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
  LMIC_setDrTxpow(DR_SF7, 14);
  // Start job
  do_send(&sendjob);
}

void loop() {
  os_runloop_once();
}

