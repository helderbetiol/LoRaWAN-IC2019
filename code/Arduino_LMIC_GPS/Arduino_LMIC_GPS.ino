/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *H
 * This example sends a valid LoRaWAN packet with payload "Hello,
 * world!", using frequency and encryption settings matching those of
 * the (early prototype version of) The Things Network.
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in g1,
 *  0.1% in g2).
 *
 * Change DEVADDR to a unique address!
 * See http://thethingsnetwork.org/wiki/AddressSpace
 *
 * Do not forget to define the radio type correctly in config.h.
 *
 *
 * Required Library: 
 *    * https://github.com/matthijskooijman/arduino-lmic 
 * 
 * Require Hardware:
 *    * LoRa Shield + Arduino
 *    * LoRa GPS Shield + Arduino 
 *    * LoRa Mini etc. 
 *******************************************************************************/

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>
TinyGPS gps;
SoftwareSerial ss(4, 3); // Arduino RX, TX to conenct to GPS module.

// LoRaWAN NwkSKey, network session key
// This is the default Semtech key, which is used by the prototype TTN
// network initially.
//ttn
static const PROGMEM u1_t NWKSKEY[16] = { 0x38, 0x56, 0x6d, 0x23, 0x21, 0x82, 0xAF, 0xB1, 0x84, 0xA8, 0xE3, 0x8F, 0x36, 0x76, 0x90, 0x12 };
// LoRaWAN AppSKey, application session key
// This is the default Semtech key, which is used by the prototype TTN
// network initially.
//ttn
static const u1_t PROGMEM APPSKEY[16] = { 0xE0, 0xA5, 0x2C, 0x51, 0xF7, 0x70, 0x87, 0x51, 0xFE, 0x0A, 0x7A, 0x39, 0x66, 0x03, 0x89, 0x8D };
//
// LoRaWAN end-device address (DevAddr)
// See http://thethingsnetwork.org/wiki/AddressSpace
// ttn
static const u4_t DEVADDR = 0x01933008;


// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static uint8_t mydata[] = "hello";
static osjob_t initjob,sendjob,blinkjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 1;//20;

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 9,
    .dio = {2, 6, 7},
};

//GPS data
String datastring1="";
String datastring2="";
String datastring3="";
String datastring4="";
uint8_t datasend[40]= "";    //Storage  longtitude,latitude and altitude
char gps_lon[50]={"\0"};  //Storage GPS info
char gps_lat[20]={"\0"}; //Storage latitude
char gps_alt[20]={"\0"}; //Storage altitude
char gps_hdp[20]={"\0"}; //Storage hdop
static void smartdelay(unsigned long ms);

void do_send(osjob_t* j){
    float flat,flon,falt,hdp;
    unsigned long age;
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.

        //get gps data
        smartdelay(500);
//        if (ss.available())
//        {
//          //ss.print(Serial.read());
//          Serial.println(F("GPS ENCODE"));
//          Serial.println(ss.read());
//          gps.encode(ss.read());
//        }
//        Serial.println(F("GPS POS"));
        gps.f_get_position(&flat, &flon, &age);
        falt=gps.f_altitude();  //get altitude 
        hdp=gps.hdop();
        flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6;//save six decimal places 
        flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6;
        falt == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : falt, 2;//save two decimal places
        hdp  == TinyGPS::GPS_INVALID_HDOP? 0.0: hdp, 5;
        Serial.println(flon);
        Serial.println(flat);
        //Serial.println(falt);
        //Serial.println(hdp);
        datastring1 +=dtostrf(flat, 0, 6, gps_lat); 
        datastring2 +=dtostrf(flon, 0, 6, gps_lon);
        datastring3 +=dtostrf(falt, 0, 2, gps_alt);
        datastring4 +=dtostrf(hdp, 0, 2, gps_hdp);
        if(flon!=1000.000000)
        {
        strcat(gps_lon,",");
        strcat(gps_lon,gps_lat); 
        strcat(gps_lon,","); 
        strcat(gps_lon,gps_alt);
        strcat(gps_lon,","); 
        strcat(gps_lon,gps_hdp);
        strcat(gps_lon,",");
        memset(datasend, 0, sizeof(datasend));
        strcpy((char *)datasend,gps_lon);//the format of datasend is longtitude,latitude,altitude,hdop,
        Serial.println((char *)datasend);
        LMIC_setTxData2(1, datasend, sizeof(datasend)-1, 0);
        }
        else
        {
        LMIC_setTxData2(1, mydata, sizeof(mydata)-1, 0);
        }
        Serial.println(F("Packet queued"));
        Serial.println(LMIC.freq);
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    Serial.println(ev);
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            break;
        case EV_RFU1:
            Serial.println(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if(LMIC.dataLen) {
                // data received in rx slot after tx
                Serial.print(F("Data Received: "));
                Serial.write(LMIC.frame+LMIC.dataBeg, LMIC.dataLen);
                Serial.println();
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
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

void setup() {
    Serial.begin(9600);
    ss.begin(9600);       // SoftSerial port to get GPS data. 
    while(!Serial);
    Serial.println(F("Starting"));
    #ifdef VCC_ENABLE
    // For Pinoccio Scout boards
    pinMode(VCC_ENABLE, OUTPUT);
    digitalWrite(VCC_ENABLE, HIGH);
    delay(1000);
    #endif

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();
    //LMIC_setClockError(MAX_CLOCK_ERROR * 1/100);
    // Set static session parameters. Instead of dynamically establishing a session
    // by joining the network, precomputed session parameters are be provided.
    #ifdef PROGMEM
    // On AVR, these values are stored in flash and only copied to RAM
    // once. Copy them to a temporary buffer here, LMIC_setSession will
    // copy them into a buffer of its own again.
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
    #else
    // If not running an AVR with PROGMEM, just use the arrays directly 
    LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
    #endif
    
    //AU
    LMIC_selectSubBand(1);
    for (int i=9;i<16;i++)
    {
      LMIC_disableChannel(i);  // only the first channel 916.8Mhz works now. 
    }

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // Set data rate and transmit power (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF10,14);

    // Start job
    do_send(&sendjob);
}

void loop() {
    os_runloop_once();
}

static void smartdelay(unsigned long ms)
{
  //Serial.println(F("smartdelay"));
  //Serial.println(ss.read());
  unsigned long start = millis();
  do 
  {
    while (ss.available())
    {
      //ss.print(Serial.read());
      //Serial.println(F("GPS ENCODE"));
      gps.encode(ss.read());
    }
  } while (millis() - start < ms);
  
  //Serial.println(F("smartdelay end"));
}

