kan#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include<TinyGPS++.h>
#include "STM32LowPower.h"
// --- Lora Port ------------
#define SCK_GPIO        PB13
#define MISO_GPIO       PB14
#define MOSI_GPIO       PB15
#define NSS_GPIO        PB12
#define RESET_GPIO      PB10
#define DIO0_GPIO       PB11
#define DIO1_GPIO       PC13
#define DIO2_GPIO       PB9

// --- GPS Port -------------
#define GPS_PORT      Serial4
#define GPS_BAUD      115200
#define GPS_RX_PIN    PC11
#define GPS_TX_PIN    PC10
#define GPS_RST_PIN   PB2
#define GPS_LS_PIN    PC6

// --- Interval Setting -----
#define SLEEP_PERIODE 60000 // in milisecond
#define TX_PERIODE    15    // in second

// --- Output Port ----------
#define PANIC         PA0  // set PA0 as panic output
#define HEARTBEAT     PA4

/*
  LoRa Parameters
  Appskey 0000000000000000886a6c0b7af4ff6d
  Newskey bd8c8c443410ba450000000000000000
  DevAddr 2cdc4ea6
  See how to obtain Lora Parameters here : https://antares.id/id/register-perangkat-lora.html
  This device Use OTAA Join Mode and Use Class A
*/
static const u1_t PROGMEM APPSKEY[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x88, 0x6a, 0x6c, 0x0b, 0x7a, 0xf4, 0xff, 0x6d };
static const u1_t PROGMEM NWKSKEY[16] = { 0xbd, 0x8c, 0x8c, 0x44, 0x34, 0x10, 0xba, 0x45, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
static const u4_t DEVADDR = 0x2cdc4ea6;
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }
const unsigned TX_INTERVAL = TX_PERIODE; // Uplink Interval
static uint8_t mydata[60];
static osjob_t sendjob;
int beatThreshold;
// Variable declaration of GPS data
float lon, lat;
bool gpsValid = false;
uint32_t deviceID = 0;
HardwareSerial Serial4(GPS_RX_PIN, GPS_TX_PIN);
TinyGPSPlus gps;
//timer for pulseData
HardwareTimer *myTim = new HardwareTimer(TIM3);
static uint8_t pulseData[3];

// GPS Initialization
void gps_init() {
  GPS_PORT.begin(GPS_BAUD, SERIAL_8N1);
  pinMode(GPS_LS_PIN, OUTPUT);
  pinMode(GPS_RST_PIN, OUTPUT);
  digitalWrite(GPS_LS_PIN,  HIGH);

  // reset GPS
  digitalWrite(GPS_RST_PIN, LOW);   delay(200);
  digitalWrite(GPS_RST_PIN, HIGH);  delay(100);
  pinMode(GPS_RST_PIN, INPUT);

  // config gps
  GPS_PORT.write("@BSSL 0x25\r\n"); delay(250);
  GPS_PORT.write("@GNS 0x3\r\n");   delay(250);
  GPS_PORT.write("@GSR\r\n"); /* hot  start */
  delay(250);
}

// GPS routine loop and data obtaining
void gps_loop() {
  while (GPS_PORT.available())
    if (gps.encode(GPS_PORT.read())) {
      if ((gpsValid = gps.location.isValid())) {}
      sprintf((char*)mydata, "[\"%08X\",\"%f\",\"%f\",\"%d\"]", deviceID, gps.location.lat(), gps.location.lng(), getpulse() );
    }
}

// LMIC GPIO configuration
const lmic_pinmap lmic_pins = {
  .nss = NSS_GPIO,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = RESET_GPIO,
  .dio = {DIO0_GPIO, DIO1_GPIO, DIO2_GPIO},
};

// LMIC routine loop
void do_send(osjob_t* j) {
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
    // Prepare upstream data transmission at the next possible time.
    LMIC_setTxData2(1, mydata, strlen((char*)mydata), 0);
    Serial.println(F("Packet queued"));
    if (gpsValid) {
      Serial.write(mydata, strlen((char*)mydata));
      Serial.println();
    } else {
      Serial.println("GPS NOT VALID");
    }
  }
}

uint32_t getpulse(){
  while(analogRead(HEARTBEAT)<beatThreshold){
  }
  uint32_t timer0 = myTim->getCount(MICROSEC_FORMAT); 
  while(analogRead(HEARTBEAT)<beatThreshold){
  }
  uint32_t timer1 = myTim->getCount(MICROSEC_FORMAT);   
  uint32_t diff = timer2-timer1;
  uint32_t pulse = 60000000/diff;
  return pulse;
}
void onEvent (ev_t ev) {
  Serial.print(os_getTime());
  Serial.print(": ");
  switch (ev) {
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
    /*
      || This event is defined but not used in the code. No
      || point in wasting codespace on it.
      ||
      || case EV_RFU1:
      ||     Serial.println(F("EV_RFU1"));
      ||     break;
    */
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      break;
    case EV_TXCOMPLETE:
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      digitalWrite(PA0, LOW); // turn off the output first
      // If there's any downlink data from gateway
      if (LMIC.dataLen != 0 || LMIC.dataBeg != 0) {
        if (LMIC.txrxFlags & TXRX_ACK)
        Serial.println(F("Received ack"));
        Serial.print(F("Received "));
        Serial.print(LMIC.dataLen);
        Serial.println(F(" bytes of payload"));
        Serial.print(F("Data Received: "));
        Serial.write(LMIC.frame + LMIC.dataBeg, LMIC.dataLen);
        int int_data = LMIC.frame[LMIC.dataBeg];
        Serial.println();
        Serial.print("Data In Interger: ");
        Serial.println(int_data);
        if (int_data == 49)  digitalWrite(PANIC, HIGH); // Device out of range, set panic pin to HIGH when get downlink data '1'
        else                 digitalWrite(PANIC, LOW);
      }

      Serial.println("ACSIP is going to sleep");
      Serial.println("Zzz");
      delay(500);
      LowPower.deepSleep(SLEEP_PERIODE);
      Serial.println("ACSIP wake up!");

      // Schedule next transmission
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);

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
    /*
      || This event is defined but not used in the code. No
      || point in wasting codespace on it.
      ||
      || case EV_SCAN_FOUND:
      ||    Serial.println(F("EV_SCAN_FOUND"));
      ||    break;
    */
    case EV_TXSTART:
      Serial.println(F("EV_TXSTART"));
      break;
    case EV_TXCANCELED:
      Serial.println(F("EV_TXCANCELED"));
      break;
    case EV_RXSTART:
      /* do not print anything -- it wrecks timing */
      break;
    case EV_JOIN_TXCOMPLETE:
      Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
      break;
    default:
      Serial.print(F("Unknown event: "));
      Serial.println((unsigned) ev);
      break;
  }
}

void setup() {
  ///pulse
  int beat;
  int highest = 0;
  int lowest = 0;
  pinMode(HEARTBEAT,OUTPUT); 
  highest = analogRead(HEARTBEAT);
  lowest = analogRead(HEARTBEAT);
  // sample average heartbeat pulse
  while(i<10){
    beat = analogRead(HEARTBEAT);
    if(beat == 0)
      Serial.print("Check Pulse Sensor Connection");
    else{
      i++;
      if(beat > highest){
        highest = beat;
      }
      else if(beat <lowest){
        lowest = beat;
      }   
    }    
  }
  beatThreshold = (highest - lowest)/2;
  pinMode(PANIC, OUTPUT);
  Serial.setTx(PA9);
  Serial.setRx(PA10);
  while (!Serial);
  Serial.begin(115200);

  // get device ID
  deviceID = DEVADDR;
  Serial.printf("Device ID: %08X\n", deviceID);

  SPI.setMISO(MISO_GPIO);
  SPI.setMOSI(MOSI_GPIO);
  SPI.setSCLK(SCK_GPIO);
  // Slave Select pin is driven by RF driver
  SPI.begin();
  Serial.println("SPI: Started!");
  delay(1000);
  // GPS Init
  gps_init();
  // LMIC init
  os_init();
  Serial.println(F("LMIC: Init!"));

  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();
  Serial.println(F("LMIC: reset!"));

#ifdef PROGMEM
  uint8_t appskey[sizeof(APPSKEY)];
  uint8_t nwkskey[sizeof(NWKSKEY)];
  memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
  memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
  LMIC_setSession (0x13, deviceID, nwkskey, appskey);
  Serial.println(F("STM: With AVR"));
#else
  // If not running an AVR with PROGMEM, just use the arrays directly
  LMIC_setSession (0x13, DEVADDR, NWKSKEY, APPSKEY);
  Serial.println(F("STM: Non AVR"));
#endif
  // Setting channel according to Antares chanel, see here : https://antares.id/id/register-perangkat-lora.html
  LMIC_setupChannel(0, 921200000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(1, 921400000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
  LMIC_setupChannel(2, 921600000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(3, 921800000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(4, 922000000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(5, 922200000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(6, 922400000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(7, 922600000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  //LMIC_setupChannel(8, 921400000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_CENTI);      // g2-band
  Serial.println(F("LMIC: Channel set!"));

  // Disable link check validation
  LMIC_setLinkCheckMode(0);
  Serial.println(F("LMIC: Check Link"));

  // Antares uses SF10 for its RX2 window.
  LMIC.dn2Dr = DR_SF10; 
  Serial.println(F("LMIC: RX2 set DR_SF10"));

  // Set data rate and transmit power for uplink
  LMIC_setDrTxpow(DR_SF10, 16);
  Serial.println(F("LMIC: TX set DR_SF10"));

  //Low Power Init
  LowPower.begin();
  Serial.println("Low Power : Init!");

  // Start job
  do_send(&sendjob);
}

void loop() {
  os_runloop_once();
  gps_loop();
}
