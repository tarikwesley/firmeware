// Including the necessary libraries
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <EmonLib.h>  // the lib emonlib needs to be the 12-bit one available. There are two, a 10-bit one for the Arduino and a 12-bit one for other boards that work with 12 bits, as is the case with the Heltec WiFi LoRa v2

#define pin_rele 23   // pin 23 = GPIO23 --> logic pin of rele
#define AmperePin 36  // pin 36 = GPIO0 --> current of imput
#define ADC_BITS 12  //define the read resolution to 12 bits
#define voltage 127.0 // Input Voltage

// --- Declaration of Variables for Reading the Current Sensor---
double current;
double power;
bool status;
uint8_t result;

// --- Sensor Object Instance ---
EnergyMonitor sensor;

// In format lsb
static const u1_t PROGMEM APPEUI[8] = { 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
void os_getArtEui(u1_t *buf) { memcpy_P(buf, APPEUI, 8);}
// In format lsb 
static const u1_t PROGMEM DEVEUI[8] = { 0xF8, 0x75, 0x05, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };
void os_getDevEui(u1_t *buf) { memcpy_P(buf, DEVEUI, 8);}
// In format msb
static const u1_t PROGMEM APPKEY[16] = { 0xB6, 0xC5, 0x1A, 0xA5, 0xE6, 0x14, 0xE0, 0xBC, 0xBC, 0xCA, 0x6A, 0xE4, 0xA3, 0x72, 0x8F, 0xB1 };
void os_getDevKey(u1_t *buf) {memcpy_P(buf, APPKEY, 16);}

static osjob_t sendjob;

const unsigned TX_INTERVAL = 1;

// Pin mapping Heltec Wifi Lora v2
const lmic_pinmap lmic_pins = {
  .nss = 18,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 14,
  .dio = { 26, 35, 34 },
};

void onEvent(ev_t ev) {
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
      LMIC_setLinkCheckMode(0);
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
      if (LMIC.txrxFlags & TXRX_ACK)
        Serial.println(F("Received ack"));
      if (LMIC.dataLen) {
        Serial.println(F("Received "));
        Serial.println(LMIC.dataLen);
        Serial.println(F(" bytes of payload"));
        //------ Added ----------------
        if (LMIC.dataLen == 1) {
            result = LMIC.frame[LMIC.dataBeg + 0];
          Serial.write(LMIC.frame + LMIC.dataBeg, LMIC.dataLen);
          if (result == 0) {
            Serial.println(" RESULT 0 = Rele off");
            digitalWrite(pin_rele, LOW);
          }
          if (result == 1) {
            Serial.println(" RESULT 1 = Rele on");
            digitalWrite(pin_rele, HIGH);
          }
        }
        Serial.println();
        //-----------------------------
      }
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
    default:
      Serial.println(F("Unknown event"));
      break;
  }
}

void do_send(osjob_t *j) {
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
    // Prepare upstream data transmission at the next possible time.
  // ========================================================================================================
    current = sensor.calcIrms(1480);
    power = current * voltage;
  // ========================================================================================================
  // --- Isolation Control ---     
    if(current > 0.25){           
      current = current;
      status = true;
    }else{
      current=0.0;
      status = false;
    }            
    if(current > 0.25){ //obs 
      power = power;
    }else{
      power=0.0;
    }
  // ========================================================================================================
    Serial.println("********************");
    Serial.print("Current = ");
    Serial.print(current);
    Serial.println(" A");
    Serial.print("Power = ");
    Serial.print(power);
    Serial.println(" W");
    Serial.print("Status = ");
    Serial.print(status);
    Serial.println("********************");
  // ========================================================================================================
    uint16_t amp = current * 100;
    uint16_t watt = power;
    uint8_t sts = status;
    
    uint8_t payload[5];
    payload[0] = amp >> 8;
    payload[1] = amp;
    payload[2] = watt >> 8;
    payload[3] = watt;
    payload[4] = sts;
    
    Serial.println("################################################################################");
    Serial.println(payload[0]);
    Serial.println(payload[1]);
    Serial.println(payload[2]);
    Serial.println(payload[3]);
    Serial.println(payload[4]);
    Serial.println("################################################################################");
    LMIC_setTxData2(1, payload, sizeof(payload), 0);
    Serial.println(F("Packet queued"));
  }
  // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
  Serial.begin(115200);
  Serial.println(F("Starting"));
  analogReadResolution(ADC_BITS);  
  sensor.current(AmperePin, 9.09);  // Current: input pin, calibration. 2000/x = 9.09 

  // pinMode(pin_rele, OUTPUT);

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
  // Start job (sending automatically starts OTAA too)
  do_send(&sendjob);
}

void loop() {
  os_runloop_once();
}