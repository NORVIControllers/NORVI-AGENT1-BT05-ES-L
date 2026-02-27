/*
*NORVI-AT01-BT5-ES-L FINAL
*2025/11/17.
*/
#include <WiFi.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_ADS1X15.h>
#include <Wire.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

#define CONFIRMED_MSG_RETRY_COUNT 3

Adafruit_ADS1115 ads1;
#define VOLTAGE_DIVIDER_RATIO 0.40112

#define BUTTON_PIN   35

#define RELAY_PIN  13

#define LED_PIN 25    
#define NUM_LEDS 1 

int analog_value = 0;
bool mode = true; 
bool initialized = false;

static const u1_t PROGMEM APPEUI[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
void os_getArtEui (u1_t* buf) {
  memcpy_P(buf, APPEUI, 8);
}
static const u1_t PROGMEM DEVEUI[8] = {0x4B, 0x11, 0x3F, 0xB1, 0x3C, 0xBE, 0xD6, 0x56};
void os_getDevEui (u1_t* buf) {
  memcpy_P(buf, DEVEUI, 8);
}
static const u1_t PROGMEM APPKEY[16] = {0xF9, 0x65, 0xE4, 0xED, 0xFF, 0x8A, 0x89, 0x27, 0x23, 0xA6, 0xB7, 0x42, 0x2F, 0x05, 0x8E, 0x9F};
void os_getDevKey (u1_t* buf) {
  memcpy_P(buf, APPKEY, 16);
}
static uint8_t mydata[] = "Hello, world!";
static osjob_t sendjob;
const unsigned TX_INTERVAL = 60;
const lmic_pinmap lmic_pins = {
  .nss = 2,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 5,
  .dio = {4, 15},
};
void printHex2(unsigned v) {
  v &= 0xff;
  if (v < 16)
    Serial.print('0');
  Serial.print(v, HEX);
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
      {
        u4_t netid = 0;
        devaddr_t devaddr = 0;
        u1_t nwkKey[16];
        u1_t artKey[16];
        LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
        Serial.print("netid: ");
        Serial.println(netid, DEC);
        Serial.print("devaddr: ");
        Serial.println(devaddr, HEX);
        Serial.print("AppSKey: ");
        for (size_t i = 0; i < sizeof(artKey); ++i) {
          if (i != 0)
            Serial.print("-");
          printHex2(artKey[i]);
        }
        Serial.println("");
        Serial.print("NwkSKey: ");
        for (size_t i = 0; i < sizeof(nwkKey); ++i) {
          if (i != 0)
            Serial.print("-");
          printHex2(nwkKey[i]);
        }
        Serial.println();
      }
      // Disable link check validation (automatically enabled
      // during join, but because slow data rates change max TX+
      // size, we don't use it in this example.
      LMIC_setLinkCheckMode(0);
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
        Serial.print(F("Received "));
        Serial.print(LMIC.dataLen);
        Serial.println(F(" bytes of payload"));
      }
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

void do_send(osjob_t* j) {
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
    // Prepare upstream data transmission at the next possible time.
    LMIC_setTxData2(1, mydata, sizeof(mydata) - 1, 0);
    Serial.println(F("Packet queued"));
  }
  // Next TX is scheduled after TX_COMPLETE event.
}

bool lastMode = true;  

int receivedValue = 0;

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);





void setup() {
 
 Serial.begin(115200);
 
 mode = true; 
    setupFirstCode(); 
    lastMode = mode;
    
 cmd();
}


void loop() {
  
  if (Serial.available()) {
    char input = Serial.read();
    
    if (input == '1') {
      mode = true;
      
   } 
   else if (input == '2') {
      mode = false; 
      
    }
  }

  
  if (mode != lastMode) {
    if (mode) {
      setupFirstCode();
    } else {
      setupSecondCode();
    }
    lastMode = mode; 
  }

  
  if (mode) {
    firstCode();
  } else {
    secondCode();
  }

  
}

void setupFirstCode() {
 Serial.begin(115200);
  
  WiFi.mode(WIFI_MODE_APSTA);
  
  strip.begin();
  strip.show(); 
  
  delay(300);
  wifi_test();
  delay(100);
  RGB_TEST();
  delay(100);
  
  pinMode(34, INPUT);
  pinMode(36, INPUT);
  pinMode(27, INPUT);
  
  pinMode(BUTTON_PIN, INPUT);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);
  
   Wire.begin(21,22);
   
   if (!ads1.begin(0x48)) {
    Serial.println("ADS1115 initialization failed");
    while (1);
  }
  ads1.setGain(GAIN_ONE); 

}

void firstCode() {
  
 
 if (Serial.available() > 0) {
    String inputString = Serial.readStringUntil('\n'); 
    
    receivedValue = inputString.toInt(); 
    
    Serial.println(receivedValue);
  }

  
 int16_t adc0, adc1, adc2, adc3;
 Serial.print("I1: ");Serial.println(digitalRead(34));
 Serial.print("I2: ");Serial.println(digitalRead(36));
 Serial.print("I3: ");Serial.println(digitalRead(27));
 Serial.println("");  
 
 Serial.print("BUTTON: ");Serial.println(digitalRead(BUTTON_PIN));
 Serial.println("");  

 
int inputState = digitalRead(BUTTON_PIN);
if (inputState == HIGH) {
    digitalWrite(RELAY_PIN, HIGH);   // Relay ON
  } else {
    digitalWrite(RELAY_PIN, LOW);    // Relay OFF
  }
  
   adc0 = ads1.readADC_SingleEnded(0);
   adc1 = ads1.readADC_SingleEnded(1);
   adc2 = ads1.readADC_SingleEnded(2);
   
   float Voltage1 = adc0 *  0.125 / 1000.0 / VOLTAGE_DIVIDER_RATIO;
   float Voltage2 = adc1 *  0.125 / 1000.0 / VOLTAGE_DIVIDER_RATIO;
   float Voltage3 = adc2 *  0.125 / 1000.0 / VOLTAGE_DIVIDER_RATIO;
    
    Serial.print("Voltage 0 ");  Serial.print(": "); Serial.print(Voltage2); Serial.println(" V");
    Serial.print("Voltage 1 ");  Serial.print(": "); Serial.print(Voltage3); Serial.println(" V");
    Serial.print("Voltage 2 ");  Serial.print(": "); Serial.print(Voltage1); Serial.println(" V");
  
  delay(1000);
 
 Serial.println("-----------------");  
}


void setupSecondCode() {
  os_init();
  LMIC_reset();
    LMIC.dn2Dr = DR_SF10;
    LMIC_setDrTxpow(DR_SF10, 26);
  LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100); // Set clock error to 1%
  LMIC_setAdrMode(1); // Enable adaptive data rate
  do_send(&sendjob);
}

void secondCode() {
  static uint8_t mydata[] = "Hello, World!";
  // Transmit unconfirmed message (set confirmed to false)
  LMIC_setTxData2(1, mydata, sizeof(mydata) - 1, true);
  os_runloop_once();
}



void RGB_TEST() {

  strip.setPixelColor(0, strip.Color(255, 0, 0)); // Red
  strip.show();
  delay(500);

  strip.setPixelColor(0, strip.Color(0, 255, 0)); // Green
  strip.show();
  delay(500);

  strip.setPixelColor(0, strip.Color(0, 0, 255)); // Blue
  strip.show();
  delay(500);

  strip.setPixelColor(0, strip.Color(255, 255, 255)); // White
  strip.show();
  delay(500);

  strip.setPixelColor(0, strip.Color(0, 0, 0)); // Off
  strip.show();
  delay(500);

 
}




void wifi_test(){
  
Serial.println("");
String str_macAddress;
byte mac[6];

WiFi.macAddress(mac);

   str_macAddress = (String(mac[0] >> 4,  HEX) + String(mac[0] & 0x0F, HEX)) + (":") +
                   (String(mac[1] >> 4,  HEX) + String(mac[1] & 0x0F, HEX)) + (":") +
                   (String(mac[2] >> 4,  HEX) + String(mac[2] & 0x0F, HEX)) + (":") +
                   (String(mac[3] >> 4,  HEX) + String(mac[3] & 0x0F, HEX)) + (":") +
                   (String(mac[4] >> 4,  HEX) + String(mac[4] & 0x0F, HEX)) + (":") +
                   (String(mac[5] >> 4,  HEX) + String(mac[5] & 0x0F, HEX));
  str_macAddress.toUpperCase();
  
  Serial.println("MAC Address: " + str_macAddress);

  String ssid =  str_macAddress; 
  const char* password = "12345678";   

  Serial.println("Setting up the Wi-Fi Access Point...");
  WiFi.softAP(ssid.c_str(), password);

  IPAddress IP = WiFi.softAPIP();
  Serial.print("Access Point IP address: ");
  Serial.println(IP);

  Serial.print("Access Point SSID: ");
  Serial.println(ssid);

  Serial.println("Wi-Fi Access Point is active!");
  Serial.println("");
}
