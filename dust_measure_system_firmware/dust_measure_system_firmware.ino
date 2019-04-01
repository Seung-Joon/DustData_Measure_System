//     Version 1.0                     
//     Last updated at 2019.04.01      
//     Made by Seung-Jun Park          
//                                     
//     - Using Sensor  list -              
//       1. PMS7003                    
//       2. DHT22                      
//       3. Voltage sensor 1 (Using for system batter voltage measure), (consist of 7500R, 30000R)       
//       4. voltage sensor 2 (Using for solar panel voltage measure), (consist of 10000K,        
//                                   
//     - Pin list -                
//        D13: DHT22
//        D9: SoftWareSerialPin(Tx)     
//        D8: SoftWareSerialPin(Rx)  
//        D7: LCD_RS_pin               
//        D6: LCD_EN_pin    
//        D5: LCD_data4_pin
//        D4: LCD_data5_pin
//        D3: LCD_data6_pin
//        D2: LCD_data7_pin
//        A7: System_voltage_measure_pin
//        A6: Solar_panel_voltage_measure_pin

/*      - Bit composition - (configurated by hexa code)
             {STARTING_BIT,
              SYSTEM_ID,
              PM1.0,
              PM2.5,
              PM10,
              Float_FLAG,
              SYSTEM_VOLTAGE,
              VOLTAGE_RESOLUTION,
              Float_FLAG,
              SOLAR_VOLTAGE,
              SOLAR_RESOLUTION,
              TEMPURATURE,
              HUMIDITY,
              ENDBIT
              }*/

#include <SoftwareSerial.h>
#include <LiquidCrystal.h>
#include "DHT.h"
#define DHTPIN 13
#define DHTTYPE DHT22

#define SYSTEM_ID '2' //System identification number
#define REQUESTING_CODE '^' // Data Requesting command
#define STARTING_BIT 0x00
#define END_BIT 0xfffe
#define VOLTAGE_RESOLUTION 100
#define TH_RESOLUTION 10
#define FLAG_FLOT 0xfe

#define ENABLE_DEBUG_MODE
SoftwareSerial Serial1(8, 9); // RX, TX
LiquidCrystal lcd(7, 6, 5, 4, 3, 2);
DHT dht(DHTPIN, DHTTYPE);

int incomingByte = 0; // for incoming serial data
const int MAX_FRAME_LEN = 64;
char frameBuf[MAX_FRAME_LEN];
int detectOff = 0;
int frameLen = MAX_FRAME_LEN;
bool inFrame = false;
char printbuf[256];
unsigned int calcChecksum = 0;

byte customChar[] = {B00110, B01001, B01001, B00110, B00000, B00000, B00000, B00000};

struct dust_data {
  byte  frameHeader[2];
  unsigned int  frameLen = MAX_FRAME_LEN;
  unsigned int  pm1;
  unsigned int  pm2_5;
  unsigned int  pm10;
  unsigned int  checksum;
};

struct system_data {
  float v_battery = 0.0;
  float v_solar = 0.0;
  char ID = '2';
  float tempurature = 0.0;
  float humidity = 0.0;
};

struct T_H_Value {
  float tempurature = 0.0;
  float humidity = 0.0;
};
struct system_data system_data;
struct dust_data dust_data;

float getV_battery(int pin) {
  return ((analogRead(pin) * 5.0) / 1024.0) / (7500.0 / ( 30000.0 + 7500.0));
}

float getV_solar(int pin) {
  return ((analogRead(pin) * 5.0) / 1024.0) / (1000.0 / ( 10000.0 + 1000.0));
}

void getTempHumidityData(struct T_H_Value &th) {
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  float f = dht.readTemperature(true);


  if (isnan(h) || isnan(t) || isnan(f)) {
    th.humidity = 0.00;
    th.tempurature = 0.00;
    return;
  }

  th.tempurature = dht.computeHeatIndex(t, h, false);
  th.humidity = h;
}

void getDustData(struct dust_data &d) {
  Serial1.begin(9600);
  bool packetReceived = false;
  calcChecksum = 0;

  while (!packetReceived) {
    if (Serial1.available() > 32) {
      int drain = Serial1.available();

      for (int i = drain; i > 0; i--) {
        Serial1.read();
      }
    }

    if (Serial1.available() > 0) {
      incomingByte = Serial1.read();

      if (!inFrame) {
        if (incomingByte == 0x42 && detectOff == 0) {
          frameBuf[detectOff] = incomingByte;
          d.frameHeader[0] = incomingByte;
          calcChecksum = incomingByte; // Checksum init
          detectOff++;
        }

        else if (incomingByte == 0x4D && detectOff == 1) {
          frameBuf[detectOff] = incomingByte;
          d.frameHeader[1] = incomingByte;
          calcChecksum += incomingByte;
          inFrame = true;
          detectOff++;
        }
      }

      else {
        frameBuf[detectOff] = incomingByte;
        calcChecksum += incomingByte;
        detectOff++;
        unsigned int  val = (frameBuf[detectOff - 1] & 0xff) + (frameBuf[detectOff - 2] << 8);
        switch (detectOff) {
          case 4:
            d.frameLen = val;
            frameLen = val + detectOff;
            break;
          case 6:
            d.pm1 = val;
            break;
          case 8:
            d.pm2_5 = val;
            break;
          case 10:
            d.pm10 = val;
            break;
          case 32:
            d.checksum = val;
            calcChecksum -= ((val >> 8) + (val & 0xFF));
            break;
          default:
            break;
        }
        if (detectOff >= frameLen) {
          packetReceived = true;
          detectOff = 0;
          inFrame = false;
        }
      }
    }
  }
  Serial1.end();
  //return (calcChecksum == d.checksum);
}

void getSystemData(struct system_data &sys) {
  struct T_H_Value th;
  getTempHumidityData(th);
  sys.v_battery = getV_battery(A7);
  sys.v_solar = getV_solar(A6);
  sys.tempurature = th.tempurature;
  sys.humidity = th.humidity;
  sys.ID = NULL;
}

void lcd_drawMain(struct system_data sys, struct dust_data D_data) {
  sys.ID = SYSTEM_ID;
  lcd.setCursor(0, 0);
  lcd.print("ID:" + String(sys.ID) + " " + String(sys.v_battery) + "V " + String(int(sys.tempurature)) + " C");
  if (sys.v_battery < 10) {
    lcd.setCursor(13, 0);
    lcd.write(byte(0));
    lcd.setCursor(15, 0);
    lcd.write(" ");
  }
  else {
    lcd.setCursor(14, 0);
    lcd.write(byte(0));

  }
  lcd.setCursor(0, 1);
  lcd.print("DuV={" + String(D_data.pm1) + "," + String(D_data.pm2_5) + "," + String(D_data.pm10) + "}");
}

void DATA_TRANSMITTION(struct system_data sys, struct dust_data D_data) {
  char tempBuffer[64];
  sprintf(tempBuffer, "%x %x %x %x %x %x %x %x %x %x %x %x %x %x", STARTING_BIT, SYSTEM_ID, D_data.pm1, D_data.pm2_5, D_data.pm10, FLAG_FLOT, int(sys.v_battery * 100), 100, FLAG_FLOT, int(sys.v_solar * 100), 100, int(sys.tempurature), int(sys.humidity), int(END_BIT));
  Serial.print(tempBuffer);
  return;
}

void setup() {
  Serial.begin(9600);
  lcd.begin(16, 2);
  dht.begin();
  lcd.createChar(0, customChar);
}

int k = 1; //convert to write 

void loop() {
  getDustData(dust_data);
  getSystemData(system_data);
  lcd_drawMain(system_data, dust_data);
  if(!k){
    Serial.print("\r\n");
    k = 1;
  } 
}

void serialEvent() {
  if (Serial.find('^')) {
    DATA_TRANSMITTION(system_data, dust_data);
    k = 0;
  }
  else return;
}
