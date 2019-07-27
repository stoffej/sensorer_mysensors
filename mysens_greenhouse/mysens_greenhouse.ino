#include <Wire.h>
#include <hd44780.h>
#include <BH1750.h>
#include <hd44780ioClass/hd44780_I2Cexp.h> // i2c expander i/o class header
#include <SPI.h>
#include "DHT.h"
#include <TimeLib.h>


#define MY_RADIO_RF24 // Enable and select radio type attached
#define MY_NODE_ID 16
//#define DEBUG_ON   // comment out to supress serial monitor output
#include <MySensors.h>

#ifdef DEBUG_ON
#define MY_DEBUG      // Enable debug prints to serial monitor mysensors
#define DEBUG_PRINT(x)   Serial.print(x)
#define DEBUG_PRINTLN(x) Serial.println(x)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#define SERIAL_START(x)
#endif

#define SKETCH_NAME "Greenhouse"
#define SKETCH_MAJOR_VER "0"
#define SKETCH_MINOR_VER "5"


#define CHILD_ID_HUM 0
#define CHILD_ID_TEMP 1
#define CHILD_ID_LIGHT 2
#define CHILD_ID_RELAY 3
#define CHILD_ID_SOILA 8
#define CHILD_ID_SOILB 9
//#define CHILD_ID_SOILC 10

#define DHT_UPDATE_INTERVAL  4000 // in milliseconds
#define WATER_PUMP_TIMEOUT   1800000UL // 30 minutes in milliseconds
#define GET_TIME 3600000UL // time request 1 hour

#define RELAY_PIN 7  // Arduino Digital I/O pin number for first relay (second on pin+1 etc)

MyMessage msgHum(CHILD_ID_HUM, V_HUM);
MyMessage msgTemp(CHILD_ID_TEMP, V_TEMP);
MyMessage msgLight(CHILD_ID_LIGHT, V_LEVEL);
MyMessage msgPrefix(CHILD_ID_LIGHT, V_UNIT_PREFIX);  //custom unit for light sensor
MyMessage msgRelay(CHILD_ID_RELAY, V_STATUS);  

MyMessage msgSoilA(CHILD_ID_SOILA, V_LEVEL);  
MyMessage msgSoilB(CHILD_ID_SOILB, V_LEVEL);  
//MyMessage msgSoilC(CHILD_ID_SOILC, V_LEVEL);  


/* for clock */
bool clockUpdating = false;
bool showTime = true;
bool metric = true;
static unsigned long watchdogTimeout;

/* default sensor value */
float humidity = 255;
float temperature = 255;
uint16_t lux = 0 ;
int moistureA = 0;
int moistureB = 0;
//int moistureC = 32000;
int relay_state=0; //default is off
int rawA;

enum State {
  Init,  Watering,  Idle,  Error
};

State state ;
State lastState;
bool initialValueSent = false;

/* send next msg type
   To avoid flooding the mysensor network
 */
enum msgPresent {
  msgSendTemperature,  msgSendHumidity,  msgSendLux,  msgSendSoilA,  msgSendSoilB, msgSendRelay
};

msgPresent msgSend = msgSendTemperature;

hd44780_I2Cexp lcd(0x27); // declare lcd object: auto locate & auto config expander chip
BH1750 lightMeter;
byte raindrop[8] = {0x4, 0x4, 0xA, 0xA, 0x11, 0xE, 0x0,}; // lcd raindrop indicator
byte clock[8] = {0x0, 0xe, 0x15, 0x17, 0x11, 0xe, 0x0}; // lcd clock indicator
byte smiley[8] = { B00000,  B10001,  B00000,  B00000,  B10001,  B01110,  B00000,}; // lcd smiley

DHT dht; //DHT22 på pinne 5



void before()
{
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, relay_state);
}

void setup()
{
   DEBUG_PRINT("current radio channel: ");
   DEBUG_PRINTLN(MY_RF24_CHANNEL);
   int status;
   lightMeter.begin();
	 status = lcd.begin(16, 2);
   if(status) // non zero status means it was unsuccesful
	 {
	   status = -status; // convert negative status value to positive number
     Serial.print("error: lcd.begin() ");
		 // begin() failed so blink error code using the onboard LED if possible
		 hd44780::fatalError(status); // does not return
	 }
   lcd.clear();
   lcd.backlight();
   lcd.createChar(0, clock);
   lcd.createChar(1, raindrop);
   lcd.createChar(2, smiley);   
  
   dht.setup(5); // data pin 5
   analogReference(DEFAULT);
  //
  lcd.setCursor(0, 0);
  lcd.print(F(" Getting Time..  "));
  lcd.setCursor(15, 0);
  lcd.write(byte(0));
  lcd.setCursor(0, 1);
  int clockCounter = 0;
  while (timeStatus() == timeNotSet && clockCounter < 21)
  {
    requestTime();
    DEBUG_PRINTLN(F("Requesting time from Gateway:"));
    wait(600);
    lcd.print(".");
    clockCounter++;
    if (clockCounter > 32)
    {
      DEBUG_PRINTLN(F("Failed initial clock synchronization!"));
      lcd.clear();
      lcd.print(F("  Failed Clock  "));
      lcd.setCursor(0, 1);
      lcd.print(F(" Syncronization "));
      wait(2000);
      break;
    }
  }
}

void presentation() 
{
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo(SKETCH_NAME, SKETCH_MAJOR_VER "." SKETCH_MINOR_VER);

  present(CHILD_ID_LIGHT, S_LIGHT);
  wait(500);
  present(CHILD_ID_HUM, S_HUM);
  wait(500);  
  present(CHILD_ID_TEMP, S_TEMP);
  wait(500);  
  present(CHILD_ID_RELAY, S_BINARY);
  wait(500);  
  present(CHILD_ID_SOILA, S_MOISTURE);
  wait(500);  
  present(CHILD_ID_SOILB, S_MOISTURE);
 
  metric = getControllerConfig().isMetric;
}

void loop() 
{
  if (!initialValueSent) {
    Serial.println("Sending initial value");
    send(msgRelay.set(relay_state),0);
    Serial.println("Requesting initial value from controller");
    request(CHILD_ID_RELAY, V_STATUS);
    wait(2000, C_SET, V_STATUS);
  }
  updateClock();
  readSensors();
  sendMsg();
  updateDisplay();
  watchdog();
}

void setRelay(int value)
{
  relay_state=value;
  saveState(CHILD_ID_RELAY, relay_state);
  if(value == 1 )
  {
    state = Watering;
    digitalWrite(RELAY_PIN, HIGH );
    watchdogTimeout=millis();
    send(msgRelay.set(relay_state),0);
  }
  else
  {
    state = Idle;
    digitalWrite(RELAY_PIN, LOW );
    send(msgRelay.set(relay_state),0);    
  }
}

void updateDisplay(void)
{
  static unsigned long lastUpdateLcdTime;
  if(millis()-lastUpdateLcdTime  >= 400)     
  {
    lcd.setCursor(0,0);
    lcd.clear();
    if(state == Watering)
    {
      lcd.write(byte(1));  
      lcd.print(" Vattnar");
    }
    else
    {
      lcd.print(" ");
      lcd.write(byte(2));    
      lcd.print("    ");
      lcd.print(hour());
      lcd.print(":");   
      lcd.print(minute());         
    }
    
    lcd.setCursor(0, 1);
    lcd.print(temperature,0);
    lcd.print((char)223);
    lcd.print(" ");
    lcd.print(moistureA);
    lcd.print("% ");
    lcd.print(moistureB);
    lcd.print("%");    
    lcd.print(" ");
    lcd.print(lux);
    lcd.print("Lx");       
    lastUpdateLcdTime = millis();
  }
}


void sendMsg(void)
{
  static int lastMinuteSendTime=0;
  if(minute() != lastMinuteSendTime)
  {
    lastMinuteSendTime = minute();
    switch(msgSend)
    {
      case msgSendTemperature:
          send(msgTemp.set(temperature, 0));
          msgSend = msgSendHumidity;
          break;
      case msgSendHumidity:
          msgSend = msgSendLux;    
          send(msgHum.set(humidity, 0));
          break;        
      case msgSendLux:
          msgSend = msgSendSoilA;    
          send(msgLight.set(lux),0);   
          break;        
      case msgSendSoilA:
          msgSend = msgSendSoilB;    
          send(msgSoilA.set(moistureA),0);
          break;        
      case msgSendSoilB:
          msgSend = msgSendRelay;    
          send(msgSoilB.set(moistureB),0);
          break;        
      case msgSendRelay:
          msgSend = msgSendTemperature;    
          send(msgRelay.set(relay_state),0);        
          break;        
      default:
          DEBUG_PRINTLN(F("STOFFE Error in msgSend enum"));
          break;
    }
  }
}


void readSensors(void)
{
  /* for DHT sensor*/
  static unsigned long lastDHTUpdateTime=0;
  static unsigned long lastSoilUpdateTime=0;
  if(millis()-lastDHTUpdateTime >= DHT_UPDATE_INTERVAL)
  {
    humidity = dht.getHumidity();
    temperature = dht.getTemperature();
    lastDHTUpdateTime = millis();
  }
  lux = lightMeter.readLightLevel();
  if(millis()-lastSoilUpdateTime >= 6000 )
  {
    wait(50);
    lastSoilUpdateTime = millis();
    DEBUG_PRINT("STOFFE ANALOG");
    int analogValue = analogRead(3);
    rawA=analogValue;
    moistureA = map(analogValue, 0, 584, 0, 100);
    DEBUG_PRINT("\tA3:");
    DEBUG_PRINT(analogValue);    
    analogValue=32000;
    wait(50);
    analogValue = analogRead(1);
    DEBUG_PRINT("\tA1:");
    DEBUG_PRINTLN(analogValue);    
    moistureB = map(analogValue, 0, 578, 0, 100);

  //    moistureC = map(analogValue, 0, 1023, 0, 100);
  //  DEBUG_PRINT("\tC2:");
  //  DEBUG_PRINT(moistureC);    
  }

}

void receiveTime(uint32_t newTime)
{
  DEBUG_PRINTLN(F("STOFFE Time value received and updated..."));
  int lastSecond = second();
  int lastMinute = minute();
  int lastHour = hour();
  setTime(newTime);
  if (((second() != lastSecond) || (minute() != lastMinute) || (hour() != lastHour)) || showTime)
  {
    DEBUG_PRINTLN(F("Clock updated...."));
    DEBUG_PRINT(F("Sensor's time currently set to:"));
    DEBUG_PRINT(hourFormat12() < 10 ? F(" 0") : F(" "));
    DEBUG_PRINT(hourFormat12());
    DEBUG_PRINT(minute() < 10 ? F(":0") : F(":"));
    DEBUG_PRINT(minute());
    DEBUG_PRINTLN(isAM() ? F("am") : F("pm"));
    DEBUG_PRINT(month());
    DEBUG_PRINT(F("/"));
    DEBUG_PRINT(day());
    DEBUG_PRINT(F("/"));
    DEBUG_PRINTLN(year());
    //DEBUG_PRINTLN(dayOfWeek[weekday()]);
    showTime = false;
  }
  else
  {
    DEBUG_PRINTLN(F("Sensor's time did NOT need adjustment greater than 1 second."));
  }
  clockUpdating = false;
}

void receive(const MyMessage &message)
{
      Serial.print("STOFFE ......... receive ");
    // We only expect one type of message from controller. But we better check anyway.
    if (message.type==V_STATUS) {
       if (!initialValueSent) {
          Serial.println("Receiving initial value from controller");
          initialValueSent = true;
        }
        // Change relay state
        setRelay(message.getInt());          
        // Write some debug info
        Serial.print(", New status: ");
        Serial.println(message.getInt() );
        Serial.print("message=");
        Serial.println(message.sensor);
     }
}
void updateClock(void)
{
  static unsigned long lastHAGetTime;
  if (millis() - lastHAGetTime >= GET_TIME) // updates clock time and gets zone times from vera once every hour
  {
    DEBUG_PRINTLN(F("Requesting time and valve data from Gateway..."));
    lcd.setCursor(15, 0);
    lcd.write(byte(0));
    clockUpdating = true;
    requestTime();
    lastHAGetTime = millis();
  }
}

void watchdog(void)
{
  static bool setOnce = true;
  
  if( state == Watering)
  {
    setOnce=true;
    if(millis()-watchdogTimeout  >= WATER_PUMP_TIMEOUT)   
    {
      setRelay(0);
      DEBUG_PRINTLN("STOFFE ..watchdog bites..");
    }
  }
  else
  {
    if(setOnce)
    {
      /* turn off relay since not in watering */
      /* set the relay to on ..but not update state*/
      setRelay(0);      
      setOnce=false;
    }

  }
  
}
