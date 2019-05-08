#include <Wire.h>
#include <hd44780.h>
#include <BH1750.h>
#include <hd44780ioClass/hd44780_I2Cexp.h> // i2c expander i/o class header
#include <SPI.h>
#include "DHT.h"

#define MY_DEBUG      // Enable debug prints to serial monitor
#define MY_RADIO_RF24 // Enable and select radio type attached
#define MY_NODE_ID 16

#include <MySensors.h>


#define SKETCH_NAME "Greenhouse"
#define SKETCH_MAJOR_VER "0"
#define SKETCH_MINOR_VER "3"


#define CHILD_ID_HUM 0
#define CHILD_ID_TEMP 1
#define CHILD_ID_LIGHT 2
#define CHILD_ID_RELAY 3

#define RELAY_PIN 7  // Arduino Digital I/O pin number for first relay (second on pin+1 etc)


// Sleep time between sensor updates (in milliseconds)
// Must be >1000ms for DHT22 and >2000ms for DHT11
static const uint64_t UPDATE_INTERVAL = 60000;

MyMessage msgHum(CHILD_ID_HUM, V_HUM);
MyMessage msgTemp(CHILD_ID_TEMP, V_TEMP);
MyMessage msgLight(CHILD_ID_LIGHT, V_LEVEL);

MyMessage msgPrefix(CHILD_ID_LIGHT, V_UNIT_PREFIX);  //custom unit for light sensor
MyMessage msgRelay(CHILD_ID_RELAY, V_STATUS);  //custom unit for light sensor


uint8_t value=1;

hd44780_I2Cexp lcd(0x27); // declare lcd object: auto locate & auto config expander chip
BH1750 lightMeter;
DHT dht; //DHT22 på pinne 5

const int LCD_COLS = 16;
const int LCD_ROWS = 2;
bool metric = true;


void before()
{
  pinMode(RELAY_PIN, OUTPUT);
  // Set relay to last known state (using eeprom storage)
  digitalWrite(RELAY_PIN, loadState(CHILD_ID_RELAY)?1:0);
}

void setup()
{
   Serial.print("current radio channel: ");
   Serial.println(MY_RF24_CHANNEL);
   int status;
   lightMeter.begin();
	 status = lcd.begin(LCD_COLS, LCD_ROWS);
   if(status) // non zero status means it was unsuccesful
	 {
	   status = -status; // convert negative status value to positive number
     Serial.print("error: lcd.begin() ");
		 // begin() failed so blink error code using the onboard LED if possible
		 hd44780::fatalError(status); // does not return
	 }
   dht.setup(5); // data pin 5
  send(msgPrefix.set("lux"));        //light sensor setup for home assistant... unit presented is lux
  int state=loadState(CHILD_ID_RELAY)?1:0;
  send(msgRelay.set(state),state);
}

void presentation() 
{
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo(SKETCH_NAME, SKETCH_MAJOR_VER "." SKETCH_MINOR_VER);

  present(CHILD_ID_LIGHT, S_LIGHT);
  present(CHILD_ID_HUM, S_HUM);
  present(CHILD_ID_TEMP, S_TEMP);
  present(CHILD_ID_RELAY, S_SPRINKLER);
  
  metric = getControllerConfig().isMetric;


  
}

void loop() 
{
  float humidity = dht.getHumidity();
  float temperature = dht.getTemperature();
  uint16_t lux = lightMeter.readLightLevel();
    

  send(msgHum.set(humidity, 1));
  send(msgTemp.set(temperature, 1));
  send(msgLight.set(lux));
  int state=loadState(CHILD_ID_RELAY)?1:0;
  send(msgRelay.set(state),state);
  lcd.setCursor(0,0);
  lcd.print("Hi Greenhouse :)");
  lcd.setCursor(0, 1);
  lcd.print(temperature,0);
  lcd.print((char)223);
  lcd.print(" ");
  lcd.print(humidity,0);
  lcd.print("%");
  lcd.print(" ");
  lcd.print(lux);
  lcd.print("lx");
  sleep(5000);
}

void receive(const MyMessage &message)
{
      Serial.print("STOFFE ......... receive ");
    // We only expect one type of message from controller. But we better check anyway.
    if (message.type==V_STATUS) {
        // Change relay state
        int state1 = message.getBool();                 
        send(msgRelay.set(state1), state1);
        digitalWrite(RELAY_PIN, state1);
        // Store state in eeprom
        saveState(message.sensor, state1);
        // Write some debug info
    
        Serial.print(message.sensor);
        Serial.print(", New status: ");
        Serial.println(state1);
    }
}
