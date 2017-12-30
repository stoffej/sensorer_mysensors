/*
 Arduino Tipping Bucket Rain Gauge

 October 6, 2017

 Version 2.1

 Arduino Tipping Bucket Rain Gauge

 Utilizing a tipping bucket sensor, your Vera home automation controller and the MySensors.org
 gateway you can measure and sense local rain.  This sketch will create two devices on your
 Vera controller.  One will display your total precipitation for the last 5 days.  The other, 
 a sensor that changes state if there is recent rain (up to last 120 hours)  above a threshold.  
 Both these settings are user definable.

 There is a build overview video here: https://youtu.be/1eMfKQaLROo

 This sketch features the following:

 * Allows you to set the rain threshold in mm
 * Allows you to determine the tripped indicator window up to 120 hours.
 * Displays the last 5 days of rain in Variable1 through Variable5
   of the Rain Sensor device
 * Configuration changes to Sensor device updated every hour
 * Should run on any Arduino
 * Will retain Tripped/Not Tripped status and data in a power interruption, saving small amount
   of data to EEPROM (Circular Buffer to maximize life of EEPROM)
 * LED status indicator
 * Optional Temp/Humidity (DHT-22 or DHT-11) and Light LUX (BH1750) sensors. To use, uncomment
   #define DHT_ON  and/or #define LUX_ON
 * Optionally send total accumulation of each day's rainfall or send only individual days rainfall totals.
   Uncomment #define USE_DAILY to display individual daily rainfall.  If it is commented out it will display
   a cumulative total rainfall (day4 = day1+day2+day3+day4 etc)

 by @BulldogLowell and @PeteWill for free public use
 
 Change Log
 2017-10-06 - Version 2.1 - Updated variable types to optimize bytes used and fixed rainBucket array size issue
 */

//#define MY_DEBUG // Enable MySensors debug prints to serial monitor

// Enable and select radio type attached
#define MY_RADIO_NRF24
//#define MY_RADIO_RFM69

//#define MY_RF24_PA_LEVEL RF24_PA_MAX //Options: RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX

#define MY_NODE_ID 30 //uncomment this line to assign a static ID

#include <math.h>
#include <TimeLib.h>
#include <MySensors.h>

#define SKETCH_NAME "Rain Gauge"
#define SKETCH_VERSION "2.1"

#define DWELL_TIME 40  // this allows for radio to come back to power after a transmission, ideally 0 

#define DEBUG_ON  // Rain gauge specific debug messages. 

#define USE_DAILY // Uncomment to display individual daily rainfall totals in the variables sent to your controller. If it's commented it will add each day to the next for a cumulative total.

#define TIP_SENSOR_PIN 3
#define CALIBRATE_FACTOR 36 // amount of rain per rain bucket tip e.g. 5 is .05mm

// Sensor manufacturer has sent information about the sensor. 0.3 mm to 1 tick
// MI-SOL Rain Guage has CALIBRATE_FACTOR = 36


#define CHILD_ID_RAIN_LOG 3  // Keeps track of accumulated rainfall
#define CHILD_ID_TRIPPED_INDICATOR 4  // Indicates Tripped when rain detected
#define EEPROM_BUFFER_LOCATION 0  // location of the EEPROM circular buffer
#define E_BUFFER_LENGTH 240 //Max size = 254
#define RAIN_BUCKET_SIZE 120 //Max size = 254

#ifdef  DEBUG_ON
  #define DEBUG_PRINT(x)   Serial.print(x)
  #define DEBUG_PRINTLN(x) Serial.println(x)
  #define SERIAL_START(x)  Serial.begin(x)
  #else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
  #define SERIAL_START(x)
#endif
//
MyMessage msgRainRate(CHILD_ID_RAIN_LOG, V_RAINRATE);
MyMessage msgRain(CHILD_ID_RAIN_LOG, V_RAIN);
//
MyMessage msgRainVAR1(CHILD_ID_RAIN_LOG, V_VAR1);
MyMessage msgRainVAR2(CHILD_ID_RAIN_LOG, V_VAR2);
MyMessage msgRainVAR3(CHILD_ID_RAIN_LOG, V_VAR3);
MyMessage msgRainVAR4(CHILD_ID_RAIN_LOG, V_VAR4);
MyMessage msgRainVAR5(CHILD_ID_RAIN_LOG, V_VAR5);
//
MyMessage msgTripped(CHILD_ID_TRIPPED_INDICATOR, V_TRIPPED);
MyMessage msgTrippedVar1(CHILD_ID_TRIPPED_INDICATOR, V_VAR1);
MyMessage msgTrippedVar2(CHILD_ID_TRIPPED_INDICATOR, V_VAR2);



uint8_t eepromIndex;
uint8_t tipSensorPin = 3; // Pin the tipping bucket is connected to. Must be interrupt capable pin
uint8_t ledPin = 5; // Pin the LED is connected to.  PWM capable pin required
#ifdef DEBUG_ON
unsigned long dataMillis;
unsigned long serialInterval = 600000UL;
#endif
const unsigned long oneHour = 3600000UL;
unsigned long lastTipTime;
unsigned long lastRainTime; //Used for rainRate calculation
uint16_t rainBucket [RAIN_BUCKET_SIZE + 1]; /* 24 hours x 5 Days = 120 hours */
uint16_t rainRate = 0;
uint8_t rainWindow = 72;         //default rain window in hours.  Will be overwritten with msgTrippedVar1.
volatile int wasTippedBuffer = 0;
uint16_t rainSensorThreshold = 50; //default rain sensor sensitivity in hundredths.  Will be overwritten with msgTrippedVar2.
uint8_t state = 0;
uint8_t oldState = 2; //Setting the default to something other than 1 or 0
uint16_t lastRainRate = 0;
uint16_t lastMeasure = 0;
bool gotTime = false;
uint8_t lastHour;
uint8_t currentHour;
//
void presentation()  {
  // Register all sensors to gw (they will be created as child devices)
  sendSketchInfo(SKETCH_NAME, SKETCH_VERSION);
  wait(DWELL_TIME);
  present(CHILD_ID_RAIN_LOG, S_RAIN);
  wait(DWELL_TIME);
  present(CHILD_ID_TRIPPED_INDICATOR, S_MOTION);
  wait(DWELL_TIME);



  DEBUG_PRINTLN(F("Sensor Presentation Complete"));
}

void setup()
{
  #ifndef MY_DEBUG
  SERIAL_START(115200);  //Start serial if MySensors debugging isn't enabled
  #endif
  //
  // Set up the IO
  pinMode(TIP_SENSOR_PIN, INPUT_PULLUP);
  attachInterrupt (digitalPinToInterrupt(TIP_SENSOR_PIN), sensorTipped, FALLING);  // depending on location of the hall effect sensor may need CHANGE
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);
  //
  //Sync time with the server
  //
  unsigned long functionTimeout = millis();
  while (timeStatus() == timeNotSet && millis() - functionTimeout < 30000UL)
  {
    requestTime();
    DEBUG_PRINTLN(F("Getting Time"));
    wait(1000); // call once per second
    DEBUG_PRINTLN(F("."));
  }
  currentHour = hour();
  lastHour = hour();
  //
  //retrieve from EEPROM stored values on a power cycle.
  //
  bool isDataOnEeprom = false;
  for (uint8_t i = 0; i < E_BUFFER_LENGTH; i++)
  {
    uint8_t locator = loadState(EEPROM_BUFFER_LOCATION + i);
    if (locator == 0xFE)  // found the EEPROM circular buffer index
    {
      eepromIndex = EEPROM_BUFFER_LOCATION + i;
      DEBUG_PRINT(F("EEPROM Index "));
      DEBUG_PRINTLN(eepromIndex);
      //Now that we have the buffer index let's populate the rainBucket[] with data from eeprom
      loadRainArray(eepromIndex);
      isDataOnEeprom = true;
      break;
    }
  }
  //
  if (!isDataOnEeprom) // Added for the first time it is run on a new Arduino
  {
    DEBUG_PRINTLN(F("I didn't find valid EEPROM Index, so I'm writing one to location 0"));
    eepromIndex = EEPROM_BUFFER_LOCATION;
    saveState(eepromIndex, 0xFE);
    saveState(eepromIndex + 1, 0xFE);
    //then I will clear out any bad data
    for (uint8_t i = 2; i <= E_BUFFER_LENGTH; i++)
    {
      saveState(i, 0x00);
    }
  }
  #ifdef DEBUG_ON
  dataMillis = millis();
  #endif
  lastTipTime = millis(); 
  //
  request(CHILD_ID_TRIPPED_INDICATOR, V_VAR1);
  wait(DWELL_TIME);
  request(CHILD_ID_TRIPPED_INDICATOR, V_VAR2);
  wait(DWELL_TIME);

  //
  transmitRainData(); //Setup complete send any data loaded from eeprom to gateway
}

void loop()
{
  if (state)
  {
    prettyFade();  // breathe if tripped
  }
  else
  {
    slowFlash();   // blink if not tripped
  }
#ifdef DEBUG_ON  // Serial Debug Block
  if ( (millis() - dataMillis) >= serialInterval)
  {
    for (int i = 24; i <= 120; i = i + 24)
    {
      updateSerialData(i);
    }
    dataMillis = millis();
  }
#endif
  //
  // let's constantly check to see if the rain in the past rainWindow hours is greater than rainSensorThreshold
  //
  uint16_t measure = 0; // Check to see if we need to show sensor tripped in this block
  for (uint8_t i = 0; i < rainWindow; i++)
  {
    measure += rainBucket [i];
    if (measure != lastMeasure)
    {
      //      DEBUG_PRINT(F("measure value (total rainBucket within rainWindow): "));
      //      DEBUG_PRINTLN(measure);
      lastMeasure = measure;
    }
  }
  //
  state = (measure >= (rainSensorThreshold * 100));
  if (state != oldState)
  {
    send(msgTripped.set(state));
    wait(DWELL_TIME);
    DEBUG_PRINT(F("New Sensor State... Sensor: "));
    DEBUG_PRINTLN(state ? "Tripped" : "Not Tripped");
    oldState = state;
  }
  //
  unsigned long tipDelay = millis() - lastRainTime;
  if (wasTippedBuffer) // if was tipped, then update the 24hour total and transmit to Vera
  {
    DEBUG_PRINTLN(F("Sensor Tipped"));
    DEBUG_PRINT(F("rainBucket [0] value: "));
    DEBUG_PRINTLN(rainBucket [0]);
    send(msgRain.set((float)rainTotal(currentHour) / 100, 1)); //Calculate the total rain for the day
    wait(DWELL_TIME);
    wasTippedBuffer--;
    rainRate = ((oneHour) / tipDelay);
    if (rainRate != lastRainRate)
    {
      send(msgRainRate.set(rainRate, 1));
      wait(DWELL_TIME);
      DEBUG_PRINT(F("RainRate= "));
      DEBUG_PRINTLN(rainRate);
      lastRainRate = rainRate;
    }
    lastRainTime = lastTipTime;
  }
  //
  currentHour = hour();
  if (currentHour != lastHour)
  {
    DEBUG_PRINTLN(F("One hour elapsed."));
    send(msgRain.set((float)rainTotal(currentHour) / 100, 1)); // send today's rainfall
    wait(DWELL_TIME);
    saveState(eepromIndex, highByte(rainBucket[0]));
    saveState(eepromIndex + 1, lowByte(rainBucket[0]));
    DEBUG_PRINT(F("Saving rainBucket[0] to eeprom. rainBucket[0] = "));
    DEBUG_PRINTLN(rainBucket[0]);
    for (int16_t i = RAIN_BUCKET_SIZE - 1; i >= 0; i--)//cascade an hour of values back into the array
    {
      rainBucket [i + 1] = rainBucket [i];
    }
    request(CHILD_ID_TRIPPED_INDICATOR, V_VAR1);
    wait(DWELL_TIME);
    request(CHILD_ID_TRIPPED_INDICATOR, V_VAR2);
    wait(DWELL_TIME);
    rainBucket[0] = 0;
    eepromIndex = eepromIndex + 2;
    if (eepromIndex > EEPROM_BUFFER_LOCATION + E_BUFFER_LENGTH)
    {
      eepromIndex = EEPROM_BUFFER_LOCATION;
    }
    DEBUG_PRINT(F("Writing to EEPROM.  Index: "));
    DEBUG_PRINTLN(eepromIndex);
    saveState(eepromIndex, 0xFE);
    saveState(eepromIndex + 1, 0xFE);
    requestTime(); // sync the time every hour
    wait(DWELL_TIME);
    transmitRainData();
    rainRate = 0;
    send(msgRainRate.set(rainRate, 1));
    wait(DWELL_TIME);
    DEBUG_PRINTLN(F("Sending rainRate is 0 to controller"));
    lastHour = hour();
  }

}
//

//

//
void sensorTipped()
{
  unsigned long thisTipTime = millis();
  if (thisTipTime - lastTipTime > 100UL)
  {
    rainBucket[0] += CALIBRATE_FACTOR; // adds CALIBRATE_FACTOR hundredths of unit each tip
    wasTippedBuffer++;
  }
  lastTipTime = thisTipTime;
}
//
uint32_t rainTotal(uint8_t hours)
{
  uint32_t total = 0;
  for (uint8_t i = 0; i <= hours; i++)
  {
    total += rainBucket [i];
  }
  return total;
}

#ifdef DEBUG_ON
void updateSerialData(int x)
{
  DEBUG_PRINT(F("Rain last "));
  DEBUG_PRINT(x);
  DEBUG_PRINTLN(F(" hours: "));
  float tipCount = 0;
  for (uint8_t i = 0; i < x; i++)
  {
    tipCount = tipCount + rainBucket [i];
  }
  tipCount = tipCount / 100;
  DEBUG_PRINTLN(tipCount);
}
#endif

void loadRainArray(int16_t eValue) // retrieve stored rain array from EEPROM on powerup
{
  for (uint8_t i = 1; i < RAIN_BUCKET_SIZE; i++)
  {
    eValue = eValue - 2;
    if (eValue < EEPROM_BUFFER_LOCATION)
    {
      eValue = EEPROM_BUFFER_LOCATION + E_BUFFER_LENGTH;
    }
    DEBUG_PRINT(F("EEPROM location: "));
    DEBUG_PRINTLN(eValue);
    uint8_t rainValueHigh = loadState(eValue);
    uint8_t rainValueLow = loadState(eValue + 1);
    uint16_t rainValue = rainValueHigh << 8;
    rainValue |= rainValueLow;
    rainBucket[i] = rainValue;
    //
    DEBUG_PRINT(F("rainBucket[ value: "));
    DEBUG_PRINT(i);
    DEBUG_PRINT(F("] value: "));
    DEBUG_PRINTLN(rainBucket[i]);
  }
}

void transmitRainData(void)
{
  DEBUG_PRINT(F("In transmitRainData. currentHour = "));
  DEBUG_PRINTLN(currentHour);
  int rainUpdateTotal = 0;
  for (int8_t i = currentHour; i >= 0; i--)
  {
    rainUpdateTotal += rainBucket[i];
    DEBUG_PRINT(F("Adding rainBucket["));
    DEBUG_PRINT(i);
    DEBUG_PRINTLN(F("] to rainUpdateTotal."));
  }
  DEBUG_PRINT(F("TX Day 1: rainUpdateTotal = "));
  DEBUG_PRINTLN((float)rainUpdateTotal / 100.0);
  send(msgRainVAR1.set((float)rainUpdateTotal / 100.0, 1)); //Send current day rain totals (resets at midnight)
  wait(DWELL_TIME);
#ifdef USE_DAILY
  rainUpdateTotal = 0;
#endif
  for (uint8_t i = currentHour + 24; i > currentHour; i--)
  {
    rainUpdateTotal += rainBucket[i];
    DEBUG_PRINT(F("Adding rainBucket["));
    DEBUG_PRINT(i);
    DEBUG_PRINTLN(F("] to rainUpdateTotal."));
  }
  DEBUG_PRINT(F("TX Day 2: rainUpdateTotal = "));
  DEBUG_PRINTLN((float)rainUpdateTotal / 100.0);
  send(msgRainVAR2.set((float)rainUpdateTotal / 100.0, 1));
  wait(DWELL_TIME);
#ifdef USE_DAILY
  rainUpdateTotal = 0;
#endif
  for (uint8_t i = currentHour + 48; i > currentHour + 24; i--)
  {
    rainUpdateTotal += rainBucket[i];
    DEBUG_PRINT(F("Adding rainBucket["));
    DEBUG_PRINT(i);
    DEBUG_PRINTLN(F("] to rainUpdateTotal."));
  }
  DEBUG_PRINT(F("TX Day 3: rainUpdateTotal = "));
  DEBUG_PRINTLN((float)rainUpdateTotal / 100.0);
  send(msgRainVAR3.set((float)rainUpdateTotal / 100.0, 1));
  wait(DWELL_TIME);
#ifdef USE_DAILY
  rainUpdateTotal = 0;
#endif
  for (uint8_t i = currentHour + 72; i > currentHour + 48; i--)
  {
    rainUpdateTotal += rainBucket[i];
    DEBUG_PRINT(F("Adding rainBucket["));
    DEBUG_PRINT(i);
    DEBUG_PRINTLN(F("] to rainUpdateTotal."));
  }
  DEBUG_PRINT(F("TX Day 4: rainUpdateTotal = "));
  DEBUG_PRINTLN((float)rainUpdateTotal / 100.0);
  send(msgRainVAR4.set((float)rainUpdateTotal / 100.0, 1));
  wait(DWELL_TIME);
#ifdef USE_DAILY
  rainUpdateTotal = 0;
#endif
  for (uint8_t i = currentHour + 96; i > currentHour + 72; i--)
  {
    rainUpdateTotal += rainBucket[i];
    DEBUG_PRINT(F("Adding rainBucket["));
    DEBUG_PRINT(i);
    DEBUG_PRINTLN(F("] to rainUpdateTotal."));
  }
  DEBUG_PRINT(F("TX Day 5: rainUpdateTotal = "));
  DEBUG_PRINTLN((float)rainUpdateTotal / 100.0);
  send(msgRainVAR5.set((float)rainUpdateTotal / 100.0, 1));
  wait(DWELL_TIME);
}

void receive(const MyMessage &message)
{
  if (message.sensor == CHILD_ID_RAIN_LOG)
  {
    // nothing to do here
  }
  else if (message.sensor == CHILD_ID_TRIPPED_INDICATOR)
  {
    if (message.type == V_VAR1)
    {
      rainWindow = atoi(message.data);
      if (rainWindow > 120)
      {
        rainWindow = 120;
      }
      else if (rainWindow < 1)
      {
        rainWindow = 1;
      }
      if (rainWindow != atoi(message.data))   // if I changed the value back inside the boundries, push that number back to Vera
      {
        send(msgTrippedVar1.set(rainWindow));
      }
    }
    else if (message.type == V_VAR2)
    {
      rainSensorThreshold = atoi(message.data);
      if (rainSensorThreshold > 10000)
      {
        rainSensorThreshold = 10000;
      }
      else if (rainSensorThreshold < 1)
      {
        rainSensorThreshold = 1;
      }
      if (rainSensorThreshold != atoi(message.data))  // if I changed the value back inside the boundries, push that number back to Vera
      {
        send(msgTrippedVar2.set(rainSensorThreshold));
      }
    }
  }
}

void prettyFade(void)
{
  float val = (exp(sin(millis() / 2000.0 * PI)) - 0.36787944) * 108.0;
  analogWrite(ledPin, val);
}

void slowFlash(void)
{
  static bool ledState = true;
  static unsigned long pulseStart = millis();
  if (millis() - pulseStart < 100UL)
  {
    digitalWrite(ledPin, !ledState);
    pulseStart = millis();
  }
}

void receiveTime(unsigned long newTime)
{
  DEBUG_PRINTLN(F("Time received..."));
  setTime(newTime);
  char theTime[6];
  sprintf(theTime, "%d:%2d", hour(), minute());
  DEBUG_PRINTLN(theTime);
}
