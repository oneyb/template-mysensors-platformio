/*
* Copyleft 2018 Brian Oney (brian dot j period oney at gmail dot com)
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; either version 2 of the License.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; if not, see <http://www.gnu.org/licenses/>.
*

 */

// Enable debug prints to serial monitor
#define MY_DEBUG
#define DEBUG_SENSORS 

#define MY_RADIO_NRF24
//#define MY_RADIO_RFM69

/* These are actually the default pins expected by the MySensors framework.
 * This means we can use the default constructor without arguments when
 * creating an instance of the Mysensors class. Other defaults will include
 * transmitting on channel 76 with a data rate of 250kbps.
 */
#if defined(ARDUINO_AVR_MEGA2560)
#define MY_RF24_CE_PIN 49
#define MY_RF24_CS_PIN 53
#elif defined(ARDUINO_AVR_PRO)
#define MY_RF24_CE_PIN 9
#define MY_RF24_CS_PIN 10
#else
#error Unsupported hardware
#endif

//#include <SPI.h>
#include <MySensors.h>
#include <Wire.h>
// #include <SparkFunHTU21D.h>
//#include <stdint.h>
//#include <math.h>

// FORCE_TRANSMIT_INTERVAL, this number of times of wakeup, the sensor is

#define FORCE_TRANSMIT_INTERVAL 3
// #define SLEEP_TIME 30000
#define SLEEP_TIME 5000
// #define SLEEP_TIME 86400000

#define CHILD_ID_HUMIDITY 0
#define CHILD_ID_TEMP 1
#define CHILD_ID_VOLTAGE 2

/*****************************/
/********* FUNCTIONS *********/
/*****************************/

//Create an instance of the object
HTU21D myHumidity;
MyMessage msgHum(CHILD_ID_HUMIDITY, V_HUM);
MyMessage msgTemp(CHILD_ID_TEMP, V_TEMP);

uint16_t measureBattery(bool force);
MyMessage msgVolt(CHILD_ID_VOLTAGE, V_VOLTAGE);

uint16_t readVcc();
void switchClock(unsigned char clk);
void receiveTime(unsigned long ts);
void readHTU21DTemperature(bool force);
void readHTU21DHumidity(bool force);
void readSI7021TempHumidity(bool force);

/*Set true to have clock throttle back, or false to not throttle*/
bool highfreq = true;

boolean receivedConfig = false;
boolean metric = true;
uint8_t loopCount = 0;
uint8_t clockSwitchCount = 0;
// string device_id = 8900;
// string device_version = 0.1;


/**********************************/
/********* IMPLEMENTATION *********/
/**********************************/
void setup()
{
  // Serial.begin(115200);
  // Serial.println("HTU21D Example!");
  // analogReference(INTERNAL);
  myHumidity.begin();

}

void presentation()
{
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo("template", "0.1");
  // sendSketchInfo(device_id, device_version);

  // Register all sensors to gateway (they will be created as child devices)
  present(CHILD_ID_VOLTAGE, S_MULTIMETER);
  present(CHILD_ID_HUMIDITY, S_HUM);
  present(CHILD_ID_TEMP, S_TEMP);
  metric = getControllerConfig().isMetric;
}


void loop()
{

  bool forceTransmit = true;
  // bool forceTransmit = false;

  loopCount++;
  clockSwitchCount++;

  if (loopCount > FORCE_TRANSMIT_INTERVAL)
  { // force a transmission
    forceTransmit = true;
    loopCount = 0;
  }

  measureBattery(forceTransmit);

  readHTU21DTemperature(forceTransmit);
  readHTU21DHumidity(forceTransmit);

  sleep(SLEEP_TIME);


}

void readHTU21DTemperature(bool force)
{
  static float lastTemp = 0;

  if (force)
  {
   lastTemp = -100;
  }
  float temp = myHumidity.readTemperature();
  
  if(lastTemp != temp)
    {
      send(msgTemp.set(temp,1));
      lastTemp = temp;
#ifdef DEBUG_SENSORS
      Serial.print(" Temperature:");
      Serial.print(temp, 1);
      Serial.print("C");
      Serial.println();
#endif
  }
}

void readHTU21DHumidity(bool force)
{
  static float lastHumidity = 0;

  if (force)
  {
    lastHumidity = -100;
  }
  float humd = myHumidity.readHumidity();

  if(lastHumidity != humd)
  {
    send(msgHum.set(humd,1));
    lastHumidity = humd;
    #ifdef DEBUG_SENSORS
    Serial.print(" Humidity:");
    Serial.print(humd, 1);
    Serial.print("%");
    Serial.println();
    #endif
  }
}


#ifdef SI7021_ENABLE
void readSI7021TempHumidity(bool force)
{
  static float lastTemp = 0;
  static float lastHum = 0;

  if (force)
  {
   lastTemp = -100;
   lastHum = 0;
  }

  si7021_env data = myHumidity.getHumidityAndTemperature();
  float temp = 1.0*data.celsiusHundredths/100;
  int humd = data.humidityPercent;

  if(lastTemp != temp)
  {
    send(msgTemp.set(temp,1));
    lastTemp = temp;
    #ifdef DEBUG_SENSORS
    Serial.print(" Temperature:");
    Serial.print(temp, 1);
    Serial.print("C");
    Serial.println();
    #endif
  }

  if(lastHum != humd)
  {
    send(msgHum.set(humd,1));
    lastHum = humd;
    #ifdef DEBUG_SENSORS
    Serial.print(" Humidity:");
    Serial.print(humd, 1);
    Serial.print("%");
    Serial.println();
    #endif
  }
}

#endif


uint16_t measureBattery(bool force)
{
  static uint16_t lastVcc = 0;

  if (force)
  {
    lastVcc = 0;
  }

  uint16_t thisVcc = readVcc();
  if(thisVcc != lastVcc)
  {
    send(msgVolt.set(thisVcc, 1));
    lastVcc = thisVcc;
    #ifdef DEBUG_SENSORS
    Serial.print("Battery voltage = ");
    Serial.println(thisVcc);
    Serial.println('\r');
    #endif
  }
  return thisVcc;

}
/**
* Measure remaining voltage in battery in millivolts
*
* From https://seeeddoc.github.io/DevDuino_Sensor_Node_V1.2-ATmega_328/
*/

uint16_t readVcc()
{
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
  ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
  ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif
  delay(75); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring
  uint8_t low = ADCL; // must read ADCL first - it then locks ADCH
  uint8_t high = ADCH; // unlocks both
  long result = (high<<8) | low;
  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  #ifdef DEBUG_SENSORS
  Serial.print("Read Vcc = ");
  Serial.println(result);
  Serial.println('\r');
  #endif
  return (uint16_t)result; // Vcc in millivolts

}

void switchClock(unsigned char clk)
{
  cli();

  CLKPR = 1<<CLKPCE; // Set CLKPCE to enable clk switching
  CLKPR = clk;
  sei();
  highfreq = false;
}
