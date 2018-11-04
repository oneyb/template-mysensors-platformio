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

// Set pins upon recognizing the type of board.
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
//#include <stdint.h>
//#include <math.h>

// FORCE_TRANSMIT_INTERVAL, this number of times of wakeup, the sensor is

#define FORCE_TRANSMIT_INTERVAL 3
// #define SLEEP_TIME 30000
#define SLEEP_TIME 5000
// #define SLEEP_TIME 86400000

#define CHILD_ID_VOLTAGE 0

/*****************************/
/********* FUNCTIONS *********/
/*****************************/

//Create an instance of the object
MyMessage msgVolt(CHILD_ID_VOLTAGE, V_VOLTAGE);


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
  metric = getControllerConfig().isMetric;
}


void loop()
{


  sleep(SLEEP_TIME);


}

