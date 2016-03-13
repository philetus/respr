//*********************************************************************
// respr demo
//
// polls resistance of conductive rubber band on analog pin 0 and
// transmits value over bluetooth connection
//
//*********************************************************************

#include <Arduino.h>
#include <SPI.h>
#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined (_VARIANT_ARDUINO_ZERO_)
  #include <SoftwareSerial.h>
#endif

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "BluefruitConfig.h"

#define SENSORPIN A0 // breath sensor pin
#define TX_DELAY 100 // sensor polling delay

// Create the bluefruit object, either software serial...uncomment these lines
// ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);


/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/
void setup(void)
{
  /* Initialise the ble module */
  ble.begin(VERBOSE_MODE);
}

/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/
void loop(void)
{
  float reading;

  initBle(); // make sure ble is connected before attempting read
  
  reading = analogRead(SENSORPIN);
  ble.println(reading);
  delay(TX_DELAY);
}

void initBle() {
  if (ble.isConnected()) return;

  /* Wait for connection */
  while (!ble.isConnected()) {
      delay(500);
  }

  /* Perform a factory reset to make sure everything is in a known state */
  if (FACTORYRESET_ENABLE) ble.factoryReset();

  /* Disable command echo from Bluefruit */
  ble.echo(false);
  ble.verbose(false);  // debug info is a little annoying after this point!

  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    // Change Mode LED Activity
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
  }

  // Set module to DATA mode
  ble.setMode(BLUEFRUIT_MODE_DATA);
}

