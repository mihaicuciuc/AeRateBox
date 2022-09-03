// Libraries needed: FlashStorage, Adafruit_Sensors, Adafruit_BME280


#include <FlashAsEEPROM.h>
#include "ArduinoLowPower.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "wiring_private.h" // pinPeripheral() function
#include "Sodaq_RN2483.h"
#include "SDS011.h"
#include <Adafruit_BME280.h>


#define debugSerial Serial
#define loraSerial Serial1

#define PIN_SDS       19
#define PIN_RED_LED   13


uint8_t app_key[16];

Adafruit_BME280 bme;
uint8_t bme_ok;

SDS011 sds;

uint8_t low_power_enable;
uint16_t measure_interval;
uint32_t nowTs;

/* from variant.cpp for Trinket M0
    PA30 - SWCLK
    PA31 - SWDIO
  // GPIO 19 & 20 (SWCLK & SWDIO)
  // --------------------------
  { PORTA, 30, PIO_TIMER, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE },
  { PORTA, 31, PIO_TIMER, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE },

  // Uart Serial1( &sercom0, PIN_SERIAL1_RX, PIN_SERIAL1_TX, PAD_SERIAL1_RX, PAD_SERIAL1_TX );
*/
Uart pmSerial(&sercom1, 20, 19, SERCOM_RX_PAD_3, UART_TX_PAD_2);

void SERCOM1_Handler()
{
  pmSerial.IrqHandler();
}

void setup()
{
  uint16_t i;
  
  measure_interval = 30;
  pinMode(PIN_RED_LED, OUTPUT);     // LED
  digitalWrite(PIN_RED_LED, HIGH);
  delay(5000);
  digitalWrite(PIN_RED_LED, LOW);

  while (loraInit() != 0)
  {
    // RN2483 error. Can't do much but complain and retry
    digitalWrite(PIN_RED_LED, HIGH);
    delay(50);
    digitalWrite(PIN_RED_LED, LOW);
    delay(50);
  }
  LoRaBee.sleep();

  debugSerial.begin(57600);

  analogReference(AR_INTERNAL1V0);
  analogReadResolution(12);

  i = analogRead(A0);
  debugSerial.printf("ADC value = %d. Low power ", i);
  if (i > 1000)
  {
    debugSerial.printf("enabled\n");
    low_power_enable = 1;
  }
  else
  {
    debugSerial.printf("disabled\n");
    low_power_enable = 0;
  }

  if (low_power_enable == 0)
  {
    // if USB attached, wait 2s for traffic on UART for programming AppKey
    debugSerial.printf("Program new AppKey in next 2s...\n");
    while (debugSerial.available() > 0) debugSerial.read();
    digitalWrite(PIN_RED_LED, HIGH);
    delay(2000);
    digitalWrite(PIN_RED_LED, LOW);
    if (debugSerial.available() >= 48)
    {
      // Read AppKey from UART buffer
      for (i = 0; i < 16; i++)
      {
        EEPROM.write(i, readHex());
      }
      EEPROM.commit();
    }
  }

  for (i = 0; i < 16; i++)
  {
    app_key[i] = EEPROM.read(i);
  }

  debugSerial.printf("AppKey = {");
  for (i = 0; i < 16; i++)
  {
    debugSerial.printf("0x%02X, ", app_key[i]);
  }
  debugSerial.printf("}\n");

  Wire.begin();
  
  bme_ok = 0;
  if (bme.begin())
  {
    bme_ok = 1;
    bme.setSampling(Adafruit_BME280::MODE_NORMAL,
                    Adafruit_BME280::SAMPLING_X1, // dataTemperature
                    Adafruit_BME280::SAMPLING_X1, // dataPressure
                    Adafruit_BME280::SAMPLING_X1, // humidity
                    Adafruit_BME280::FILTER_OFF,
                    Adafruit_BME280::STANDBY_MS_1000);
  }

  pmSerial.begin(9600);
  pinPeripheral(20, PIO_SERCOM_ALT); // Assign pin 20 (SWDIO - pad away from microUSB connector) SERCOM functionality (RX)

  sds.begin(&pmSerial);
  pinMode(PIN_SDS, OUTPUT);         // High-side switch
  digitalWrite(PIN_SDS, LOW);       // SDS off

  nowTs = 0;
}



/*
  LoRa payload, all values MSB first
  [0-1] battery (mV)
  [2-3] dataTemp * 100 (100*C)
  [4-5] dataRH * 100 (100*%)
  [6-9] dataPressure * 100 (10000*hPa)
  [10-11] p2.5 * 10
  [12-13] p10 * 10
  [14-15] packet counter
  [16-17] measurement interval (s)
*/

#define ST_NOT_INIT         0
#define ST_NOT_JOINED       1
#define ST_MEASURE_START    2
#define ST_MEASURE_WARMUP   3
#define ST_MEASURE          4
#define ST_TX               5
#define ST_SLEEP            6
#define MEASURE_TIME        13


void loop()
{
  // timestamps (Ts) are VERY rough numbers of seconds. millis() doesn't work because of sleep,
  // so every time we put in a delay we increment nowTs by that number of seconds
  static uint32_t lastJoinAttemptTs = 0x80000000;
  static uint32_t measureStartTs = 0;
  static uint16_t counter = 0;
  static uint8_t nacks = 0; // Number of unacknowledged transmissions
  static uint8_t state = ST_NOT_JOINED;

  static uint8_t dataTx[18];

  static uint16_t dataP25, dataP10;
  static float dataADC, dataTemp, dataRH, dataPressure;
  static uint16_t nDataADC;

  uint16_t rxSize;
  float fdataTemp;

  if (state == ST_NOT_INIT)
  {
    debugSerial.printf("State = ST_NOT_INIT\n");
    if (loraInit() == 0)
    {
      state = ST_NOT_JOINED;
      LoRaBee.sleep();
      
      // Add 1s delay just to make sure we don't immediately start sending commands
      delayS(1);
    }
    else
    {
      delayS(60);
    }
  }
  if (state == ST_NOT_JOINED)
  {
    debugSerial.printf("State = ST_NOT_JOINED\n");
    if (nowTs - lastJoinAttemptTs > 60)
    {
      lastJoinAttemptTs = nowTs;
      digitalWrite(PIN_RED_LED, HIGH);
      LoRaBee.wakeUp();
      if (loraJoin() == 0)
      {
        state = ST_MEASURE_START;
      }
      LoRaBee.sleep();
      digitalWrite(PIN_RED_LED, LOW);
    }
    delayS(10);
  }
  else if (state == ST_MEASURE_START)
  {
    for (uint8_t i = 0; i < 18; i++)
    {
      dataTx[i] = 0xFF;
    }
    
    dataP25 = dataP10 = 0xFFFF;
    dataTemp = dataRH = dataPressure = -100000;
    dataADC = 0;
    nDataADC = 0;
    
    measureStartTs = nowTs;
    digitalWrite(PIN_SDS, HIGH);
    state = ST_MEASURE_WARMUP;
  }
  else if (state == ST_MEASURE_WARMUP)
  {
    delayS(1);

    dataADC += analogRead(A0);
    nDataADC++;

    if (nowTs - measureStartTs >= MEASURE_TIME)
    {
      // SDS011 warmed up. Take measurements now
      while (pmSerial.available() > 0) pmSerial.read();
      state = ST_MEASURE;
    }
  }
  else if (state == ST_MEASURE)
  {
    // Two seconds from buffer clear just to make sure we
    // get at least one full measurement from SDS011
    delay(2000);
    nowTs += 2;
    
    dataADC += analogRead(A0);
    nDataADC++;

    fdataTemp = dataADC / nDataADC;
    fdataTemp *= (670.0 + 6800.0) * 1000.0;
    fdataTemp /= (670.0 * 4096.0);
    debugSerial.printf("Sending %.2f mV\n", fdataTemp);
    dataTx[0] = ((uint16_t)fdataTemp) >> 8;
    dataTx[1] = ((uint16_t)fdataTemp) & 0xFF;

    if (bme_ok == 1)
    {
      dataTemp = bme.readTemperature();
      dataRH = bme.readHumidity();
      dataPressure = bme.readPressure();
      
      fdataTemp = dataTemp*100;
      dataTx[2] = (((int16_t)fdataTemp) >> 8) & 0xFF;
      dataTx[3] = ((int16_t)fdataTemp) & 0xFF;
      
      fdataTemp = dataRH*100;
      dataTx[4] = ((uint16_t)fdataTemp) >> 8;
      dataTx[5] = ((uint16_t)fdataTemp) & 0xFF;

      fdataTemp = dataPressure*100;
      dataTx[6] = ((uint32_t)fdataTemp) >> 24;
      dataTx[7] = (((uint32_t)fdataTemp) >> 16) & 0xFF;
      dataTx[8] = (((uint32_t)fdataTemp) >> 8) & 0xFF;
      dataTx[9] = ((uint32_t)fdataTemp) & 0xFF;
    }

    sds.read(&dataP25, &dataP10);
    dataTx[10] = ((uint16_t)dataP25) >> 8;
    dataTx[11] = ((uint16_t)dataP25) & 0xFF;
    dataTx[12] = ((uint16_t)dataP10) >> 8;
    dataTx[13] = ((uint16_t)dataP10) & 0xFF;

    dataTx[14] = counter >> 8;
    dataTx[15] = counter & 0xFF;

    dataTx[16] = measure_interval >> 8;
    dataTx[17] = measure_interval & 0xFF;

    debugSerial.printf("ADC: %.2f, temp %.2f, RH %.2f%%, pres %.2f, PM10 %d, PM25 %d\n", dataADC/nDataADC, dataTemp, dataRH, dataPressure, dataP25, dataP10);
    debugSerial.printf("Going to TX. Switching off SDS\n");
    state = ST_TX;
    digitalWrite(PIN_SDS, LOW);
  }
  else if (state == ST_TX)
  {
    // By default will go to ST_SLEEP afterwards. But this may be overridden later
    state = ST_SLEEP;

    LoRaBee.wakeUp();
    switch (LoRaBee.send(1, dataTx, 18))
    {
      case NoError:
        debugSerial.println("Successful transmission.");
        nacks = 0;
        break;
      case NoAcknowledgment:
        debugSerial.println("There was no acknowledgment sent back!");
        nacks++;
        if (nacks == 10)
        {
          // something seems bad. Take it from the top.
          state = ST_NOT_INIT;
        }
        break;
      case NoResponse:
        debugSerial.println("NoResponse");
        state = ST_NOT_INIT;
        break;
      case Timeout:
        debugSerial.println("Timeout.");
        // Clear buffer as something is weird
        // Send sync char just in case the last thing it saw was a break
        loraSerial.write((uint8_t)0x55);
        loraSerial.flush();
        loraSerial.printf("\r\n");
        delay(1000);
        while (loraSerial.available() > 0) loraSerial.read();
        state = ST_NOT_INIT;
        break;
      case PayloadSizeError:
        debugSerial.println("PayloadSizeError");
        break;
      case Busy:
        debugSerial.println("Busy");
        delay(1000);
        state = ST_NOT_INIT;
        break;
      case InternalError:
        debugSerial.println("InternalError");
        delay(1000);
        state = ST_NOT_INIT;
        break;
      case NetworkFatalError:
        debugSerial.println("NetworkFatalError");
        delay(1000);
        state = ST_NOT_INIT;
        break;
      case NotConnected:
        debugSerial.println("NotConnected");
        delay(1000);
        state = ST_NOT_INIT;
        //NVIC_SystemReset();
        break;
      default:
        debugSerial.println("default");
        delay(1000);
        state = ST_NOT_INIT;
        break;
    }
    LoRaBee.sleep();

    // {"cmd":1, "param":60}
    rxSize = LoRaBee.receive(dataTx, 10);
    if (rxSize == 3)
    {
      debugSerial.printf("Rx data[0] = %d, data[1] = %d, data[2] = %d\n", dataTx[0], dataTx[1], dataTx[2]);
      switch (dataTx[0])
      {
        case 0:
          if (dataTx[2] == 0) digitalWrite(PIN_RED_LED, LOW);
          else  digitalWrite(PIN_RED_LED, HIGH);
          break;
        case 1:
          measure_interval = ((uint16_t)dataTx[1] << 8) | dataTx[2];
          break;
        default:
          break;
      }
    }

    counter++;
  }
  else if (state == ST_SLEEP)
  {
    delayS(10);
    if (nowTs - measureStartTs > measure_interval)
    {
      debugSerial.printf("Going to measure.\n");
      state = ST_MEASURE_START;
    }
  }
}

int8_t loraInit()
{
  loraSerial.begin(LoRaBee.getDefaultBaudRate());

  // Send sync char just in case the last thing it saw was a break
  loraSerial.write((uint8_t)0x55);
  loraSerial.flush();

  // clear buffer
  loraSerial.printf("\r\n");
  delay(1000);
  while (loraSerial.available() > 0) loraSerial.read();

  // send true break. But a really long one, just in case the autodetected baudrate up to now is something crazy
  loraSerial.end();
  pinMode(4, OUTPUT);
  digitalWrite(4, LOW);
  delay(1000);
  digitalWrite(4, HIGH);

  // resend sync char this time to really set the baud rate to what we want.
  loraSerial.begin(LoRaBee.getDefaultBaudRate());
  loraSerial.write((uint8_t)0x55);
  loraSerial.flush();
  delay(1000);

  // we should be good to go and never have a problem
  if (!LoRaBee.init(loraSerial, -1, true, true))
  {
    return -1;
  }
  else
  {
    return 0;
  }
}

int8_t loraJoin()
{
  uint8_t app_eui[16] = {};
  uint8_t dev_eui[8];
  uint8_t i;

  debugSerial.printf("loraJoin()\n");

  LoRaBee.getHWEUI(dev_eui, 8);
  debugSerial.printf("dev_eui: ");
  for (i = 0; i < 8; i++)
  {
    debugSerial.printf("%02X", dev_eui[i]);
  }
  debugSerial.printf("\n");

  if (LoRaBee.initOTA(loraSerial, dev_eui, app_eui, app_key, true))
  {
    debugSerial.println("Connection to the network was successful.");
    return 0;
  }
  else
  {
    debugSerial.println("Connection to the network failed!");
    return -1;
  }
}

void delayS(uint16_t s)
{
  if (low_power_enable == 1)
  {
    USBDevice.detach();
    LowPower.sleep(s * 1000);
    USBDevice.attach();
  }
  else
  {
    delay(s * 1000);
  }
  nowTs += s;
}

uint8_t readHex()
{
  uint8_t retVal = 0;
  uint8_t chr;
  if (debugSerial.available() >= 2)
  {
    chr = debugSerial.read();
    if (chr >= '0' && chr <= '9')
    {
      chr -= '0';
    }
    else if (chr >= 'A' && chr <= 'F')
    {
      chr -= 'A';
      chr += 10;
    }
    else if (chr >= 'a' && chr <= 'f')
    {
      chr -= 'a';
      chr += 10;
    }
    retVal = chr << 4;
    
    chr = debugSerial.read();
    if (chr >= '0' && chr <= '9')
    {
      chr -= '0';
    }
    else if (chr >= 'A' && chr <= 'F')
    {
      chr -= 'A';
      chr += 10;
    }
    else if (chr >= 'a' && chr <= 'f')
    {
      chr -= 'a';
      chr += 10;
    }
    retVal |= chr;
  }
  
  while (debugSerial.peek() == ' ')
  {
    debugSerial.read();
  }
  
  return retVal;
}
