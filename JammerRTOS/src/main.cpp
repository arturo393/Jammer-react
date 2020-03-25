#include "MemoryFree.h"
#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <EEPROM.h>
#include <avr/wdt.h>
#include <timers.h>

#define XTERM
/* Useful Constants */
#define TIME_AFTER_START 15
#define TIME_AFTER_STOP 10
#define TIME_TO_STOP_ENGINE 120 // secs
#define TIME_TO_OPEN_DOOR 30
#define DOOR_ENABLE_SECONDS 7
#define NOTIFICATION_TIME 30
#define TIME_AFTER_OPEN_DOOR 60 // secs after the door was opened
#define TIME_JAMMING_SECURE 120
#define TIME_RESET_HW 720 // * 10 seconds
#define CCON LOW
#define CCOFF HIGH
#define DOOROPEN LOW
#define DOORCLOSE HIGH
#define NORMAL_STATE 1   /* cirtcuit normal state */
#define PROTOCOL_STATE 2 /* CC avtivated sate */
#define SUSPEND_STATE 3  /* CC deactivated and all the task suspended */
#define JAMMED_STATE 4
#define GPS_STATE 5
#define BLUETOOTH_STATE 6
#define UNKNOWN_STATE 69
#define ID 5
#define BUFF_SIZE 20
#define MINIMUM_SIZE 5

// inputs
#define CCDisInPin 2  // Enable/disable jamming detection HIGH = on & LOW = off
#define JamInPin 4    // jammer signal
#define DNegInPin 5   // Door sensor  HIGH = open / LOW = close
#define DisarmInPin 6 // disarm negative pin
#define DPosInPin 15  // Door positive pin
#define IgnInPin 3    // (with pull down resistor)

// output
#define LedOutPin 9 // neg
#define CCOutPin 10 // neg

#define TNegOutPin 16 // neg
#define TPosOutPin 8  // pos
#define TinitOutPin 7 // pos

#define BlEnOutPin 14 // neg with diode
const char VERSION[] = "3.0.9";
int8_t DEBUG = 0;
int8_t BLEDEBUG = 0;

uint8_t stateAddress = 20;
uint8_t address = 0;
uint16_t c_off = 0;

TaskHandle_t cch;
TaskHandle_t jh;
TaskHandle_t ph;
TaskHandle_t bh;
TaskHandle_t ioh;

TimerHandle_t xTimerNotify;
TimerHandle_t xTimerRst;
TimerHandle_t xtimerMem;

static void vProtocolTask(void *pvParameters);
static void vJammingTask(void *pvParameters);
static void vCCTask(void *pvParameters);
static void vIOTask(void *pvParameters);
static void vBlueTask(void *pvParameters);
static void vTestTask(void *pvParameters);

void vTimerCallback(TimerHandle_t xTimer);
void vTimerMemCallback(TimerHandle_t xTimer);
void vTimerRestart(TimerHandle_t xTimer);

void restartHW();
void updateState(int8_t _state);
void printBle(char *_buff);
void getHighWaterMark();
void getDIO();
void getState();

void setup()
{
  Serial1.begin(9600);

#ifdef XTERM
  Serial.begin(9600);
#endif
  wdt_enable(WDTO_2S);
  xTaskCreate(vBlueTask, "Blue", configMINIMAL_STACK_SIZE + 40, NULL,
              tskIDLE_PRIORITY + 1, &bh);
  xTaskCreate(vCCTask, "CC", configMINIMAL_STACK_SIZE + 20, NULL,
              tskIDLE_PRIORITY, &cch);
  xTaskCreate(vProtocolTask, "Protocol", configMINIMAL_STACK_SIZE + 20, NULL,
              tskIDLE_PRIORITY, &ph);
  xTaskCreate(vJammingTask, "Jamming", configMINIMAL_STACK_SIZE + 20, NULL,
              tskIDLE_PRIORITY, &jh);
  xTaskCreate(vIOTask, "IO", configMINIMAL_STACK_SIZE, NULL,
              tskIDLE_PRIORITY, &ioh);
  //  xTaskCreate(vTestTask, "Test", configMINIMAL_STACK_SIZE, NULL,
  //              tskIDLE_PRIORITY, &test_handler);

  xTimerNotify = xTimerCreate("TimerNotify", configTICK_RATE_HZ * 15, pdTRUE,
                              (void *)0, vTimerCallback);
  xTimerRst = xTimerCreate("TimerRst", configTICK_RATE_HZ * 10, pdTRUE,
                           (void *)0, vTimerRestart);
  xtimerMem = xTimerCreate("TimerMem", configTICK_RATE_HZ * 5, pdTRUE,
                           (void *)0, vTimerMemCallback);

  pinMode(LedOutPin, OUTPUT);
  pinMode(CCOutPin, OUTPUT);
  pinMode(TNegOutPin, OUTPUT);
  pinMode(TPosOutPin, OUTPUT);
  pinMode(TinitOutPin, OUTPUT);
  pinMode(BlEnOutPin, OUTPUT);

  pinMode(CCDisInPin, INPUT_PULLUP);
  pinMode(JamInPin, INPUT_PULLUP);
  pinMode(DNegInPin, INPUT_PULLUP);
  pinMode(DisarmInPin, INPUT_PULLUP);
  pinMode(DPosInPin, INPUT_PULLUP);
  pinMode(IgnInPin, INPUT_PULLUP);

  digitalWrite(BlEnOutPin, HIGH);

  int8_t savedState = EEPROM.read(stateAddress);
  if (savedState == NORMAL_STATE)
    xTaskNotify(cch, 0x01, eSetValueWithOverwrite);
  else if (savedState == PROTOCOL_STATE)
    xTaskNotify(cch, 0x05, eSetValueWithOverwrite);
  else if (savedState == SUSPEND_STATE)
    xTaskNotify(cch, 0x03, eSetValueWithOverwrite);
  else if (savedState == JAMMED_STATE)
    xTaskNotify(cch, 0x05, eSetValueWithOverwrite);
  else if (savedState == GPS_STATE)
    xTaskNotify(cch, 0x05, eSetValueWithOverwrite);
  else if (savedState == BLUETOOTH_STATE)
    xTaskNotify(cch, 0x05, eSetValueWithOverwrite);

  // Start the real time scheduler.
  vTaskStartScheduler();
  // Will not get here unless there is insufficient
  // RAM.
  Serial.println("xNo Ram avaliable");
}
//------------------------------------------------------------------------------
// WARNING idle loop has a very small stack (configMINIMAL_STACK_SIZE)
// loop must never block
void loop() {}
//-----------------------------------------------------------------------------
static void vProtocolTask(void *pvParameters)
{
  uint16_t c_ign_on = 0;  /* engine on counter */
  uint16_t c_ign_off = 0; /* engine off counter */
  uint8_t armed = false;
  uint8_t disarm = false;
  uint8_t ignition;
  uint8_t reading;
  uint8_t c_disarm;
  uint8_t c_add;

  uint8_t dPosS = DOORCLOSE;                         // door Positive State
  uint8_t lastDPosS = DOORCLOSE;                     // last door Positive State
  uint8_t dPosR = DOORCLOSE;                         // door Positive reading
  uint32_t lastDPosDebounceTime = 0;                 // the last time the output pin was toggled
  uint32_t debounceDDelay = pdMS_TO_TICKS(1000) * 2; // the debounce time;

  uint8_t dNegS = DOORCLOSE;     // door negative state
  uint8_t lastDNegS = DOORCLOSE; // last door negative state
  uint8_t dNegR = DOORCLOSE;     // door negative reading
  TickType_t lastDNegDebounceTime =
      0; // the last time the output pin was toggled

  uint8_t disarmState = HIGH;
  uint8_t lastDisarmState = HIGH;

  TickType_t lastDebounceTime = 0;               // the last time the output pin was toggled
  TickType_t debounceDelay = pdMS_TO_TICKS(100); // the debounce time;

  // to aviod blocked at the first time
  dPosS = digitalRead(DPosInPin);
  lastDPosS = dPosS;
  dPosR = dPosS;
  dNegS = digitalRead(DNegInPin);
  lastDNegS = dNegS;
  dNegR = dNegS;

  for (;;)
  {
    ignition = !digitalRead(IgnInPin);
    // check ignition ON
    c_ign_on = 0;
    c_ign_off = 0;

    while (ignition && !armed)
    {
      c_ign_on++;
      vTaskDelay(pdMS_TO_TICKS(1000));
      if (BLEDEBUG)
      {
        Serial1.print(F("IgnON "));
        Serial1.print(TIME_AFTER_START - c_ign_on);
        Serial1.print(F("\n"));
      }
      ignition = !digitalRead(IgnInPin);
      if (c_ign_on >= TIME_AFTER_START)
      {
        if (BLEDEBUG)
          Serial1.print(F("xArmed\n"));
        armed = true;
      }
    }
    // end

    // check ignition OFF
    while (!ignition && armed)
    {
      c_ign_off++;
      vTaskDelay(pdMS_TO_TICKS(1000));
      if (BLEDEBUG)
      {
        Serial1.print(F("IgnOFF "));
        Serial1.print(TIME_AFTER_STOP - c_ign_off);
        Serial1.print(F("\n"));
      }
      ignition = !digitalRead(IgnInPin);
      if (c_ign_off >= TIME_AFTER_STOP)
      {
        if (BLEDEBUG)
          Serial1.print(F("xDisarmed\n"));
        armed = false;
      }
    }
    // end
    if (armed)
    {
      digitalWrite(LedOutPin, HIGH);
      // check door status
      dPosR = digitalRead(DPosInPin);
      if (dPosR != lastDPosS)
      {
        lastDPosDebounceTime = xTaskGetTickCount();
        if (BLEDEBUG)
          Serial1.print(F("PosDoor Change\n"));
      }
      if ((xTaskGetTickCount() - lastDPosDebounceTime) > debounceDDelay)
      {
        if (dPosR != dPosS)
        {
          dPosS = dPosR;
          if (dPosS == DOOROPEN)
          {
            xTaskNotify(cch, 0x02, eSetValueWithOverwrite);
            armed = false;
          }
        }
      }
      lastDPosS = dPosR;

      // check door status
      dNegR = digitalRead(DNegInPin);
      if (dNegR != lastDNegS)
      {
        lastDNegDebounceTime = xTaskGetTickCount();
        if (BLEDEBUG)
          Serial1.print(F("NegDoor Change\n"));
      }
      if ((xTaskGetTickCount() - lastDNegDebounceTime) > debounceDDelay)
      {
        if (dNegR != dNegS)
        {
          dNegS = dNegR;
          if (dNegS == DOOROPEN)
          {
            xTaskNotify(cch, 0x02, eSetValueWithOverwrite);
            armed = false;
          }
        }
      }
      lastDNegS = dNegR;

      reading = !digitalRead(DisarmInPin);
      // If the switch changed, due to noise or pressing:
      if (reading != lastDisarmState)
      {
        // reset the debouncing timer
        lastDebounceTime = xTaskGetTickCount();
      }
      if ((xTaskGetTickCount() - lastDebounceTime) > debounceDelay)
      {
        if (reading != disarmState)
        {
          disarmState = reading;
          if (disarmState == LOW)
          {
            disarm = false;
          }
          else
          {
            disarm = pdTRUE;
          }
        }
      }
      lastDisarmState = reading;

      if (disarm)
      {
        digitalWrite(LedOutPin, LOW);
        c_disarm = 0;
        c_add = 0;
        if (BLEDEBUG)
          Serial1.print("xDisarmed\n");
        do
        {
          disarm = !digitalRead(DisarmInPin);
          c_disarm++;
          if (BLEDEBUG)
          {
            Serial1.print(F("xD "));
            Serial1.println(TIME_AFTER_OPEN_DOOR + c_add - c_disarm);
          }
          // if button is pressed , add 20 seconds more to te suspended task
          if (disarm && c_add <= 60)
            c_add += 20;
          vTaskDelay(pdMS_TO_TICKS(1000));
        } while (c_disarm < (TIME_TO_OPEN_DOOR + c_add));

        // to avoid blocked after disarm
        digitalWrite(LedOutPin, HIGH);
        dPosS = digitalRead(DPosInPin);
        lastDPosS = dPosS;
        dNegS = digitalRead(DNegInPin);
        lastDNegS = dNegS;
      }
    } /* end for(;;) */
  }
}
//-----------------------------------------------------------------------------
static void vJammingTask(void *pvParameters)
{
  uint8_t jammed = false;
  uint16_t c_jammed = 0;

  uint8_t dPosS = DOORCLOSE;     // door Positive State
  uint8_t lastDPosS = DOORCLOSE; // last door Positive State
  uint8_t dPosR = DOORCLOSE;     // door Positive reading
  TickType_t lastDPosDebounceTime =
      0; // the last time the output pin was toggled
  TickType_t debounceDDelay =
      pdMS_TO_TICKS(1000) *
      2; // the debounce time; increase if the output flickers

  uint8_t dNegS = DOORCLOSE;     // door negative state
  uint8_t lastDNegS = DOORCLOSE; // last door negative state
  uint8_t dNegR = DOORCLOSE;     // door negative reading
  TickType_t lastDNegDebounceTime =
      0; // the last time the output pin was toggled

  uint8_t jammedState = HIGH;
  uint8_t lastJammedState = HIGH;
  TickType_t lastDebounceTime = 0; // the last time the output pin was toggled
  TickType_t debounceDelay =
      pdMS_TO_TICKS(500); // the debounce time; increase if the output flickers
  int reading;
  // to aviod blocked at the first time
  dPosS = digitalRead(DPosInPin);
  lastDPosS = dPosS;
  dPosR = dPosS;
  dNegS = digitalRead(DNegInPin);
  lastDNegS = dNegS;
  dNegR = dNegS;

  for (;;)
  {
    // if jamming is detected after 2 minute from GPS*/
    reading = !digitalRead(JamInPin);
    // If the switch changed, due to noise or pressing:
    if (reading != lastJammedState)
    {
      // reset the debouncing timer
      lastDebounceTime = xTaskGetTickCount();
    }
    if ((xTaskGetTickCount() - lastDebounceTime) > debounceDelay)
    {
      if (reading != jammedState)
      {
        jammedState = reading;
        if (jammedState == LOW)
        {
          jammed = false;
        }
        else
          jammed = pdTRUE;
      }
    }
    lastJammedState = reading;

    if (!jammed)
      c_jammed = 0;

    if (jammed)
    {
      if (BLEDEBUG)
      {
        Serial1.print(F("xJ "));
        Serial1.print(TIME_JAMMING_SECURE - c_jammed);
      }
      // check door status
      dPosR = digitalRead(DPosInPin);
      if (dPosR != lastDPosS)
        lastDPosDebounceTime = xTaskGetTickCount();
      if ((xTaskGetTickCount() - lastDPosDebounceTime) > debounceDDelay)
      {
        if (dPosR != dPosS)
        {
          dPosS = dPosR;
          if (dPosS == DOOROPEN)
          {
            xTaskNotify(cch, 0x04, eSetValueWithOverwrite);
            jammed = false;
          }
        }
      }
      lastDPosS = dPosR;

      // check door status
      dNegR = digitalRead(DNegInPin);
      if (dNegR != lastDNegS)
        lastDNegDebounceTime = xTaskGetTickCount();
      if ((xTaskGetTickCount() - lastDNegDebounceTime) > debounceDDelay)
      {
        if (dNegR != dNegS)
        {
          dNegS = dNegR;
          if (dNegS == DOOROPEN)
          {
            xTaskNotify(cch, 0x04, eSetValueWithOverwrite);
            jammed = false;
          }
        }
      }
      lastDNegS = dNegR;

      c_jammed++;
      vTaskDelay(pdMS_TO_TICKS(1000));
      if (c_jammed >= TIME_JAMMING_SECURE)
      {
        xTaskNotify(cch, 0x04, eSetValueWithOverwrite);
        jammed = false;
      }
    }
  } /* end for (;;)*/
}
//-----------------------------------------------------------------------------
static void vBlueTask(void *pvParameters)
{
  uint8_t c_data = 0;
  char readingString[BUFF_SIZE]; /* reading string from bluetooth */
  TickType_t xLastDataReceived;

  readingString[0] = '\0';
  xLastDataReceived = xTaskGetTickCount();
  xTimerStart(xtimerMem, 0);
  xTimerStart(xTimerRst, 0);

  for (;;)
  {

    if (EEPROM.read(stateAddress) > 6)
    {
      Serial1.print(F("AT+DEFAULT\r\n"));
      vTaskDelay(configTICK_RATE_HZ);
      Serial1.print(F("AT+RESET\r\n"));
      vTaskDelay(configTICK_RATE_HZ);
      Serial1.print(F("AT+NAMEBLACKGPS\r\n"));
      vTaskDelay(configTICK_RATE_HZ);
      Serial1.print(F("AT+NOTI1\r\n"));
      vTaskDelay(configTICK_RATE_HZ);
      Serial1.print(F("AT+RESET\r\n"));
    }

    if (Serial1.available() > 0)
    {
      readingString[c_data] = Serial1.read();
      readingString[c_data + 1] = '\0';
      // Serial.println(readingString);
      xLastDataReceived = xTaskGetTickCount();
      c_data++;
      // for overflow
      if (c_data >= BUFF_SIZE)
      {
        readingString[0] = '\0';
        c_data = 0;
      }
    }

    if ((xTaskGetTickCount() - xLastDataReceived) > 30)
    {
      c_data = 0;
      vTaskDelay(pdMS_TO_TICKS(300));
    }

    if (strcmp("suspender", readingString) == 0)
    {
      xTaskNotify(cch, 0x03, eSetValueWithOverwrite);
      readingString[0] = '\0';
    }
    if (strcmp("normal", readingString) == 0)
    {
      xTaskNotify(cch, 0x10, eSetValueWithOverwrite);
      readingString[0] = '\0';
    }
    if (strcmp("bloquear", readingString) == 0)
    {
      xTaskNotify(cch, 0x05, eSetValueWithOverwrite);
      readingString[0] = '\0';
    }
    if (strcmp("reset", readingString) == 0)
    {
      restartHW();
    }
    if (strcmp("estado", readingString) == 0)
    {
      getState();
      readingString[0] = '\0';
    }
    if (strcmp("version", readingString) == 0)
    {
      if (BLEDEBUG)
      {
        Serial1.print(F("v"));
        Serial1.println(VERSION);
      }
      readingString[0] = '\0';
    }
    if (strcmp("test", readingString) == 0)
    {
      // Serial1.println("testing");
      //    xTaskNotifyGive(test_handler);
      readingString[0] = '\0';
    }
    if (strcmp("getdio", readingString) == 0)
    {
      getDIO();
    }
    if (strcmp("getmem", readingString) == 0)
    {
      getHighWaterMark();
    }
    if (strcmp("OK+CONN", readingString) == 0)
    {
#ifdef XTERM
      Serial.print(F("BLE connected\n"));
#endif
      xTimerStart(xTimerNotify, 0);
      xTimerStop(xTimerRst, 0);
      vTaskDelay(configTICK_RATE_HZ);
      DEBUG = 1;
      BLEDEBUG = 1;
      for (int i = 1; i <= 2; i++)
      {
        vTaskDelay(configTICK_RATE_HZ);
        getState();
        readingString[0] = '\0';
      }
    }
    if (strcmp("OK+LOST", readingString) == 0)
    {
#ifdef XTERM
      Serial.print(F("BLE disconnected\n"));
#endif
      xTimerStop(xTimerNotify, 0);
      xTimerStart(xTimerRst, 0);
      // restart the bluetooth
      digitalWrite(BlEnOutPin, LOW);
      vTaskDelay(configTICK_RATE_HZ);
      digitalWrite(BlEnOutPin, HIGH);
      DEBUG = 0;
      BLEDEBUG = 0;
      readingString[0] = '\0';
    }
  } /* end for (;;) */
}
//------------------------------------------------------------------------------
static void vCCTask(void *pvParameters)
{
  TickType_t xFrequency = pdMS_TO_TICKS(200);
  uint32_t ulNotifiedValue = 0x00;
  uint8_t savedState;

  for (;;)
  {
    xTaskNotifyWait(
        0x00,             /* Don't clear any notification bits on entry. */
        0xFF,             /* Reset the notification value to 0 on exit. */
        &ulNotifiedValue, /* Notified value pass out in ulNotifiedValue. */
        xFrequency);      /* Block indefinitely. */

    savedState = EEPROM.read(stateAddress);

    if ((ulNotifiedValue == 0x01))
    {
      digitalWrite(CCOutPin, CCOFF);
    }

    /* Activate CC after 1 minute */
    if ((ulNotifiedValue == 0x02))
    {
      uint8_t c_open = 0;
      updateState(PROTOCOL_STATE);
      if (jh != NULL)
        vTaskDelete(jh);
      if (ph != NULL)
        vTaskDelete(ph);
      jh = NULL;
      ph = NULL;

      vTaskSuspend(ioh);

      if (BLEDEBUG)
        Serial1.print(F("Protocol CC Activation "));
      do
      {
        if (BLEDEBUG)
        {
          Serial1.print(F("xCCON"));
          Serial1.println(TIME_TO_STOP_ENGINE - c_open);
        }
        c_open++;

        vTaskDelay(pdMS_TO_TICKS(1000));
      } while (c_open <= TIME_TO_STOP_ENGINE);

      digitalWrite(LedOutPin, HIGH);
      digitalWrite(CCOutPin, CCON);
    }

    /* Deactivate CC and suspend tasks*/
    if ((ulNotifiedValue == 0x03) && (savedState != SUSPEND_STATE))
    {
      updateState(SUSPEND_STATE);
      if (jh != NULL)
        vTaskDelete(jh);
      if (ph != NULL)
        vTaskDelete(ph);
      jh = NULL;
      ph = NULL;

      digitalWrite(CCOutPin, CCOFF);
      digitalWrite(LedOutPin, LOW);
    }

    /* Activate CC with digital CCOutPin in a On/Off secuence */
    if ((ulNotifiedValue == 0x04))
    {
      updateState(JAMMED_STATE);
      if (BLEDEBUG)
        Serial.print(F("Jammed CC activation\n"));

      if (jh != NULL)
        vTaskDelete(jh);
      if (ph != NULL)
        vTaskDelete(ph);

      jh = NULL;
      ph = NULL;

      for (int j = 0; j < 2; j++)
      {
        /* turn on for 2 seconds */
        digitalWrite(CCOutPin, CCON);
        vTaskDelay(pdMS_TO_TICKS(5000));
        /* turn off for 20 seconds */
        digitalWrite(CCOutPin, CCOFF);
        vTaskDelay(configTICK_RATE_HZ * 20);
      }
      /* finally turn off */
      digitalWrite(CCOutPin, CCON);
      digitalWrite(LedOutPin, LOW);
    }
    /* Activate CC with and suspend tasks */
    if ((ulNotifiedValue == 0x05) && (savedState != BLUETOOTH_STATE))
    {
      updateState(BLUETOOTH_STATE);
      if (jh != NULL)
        vTaskDelete(jh);
      if (ph != NULL)
        vTaskDelete(ph);
      jh = NULL;
      ph = NULL;

      digitalWrite(CCOutPin, CCON);
      digitalWrite(LedOutPin, LOW);
    }
    /* Activate CC with and suspend tasks */
    if ((ulNotifiedValue == 0x06) && (savedState != BLUETOOTH_STATE))
    {
      updateState(GPS_STATE);
      if (jh != NULL)
        vTaskDelete(jh);
      if (ph != NULL)
        vTaskDelete(ph);
      jh = NULL;
      ph = NULL;

      digitalWrite(CCOutPin, CCON);
      digitalWrite(LedOutPin, LOW);
    }
    if ((ulNotifiedValue == 0x10) && (savedState != NORMAL_STATE))
    {
      updateState(NORMAL_STATE);
      if (BLEDEBUG)
        Serial.print(F("\nSystem Activate!\n"));
      digitalWrite(CCOutPin, CCOFF);

      xTaskCreate(vProtocolTask, "Protocol", configMINIMAL_STACK_SIZE + 20,
                  NULL, tskIDLE_PRIORITY, &ph);
      xTaskCreate(vJammingTask, "Jamming", configMINIMAL_STACK_SIZE - 10, NULL,
                  tskIDLE_PRIORITY, &jh);
      vTaskResume(ioh);
    }
  } /* end for(;;) */
}
//------------------------------------------------------------------------------
static void vIOTask(void *pvParameters)
{
  uint8_t blocked = false;
  uint8_t _last_state;    /* CCDisInPin pin last state*/
  uint8_t _reading_state; /* CCDisInPin pin reading state */
  uint8_t _current_state; /* CCDisInPin pin current state */
  TickType_t lastDebounceTime = 0;
  TickType_t debounceDelay = pdMS_TO_TICKS(500);

  _reading_state = digitalRead(CCDisInPin);
  _last_state = _reading_state;

  int8_t savedState = EEPROM.read(stateAddress);

  if (savedState == JAMMED_STATE || savedState == PROTOCOL_STATE ||
      savedState == GPS_STATE || savedState == BLUETOOTH_STATE)
  {
    _last_state = HIGH;    /* CCDisInPin pin last state*/
    _reading_state = HIGH; /* CCDisInPin pin reading state */
    _current_state = HIGH; /* CCDisInPin pin current state */
  }
  if (savedState == SUSPEND_STATE || savedState == NORMAL_STATE)
  {
    _last_state = LOW;    /* CCDisInPin pin last state*/
    _reading_state = LOW; /* CCDisInPin pin reading state */
    _current_state = LOW; /* CCDisInPin pin current state */
  }

  for (;;)
  {

    savedState = EEPROM.read(stateAddress);
    if (savedState == JAMMED_STATE || savedState == PROTOCOL_STATE ||
        savedState == GPS_STATE || savedState == BLUETOOTH_STATE)
    {
      blocked = pdTRUE;
    }
    if (savedState == SUSPEND_STATE || savedState == NORMAL_STATE)
    {
      blocked = !pdTRUE;
    }

    _reading_state = digitalRead(CCDisInPin);
    if (_reading_state != _last_state)
    {
      lastDebounceTime = xTaskGetTickCount();
    }

    if ((xTaskGetTickCount() - lastDebounceTime) > debounceDelay)
    {
      if (_reading_state != _current_state)
      {
        _current_state = _reading_state;
        if (_current_state == LOW && blocked)
        {
          xTaskNotify(cch, 0x10, eSetValueWithOverwrite);
        }
        // if CCDisInPin is LOW then CC ON
        if (_current_state == HIGH && !blocked)
        {
          xTaskNotify(cch, 0x06, eSetValueWithOverwrite);
        }
      }
    }
    _last_state = _reading_state;
  }
}

static void vTestTask(void *pvParameters)
{
  int inputs = 6;
  uint8_t outputs = 2;
  uint8_t open[inputs];  // open oputpus open
  uint8_t close[inputs]; // door positive closed
  uint8_t ready = 0;
  uint8_t reading[inputs]; // door positive reading
  uint8_t input[inputs];
  uint8_t output[outputs];
  uint8_t _isOk[inputs]; // door positive closed

  uint8_t i = 0;
  TickType_t timeout = pdMS_TO_TICKS(1000) * 20;
  TickType_t lastDebounceTime = 0;

  output[0] = LedOutPin;
  output[1] = CCOutPin;

  input[0] = CCDisInPin;
  input[1] = IgnInPin;
  input[2] = JamInPin;
  input[3] = DNegInPin;
  input[4] = DisarmInPin;
  input[5] = DPosInPin;

  // open and close states are closed
  for (int j = 0; j < inputs; j++)
  {
    open[j] = false;
    close[j] = false;
    _isOk[j] = false;
  }

  for (;;)
  {

    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    xTimerStop(xTimerNotify, 0);
    vTaskSuspend(cch);
    vTaskSuspend(jh);
    vTaskSuspend(ph);
    vTaskSuspend(ioh);

    for (int j = 0; j < outputs; j++)
    {
      digitalWrite(output[j], LOW);
    }

    lastDebounceTime = xTaskGetTickCount();

    for (int o = 0; o < outputs; o++)
    {
      Serial1.print("Trying Output: ");
      Serial1.println(o + 1);
      digitalWrite(output[o], HIGH);

      while (ready != inputs)
      {

        reading[i] = digitalRead(input[i]);
        vTaskDelay(pdMS_TO_TICKS(200));

        // check reading status
        if (reading[i] == HIGH)
          open[i] = pdTRUE;
        if (reading[i] == LOW) // door positive opened
          close[i] = pdTRUE;

        // in case there is and input change
        if (open[i] && close[i] && !_isOk[i])
        {
          _isOk[i] = pdTRUE;
          if (ready == 0)
          {
            Serial1.print("Output  ");
            Serial1.print(o + 1);
            Serial1.print("/");
            Serial1.print(outputs);

            if (o == 0)
              Serial1.print(" Led ");
            if (o == 1)
              Serial1.print(" CC ");

            Serial1.println(" OK");
          }

          Serial1.print("Input:");
          Serial1.print(i + 1);
          Serial1.print(" - ");
          Serial1.print(ready + 1);
          Serial1.print("/");
          Serial1.print(inputs);

          if (i == 0)
            Serial1.print(" CCDisable ");
          if (i == 1)
            Serial1.print(" Ignition ");
          if (i == 2)
            Serial1.print(" Jamming ");
          if (i == 3)
            Serial1.print(" DoorNegative ");
          if (i == 4)
            Serial1.print(" Disarm ");
          if (i == 5)
            Serial1.print(" DoorPositive ");

          Serial1.println(" OK");
          open[i] = false;
          close[i] = false;
          ready++;
        }
        i++;

        if ((xTaskGetTickCount() - lastDebounceTime) > timeout)
        {
          Serial1.println("Timeout!");
          //  Serial1.println("Check Device!");
          ready = inputs;
          digitalWrite(output[o], LOW);
          lastDebounceTime = xTaskGetTickCount();
          vTaskDelay(pdMS_TO_TICKS(1000) * 5);
        }
        if (i == inputs)
          i = 0;
      } // end while

      digitalWrite(output[o], LOW);
      for (int j = 0; j < inputs; j++)
      {
        if (_isOk[j] == false)
        {
          if (j == 0)
            Serial1.print(" CCDisable ");
          if (j == 1)
            Serial1.print(" Ignition ");
          if (j == 2)
            Serial1.print(" Jamming ");
          if (j == 3)
            Serial1.print(" DoorNegative ");
          if (j == 4)
            Serial1.print(" Disarm ");
          if (j == 5)
            Serial1.print(" DoorPositive ");
          Serial1.println(" ERROR!");
        }
        open[j] = false;
        close[j] = false;
        _isOk[j] = false;
      }
      ready = 0;
    } // end for

    xTimerStart(xTimerNotify, 0);
    vTaskResume(jh);
    vTaskResume(ph);
    vTaskResume(ioh);
    vTaskResume(cch);
    Serial1.println("Test ended");
    restartHW();
  }
}

void restartHW()
{
  do
  {
    wdt_enable(WDTO_15MS);
    for (;;)
    {
    }
  } while (0);
}
void vTimerCallback(TimerHandle_t xTimer)
{
  getState();
}
void vTimerRestart(TimerHandle_t xTimer)
{
  // restart the bluetooth
  digitalWrite(BlEnOutPin, LOW);
  vTaskDelay(configTICK_RATE_HZ);
  vTaskDelay(configTICK_RATE_HZ);
  digitalWrite(BlEnOutPin, HIGH);

  int16_t savedState = EEPROM.read(stateAddress);
  configASSERT(pxTimer);

  if (digitalRead(IgnInPin) || savedState == PROTOCOL_STATE ||
      savedState == JAMMED_STATE || savedState == GPS_STATE ||
      savedState == BLUETOOTH_STATE) // if ignition off
    c_off++;
  else
    c_off = 0;
#ifdef XTERM
  Serial.print("Restart in ");
  Serial.println(TIME_RESET_HW - c_off);
#endif
  if (c_off >= TIME_RESET_HW)
    restartHW();
}
void updateState(int8_t _state)
{
  EEPROM.update(stateAddress, _state);
  if (BLEDEBUG)
    getState();
}
void vTimerMemCallback(TimerHandle_t xTimer)
{
  Serial.print(F("xStack Remain\n"));

  Serial.print(pcTaskGetName(jh));
  Serial.print("\t");
  Serial.print(pcTaskGetName(ph));
  Serial.print(F("\t"));
  Serial.print(pcTaskGetName(ioh));
  Serial.print(F("\t"));
  Serial.print(pcTaskGetName(bh));
  Serial.print(F("\t"));
  Serial.print(pcTaskGetName(cch));
  Serial.print(F("\t"));
  Serial.print(F("freeMem "));
  Serial.print(F("\n"));
  Serial.print(uxTaskGetStackHighWaterMark(jh));
  Serial.print(F("\t"));
  Serial.print(uxTaskGetStackHighWaterMark(ph));
  Serial.print(F("\t"));
  Serial.print(uxTaskGetStackHighWaterMark(ioh));
  Serial.print(F("\t"));
  Serial.print(uxTaskGetStackHighWaterMark(bh));
  Serial.print(F("\t"));
  Serial.print(uxTaskGetStackHighWaterMark(cch));
  Serial.print(F("\t"));
  Serial.print(freeMemory());
  Serial.print(F("\n"));
}

/*
void getHighWaterMark() {
  char _message[2];
  UBaseType_t uxHighWaterMark;
  printBle("xStack Remain\n");

  printBle("freeMemory()= ");
  Serial1.println(freeMemory());

  uxHighWaterMark = uxTaskGetStackHighWaterMark(jh);
  printBle("xJammingTask ");
  sprintf(_message, "%d\n", uxHighWaterMark);
  printBle(_message);

  uxHighWaterMark = uxTaskGetStackHighWaterMark(ph);
  printBle("xProtocolTask ");
  sprintf(_message, "%d\n", uxHighWaterMark);
  printBle(_message);

  uxHighWaterMark = uxTaskGetStackHighWaterMark(ioh);
  printBle("xIOTask ");
  sprintf(_message, "%d\n", uxHighWaterMark);
  printBle(_message);

  uxHighWaterMark = uxTaskGetStackHighWaterMark(bh);
  printBle("xBlueTask ");
  sprintf(_message, "%d\n", uxHighWaterMark);
  printBle(_message);

  uxHighWaterMark = uxTaskGetStackHighWaterMark(cch);
  printBle("xCCTask ");
  sprintf(_message, "%d\n", uxHighWaterMark);
  printBle(_message);

  printBle("freeMemory()= ");
  Serial1.println(freeMemory());
}
*/
void getHighWaterMark()
{

  Serial.print(F("xStack Remain\n"));

  Serial1.print(pcTaskGetName(jh));
  Serial1.print(F("\t"));
  Serial1.print(pcTaskGetName(ph));
  Serial1.print(F("\t"));
  Serial1.print(pcTaskGetName(ioh));
  Serial1.print(F("\t"));
  Serial1.print(pcTaskGetName(bh));
  Serial1.print(F("\t"));
  Serial1.print(pcTaskGetName(cch));
  Serial1.print(F("\t"));
  Serial1.print(F("freeMemory "));
  Serial1.print(F("\n"));

  Serial1.print(uxTaskGetStackHighWaterMark(jh));
  Serial1.print(F("\t"));
  Serial1.print(uxTaskGetStackHighWaterMark(ph));
  Serial1.print(F("\t"));
  Serial1.print(uxTaskGetStackHighWaterMark(ioh));
  Serial1.print(F("\t"));
  Serial1.print(uxTaskGetStackHighWaterMark(bh));
  Serial1.print(F("\t"));
  Serial1.print(uxTaskGetStackHighWaterMark(cch));
  Serial1.print(F("\t"));
  Serial1.print(freeMemory());
  Serial1.print(F("\n"));
}
void getDIO()
{
  if (!digitalRead(IgnInPin))
    Serial1.print(F("Ignition ON\n"));
  else
    Serial1.print(F("Igntion OFF\n"));
  if (digitalRead(DPosInPin) == DOOROPEN)
    Serial1.print(F("PosDoor Open\n"));
  else
    Serial1.print(F("PosDoor Closed\n"));

  if (digitalRead(DNegInPin) == DOOROPEN)
    Serial1.print(F("NegDoor Open\n"));
  else
    Serial1.print(F("NegDoor Closed\n"));
  if (!digitalRead(DisarmInPin))
    Serial1.print(F("Disarmed\n"));
  else
    Serial1.print(F("Armed\n"));

  if (digitalRead(JamInPin) == LOW)
    Serial1.print(F("Jammed ON\n"));
  else
    Serial1.print(F("Jammed OFF\n"));
  if (digitalRead(CCDisInPin) == LOW)
    Serial1.print(F("CCDisable OFF\n"));
  else
    Serial1.print(F("CCDisable ON\n"));
}
void getState()
{
  if (BLEDEBUG)
  {
    Serial1.print(F("\ns"));
    Serial1.println(EEPROM.read(stateAddress));
    Serial1.print(F("v"));
    Serial1.println(VERSION);
  }
}
