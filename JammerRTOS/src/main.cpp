#include "MemoryFree.h"
#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <EEPROM.h>
#include <avr/wdt.h>
#include <timers.h>

<<<<<<< HEAD
const char VERSION[] = "3.0.9";
int8_t DEBUG = 0;
=======
const char VERSION[] = "3.0.8";

>>>>>>> b4c199183646342b025353b55e717f9dbb9d1057
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

uint8_t stateAddress = 20;
uint8_t address = 0;
uint16_t c_off = 0;

TaskHandle_t cch;
TaskHandle_t jh;
TaskHandle_t ph;
TaskHandle_t bh;
TaskHandle_t ioh;
TaskHandle_t test_handler;

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
<<<<<<< HEAD
void printBle(int8_t _print, char *_buff);
void getHighWaterMark();
void getDIO();
void getState();
=======
void getHighWaterMark();
>>>>>>> b4c199183646342b025353b55e717f9dbb9d1057

void setup() {

  Serial1.begin(9600);
  Serial.begin(9600);

  wdt_enable(WDTO_2S);
  xTaskCreate(vBlueTask, "Blue", configMINIMAL_STACK_SIZE + 40, NULL,
              tskIDLE_PRIORITY + 1, &bh);
  xTaskCreate(vCCTask, "CC", configMINIMAL_STACK_SIZE - 30, NULL,
              tskIDLE_PRIORITY, &cch);
  xTaskCreate(vProtocolTask, "Protocol", configMINIMAL_STACK_SIZE, NULL,
              tskIDLE_PRIORITY, &ph);
<<<<<<< HEAD
  xTaskCreate(vJammingTask, "Jamming", configMINIMAL_STACK_SIZE - 10, NULL,
=======
  xTaskCreate(vJammingTask, "Jamming", configMINIMAL_STACK_SIZE - 20, NULL,
>>>>>>> b4c199183646342b025353b55e717f9dbb9d1057
              tskIDLE_PRIORITY, &jh);
  xTaskCreate(vIOTask, "IO", configMINIMAL_STACK_SIZE - 30, NULL,
              tskIDLE_PRIORITY, &ioh);
  //  xTaskCreate(vTestTask, "Test", configMINIMAL_STACK_SIZE, NULL,
  //              tskIDLE_PRIORITY, &test_handler);

  xTimerNotify = xTimerCreate("TimerNotify", configTICK_RATE_HZ * 15, pdTRUE,
                              (void *)0, vTimerCallback);
  xTimerRst = xTimerCreate("TimerRst", configTICK_RATE_HZ * 10, pdTRUE,
                           (void *)0, vTimerRestart);
<<<<<<< HEAD
  //  xtimerMem = xTimerCreate("TimerMem", configTICK_RATE_HZ * 5, pdTRUE,
=======
  //  xtimerMem = xTimerCreate("TimerMem", configTICK_RATE_HZ * 120, pdTRUE,
>>>>>>> b4c199183646342b025353b55e717f9dbb9d1057
  //                           (void *)0, vTimerMemCallback);

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
  //  Serial1.println("xNo Ram avaliable");
}
//------------------------------------------------------------------------------
// WARNING idle loop has a very small stack (configMINIMAL_STACK_SIZE)
// loop must never block
void loop() {}
//-----------------------------------------------------------------------------
static void vProtocolTask(void *pvParameters) {

  uint16_t c_ign_on = 0;  /* engine on counter */
  uint16_t c_ign_off = 0; /* engine off counter */
  uint8_t armed = false;
  uint8_t disarm = false;
  uint8_t ignition;

  uint8_t dPosS = DOORCLOSE;         // door Positive State
  uint8_t lastDPosS = DOORCLOSE;     // last door Positive State
  uint8_t dPosR = DOORCLOSE;         // door Positive reading
  uint32_t lastDPosDebounceTime = 0; // the last time the output pin was toggled
  uint32_t debounceDDelay = pdMS_TO_TICKS(1000) * 2; // the debounce time;

  uint8_t dNegS = DOORCLOSE;     // door negative state
  uint8_t lastDNegS = DOORCLOSE; // last door negative state
  uint8_t dNegR = DOORCLOSE;     // door negative reading
  TickType_t lastDNegDebounceTime =
      0; // the last time the output pin was toggled

  uint8_t disarmState = HIGH;
  uint8_t lastDisarmState = HIGH;

  TickType_t lastDebounceTime = 0; // the last time the output pin was toggled
  TickType_t debounceDelay = pdMS_TO_TICKS(100); // the debounce time;

  // to aviod blocked at the first time
  dPosS = digitalRead(DPosInPin);
  lastDPosS = dPosS;
  dPosR = dPosS;
  dNegS = digitalRead(DNegInPin);
  lastDNegS = dNegS;
  dNegR = dNegS;

  for (;;) {
    ignition = !digitalRead(IgnInPin);
    // check ignition ON
    c_ign_on = 0;
    c_ign_off = 0;

    while (ignition && !armed) {
      char _message[13];
      c_ign_on++;
      vTaskDelay(pdMS_TO_TICKS(1000));
      sprintf(_message, "IgnOn %d\n", c_ign_on);
      printBle(DEBUG, _message);
      ignition = !digitalRead(IgnInPin);
      if (c_ign_on >= TIME_AFTER_START) {
        printBle(DEBUG, "xArmed\n");
        armed = true;
      }
    }
    // end

    // check ignition OFF
    while (!ignition && armed) {
      c_ign_off++;
      char _message[13];
      vTaskDelay(pdMS_TO_TICKS(1000));
      sprintf(_message, "IgnOFF %d\n", c_ign_off);
      printBle(DEBUG, _message);
      ignition = !digitalRead(IgnInPin);
      if (c_ign_off >= TIME_AFTER_STOP) {
        armed = false;
      }
    }
    // end
    if (armed) {
      // check door status
      digitalWrite(LedOutPin, HIGH);
      vTaskDelay(pdMS_TO_TICKS(200));
      digitalWrite(LedOutPin, LOW);

      dPosR = digitalRead(DPosInPin);

      if (dPosR != lastDPosS) {
        lastDPosDebounceTime = xTaskGetTickCount();
        printBle(DEBUG, "PosDoor Change\n");
      }

      if ((xTaskGetTickCount() - lastDPosDebounceTime) > debounceDDelay) {
        if (dPosR != dPosS) {
          dPosS = dPosR;
          if (dPosS == DOOROPEN) {
            updateState(PROTOCOL_STATE);
            xTaskNotify(cch, 0x02, eSetValueWithOverwrite);
            armed = false;
          }
        }
      }
      lastDPosS = dPosR;

      // check door status
      dNegR = digitalRead(DNegInPin);

      if (dNegR != lastDNegS) {
        lastDNegDebounceTime = xTaskGetTickCount();
        printBle(DEBUG, "NegDoor Change\n");
      }

      if ((xTaskGetTickCount() - lastDNegDebounceTime) > debounceDDelay) {
        if (dNegR != dNegS) {
          dNegS = dNegR;
          if (dNegS == DOOROPEN) {
            updateState(PROTOCOL_STATE);
            xTaskNotify(cch, 0x02, eSetValueWithOverwrite);
            armed = false;
          }
        }
      }
      lastDNegS = dNegR;

      int8_t reading = !digitalRead(DisarmInPin);

      // If the switch changed, due to noise or pressing:
      if (reading != lastDisarmState) {
        // reset the debouncing timer
        lastDebounceTime = xTaskGetTickCount();
      }

      if ((xTaskGetTickCount() - lastDebounceTime) > debounceDelay) {
        if (reading != disarmState) {
          disarmState = reading;
          if (disarmState == LOW) {
            disarm = false;
          } else {
            disarm = pdTRUE;
          }
        }
      }
      lastDisarmState = reading;

      if (disarm) {
        digitalWrite(LedOutPin, LOW);
        int8_t c_disarm = 0;
        int8_t toggle = false;
        int8_t c_add = 0;
        printBle(DEBUG, "xDisarmed\n");
        char _message[4];
        do {

          disarm = !digitalRead(DisarmInPin);
          digitalWrite(LedOutPin, toggle);
          c_disarm++;
          toggle = !toggle;
          sprintf(_message, "%d ", (TIME_TO_OPEN_DOOR + c_add) - c_disarm);
          printBle(DEBUG, _message);
          // if button is pressed , add 20 seconds more to te suspended task
          if (disarm && c_add <= 60)
            c_add += 20;
          vTaskDelay(pdMS_TO_TICKS(1000));
        } while (c_disarm <= (TIME_TO_OPEN_DOOR + c_add));

        // to avoid blocked after disarm
        digitalWrite(LedOutPin, HIGH);
        dPosS = digitalRead(DPosInPin);
        lastDPosS = dPosS;
        dNegS = digitalRead(DNegInPin);
        lastDNegS = dNegS;
      }
    }
  } /* end for(;;) */
}
//-----------------------------------------------------------------------------
static void vJammingTask(void *pvParameters) {

  uint8_t jammed = false;
  uint16_t c_jammed = 0;

  uint8_t dPosS = DOORCLOSE;     // door Positive State
  uint8_t lastDPosS = DOORCLOSE; // last door Positive State
  uint8_t dPosR = DOORCLOSE;     // door Positive reading
  TickType_t long lastDPosDebounceTime =
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

  // to aviod blocked at the first time
  dPosS = digitalRead(DPosInPin);
  lastDPosS = dPosS;
  dPosR = dPosS;
  dNegS = digitalRead(DNegInPin);
  lastDNegS = dNegS;
  dNegR = dNegS;

  for (;;) {
    // if jamming is detected after 2 minute from GPS*/
    int reading = !digitalRead(JamInPin);
    // If the switch changed, due to noise or pressing:
    if (reading != lastJammedState) {
      // reset the debouncing timer
      lastDebounceTime = xTaskGetTickCount();
    }
    if ((xTaskGetTickCount() - lastDebounceTime) > debounceDelay) {
      if (reading != jammedState) {
        jammedState = reading;
        if (jammedState == LOW) {
          jammed = false;
        } else
          jammed = pdTRUE;
      }
    }
    lastJammedState = reading;

    if (!jammed)
      c_jammed = 0;

    if (jammed) {
      char _message[13];
      sprintf(_message, "xJammed %d\n", c_jammed);
      printBle(DEBUG, _message);

      // check door status
      dPosR = digitalRead(DPosInPin);

      if (dPosR != lastDPosS)
        lastDPosDebounceTime = xTaskGetTickCount();

      if ((xTaskGetTickCount() - lastDPosDebounceTime) > debounceDDelay) {
        if (dPosR != dPosS) {
          dPosS = dPosR;
          if (dPosS == DOOROPEN) {
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

      if ((xTaskGetTickCount() - lastDNegDebounceTime) > debounceDDelay) {
        if (dNegR != dNegS) {
          dNegS = dNegR;
          if (dNegS == DOOROPEN) {
            updateState(JAMMED_STATE);
            xTaskNotify(cch, 0x04, eSetValueWithOverwrite);
            jammed = false;
          }
        }
      }
      lastDNegS = dNegR;

      c_jammed++;
      vTaskDelay(pdMS_TO_TICKS(1000));
      if (c_jammed >= TIME_JAMMING_SECURE) {
        updateState(JAMMED_STATE);
        xTaskNotify(cch, 0x04, eSetValueWithOverwrite);
        jammed = false;
      }
    }
  } /* end for (;;)*/
}
//-----------------------------------------------------------------------------
static void vBlueTask(void *pvParameters) {
  uint8_t c_data = 0;
  char readingString[BUFF_SIZE]; /* reading string from bluetooth */
  TickType_t xLastDataReceived;
<<<<<<< HEAD
=======
  uint8_t config = pdTRUE;
  UBaseType_t uxHighWaterMark;

  int i;
  String membuff = "0";
>>>>>>> b4c199183646342b025353b55e717f9dbb9d1057

  i = 0;
  readingString[0] = '\0';
  xLastDataReceived = xTaskGetTickCount();
  //  xTimerStart(xtimerMem, 0);
  xTimerStart(xTimerRst, 0);

  for (;;) {

<<<<<<< HEAD
    if (EEPROM.read(stateAddress) > 6) {
=======
    if (config) {
>>>>>>> b4c199183646342b025353b55e717f9dbb9d1057
      Serial1.print("AT+DEFAULT\r\n");
      vTaskDelay(configTICK_RATE_HZ);
      Serial1.print("AT+RESET\r\n");
      vTaskDelay(configTICK_RATE_HZ);
      Serial1.print("AT+NAMEBLACKGPS\r\n");
      vTaskDelay(configTICK_RATE_HZ);
      Serial1.print("AT+NOTI1\r\n");
      vTaskDelay(configTICK_RATE_HZ);
      Serial1.print("AT+RESET\r\n");
    }

    if (Serial1.available() > 0) {
      readingString[c_data] = Serial1.read();
      readingString[c_data + 1] = '\0';
      // Serial.println(readingString);
      xLastDataReceived = xTaskGetTickCount();
      c_data++;
      // for overflow
      if (c_data >= BUFF_SIZE) {
        readingString[0] = '\0';
        c_data = 0;
      }
    }

    if ((xTaskGetTickCount() - xLastDataReceived) > 30) {
      c_data = 0;
      vTaskDelay(pdMS_TO_TICKS(300));
    }

    if (strcmp("suspender", readingString) == 0) {
      updateState(SUSPEND_STATE);
      xTaskNotify(cch, 0x03, eSetValueWithOverwrite);
      readingString[0] = '\0';
    }
    if (strcmp("normal", readingString) == 0) {
      updateState(NORMAL_STATE);
      xTaskNotify(cch, 0x10, eSetValueWithOverwrite);
      readingString[0] = '\0';
    }
    if (strcmp("bloquear", readingString) == 0) {
      updateState(BLUETOOTH_STATE);
      xTaskNotify(cch, 0x05, eSetValueWithOverwrite);
      readingString[0] = '\0';
    }
    if (strcmp("reset", readingString) == 0) {
      xTaskNotify(cch, 0x10, eSetValueWithOverwrite);
      //  Serial1.println("Reset");
      readingString[0] = '\0';
      vTaskDelay(pdMS_TO_TICKS(500));
      do {
        wdt_enable(WDTO_15MS);
        for (;;) {
        }
      } while (0);
    }
    if (strcmp("estado", readingString) == 0) {
      getState();
      readingString[0] = '\0';
    }
    if (strcmp("version", readingString) == 0) {
      Serial1.print("v");
      Serial1.println(VERSION);
      readingString[0] = '\0';
    }
    if (strcmp("test", readingString) == 0) {
      // Serial1.println("testing");
      //    xTaskNotifyGive(test_handler);
      readingString[0] = '\0';
    }
<<<<<<< HEAD
    if (strcmp("getdio", readingString) == 0) {
      getDIO();
    }
    if (strcmp("getmem", readingString) == 0) {
      getHighWaterMark();
=======
    if (strcmp("memtest", readingString) == 0) {
      getHighWaterMark();
      readingString[0] = '\0';
    }
    if (strcmp("alocate", readingString) == 0) {
      membuff.concat("corega");
      Serial1.println(membuff);
      uxHighWaterMark = uxTaskGetStackHighWaterMark(bh);
      Serial1.print("xBlueTask ");
      Serial1.print(uxHighWaterMark);
      Serial1.println(" bytes ");
      i = 1 + i;
      //    Serial1.print(i);
      //    Serial1.println(" Bytes");
      vTaskDelay(configTICK_RATE_HZ / 10);
>>>>>>> b4c199183646342b025353b55e717f9dbb9d1057
    }
    if (strcmp("OK+CONN", readingString) == 0) {
      xTimerStart(xTimerNotify, 0);
      xTimerStop(xTimerRst, 0);
      vTaskDelay(configTICK_RATE_HZ * 1);
      int8_t savedState = EEPROM.read(stateAddress);
      DEBUG = 1;
      for (int i = 1; i <= 2; i++) {
        vTaskDelay(configTICK_RATE_HZ);
        Serial1.println();
        Serial1.print("s");
        Serial1.println(savedState);
        Serial1.print("v");
        Serial1.println(VERSION);
        readingString[0] = '\0';
      }
    }
    if (strcmp("OK+LOST", readingString) == 0) {

      xTimerStop(xTimerNotify, 0);
      xTimerStart(xTimerRst, 0);
      // restart the bluetooth
      digitalWrite(BlEnOutPin, LOW);
      vTaskDelay(configTICK_RATE_HZ);
      digitalWrite(BlEnOutPin, HIGH);
      DEBUG = 0;
      readingString[0] = '\0';
    }
  } /* end for (;;) */
}
//------------------------------------------------------------------------------
static void vCCTask(void *pvParameters) {
  TickType_t xFrequency = pdMS_TO_TICKS(10);
  uint32_t ulNotifiedValue = 0x00;

  for (;;) {
    xTaskNotifyWait(
        0x00,             /* Don't clear any notification bits on entry. */
        0xFF,             /* Reset the notification value to 0 on exit. */
        &ulNotifiedValue, /* Notified value pass out in ulNotifiedValue. */
        xFrequency);      /* Block indefinitely. */

    if ((ulNotifiedValue == 0x01)) {
      digitalWrite(CCOutPin, CCOFF);
    }

    /* Activate CC after 1 minute */
    if ((ulNotifiedValue == 0x02)) {
      vTaskDelete(jh);
      vTaskDelete(ph);
      vTaskSuspend(ioh);
      int8_t c_open = 0;
      int8_t toggle = false;
      printBle(DEBUG, "Protocol CC Activation ");
      do {
        char _message[3];
        sprintf(_message, "%d ", TIME_TO_STOP_ENGINE - c_open);
        printBle(DEBUG, _message);
        c_open++;
        toggle = !toggle;
        vTaskDelay(pdMS_TO_TICKS(1000));
      } while (c_open <= TIME_TO_STOP_ENGINE);

      digitalWrite(LedOutPin, HIGH);
      digitalWrite(CCOutPin, CCON);
      vTaskDelay(pdMS_TO_TICKS(500));
    }

    /* Deactivate CC and suspend tasks*/
    if ((ulNotifiedValue == 0x03)) {
      vTaskDelete(jh);
      vTaskDelete(ph);
      digitalWrite(CCOutPin, CCOFF);
      digitalWrite(LedOutPin, LOW);
      vTaskDelay(pdMS_TO_TICKS(500));
    }

    /* Activate CC with digital CCOutPin in a On/Off secuence */
    if ((ulNotifiedValue == 0x04)) {

      printBle(DEBUG, "Jammed CC activation \n");
      for (int j = 0; j < 2; j++) {
<<<<<<< HEAD

        vTaskDelete(jh);
        vTaskDelete(ph);
=======
        vTaskDelete(jh);
        vTaskDelete(ph);
        vTaskSuspend(ioh);
>>>>>>> b4c199183646342b025353b55e717f9dbb9d1057
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
    if ((ulNotifiedValue == 0x05)) {
      vTaskDelete(jh);
      vTaskDelete(ph);
      digitalWrite(CCOutPin, CCON);
      digitalWrite(LedOutPin, LOW);
    }
    if ((ulNotifiedValue == 0x10)) {
<<<<<<< HEAD
      printBle(DEBUG, "\nSystem Activate!\n");
=======
      Serial1.println("System Activated!");
>>>>>>> b4c199183646342b025353b55e717f9dbb9d1057
      digitalWrite(CCOutPin, CCOFF);
      xTaskCreate(vProtocolTask, "Protocol", configMINIMAL_STACK_SIZE - 30,
                  NULL, tskIDLE_PRIORITY, &ph);
      xTaskCreate(vJammingTask, "Jamming", configMINIMAL_STACK_SIZE - 30, NULL,
                  tskIDLE_PRIORITY, &jh);
      vTaskResume(ioh);
    }
  } /* end for(;;) */
}
//------------------------------------------------------------------------------
static void vIOTask(void *pvParameters) {
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
      savedState == GPS_STATE || savedState == BLUETOOTH_STATE) {
    _last_state = HIGH;    /* CCDisInPin pin last state*/
    _reading_state = HIGH; /* CCDisInPin pin reading state */
    _current_state = HIGH; /* CCDisInPin pin current state */
  }
  if (savedState == SUSPEND_STATE || savedState == NORMAL_STATE) {
    _last_state = LOW;    /* CCDisInPin pin last state*/
    _reading_state = LOW; /* CCDisInPin pin reading state */
    _current_state = LOW; /* CCDisInPin pin current state */
  }

  for (;;) {

    int8_t savedState = EEPROM.read(stateAddress);
    if (savedState == JAMMED_STATE || savedState == PROTOCOL_STATE ||
        savedState == GPS_STATE || savedState == BLUETOOTH_STATE) {
      blocked = pdTRUE;
    }
    if (savedState == SUSPEND_STATE || savedState == NORMAL_STATE) {
      blocked = !pdTRUE;
    }

    _reading_state = digitalRead(CCDisInPin);
    if (_reading_state != _last_state) {
      lastDebounceTime = xTaskGetTickCount();
    }

    if ((xTaskGetTickCount() - lastDebounceTime) > debounceDelay) {
      if (_reading_state != _current_state) {
        _current_state = _reading_state;
        if (_current_state == LOW && blocked) {
          updateState(NORMAL_STATE);
          xTaskNotify(cch, 0x10, eSetValueWithOverwrite);
        }
        // if CCDisInPin is LOW then CC ON
        if (_current_state == HIGH && !blocked) {
          updateState(GPS_STATE);
          xTaskNotify(cch, 0x05, eSetValueWithOverwrite);
        }
      }
    }

    _last_state = _reading_state;
  }
}

static void vTestTask(void *pvParameters) {

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
  for (int j = 0; j < inputs; j++) {
    open[j] = false;
    close[j] = false;
    _isOk[j] = false;
  }

  for (;;) {

    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    xTimerStop(xTimerNotify, 0);
    vTaskSuspend(cch);
    vTaskSuspend(jh);
    vTaskSuspend(ph);
    vTaskSuspend(ioh);

    for (int j = 0; j < outputs; j++) {
      digitalWrite(output[j], LOW);
    }

    lastDebounceTime = xTaskGetTickCount();

    for (int o = 0; o < outputs; o++) {
      Serial1.print("Trying Output: ");
      Serial1.println(o + 1);
      digitalWrite(output[o], HIGH);

      while (ready != inputs) {

        reading[i] = digitalRead(input[i]);
        vTaskDelay(pdMS_TO_TICKS(200));

        // check reading status
        if (reading[i] == HIGH)
          open[i] = pdTRUE;
        if (reading[i] == LOW) // door positive opened
          close[i] = pdTRUE;

        // in case there is and input change
        if (open[i] && close[i] && !_isOk[i]) {
          _isOk[i] = pdTRUE;
          if (ready == 0) {
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

        if ((xTaskGetTickCount() - lastDebounceTime) > timeout) {
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
      for (int j = 0; j < inputs; j++) {
        if (_isOk[j] == false) {
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

void printBle(int8_t _print, char *_buff) {
  if (_print) {
    int _size = strlen(_buff);
    if (_size > 0) {
      Serial1.print(_buff);
    }
  }
}

void restartHW() {
  do {
    wdt_enable(WDTO_15MS);
    for (;;) {
    }
  } while (0);
}
void vTimerCallback(TimerHandle_t xTimer) { getState(); }
void vTimerRestart(TimerHandle_t xTimer) {
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

  Serial.println(c_off);
  if (c_off >= TIME_RESET_HW)
    restartHW();
}
void updateState(int8_t _state) {
  char _message[4];
  EEPROM.update(stateAddress, _state);
  sprintf(_message, "s%d\n", _state);
  printBle(DEBUG, _message);
}
void vTimerMemCallback(TimerHandle_t xTimer) { getHighWaterMark(); }
<<<<<<< HEAD
=======

>>>>>>> b4c199183646342b025353b55e717f9dbb9d1057
void getHighWaterMark() {
  /* Inspect our own high water mark on entering the task. */
  UBaseType_t uxHighWaterMark;
  char _message[10];
  printBle(DEBUG, "xStack Remain\n");

  uxHighWaterMark = uxTaskGetStackHighWaterMark(jh);
<<<<<<< HEAD
  printBle(DEBUG, "xJammingTask ");
  sprintf(_message, "%d bytes\n", uxHighWaterMark);
  printBle(DEBUG, _message);

  uxHighWaterMark = uxTaskGetStackHighWaterMark(ph);
  printBle(DEBUG, "xProtocolTask ");
  sprintf(_message, "%d bytes\n", uxHighWaterMark);
  printBle(DEBUG, _message);

  uxHighWaterMark = uxTaskGetStackHighWaterMark(ioh);
  printBle(DEBUG, "xIOTask ");
  sprintf(_message, "%d bytes\n", uxHighWaterMark);
  printBle(DEBUG, _message);

  uxHighWaterMark = uxTaskGetStackHighWaterMark(bh);
  printBle(DEBUG, "xBlueTask ");
  sprintf(_message, "%d bytes\n", uxHighWaterMark);
  printBle(DEBUG, _message);

  uxHighWaterMark = uxTaskGetStackHighWaterMark(cch);
  printBle(DEBUG, "xCCTask ");
  sprintf(_message, "%d bytes\n", uxHighWaterMark);
  printBle(DEBUG, _message);

  sprintf(_message, "freeMemory()= %d", freeMemory());

  //  uxHighWaterMark = uxTaskGetStackHighWaterMark(test_handler);
  //  printBle(DEBUG, "xTestTask ");
  //  sprintf(_message, "%d bytes\n", uxHighWaterMark);
  //  printBle(DEBUG, _message);
}
void getDIO() {
  char _message[17];

  if (!digitalRead(IgnInPin))
    sprintf(_message, "Ignition ON\n");
  else
    sprintf(_message, "Igntion OFF\n");
  printBle(DEBUG, _message);
  if (digitalRead(DPosInPin) == DOOROPEN)
    sprintf(_message, "PosDoor Open\n");
  else
    sprintf(_message, "PosDoor Closed\n");
  printBle(DEBUG, _message);
  if (digitalRead(DNegInPin) == DOOROPEN)
    sprintf(_message, "NegDoor Open\n");
  else
    sprintf(_message, "NegDoor Closed\n");
  printBle(DEBUG, _message);
  if (!digitalRead(DisarmInPin))
    sprintf(_message, "Disarmed\n");
  else
    sprintf(_message, "Armed\n");
  printBle(DEBUG, _message);
  if (digitalRead(JamInPin) == LOW)
    sprintf(_message, "Jammed ON\n");
  else
    sprintf(_message, "Jammed OFF \n");
  printBle(DEBUG, _message);
  if (digitalRead(CCDisInPin) == LOW)
    sprintf(_message, "CCDisable OFF\n");
  else
    sprintf(_message, "CCDisable ON \n");
  printBle(DEBUG, _message);
}
void getState() {
  char _message[2 + 5 + 1];
  int8_t savedState = EEPROM.read(stateAddress);
  sprintf(_message, "s%d\nv%s\n", savedState, VERSION);
  printBle(DEBUG, _message);
=======
  Serial1.println("xStack Remain  ");
  Serial1.print("xJammingTask ");
  Serial1.print(uxHighWaterMark * 2);
  Serial1.println(" bytes ");
  uxHighWaterMark = uxTaskGetStackHighWaterMark(ph);
  Serial1.print("xProtocolTask ");
  Serial1.print(uxHighWaterMark * 2);
  Serial1.println(" bytes ");
  uxHighWaterMark = uxTaskGetStackHighWaterMark(ioh);
  Serial1.print("xIOTask ");
  Serial1.print(uxHighWaterMark * 2);
  Serial1.println(" bytes ");
  uxHighWaterMark = uxTaskGetStackHighWaterMark(bh);
  Serial1.print("xBlueTask ");
  Serial1.print(uxHighWaterMark * 2);
  Serial1.println(" bytes ");
  uxHighWaterMark = uxTaskGetStackHighWaterMark(cch);
  Serial1.print("xCCTask ");
  Serial1.print(uxHighWaterMark * 2);
  Serial1.println(" bytes ");
  uxHighWaterMark = uxTaskGetStackHighWaterMark(test_handler);
  Serial1.print("xTestTask ");
  Serial1.print(uxHighWaterMark * 2);
  Serial1.println(" bytes ");
>>>>>>> b4c199183646342b025353b55e717f9dbb9d1057
}
