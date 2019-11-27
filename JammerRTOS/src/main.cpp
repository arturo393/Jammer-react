#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <EEPROM.h>
#include <avr/wdt.h>
#include <timers.h>

const char VERSION[] = "3.0.4";

/* Useful Constants */

#define TIME_AFTER_START 15
#define TIME_AFTER_STOP 10
#define TIME_TO_STOP_ENGINE 40
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

TaskHandle_t cc_handler;
TaskHandle_t jammer_handler;
TaskHandle_t protocol_handler;
TaskHandle_t blue_handler;
TaskHandle_t io_handler;
TaskHandle_t test_handler;

TimerHandle_t xTimerNotification;
TimerHandle_t xTimerRestart;
TimerHandle_t xTimerMemoryCheck;

static void vProtocolTask(void *pvParameters);
static void vJammingTask(void *pvParameters);
static void vCCTask(void *pvParameters);
static void vIOTask(void *pvParameters);
static void vBlueTask(void *pvParameters);
static void vTestTask(void *pvParameters);

void vTimerCallback(TimerHandle_t xTimer);
void vTimerMemoryCheckCallback(TimerHandle_t xTimer);
void vTimerRestart(TimerHandle_t xTimer);

void restartHW();
void updateState(int8_t _state);

void setup() {
  Serial1.begin(9600);
  // Serial.begin(9600);

  wdt_enable(WDTO_2S);

  //      xTaskCreate(vProtocolTask
  //                  ,"Protocol"
  //                  ,configMINIMAL_STACK_SIZE
  //                  ,NULL,tskIDLE_PRIORITY
  //                  ,&protocol_handler);

  //    xTaskCreate(vJammingTask
  //                ,"Jamming"
  //                ,configMINIMAL_STACK_SIZE
  //                ,NULL
  //                ,tskIDLE_PRIORITY
  //                ,&jammer_handler);
  ///    xTaskCreate(vIOTask,"IO",configMINIMAL_STACK_SIZE,NULL,tskIDLE_PRIORITY,&io_handler);
  xTaskCreate(vBlueTask, "Blue", configMINIMAL_STACK_SIZE + 20, NULL,
              tskIDLE_PRIORITY, &blue_handler);
  xTaskCreate(vCCTask, "CC", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY,
              &cc_handler);
  xTaskCreate(vTestTask, "Test", configMINIMAL_STACK_SIZE + 50, NULL,
              tskIDLE_PRIORITY, &test_handler);

  /* restart notification callback */
  xTimerNotification = xTimerCreate("Timer", configTICK_RATE_HZ * 5, pdTRUE,
                                    (void *)0, vTimerCallback);

  xTimerRestart = xTimerCreate("Timer", configTICK_RATE_HZ * 10, pdTRUE,
                               (void *)0, vTimerRestart);
  /*  xTimerMemoryChecknotification callback */
  //  xTimerMemoryCheck = xTimerCreate(
  //    "Timer",
  //    configTICK_RATE_HZ*5,
  //    pdTRUE,
  //    (void *) 0,
  //     vTimerMemoryCheckCallback
  //  );

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

  //    EEPROM.update(stateAddress, NORMAL_STATE);
  int8_t savedState = EEPROM.read(stateAddress);
  if (savedState == NORMAL_STATE)
    xTaskNotify(cc_handler, 0x01, eSetValueWithOverwrite);
  else if (savedState == PROTOCOL_STATE)
    xTaskNotify(cc_handler, 0x05, eSetValueWithOverwrite);
  else if (savedState == SUSPEND_STATE)
    xTaskNotify(cc_handler, 0x03, eSetValueWithOverwrite);
  else if (savedState == JAMMED_STATE)
    xTaskNotify(cc_handler, 0x05, eSetValueWithOverwrite);
  else if (savedState == GPS_STATE)
    xTaskNotify(cc_handler, 0x05, eSetValueWithOverwrite);
  else if (savedState == BLUETOOTH_STATE)
    xTaskNotify(cc_handler, 0x05, eSetValueWithOverwrite);

  // Start the real time scheduler.
  vTaskStartScheduler();
  // Will not get here unless there is insufficient RAM.
  Serial1.println("xNo Ram avaliable");
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
  uint8_t ignition = false;
  uint8_t disarm = false;

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
      c_ign_on++;
      vTaskDelay(pdMS_TO_TICKS(1000));
      if (c_ign_on >= TIME_AFTER_START) {
        digitalWrite(LedOutPin, HIGH);
        armed = true;
      }
      Serial1.print("Ign on ");
      Serial1.println(c_ign_on);
      ignition = !digitalRead(IgnInPin);
    }
    // end

    // check ignition OFF
    while (!ignition && armed) {
      c_ign_off++;
      vTaskDelay(pdMS_TO_TICKS(1000));
      if (c_ign_off >= TIME_AFTER_STOP) {
        digitalWrite(LedOutPin, LOW);
        armed = false;
      }
      Serial1.print("xIgn off ");
      Serial1.println(c_ign_off);
      ignition = !digitalRead(IgnInPin);
    }

    // end
    if (armed) {
      // check door status
      dPosR = digitalRead(DPosInPin);

      if (dPosR != lastDPosS) {
        lastDPosDebounceTime = xTaskGetTickCount();
        Serial1.println("PosDoor Change");
      }

      if ((xTaskGetTickCount() - lastDPosDebounceTime) > debounceDDelay) {
        if (dPosR != dPosS) {
          dPosS = dPosR;
          if (dPosS == DOOROPEN) {
            updateState(PROTOCOL_STATE);
            xTaskNotify(cc_handler, 0x02, eSetValueWithOverwrite);
            armed = false;
          }
        }
      }
      lastDPosS = dPosR;

      // check door status
      dNegR = digitalRead(DNegInPin);

      if (dNegR != lastDNegS) {
        lastDNegDebounceTime = xTaskGetTickCount();
        Serial1.println("NegDoor Change");
      }
      if ((xTaskGetTickCount() - lastDNegDebounceTime) > debounceDDelay) {
        if (dNegR != dNegS) {
          dNegS = dNegR;
          if (dNegS == DOOROPEN) {
            updateState(PROTOCOL_STATE);
            xTaskNotify(cc_handler, 0x02, eSetValueWithOverwrite);
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
        int8_t c_disarm = 0;
        int8_t toggle = false;
        int8_t c_add = 0;
        Serial1.println("xdisarm");
        do {
          disarm = !digitalRead(DisarmInPin);
          digitalWrite(LedOutPin, toggle);
          c_disarm++;
          toggle = !toggle;
          Serial1.print((TIME_TO_OPEN_DOOR + c_add) - c_disarm);
          Serial1.print(" ");
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
      Serial1.print("xjammed");
      Serial1.println(c_jammed);

      // check door status
      dPosR = digitalRead(DPosInPin);

      if (dPosR != lastDPosS)
        lastDPosDebounceTime = xTaskGetTickCount();

      if ((xTaskGetTickCount() - lastDPosDebounceTime) > debounceDDelay) {
        if (dPosR != dPosS) {
          dPosS = dPosR;
          if (dPosS == DOOROPEN) {
            xTaskNotify(cc_handler, 0x04, eSetValueWithOverwrite);
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
            xTaskNotify(cc_handler, 0x04, eSetValueWithOverwrite);
            jammed = false;
          }
        }
      }
      lastDNegS = dNegR;

      c_jammed++;
      vTaskDelay(pdMS_TO_TICKS(1000));
      if (c_jammed >= TIME_JAMMING_SECURE) {
        updateState(JAMMED_STATE);
        xTaskNotify(cc_handler, 0x04, eSetValueWithOverwrite);
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
  uint8_t config = pdTRUE;

  readingString[0] = '\0';
  xLastDataReceived = xTaskGetTickCount();

  for (;;) {
    if (config) {
      Serial1.print("AT+DEFAULT\r\n");
      vTaskDelay(configTICK_RATE_HZ);
      Serial1.print("AT+RESET\r\n");
      vTaskDelay(configTICK_RATE_HZ);
      Serial1.print("AT+NAMEBLACKGPS\r\n");
      vTaskDelay(configTICK_RATE_HZ);
      Serial1.print("AT+NOTI1\r\n");
      vTaskDelay(configTICK_RATE_HZ);
      Serial1.print("AT+RESET\r\n");
      config = false;
    }

    if (Serial1.available() > 0) {
      readingString[c_data] = Serial1.read();
      readingString[c_data + 1] = '\0';
      Serial.println(readingString);
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
      xTaskNotify(cc_handler, 0x03, eSetValueWithOverwrite);
      readingString[0] = '\0';
    }
    if (strcmp("normal", readingString) == 0) {
      updateState(NORMAL_STATE);
      xTaskNotify(cc_handler, 0x10, eSetValueWithOverwrite);
      readingString[0] = '\0';
    }
    if (strcmp("bloquear", readingString) == 0) {
      updateState(BLUETOOTH_STATE);
      xTaskNotify(cc_handler, 0x05, eSetValueWithOverwrite);
      readingString[0] = '\0';
    }
    if (strcmp("reset", readingString) == 0) {
      xTaskNotify(cc_handler, 0x10, eSetValueWithOverwrite);
      Serial1.println("Reset");
      readingString[0] = '\0';
      vTaskDelay(pdMS_TO_TICKS(500));
      do {
        wdt_enable(WDTO_15MS);
        for (;;) {
        }
      } while (0);
    }
    if (strcmp("estado", readingString) == 0) {
      int8_t savedState = EEPROM.read(stateAddress);
      Serial1.print("s");
      Serial1.println(savedState);
      readingString[0] = '\0';
    }
    if (strcmp("version", readingString) == 0) {
      Serial1.print("v");
      Serial1.println(VERSION);
      readingString[0] = '\0';
    }
    if (strcmp("test", readingString) == 0) {
      Serial1.println("testing");

      xTaskNotifyGive(test_handler);

      readingString[0] = '\0';
    }
    if (strcmp("OK+CONN", readingString) == 0) {
      xTimerStart(xTimerNotification, 0);
      xTimerStop(xTimerRestart, 0);
      vTaskDelay(configTICK_RATE_HZ * 1);
      int8_t savedState = EEPROM.read(stateAddress);
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
      xTimerStop(xTimerNotification, 0);
      xTimerStart(xTimerRestart, 0);
      // restart the bluetooth
      digitalWrite(BlEnOutPin, LOW);
      vTaskDelay(configTICK_RATE_HZ);
      digitalWrite(BlEnOutPin, HIGH);
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

    /* Deactivate CC with CCOutPin */
    if ((ulNotifiedValue == 0x01)) {
      digitalWrite(CCOutPin, CCOFF);
    }

    /* Activate CC after 1 minute */
    if ((ulNotifiedValue == 0x02)) {
      vTaskSuspend(jammer_handler);
      vTaskSuspend(protocol_handler);
      vTaskSuspend(io_handler);
      int8_t c_open = 0;
      int8_t toggle = false;
      Serial1.println("Door Open");
      do {
        digitalWrite(LedOutPin, toggle);
        c_open++;
        toggle = !toggle;
        Serial1.print((TIME_TO_OPEN_DOOR * 2) - c_open);
        Serial1.print(" ");
        vTaskDelay(pdMS_TO_TICKS(1000 / 2));
      } while (c_open <= (TIME_AFTER_OPEN_DOOR * 2));

      digitalWrite(LedOutPin, LOW);
      digitalWrite(CCOutPin, CCON);
      vTaskDelay(pdMS_TO_TICKS(500));
      vTaskResume(io_handler);
    }

    /* Deactivate CC and suspend tasks*/
    if ((ulNotifiedValue == 0x03)) {
      vTaskSuspend(jammer_handler);
      vTaskSuspend(protocol_handler);
      digitalWrite(CCOutPin, CCOFF);
      digitalWrite(LedOutPin, LOW);
      vTaskDelay(pdMS_TO_TICKS(500));
    }

    /* Activate CC with digital CCOutPin in a On/Off secuence */
    if ((ulNotifiedValue == 0x04)) {
      for (int j = 0; j < 2; j++) {
        vTaskSuspend(jammer_handler);
        vTaskSuspend(protocol_handler);
        vTaskSuspend(io_handler);
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
      /* update state of the coda */
      vTaskDelay(pdMS_TO_TICKS(500));
      vTaskResume(io_handler);
    }

    /* Activate CC with and suspend tasks */
    if ((ulNotifiedValue == 0x05)) {
      vTaskSuspend(jammer_handler);
      vTaskSuspend(protocol_handler);
      digitalWrite(CCOutPin, CCON);
      digitalWrite(LedOutPin, LOW);
      vTaskDelay(pdMS_TO_TICKS(500));
    }
    /* Watchdog System-Reset */
    if ((ulNotifiedValue == 0x10)) {
      Serial1.println("System Restart!");
      digitalWrite(CCOutPin, CCOFF);
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
          Serial1.println("CC OFF");
          //    xTaskNotify(cc_handler,0x10, eSetValueWithOverwrite );
        }
        // if CCDisInPin is LOW then CC ON
        if (_current_state == HIGH && !blocked) {
          updateState(GPS_STATE);
          Serial1.println("CC ON");
          xTaskNotify(cc_handler, 0x05, eSetValueWithOverwrite);
        }
      }
    }

    _last_state = _reading_state;
  }
}

static void vTestTask(void *pvParameters) {

  int inputs = 6;
  uint8_t outputs = 2;
  uint8_t open[inputs];
  uint8_t close[inputs]; // door positive closed
  uint8_t ready = 0;
  uint8_t reading[inputs]; // door positive reading
  uint8_t input[inputs];
  uint8_t output[outputs];

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

  for (int j = 0; j < inputs; j++) {
    open[j] = false;
    close[j] = false;
  }

  for (int j = 0; j < outputs; j++) {
    digitalWrite(output[j], LOW);
  }

  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    xTimerStop(xTimerNotification, 0);
    vTaskSuspend(cc_handler);
    //    vTaskSuspend(jammer_handler);
    //    vTaskSuspend(protocol_handler);
    //    vTaskSuspend(io_handler);

    lastDebounceTime = xTaskGetTickCount();

    for (int o = 0; o < outputs; o++) {
      Serial1.print("Try: ");
      Serial1.println(o);
      digitalWrite(output[o], HIGH);
      while (ready != inputs) {
        reading[i] = digitalRead(input[i]);
        if (reading[i] == HIGH)
          open[i] = pdTRUE;
        if (reading[i] == LOW) // door positive opened
          close[i] = pdTRUE;
        if (open[i] && close[i]) {
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
          Serial1.print("Input  ");
          Serial1.print(i + 1);
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
          ready = inputs;
        if (i == inputs)
          i = 0;
      } // end while
      digitalWrite(output[o], LOW);
      for (int j = 0; j < inputs; j++) {
        open[j] = false;
        close[j] = false;
      }
      ready = 0;
    } // end for

    vTaskResume(cc_handler);
    xTimerStart(xTimerNotification, 0);

    for (int j = 0; j < outputs; j++) {
      digitalWrite(output[j], LOW);
    }
    //    vTaskResume(jammer_handler);
    //    vTaskResume(protocol_handler);
    //    vTaskResume(io_handler);
    Serial1.println("Test ended");
  }
}

void restartHW() {
  do {
    wdt_enable(WDTO_15MS);
    for (;;) {
    }
  } while (0);
}

void vTimerCallback(TimerHandle_t xTimer) {
  int8_t savedState = EEPROM.read(stateAddress);
  Serial1.print("s");
  Serial1.println(savedState);
  Serial1.print("v");
  Serial1.println(VERSION);
}

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

  if (c_off >= TIME_RESET_HW)
    restartHW();
}

void updateState(int8_t _state) {
  EEPROM.update(stateAddress, _state);
  Serial1.print("s");
  Serial1.println(_state);
}
void vTimerMemoryCheckCallback(TimerHandle_t xTimer) {
  /* Inspect our own high water mark on entering the task. */
  UBaseType_t uxHighWaterMark;
  uxHighWaterMark = uxTaskGetStackHighWaterMark(protocol_handler);
  Serial.println("xStack Remain  ");
  Serial.print("xProtocolTask ");
  Serial.println(uxHighWaterMark);
  uxHighWaterMark = uxTaskGetStackHighWaterMark(jammer_handler);
  Serial.print("xJammingTask ");
  Serial.println(uxHighWaterMark);
  uxHighWaterMark = uxTaskGetStackHighWaterMark(cc_handler);
  Serial.print("xCCTask ");
  Serial.println(uxHighWaterMark);
  uxHighWaterMark = uxTaskGetStackHighWaterMark(blue_handler);
  Serial.print("xBlueTask ");
  Serial.println(uxHighWaterMark);
}
