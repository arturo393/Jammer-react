#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <avr/wdt.h>
#include <EEPROM.h>
#include "timers.h"

const char VERSION[] = "2.9.5";

/* Useful Constants */
#define SECS_PER_MIN  (60UL)
#define SECS_PER_HOUR (3600UL)
#define SECS_PER_DAY  (SECS_PER_HOUR * 24L)

/* Useful Macros for getting elapsed time */
#define numberOfSeconds(_time_) (_time_ % SECS_PER_MIN)
#define numberOfMinutes(_time_) ((_time_ / SECS_PER_MIN) % SECS_PER_MIN)
#define numberOfHours(_time_) (( _time_% SECS_PER_DAY) / SECS_PER_HOUR)
#define elapsedDays(_time_) ( _time_ / SECS_PER_DAY)

// inputs
#define CCDisable              2      // Enable/disable jamming detection HIGH = on & LOW = off
#define IgnitionPin            3  // (with pull down resistor) Engine power HIGH = on & LOW = off
#define JamDetectionPin        4
#define DoorNegativePin        5     // Door sensor  HIGH = open / LOW = close
#define DisarmPin              6
#define LedPin                 9
#define CCPin                  10    // Corta corriente HIGH = shutdown / LOW = normal
#define BuzzerPin              14
#define DoorPositivePin        15
#define EngineStatusPin        8    // Jamming alert ping HIGH =
#define BlueDisablePin         7   // 10 Protocol block - 11 suspended

#define TIME_AFTER_START       15
#define TIME_AFTER_STOP        10
#define TIME_TO_STOP_ENGINE    40
#define TIME_TO_OPEN_DOOR      30
#define DOOR_ENABLE_SECONDS    7
#define NOTIFICATION_TIME      30
#define TIME_AFTER_OPEN_DOOR   60 // secs after the door was opened
#define TIME_JAMMING_SECURE    120
#define TIME_RESET_HW          720 // * 10 seconds
#define CCON           LOW
#define CCOFF          HIGH
#define NORMAL_STATE   1        /* cirtcuit normal state */
#define BLOCKED_STATE  2        /* CC avtivated sate */
#define SUSPEND_STATE  3        /* CC deactivated and all the task suspended */
#define JAMMED_STATE   4
#define PROTOCOL_STATE 5
#define RESET_STATE    6
#define UNKNOWN_STATE 69
#define ID 5

int stateAddress = 20;
int address = 0;
uint32_t c_off = 0;

TaskHandle_t cc_handler;
TaskHandle_t jammer_handler;
TaskHandle_t protocol_handler;
TaskHandle_t blue_handler;

TimerHandle_t xTimerNotification;
TimerHandle_t xTimerMemoryCheck;


static void vProtocolTask(void *pvParameters);
static void vJammingTask(void *pvParameters);
static void vCCTask(void *pvParameters);
static void vBlueTask(void *pvParameters);

void vApplicationMallocFailedHook( void );
void vApplicationStackOverflowHook( void );
void vTimerCallback( TimerHandle_t xTimer );
void vTimerMemoryCheckCallback( TimerHandle_t xTimer);
void restartHW();
bool checkSavedState();

void setup() {

  wdt_enable(WDTO_8S);

  Serial1.begin(9600);

  BaseType_t xReturned;
  Serial1.print("AT+DEFAULT\r\n");
  Serial1.print("AT+RESET\r\n");
  Serial1.print("AT+ROLE0\r\n");
  Serial1.print("AT+NAMEBLACKDEMO3\r\n");

  xReturned = xTaskCreate(vProtocolTask
    ,"Protocol"
    ,configMINIMAL_STACK_SIZE
    ,NULL,tskIDLE_PRIORITY
    ,&protocol_handler);
  if(xReturned == pdTRUE)
   Serial1.println("xProtocoloTask Created!");

   xReturned = xTaskCreate(vJammingTask
     ,"Jamming"
     ,configMINIMAL_STACK_SIZE
     ,NULL
     ,tskIDLE_PRIORITY
     ,&jammer_handler);
   if(xReturned == pdTRUE)
   Serial1.println("xJammingTask Created!");

  xReturned = xTaskCreate(vBlueTask
    ,"Blue"
    ,configMINIMAL_STACK_SIZE
    ,NULL
    ,tskIDLE_PRIORITY
    ,&blue_handler);
  if(xReturned == pdTRUE)
   Serial1.println("xBlueTask Created!");

  xReturned = xTaskCreate(vCCTask
    ,"CC"
    ,configMINIMAL_STACK_SIZE
    ,NULL
    ,tskIDLE_PRIORITY
    ,&cc_handler);
  if(xReturned == pdTRUE)
   Serial1.println("xCCTask Created!");

  /* bluetooth notification callback */
  xTimerNotification = xTimerCreate(
    "Timer",
    configTICK_RATE_HZ*10,
    pdTRUE,
    (void *) 0,
    vTimerCallback
  );
   Serial1.println("xTimerNotification Created!");

//  xTimerMemoryCheck = xTimerCreate(
//    "Memory",
//    configTICK_RATE_HZ*5,
//    pdTRUE,
//    (void *) 0,
//    vTimerMemoryCheckCallback
//  );
//  Serial1.println("xTimerMemoryCheck Created!");

  pinMode(BlueDisablePin, OUTPUT);
  pinMode(CCPin, OUTPUT);
  pinMode(LedPin, OUTPUT);
  pinMode(BuzzerPin, OUTPUT);
  pinMode(DisarmPin, INPUT_PULLUP);
  pinMode(DoorNegativePin, INPUT_PULLUP);
  pinMode(JamDetectionPin, INPUT_PULLUP);
  pinMode(DoorPositivePin, INPUT_PULLUP);
  pinMode(IgnitionPin, INPUT_PULLUP);
  pinMode(CCDisable, INPUT_PULLUP);

Serial1.println("xPinout seated!");

//    EEPROM.update(stateAddress, NORMAL_STATE);
int8_t savedState = EEPROM.read(stateAddress);

if(savedState == JAMMED_STATE)
  xTaskNotify(cc_handler,0x06, eSetValueWithOverwrite );
else
if(savedState == BLOCKED_STATE)
  xTaskNotify(cc_handler,0x03, eSetValueWithOverwrite );
else
if(savedState == SUSPEND_STATE)
  xTaskNotify(cc_handler,0x04, eSetValueWithOverwrite );
else
if(savedState == NORMAL_STATE)
  xTaskNotify(cc_handler,0x01, eSetValueWithOverwrite );

// Start the real time scheduler.
vTaskStartScheduler();
// Will not get here unless there is insufficient RAM.
Serial1.println("xNo Ram avaliable");

}
//------------------------------------------------------------------------------
// WARNING idle loop has a very small stack (configMINIMAL_STACK_SIZE)
// loop must never block
void loop() {
}
//-----------------------------------------------------------------------------
static void vProtocolTask(void *pvParameters)
{

  uint32_t c_ign_on = 0;                            /* engine on counter */
  uint32_t c_ign_off = 0;                           /* engine off counter */
  bool armed = false;
  bool ignition = false;
  bool disarm = false;
  bool opened = false;
  bool doorPositiveState = HIGH;
  bool lastDoorPositiveState = HIGH;
  bool doorNegativeState = LOW;
  bool lastDoorNegativeState = LOW;

  bool disarmState = HIGH;
  bool lastDisarmState = HIGH;
  unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
  unsigned long debounceDelay = pdMS_TO_TICKS(100);    // the debounce time; increase if the output flickers

  // to aviod blocked at the first time
  doorPositiveState = digitalRead(DoorPositivePin);
  lastDoorPositiveState = doorPositiveState;
  doorNegativeState = digitalRead(DoorNegativePin);
  lastDoorNegativeState = doorNegativeState;

  for(;;)
  {
      ignition = !digitalRead(IgnitionPin);
      // check ignition ON
      c_ign_on = 0;
      c_ign_off = 0;
      while(ignition && !armed)
      {
        c_ign_on++;
        vTaskDelay(pdMS_TO_TICKS(1000));
        if(c_ign_on >= TIME_AFTER_START)
        {
          digitalWrite(LedPin,HIGH);
          armed = true;
        }
        Serial1.print("Ign on ");
        Serial1.println(c_ign_on);
        ignition = !digitalRead(IgnitionPin);
      }
      // end

      // check ignition OFF
      while(!ignition && armed)
      {
        c_ign_off++;
        vTaskDelay(pdMS_TO_TICKS(1000));
        if(c_ign_off >= TIME_AFTER_STOP)
        {
          digitalWrite(LedPin,LOW);
          armed = false;
        }
        Serial1.print("xIgn off ");
        Serial1.println(c_ign_off);
        ignition = !digitalRead(IgnitionPin);
      }
      // end
      if(armed)
      {
        // check door status
        doorPositiveState = digitalRead(DoorPositivePin);
        if(doorPositiveState != lastDoorPositiveState)
        {
          vTaskDelay(pdMS_TO_TICKS(500));
          if(doorPositiveState == LOW)
          {
            xTaskNotify(cc_handler,0x05, eSetValueWithOverwrite );
            armed = false;
          }
        }
        lastDoorPositiveState = doorPositiveState;

        doorNegativeState = digitalRead(DoorNegativePin);
        if(doorNegativeState != lastDoorNegativeState)
        {
          vTaskDelay(pdMS_TO_TICKS(500));
          if(doorNegativeState == LOW)
          {
            xTaskNotify(cc_handler,0x05, eSetValueWithOverwrite );
            armed = false;           }
          }
          lastDoorNegativeState = doorNegativeState;
          // end door status


          int reading  = !digitalRead(DisarmPin);

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
              }else{
                disarm = pdTRUE;
              }
            }
          }
          lastDisarmState = reading;



          if(disarm)
          {
            int c_disarm = 0;
            int c_add = 0;
            bool toggle = false;
            Serial1.println("xdisarm");
            do
            {
              disarm = !digitalRead(DisarmPin);
              digitalWrite(LedPin,toggle);
              c_disarm++;
              toggle = !toggle;
              Serial1.print((TIME_TO_OPEN_DOOR+c_add)-c_disarm);
              Serial1.print(" ");
              // if button is pressed , add 20 seconds more to te suspended task
              if(disarm && c_add <= 60  ) c_add += 20;
              vTaskDelay(pdMS_TO_TICKS(1000));
            } while(c_disarm <= (TIME_TO_OPEN_DOOR + c_add) );

            // to avoid blocked after disarm
            digitalWrite(LedPin,HIGH);
            doorPositiveState = digitalRead(DoorPositivePin);
            lastDoorPositiveState = doorPositiveState;
            doorNegativeState = digitalRead(DoorNegativePin);
            lastDoorNegativeState = doorNegativeState;
          }

        }
    } /* end for(;;) */
  }
//-----------------------------------------------------------------------------
static void vJammingTask(void *pvParameters)
{
    uint32_t c_ign_on = 0;                            /* engine on counter */
    uint32_t c_ign_off = 0;                           /* engine off counter */
    bool jammed = false;
    bool open = false;
    uint32_t c_jammed = 0;

    bool doorPositiveState = HIGH;
    bool lastDoorPositiveState = HIGH;
    bool doorNegativeState = LOW;
    bool lastDoorNegativeState = LOW;

    bool jammedState = HIGH;
    bool lastJammedState = HIGH;
    unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
    unsigned long debounceDelay = pdMS_TO_TICKS(500);    // the debounce time; increase if the output flickers

    // to aviod blocked at the first time
    doorPositiveState = digitalRead(DoorPositivePin);
    lastDoorPositiveState = doorPositiveState;
    doorNegativeState = digitalRead(DoorNegativePin);
    lastDoorNegativeState = doorNegativeState;

    for(;;)
    {
        // if jamming is detected after 2 minute from GPS*/
        int reading = !digitalRead(JamDetectionPin);
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
            }
            else
              jammed = pdTRUE;
          }
        }
        lastJammedState = reading;

        if(!jammed) c_jammed = 0;

        if(jammed)
        {
          Serial1.print("xjammed");
          Serial1.println(c_jammed);
          // check Positive door status
          doorPositiveState = digitalRead(DoorPositivePin);
          if(doorPositiveState != lastDoorPositiveState)
          {
            vTaskDelay(pdMS_TO_TICKS(1000));
            if(doorPositiveState == LOW)
            {
              xTaskNotify(cc_handler,0x02, eSetValueWithOverwrite );
              jammed = false;
            }
          }
          lastDoorPositiveState = doorPositiveState;
          // check negative door Status
          doorNegativeState = digitalRead(DoorNegativePin);
          if(doorNegativeState != lastDoorNegativeState)
          {
            vTaskDelay(pdMS_TO_TICKS(500));
            if(doorNegativeState == LOW)
            {
              xTaskNotify(cc_handler,0x02, eSetValueWithOverwrite );
              jammed = false;
            }
          }
          lastDoorNegativeState = doorNegativeState;
          // end door status

          c_jammed++;
          vTaskDelay(pdMS_TO_TICKS(1000));
          if(c_jammed >= TIME_JAMMING_SECURE){
            xTaskNotify(cc_handler,0x02, eSetValueWithOverwrite );
            jammed = false;
          }
        }
    } /* end for (;;)*/
}
//-----------------------------------------------------------------------------
static void vBlueTask(void *pvParameters)
{
    int c_data = 0;
    TickType_t xTimeLow;
    char readingString[10];              /* reading string from bluetooth */
    char reading;
    TickType_t xLastDataReceived;

    xTimerStart(xTimerNotification,0);
  //  xTimerStart(xTimerMemoryCheck,0);
    xLastDataReceived = xTaskGetTickCount();

    for  (;;)
    {
      if(Serial1.available() > 0)
      {
        readingString[c_data] = Serial1.read();
        readingString[c_data+1] = '\0';
        xLastDataReceived = xTaskGetTickCount();
        c_data++;
      }
      if((xTaskGetTickCount() - xLastDataReceived) > 30)
      {
        c_data = 0;
        vTaskDelay(pdMS_TO_TICKS(300));
      }
      if (strcmp("suspender",readingString) == 0)
      {
        xTaskNotify(cc_handler,0x04, eSetValueWithOverwrite );
        Serial1.print("s");
        Serial1.println(SUSPEND_STATE);
        readingString[0] = '\0';
      }
      if(strcmp("normal", readingString) == 0)
      {
        xTaskNotify(cc_handler,0x10, eSetValueWithOverwrite );
        Serial1.print("s");
        Serial1.println(NORMAL_STATE);
        readingString[0] = '\0';
      }
      if (strcmp("bloquear",readingString) == 0)
      {
        xTaskNotify(cc_handler,0x03, eSetValueWithOverwrite );
        Serial1.print("s");
        Serial1.println(BLOCKED_STATE);
        readingString[0] = '\0';
      }
      if (strcmp("reset",readingString) == 0)
      {
        xTaskNotify(cc_handler,0x10, eSetValueWithOverwrite );
        Serial1.println("Reset");
        readingString[0] = '\0';
      }
      if(strcmp("estado", readingString) == 0)
      {
        Serial1.print("s");
        switch (EEPROM.read(stateAddress))
        {
          case NORMAL_STATE:
          Serial1.println(NORMAL_STATE);
          break;
          case SUSPEND_STATE:
          Serial1.println(SUSPEND_STATE);
          break;
          case BLOCKED_STATE:
          Serial1.println(BLOCKED_STATE);
          break;
          case JAMMED_STATE:
          Serial1.println(JAMMED_STATE);
          break;
          default:
          Serial1.println(UNKNOWN_STATE);
          break;
        }
        readingString[0] = '\0';
      }
      if (strcmp("version",readingString) == 0)
      {
        Serial1.print("v");
        Serial1.println(VERSION);
        readingString[0] = '\0';
      }
      if (strcmp("cmd",readingString) == 0)
      {
        Serial1.println("suspender - suspende el corta corriente del AJ");
        Serial1.println("normal - reinicia el sistema");
        Serial1.println("bloquear - activa el corta corriente");
        Serial1.println("version - entrega la version del software");
        Serial1.println("estado - entrega el estado actual del AJ");
        readingString[0] = NULL;
      }
    } /* end for (;;) */
  }
//------------------------------------------------------------------------------
static void vCCTask(void *pvParameters)
{
    bool _last_state     = CCOFF;              /* CCDisable pin last state*/
    bool _reading_state  = CCOFF;              /* CCDisable pin reading state */
    bool _current_state  = CCOFF;              /* CCDisable pin current state */
    bool blocked = false;
    bool normal = false;

    TickType_t lastDebounceTime;
    TickType_t debounceDelay;
    TickType_t xFrequency = pdMS_TO_TICKS(10);
    uint32_t ulNotifiedValue = 0x00;


    _reading_state = digitalRead(CCDisable);
     _last_state   = _reading_state;

    for (;;)
    {
      xTaskNotifyWait( 0x00,      /* Don't clear any notification bits on entry. */
      0xFF, /* Reset the notification value to 0 on exit. */
      &ulNotifiedValue, /* Notified value pass out in ulNotifiedValue. */
      xFrequency  );  /* Block indefinitely. */

      // for pin disable

      _reading_state = digitalRead(CCDisable);
      if(_reading_state != _last_state )
      {
        vTaskDelay(pdMS_TO_TICKS(500));

        if(_reading_state == LOW){
          normal = true;
          Serial1.println("CC OFF");
        }
        // if CCDisable is LOW then CC ON
        if(_reading_state == HIGH){
          blocked = true;
          Serial1.println("CC ON");
        }
      }
      _last_state = _reading_state;


      /* Deactivate CC with CCPin */
      if( ( ulNotifiedValue == 0x01 ))
      {
        digitalWrite(CCPin, CCOFF);
        EEPROM.update(stateAddress, NORMAL_STATE);
      }
      /* Activate CC with digital CCPin in a On/Off secuence */
      if( ( ulNotifiedValue == 0x02 ))
      {
        EEPROM.update(stateAddress, JAMMED_STATE);
        for (int j = 0; j  < 2 ; j++ )
        {
          vTaskSuspend(jammer_handler);
          vTaskSuspend(protocol_handler);
          /* turn on for 2 seconds */
          digitalWrite(CCPin, CCON);
          vTaskDelay(pdMS_TO_TICKS(5000));
          /* turn off for 20 seconds */
          digitalWrite(CCPin, CCOFF);
          vTaskDelay(configTICK_RATE_HZ*20);
        }
        /* finally turn off */
        digitalWrite(CCPin, CCON);
        digitalWrite(LedPin,LOW);
        /* update state of the coda */
        vTaskDelay(pdMS_TO_TICKS(500));
      }

      /* Activate CC with digital CCPin */
      if( ( ulNotifiedValue == 0x03) || blocked)
      {
        vTaskSuspend(jammer_handler);
        vTaskSuspend(protocol_handler);
        digitalWrite(CCPin, CCON);
        digitalWrite(LedPin,LOW);
        EEPROM.update(stateAddress, BLOCKED_STATE);
        blocked = false;
        vTaskDelay(pdMS_TO_TICKS(500));
      }
      /* Deactivate CC with digital CCPin and suspend*/
      if( ( ulNotifiedValue == 0x04) )
      {
        vTaskSuspend(jammer_handler);
        vTaskSuspend(protocol_handler);
        digitalWrite(CCPin, CCOFF);
        digitalWrite(LedPin,LOW);
        EEPROM.update(stateAddress, SUSPEND_STATE);
        vTaskDelay(pdMS_TO_TICKS(500));
      }

      /* Activate CC with digital CCPin */
      if( ( ulNotifiedValue == 0x05) )
      {
        vTaskSuspend(jammer_handler);
        vTaskSuspend(protocol_handler);
        EEPROM.update(stateAddress, BLOCKED_STATE);
        int c_open = 0;
        int c_add = 0;
        bool toggle = false;
        Serial.println("open");
        do
        {
          digitalWrite(LedPin,toggle);
          c_open++;
          toggle = !toggle;
          Serial.print((TIME_TO_OPEN_DOOR*2)-c_open);
          Serial.print(" ");
          vTaskDelay(pdMS_TO_TICKS(1000/2));
        } while(c_open <= (TIME_AFTER_OPEN_DOOR*2));

        digitalWrite(LedPin,LOW);
        digitalWrite(CCPin, CCON);
        vTaskDelay(pdMS_TO_TICKS(500));
      }

      if( ( ulNotifiedValue == 0x06) )
      {
        vTaskSuspend(jammer_handler);
        vTaskSuspend(protocol_handler);
        digitalWrite(CCPin, CCON);
        digitalWrite(LedPin,LOW);
        EEPROM.update(stateAddress, JAMMED_STATE);
        vTaskDelay(pdMS_TO_TICKS(500));
      }

      /* Watchdog System-Reset */
      if( ( ulNotifiedValue == 0x10 ) || normal )
      {
        digitalWrite(CCPin, CCOFF);
        EEPROM.update(stateAddress, NORMAL_STATE);
        do{
          wdt_enable(WDTO_15MS);
          for(;;){ }
        } while(0);
      }
    } /* end for(;;) */
  }
/*-----------------------------------------------------------*/
void vApplicationMallocFailedHook( void ){
    Serial.println("faliedhook Memory");
  }
/*-----------------------------------------------------------*/
void vApplicationStackOverflowHook( TaskHandle_t xTask, signed char *pcTaskName ){
    Serial1.println(*pcTaskName);
  }
//------------------------------------------------------------------------------
bool checkSavedState()
{
    int8_t savedState = EEPROM.read(stateAddress);

    if(savedState == JAMMED_STATE){
      xTaskNotify(cc_handler,0x06, eSetValueWithOverwrite );
      return false;
    }
    if(savedState == BLOCKED_STATE){
      xTaskNotify(cc_handler,0x03, eSetValueWithOverwrite );
      return false;
    }
    if(savedState == SUSPEND_STATE){
      xTaskNotify(cc_handler,0x04, eSetValueWithOverwrite );
      return false;
    }
    EEPROM.update(stateAddress, NORMAL_STATE);
    return true;
  }
void restartHW()
{
    do{
      wdt_enable(WDTO_15MS);
      for(;;){ }
    } while(0);
  }

void vTimerCallback( TimerHandle_t xTimer )
{

  int8_t savedState = EEPROM.read(stateAddress);
  configASSERT( pxTimer );

  Serial1.print("s");
  Serial1.println(savedState);
  if(digitalRead(IgnitionPin) || savedState == BLOCKED_STATE || savedState == JAMMED_STATE ) // if ignition off
    c_off++;
  else
    c_off = 0;

  if(c_off >= TIME_RESET_HW)
    restartHW();

    Serial1.print("Apagado en ");
    Serial1.println(TIME_RESET_HW - c_off);
}

void vTimerMemoryCheckCallback( TimerHandle_t xTimer )
{
  /* Inspect our own high water mark on entering the task. */
  UBaseType_t uxHighWaterMark;
  uxHighWaterMark = uxTaskGetStackHighWaterMark( protocol_handler );
  Serial1.println("xStack Remain  ");
  Serial1.print("xProtocolTask ");
  Serial1.println(uxHighWaterMark);
  uxHighWaterMark = uxTaskGetStackHighWaterMark( jammer_handler );
  Serial1.print("xJammingTask ");
  Serial1.println(uxHighWaterMark);
  uxHighWaterMark = uxTaskGetStackHighWaterMark( cc_handler );
  Serial1.print("xCCTask ");
  Serial1.println(uxHighWaterMark);
  uxHighWaterMark = uxTaskGetStackHighWaterMark( blue_handler );
  Serial1.print("xBlueTask ");
  Serial1.println(uxHighWaterMark);
}
