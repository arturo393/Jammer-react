#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <avr/wdt.h>
#include <EEPROM.h>
#include "timers.h"

const char VERSION[] = "2.9";

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
#define DoorNegativePin                5     // Door sensor  HIGH = open / LOW = close
#define DisarmPin          6
//#define JamDetectionAuxPin     7
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
#define TIME_JAMMING_HANG      30*SECS_PER_MIN
//#define TIME_JAMMING_HANG      3*SECS_PER_MIN

#define TIME_JAMMING_AFTER_SECURE 120

#define NORMAL_STATE   1        /* cirtcuit normal state */
#define BLOCKED_STATE  2        /* CC avtivated sate */
#define SUSPEND_STATE  3        /* CC deactivated and all the task suspended */
#define JAMMED_STATE   4
#define PROTOCOL_STATE 5
#define PHONE_STATE    6
#define UNKNOWN_STATE 69

#define ID 5
int stateAddress = 20;

/** the current address in the EEPROM (i.e. which byte we're going to write to next) **/
int address = 0;

TaskHandle_t cc_handler;
TaskHandle_t jammer_handler;
TaskHandle_t disable_handler;
TaskHandle_t protocol_handler;
TaskHandle_t blue_handler;
TimerHandle_t xTimerNotification;
TimerHandle_t xTimerBlueReset;

void vTimerCallback( TimerHandle_t xTimer );
void vBlueResetCallBack( TimerHandle_t xTimer );

void restartHW();
bool checkSavedState();

static void vProtocolTask(void *pvParameters);
static void vJammingTask(void *pvParameters);
static void vCCTask(void *pvParameters);
static void vBlueTask(void *pvParameters);

void vApplicationMallocFailedHook( void );
void vApplicationStackOverflowHook( void );

void setup() {

  Serial.begin(9600);
  Serial1.begin(9600);

  BaseType_t xReturned;

  Serial1.print("AT+DEFAULT\r\n");
  Serial1.print("AT+RESET\r\n");
  Serial1.print("AT+ROLE0\r\n");
  Serial1.print("AT+NAMEBLACKDEMO3\r\n");


//  xReturned = xTaskCreate(vBlueTask,"Blue",configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY , &blue_handler);
//  if(xReturned == pdTRUE){
//    Serial1.println("BlueTask Created!");
//  }
//  xReturned = xTaskCreate(vCCTask,"CC",configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY,&cc_handler);
//  if(xReturned == pdTRUE){
//      Serial1.println("CCTask Created!");
//    }
  xReturned = xTaskCreate(vProtocolTask,"Protocol",configMINIMAL_STACK_SIZE,NULL,tskIDLE_PRIORITY,&protocol_handler);
  if(xReturned == pdTRUE){
      Serial1.println("ProtocoloTask Created!");
    }
//  xReturned = xTaskCreate(vJammingTask,"Jamming",configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY , &jammer_handler);
//  if(xReturned == pdTRUE){
//      Serial1.println("JammingTask Created!");
//  }

  /* bluetooth notification callback */
  xTimerNotification = xTimerCreate(
    "Timer",
    configTICK_RATE_HZ*10,
    pdTRUE,
    (void *) 0,
    vTimerCallback
  );

  /* bluetooth notification callback */
  xTimerBlueReset = xTimerCreate(
    "Reset",
    configTICK_RATE_HZ*20,
    pdTRUE,
    (void *) 0,
    vBlueResetCallBack
  );

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

}
//------------------------------------------------------------------------------
// WARNING idle loop has a very small stack (configMINIMAL_STACK_SIZE)
// loop must never block
void loop() {
}

//-----------------------------------------------------------------------------
/* This task activate send notitfication to activate CC ON using
* door notification button satate detection */
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


  // to aviod blocked at the first time
  doorPositiveState = digitalRead(DoorPositivePin);
  lastDoorPositiveState = doorPositiveState;
  doorNegativeState = digitalRead(DoorNegativePin);
  lastDoorNegativeState = doorNegativeState;


  /* Inspect our own high water mark on entering the task. */
  UBaseType_t uxHighWaterMark;
  uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
  Serial1.print("Stack Remain  ");
  Serial1.println(uxHighWaterMark);

  for(;;)
  {
    vTaskDelay(pdMS_TO_TICKS(500));
    if(checkSavedState())
    {
      Serial.println("Protocol");
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
        Serial.print("Ign on ");
        Serial.println(c_ign_on);
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
        Serial.print("Ign off ");
        Serial.println(c_ign_off);
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


        disarm = !digitalRead(DisarmPin);
        if(disarm)
        {
          int c_disarm = 0;
          int c_add = 0;
          bool toggle = false;
          Serial.println("disarm");
          do
          {
            disarm = !digitalRead(DisarmPin);
            digitalWrite(LedPin,toggle);
            c_disarm++;
            toggle = !toggle;
            Serial.print((TIME_TO_OPEN_DOOR+c_add)-c_disarm);
            Serial.print(" ");
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
    }
    else
    {
      vTaskDelay(pdMS_TO_TICKS(5000));
      Serial.println("Protocol Suspended");
      digitalWrite(LedPin,LOW);
      vTaskSuspend(NULL);
      c_ign_on = 0;
      c_ign_off = 0;
      armed = false;
      ignition = false;
      disarm = false;
      opened = false;
    }
  } /* end for(;;) */
}
//-----------------------------------------------------------------------------
/* This task check the igntion status and  send notitfication to
* activate/deactivate  vProtocolTask */
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
  // to aviod blocked at the first time
  doorPositiveState = digitalRead(DoorPositivePin);
  lastDoorPositiveState = doorPositiveState;
  doorNegativeState = digitalRead(DoorNegativePin);
  lastDoorNegativeState = doorNegativeState;

  for(;;)
  {
    if (checkSavedState())
    {
      // if jamming is detected after 2 minute from GPS*/
      jammed = !digitalRead(JamDetectionPin);
      if(!jammed) c_jammed = 0;

      if(jammed)
      {
        Serial.println("jammed");
        Serial.println(c_jammed);

        // check door status
         doorPositiveState = digitalRead(DoorPositivePin);
         if(doorPositiveState != lastDoorPositiveState)
         {
           vTaskDelay(pdMS_TO_TICKS(500));
           if(doorPositiveState == LOW)
           {
             xTaskNotify(cc_handler,0x02, eSetValueWithOverwrite );
             jammed = false;
          }
         }
       lastDoorPositiveState = doorPositiveState;

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
    }
    else
    {
      Serial.println("Jammed Suspended");
      vTaskSuspend(NULL);
       c_ign_on = 0;                            /* engine on counter */
       c_ign_off = 0;                           /* engine off counter */
       jammed = false;
       open = false;
       c_jammed = 0;
    }
  } /* end for (;;)*/
}
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
/* This task activate send notitfication to activate CC ON using
* door notification and jamming detection */
static void vBlueTask(void *pvParameters)
{
  int c_data = 0;
  TickType_t xTimeLow;
  char readingString[10];              /* reading string from bluetooth */
  char reading;
  TickType_t xLastDataReceived;

  xTimerStart(xTimerNotification,0);
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
      xTaskNotify(cc_handler,0x01, eSetValueWithOverwrite );
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
  /* This task recieve notifications from other task to activate / deactivate the CC
  *  and restart the MCU by hardware */
static void vCCTask(void *pvParameters)
{
  bool _last_state     = HIGH;              /* CCDisable pin last state*/
  bool _reading_state  = HIGH;              /* CCDisable pin reading state */
  bool _current_state  = HIGH;              /* CCDisable pin current state */
  bool blocked = false;
  bool normal = false;
  TickType_t lastDebounceTime;
  TickType_t debounceDelay;
  TickType_t xFrequency = pdMS_TO_TICKS(10);
  uint32_t ulNotifiedValue = 0x00;
  for (;;)
  {
    // for pin disable
    _reading_state = digitalRead(CCDisable);
    // if CCDisable any change update the debounce time */
    if(_reading_state != _last_state )
    {
        Serial1.println("change CCDisable");
        vTaskDelay(pdMS_TO_TICKS(500));

       if(_reading_state == HIGH){
        normal = true;
        Serial1.println("CCDisable OFF");
      }
        // if CCDisable is LOW then CC ON */
       if(_reading_state == LOW){
          blocked = true;
          Serial1.println("CCDisable ON");
        }
     }
      /* end if any change */

      _last_state = _reading_state;

      xTaskNotifyWait( 0x00,      /* Don't clear any notification bits on entry. */
      0xFF, /* Reset the notification value to 0 on exit. */
      &ulNotifiedValue, /* Notified value pass out in ulNotifiedValue. */
      xFrequency  );  /* Block indefinitely. */

      /* Deactivate CC with CCPin */
      if( ( ulNotifiedValue == 0x01 ) || normal )
      {

        digitalWrite(CCPin, LOW);
        EEPROM.update(stateAddress, NORMAL_STATE);
        vTaskResume(jammer_handler);
        vTaskResume(protocol_handler);
        normal = false;
        vTaskDelay(pdMS_TO_TICKS(500));

      }
      /* Activate CC with digital CCPin in a On/Off secuence */
      if( ( ulNotifiedValue == 0x02 ))
      {
        EEPROM.update(stateAddress, JAMMED_STATE);
        for (int j = 0; j  < 2 ; j++ )
        {
          /* turn on for 2 seconds */
          digitalWrite(CCPin, HIGH);
          vTaskDelay(pdMS_TO_TICKS(2000));
          /* turn off for 20 seconds */
          digitalWrite(CCPin, LOW);
          vTaskDelay(configTICK_RATE_HZ*20);
        }
        /* finally turn off */
        digitalWrite(CCPin, HIGH);
        /* update state of the coda */
        vTaskDelay(pdMS_TO_TICKS(500));
      }

      /* Activate CC with digital CCPin */
      if( ( ulNotifiedValue == 0x03) || blocked)
      {
        digitalWrite(CCPin, HIGH);
        EEPROM.update(stateAddress, BLOCKED_STATE);
        blocked = false;
        vTaskDelay(pdMS_TO_TICKS(500));
      }
      /* Deactivate CC with digital CCPin and suspend*/
      if( ( ulNotifiedValue == 0x04) )
      {
        digitalWrite(CCPin, LOW);
        EEPROM.update(stateAddress, SUSPEND_STATE);
        vTaskDelay(pdMS_TO_TICKS(500));
      }

      /* Activate CC with digital CCPin */
      if( ( ulNotifiedValue == 0x05) )
      {
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
        digitalWrite(CCPin, HIGH);
        vTaskDelay(pdMS_TO_TICKS(500));
      }
      /* Activate CC with digital CCPin */
      if( ( ulNotifiedValue == 0x06) )
      {
        digitalWrite(CCPin, HIGH);
        EEPROM.update(stateAddress, JAMMED_STATE);
        vTaskDelay(pdMS_TO_TICKS(500));
      }
      /* Watchdog System-Reset */
      if( ( ulNotifiedValue == 0x10 ) )
      {
        digitalWrite(CCPin, LOW);
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
    Serial.println(*pcTaskName);
  }
  //------------------------------------------------------------------------------
  /* Read and string from bluetooth , store it in _readings and return the number
  * of characters readed */

  //-----------------------------------------------------------------------------
bool checkSavedState()
{
  int8_t savedState = EEPROM.read(stateAddress);

  if(savedState == JAMMED_STATE){
    Serial.println("jammed");
   xTaskNotify(cc_handler,0x06, eSetValueWithOverwrite );
    return false;
    }
  if(savedState == BLOCKED_STATE){
    Serial.println("bloqued");
      xTaskNotify(cc_handler,0x03, eSetValueWithOverwrite );
      return false;
    }
    if(savedState == SUSPEND_STATE){
      Serial.println("suspended");
      xTaskNotify(cc_handler,0x04, eSetValueWithOverwrite );
      return false;
    }
      Serial.println("normal");
      EEPROM.update(stateAddress, NORMAL_STATE);
      return true;
}

  /* Restart by hardware */
  void restartHW()
  {
    do{
      wdt_enable(WDTO_15MS);
      for(;;){ }
    } while(0);
  }

  void vTimerCallback( TimerHandle_t xTimer )
  {
    configASSERT( pxTimer );
    Serial1.print("s");
    Serial1.println(EEPROM.read(stateAddress));
  }

  void vBlueResetCallBack( TimerHandle_t xTimer )
  {
    Serial.println("Blue Disable");
    digitalWrite(BlueDisablePin,LOW);
    vTaskDelay(configTICK_RATE_HZ*5);
    digitalWrite(BlueDisablePin,HIGH);
    Serial.println("Blue Enable");

  }
