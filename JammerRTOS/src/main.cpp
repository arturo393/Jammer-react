#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <avr/wdt.h>
#include <EEPROM.h>
#include "timers.h"

const char VERSION[] = "2.2";

//#define DEBUG 1
//#define PRINT_DOORSTATUS
//#define PRINT_PROTOCOL_STATUS
//#define PRINT_IGNITION_STATUS
//#define PRINT_JAMMING_STATUS
//#define PRINT_PASSWORD_STATUS
//#define PRINT_CC_STATUS
//#define PRINT_DISABLE_STATUS



//#define PROTOCOLFREE_BYTES
//#define DISABLE_FREE_BYTES
//#define IGNITION_FREE_BYTES
//#define DOOR_FREE_BYTES
//#define PASSWORD_FREE_BYTES2
//#define CC_FREE_BYTES
//#define JAMMING_FREE_BYTES

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
#define DoorPin                5     // Door sensor  HIGH = open / LOW = close
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
#define TIME_JAMMING_SECURE    60
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

#define PASSLENGTH 6
#define DEVICE_ID              001

/** the current address in the EEPROM (i.e. which byte we're going to write to next) **/
int address = 0;

TaskHandle_t cc_handler;
TaskHandle_t jammer_handler;
TaskHandle_t disable_handler;
TaskHandle_t protocol_handler;
TaskHandle_t ignition_handler;
TaskHandle_t door_handler;
TaskHandle_t blue_handler;
TimerHandle_t xTimerNotification;
TimerHandle_t xTimerBlueReset;

void time(long val);
void printDigits(byte digits);

void vTimerCallback( TimerHandle_t xTimer );
void vBlueResetCallBack( TimerHandle_t xTimer );

void keyGenerator(char * _temp);
int8_t readSerialBluetooth(char *_reading);
int8_t bluetoothInit( );
int8_t sendATCmd(char _cmd[],char *_response);

void suspendOperationalTasks();
void restartHW();
void suspendTasks();

bool checkDoorStatus(int8_t _pin, bool _open);
bool checkSavedState();

static void vProtocolTask(void *pvParameters);
static void vDisableTask(void *pvParameters);
static void vIgnitionNotification(void *pvParameters);
static void vJammingTask(void *pvParameters);
static void vCCTask(void *pvParameters);
static void vDoorStatusCheckTask(void *pvParameters);
static void vBlueTask(void *pvParameters);

void vApplicationMallocFailedHook( void );
void vApplicationStackOverflowHook( void );

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
  //wdt_enable(WDTO_8S);
  xTaskCreate(vBlueTask,"Blue",configMINIMAL_STACK_SIZE+40+20, NULL, tskIDLE_PRIORITY , &blue_handler);
  //xTaskCreate(vDisableTask,"Disable",configMINIMAL_STACK_SIZE+50+20,NULL,tskIDLE_PRIORITY,&disable_handler);
//  xTaskCreate(vCCTask,"CC",configMINIMAL_STACK_SIZE+10, NULL, tskIDLE_PRIORITY,&cc_handler);
  //xTaskCreate(vIgnitionNotification,"Ignition",configMINIMAL_STACK_SIZE+31,NULL,tskIDLE_PRIORITY,&ignition_handler);
//  xTaskCreate(vProtocolTask,"Protocol",configMINIMAL_STACK_SIZE,NULL,tskIDLE_PRIORITY,&protocol_handler);
  // xTaskCreate(vJammingTask,"Jamming",configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY , &jammer_handler);

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
  pinMode(DoorPin, INPUT_PULLUP);
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
  bool open = false;

  for(;;)
  {
    if(checkSavedState())
    {
      Serial.println("Protocol Normal");

      ignition = !digitalRead(IgnitionPin);
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

      if(armed)
      {
        disarm = !digitalRead(DisarmPin);
        open = checkDoorStatus(DoorPositivePin,LOW);

        if(open){
          xTaskNotify(cc_handler,0x05, eSetValueWithOverwrite );
          armed = false;
        }

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
        }
      }
    }
    else
    {
      vTaskDelay(pdMS_TO_TICKS(5000));
      Serial.println("Jammer Suspended");
      vTaskSuspend(NULL);
      c_ign_on = 0;
      c_ign_off = 0;
      armed = false;
      ignition = false;
      disarm = false;
      open = false;
    }
  } /* end for(;;) */
}
//-----------------------------------------------------------------------------
/* This task check the igntion status and  send notitfication to
* activate/deactivate  vProtocolTask */
static void vJammingTask(void *pvParameters) {
  uint32_t c_ign_on = 0;                            /* engine on counter */
  uint32_t c_ign_off = 0;                           /* engine off counter */
  bool jammed = false;
  bool open = false;
  uint32_t c_jammed = 0;

  for(;;)
  {

    if (checkSavedState())
    {
      Serial.println("Protocol Normal");
      /* if jamming is detected after 1 minute from GPS*/
      jammed = !digitalRead(JamDetectionPin);
      if(!jammed) c_jammed = 0;

      if(jammed)
      {
        Serial.println("jammed");
        Serial.println(c_jammed);
        open = checkDoorStatus(DoorPositivePin,LOW);

        if(open){
          xTaskNotify(cc_handler,0x02, eSetValueWithOverwrite );
          jammed = false;
        }
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
      Serial.println("Protocol Suspended");
      vTaskSuspend(NULL);
       c_ign_on = 0;                            /* engine on counter */
       c_ign_off = 0;                           /* engine off counter */
       jammed = false;
       open = false;
       c_jammed = 0;
    }
  } /* end for (;;)*/
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

//-----------------------------------------------------------------------------
/* This task activate send notitfication to activate CC ON using
* door notification and jamming detection */
static void vDisableTask2(void *pvParameters)
{
  bool _last_state     = HIGH;              /* CCDisable pin last state*/
  bool _reading_state  = HIGH;              /* CCDisable pin reading state */
  bool _current_state  = HIGH;              /* CCDisable pin current state */
  bool suspendState    = false;             /* button suspended state */
  bool checkState      = false;
  int i = 0;
  TickType_t xTimeLow;
  int16_t lastDebounceTime = 0;                 /* the last time the DisarmPin pin was toggled */
  int16_t debounceDelay = pdMS_TO_TICKS(500);   /* the debounce time */
  int16_t ecount = 0;                           /*counter for DisarmPin on*/
  int16_t elimit  = 0;                          /* limit for vCCTask notfiication*/

  bool suspend         = false;          /* bluetooth suspended state */
  char readingString[10];              /* reading string from bluetooth */
  char _id = 20;                      /* local store for ID device */


  #ifdef PASSWORD_FREE_BYTES
  UBaseType_t uxHighWaterMark;
  #endif

  bluetoothInit();
  int8_t savedState = EEPROM.read(stateAddress);

  xTimerStart(xTimerNotification,0);
  Serial1.println("End blueconfig");
  for  (;;) {



    #ifdef DISABLE_FREE_BYTES
    Serial.print("DisableFreeWords ");
    uxHighWaterMark = uxTaskGetStackHighWaterMark(disable_handler);
    Serial.println(uxHighWaterMark);
    vTaskDelay(pdMS_TO_TICKS(5000));
    #endif

    ecount = 0;
    elimit = random(45,120);

    /* repeat while button is pressed */
    while(digitalRead(DisarmPin) == LOW)
    {
      #ifdef PRINT_DISABLE_STATUS
      if((elimit-ecount) % 20  == 0)
      {
        Serial.print(elimit-ecount);
        Serial.print(" ");
      }
      #endif
      /* when reach the limit */
      if(ecount == elimit)
      {
        /* if already suspended */
        if (EEPROM.read(stateAddress) == SUSPEND_STATE)
        {
          vTaskResume(cc_handler);
          Serial1.println("\nNormal");
          xTaskNotify(cc_handler,0x10, eSetValueWithOverwrite );
          ecount = 0;
        }
        /* if not suspended */
        else
        {
          xTaskNotify(cc_handler,0x04, eSetValueWithOverwrite );
          Serial1.println("\nSuspend");
          ecount = 0;
        }
      }

      /* count every 1 second */
      vTaskDelay(pdMS_TO_TICKS(100));
      ecount++;
    } /* end while DisarmPin */


    /* CCDisable pin readings every 10 ms */
    vTaskDelay(pdMS_TO_TICKS(10));
    _reading_state = digitalRead(CCDisable);

    /* if CCDisable any change update the debounce time */
    if(_reading_state != _last_state )
    lastDebounceTime = xTaskGetTickCount();

    /* if there is no change after debounceDelay */
    if (( xTaskGetTickCount() - lastDebounceTime) > debounceDelay){
      /* if the current reading is different from lastnotification value */
      if (_reading_state != _current_state )
      {
        _current_state = _reading_state;

        #ifdef PRINT_DISABLE_STATUS
        if( _reading_state == HIGH )
        Serial.println("CCDisable HIGH");
        else
        Serial.println("CCDisable LOW");
        #endif

        /* if CCDisable is HIGH then CC OFF */
        if(_current_state == HIGH)
        {
          /* send CC OFF notification */
          xTaskNotify(cc_handler,0x10, eSetValueWithOverwrite );
          Serial1.println("\nBlocked");
          #ifdef PRINT_DISABLE_STATUS
          Serial.println("CC OFF by SAFECAR");
          #endif
        }
        /* if CCDisable is LOW then CC ON */
        if(_current_state == LOW)
        {
          /* send CC ON notification */
          xTaskNotify(cc_handler,0x03, eSetValueWithOverwrite );
          digitalWrite(BlueDisablePin, HIGH);
          #ifdef PRINT_DISABLE_STATUS
          Serial.println("CC ON by SAFECAR");
          #endif
        }
      } /* end if any change */
    } /* end if change after debounceDelay */
    _last_state = _reading_state;



    if (readSerialBluetooth(readingString) > 0)
    {
      if (strcmp("suspender",readingString) == 0)
      {
        xTaskNotify(cc_handler,0x04, eSetValueWithOverwrite );
        Serial1.print("s");
        Serial1.println(SUSPEND_STATE);
      }
      else
      if(strcmp("normal", readingString) == 0)
      {
        for(int i=0 ; i<3 ; i++)
        {
          digitalWrite(BuzzerPin, HIGH);
          vTaskDelay(pdMS_TO_TICKS(50));
          digitalWrite(BuzzerPin, LOW);
          vTaskDelay(pdMS_TO_TICKS(100));
        }
        EEPROM.update(stateAddress, NORMAL_STATE);
        Serial1.print("s");
        Serial1.println(NORMAL_STATE);
        restartHW();
      }
      else
      if(strcmp("estado", readingString) == 0)
      {
        switch (EEPROM.read(stateAddress))
        {
          Serial1.print("s");
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
      }
      else
      if (strcmp("bloquear",readingString) == 0)
      {
        digitalWrite(LedPin,LOW);
        EEPROM.update(stateAddress, BLOCKED_STATE);

        /* send CC on notification */
        xTaskNotify(cc_handler,0x03, eSetValueWithOverwrite );
        digitalWrite(BlueDisablePin, HIGH);
        Serial1.print("s");
        Serial1.println(BLOCKED_STATE);
      }
      else
      if (strcmp("version",readingString) == 0)
      {
        Serial1.print("v");
        Serial1.println(VERSION);
      }
      else
      Serial1.println("No command available");
    }

  } /* end for (;;) */
}


//-----------------------------------------------------------------------------
/* This task activate send notitfication to activate CC ON using
* door notification and jamming detection */
static void vBlueTask(void *pvParameters)
{
  int c_data = 0;
  TickType_t xTimeLow;
  char readingString[10];              /* reading string from bluetooth */
  char reading;
  xTimerStart(xTimerNotification,0);
  //xTimerStart(xTimerBlueReset,0);
  TickType_t xLastDataReceived;

  Serial1.print("AT+DEFAULT\r\n");
  Serial1.print("AT+RESET\r\n");
  Serial1.print("AT+ROLE0\r\n");
  Serial1.print("AT+NAMEBLACKDEMO2\r\n");

  xLastDataReceived = xTaskGetTickCount();

  for  (;;)
  {

    if(Serial1.available() > 0)
    {
    //  Serial.println(xTaskGetTickCount() - xLastDataReceived);
      readingString[c_data] = Serial1.read();
      readingString[c_data+1] = NULL;
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
        readingString[0] = NULL;
    }

    if(strcmp("normal", readingString) == 0)
    {
      xTaskNotify(cc_handler,0x01, eSetValueWithOverwrite );
      Serial1.print("s");
      Serial1.println(NORMAL_STATE);
      readingString[0] = NULL;
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
        readingString[0] = NULL;
      }

      if (strcmp("bloquear",readingString) == 0)
      {
        xTaskNotify(cc_handler,0x03, eSetValueWithOverwrite );
        Serial1.print("s");
        Serial1.println(BLOCKED_STATE);
        readingString[0] = NULL;
      }

      if (strcmp("version",readingString) == 0)
      {
        Serial1.print("v");
        Serial1.println(VERSION);
        readingString[0] = NULL;
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

    TickType_t xFrequency = pdMS_TO_TICKS(10);
    uint32_t ulNotifiedValue = 0x00;
    for (;;)
    {

      xTaskNotifyWait( 0x00,      /* Don't clear any notification bits on entry. */
      0xFF, /* Reset the notification value to 0 on exit. */
      &ulNotifiedValue, /* Notified value pass out in ulNotifiedValue. */
      xFrequency  );  /* Block indefinitely. */

      /* Deactivate CC with CCPin */
      if( ( ulNotifiedValue == 0x01 ) )
      {
        digitalWrite(CCPin, LOW);
        vTaskResume(jammer_handler);
        vTaskResume(protocol_handler);
        EEPROM.update(stateAddress, NORMAL_STATE);
      }

      /* Activate CC with digital CCPin in a On/Off secuence */
      if( ( ulNotifiedValue == 0x02 ))
      {
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
        EEPROM.update(stateAddress, JAMMED_STATE);
      }

      /* Activate CC with digital CCPin */
      if( ( ulNotifiedValue == 0x03) )
      {
        digitalWrite(CCPin, HIGH);
        EEPROM.update(stateAddress, BLOCKED_STATE);
      }
      /* Deactivate CC with digital CCPin and suspend*/
      if( ( ulNotifiedValue == 0x04) )
      {
        digitalWrite(CCPin, LOW);
        EEPROM.update(stateAddress, SUSPEND_STATE);
      }

      /* Activate CC with digital CCPin */
      if( ( ulNotifiedValue == 0x05) )
      {
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
        EEPROM.update(stateAddress, BLOCKED_STATE);
      }
      /* Activate CC with digital CCPin */
      if( ( ulNotifiedValue == 0x06) )
      {
        digitalWrite(CCPin, HIGH);
        EEPROM.update(stateAddress, JAMMED_STATE);
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

  //------------------------------------------------------------------------------
  /* This task recieve notifications from other task to activate / deactivate the CC
  *  and restart the MCU by hardware */
  static void vCCTask2(void *pvParameters)
  {

    TickType_t xFrequency = pdMS_TO_TICKS(10);
    uint32_t ulNotifiedValue = 0x00;

    #ifdef CC_FREE_BYTES
    UBaseType_t uxHighWaterMark;
    #endif

    for (;;)
    {

      #ifdef CC_FREE_BYTES
      Serial.print("CCFreeWords ");
      uxHighWaterMark = uxTaskGetStackHighWaterMark(cc_handler);
      Serial.println(uxHighWaterMark);
      vTaskDelay(pdMS_TO_TICKS(5000));
      #endif

      xTaskNotifyWait( 0x00,      /* Don't clear any notification bits on entry. */
      0xFF, /* Reset the notification value to 0 on exit. */
      &ulNotifiedValue, /* Notified value pass out in ulNotifiedValue. */
      xFrequency  );  /* Block indefinitely. */

      /* Deactivate CC with CCPin */
      if( ( ulNotifiedValue == 0x01 ) )
      {
        vTaskResume(ignition_handler);
        vTaskResume(protocol_handler);
        vTaskResume(door_handler);
        vTaskResume(jammer_handler);
        digitalWrite(CCPin, LOW);
        EEPROM.update(stateAddress, NORMAL_STATE);

        #ifdef PRINT_CC_STATUS
        Serial.print("Resumed JammingTask ProtolTask IgnitionTask DoorNotificationTask\t");
        Serial.println("CC = OFF Normal");
        #endif
      }

      /* Activate CC with digital CCPin in a On/Off secuence */
      if( ( ulNotifiedValue == 0x02 ))
      {
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
        EEPROM.update(stateAddress, JAMMED_STATE);
      }
      /* Activate CC with digital CCPin */
      if( ( ulNotifiedValue == 0x03) )
      {
        suspendOperationalTasks();
        vTaskResume(jammer_handler);
        xTaskNotify(jammer_handler,0x02, eSetValueWithOverwrite );
        digitalWrite(CCPin, HIGH);
        /* update state of the coda */
        #ifdef PRINT_CC_STATUS
        Serial.print("Suspended JammingTask ProtolTask IgnitionTask DoorNotificationTask\t");
        Serial.println("CC = ON blocked");
        #endif
      }
      /* Deactivate CC with digital CCPin and suspend*/
      if( ( ulNotifiedValue == 0x04) )
      {
        digitalWrite(BuzzerPin, HIGH);
        vTaskDelay(pdMS_TO_TICKS(100));
        digitalWrite(BuzzerPin, LOW);
        suspendOperationalTasks();
        digitalWrite(CCPin, LOW);
        /* update state of the coda */
        EEPROM.update(stateAddress, SUSPEND_STATE);

        #ifdef PRINT_CC_STATUS
        Serial.print("Suspended JammingTask ProtolTask IgnitionTask DoorNotificationTask\t");
        Serial.println("CC OFF ");
        #endif
        vTaskSuspend(NULL);
      }
      /* Activate CC with digital CCPin */
      if( ( ulNotifiedValue == 0x05) )
      {
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
        EEPROM.update(stateAddress, BLOCKED_STATE);
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

  bool checkDoorStatus(int8_t _pin, bool _open)
  {
    bool reading =  digitalRead(_pin); // LOW equals Open
    bool current = reading;
    while(true){
      reading = digitalRead(_pin);
      if(reading != current)
      vTaskDelay(pdMS_TO_TICKS(500));
      current = reading;
      if(current == _open)
      return true;
      else
      return false;
    }
  }

bool checkSavedState()
{
  int8_t savedState = EEPROM.read(stateAddress);

  if(savedState == JAMMED_STATE){
    Serial.println("jammed");
  //  xTaskNotify(cc_handler,0x06, eSetValueWithOverwrite );
    return false;
    }
  if(savedState == BLOCKED_STATE){
    Serial.println("bloqued");
    //  xTaskNotify(cc_handler,0x03, eSetValueWithOverwrite );
      return false;
    }
    if(savedState == SUSPEND_STATE){
      Serial.println("suspended");
    //  xTaskNotify(cc_handler,0x04, eSetValueWithOverwrite );
      return false;
    }
      Serial.println("normal");
    //  EEPROM.update(stateAddress, NORMAL_STATE);
      return true;
}

  /* Suspend vIgnitionNotification , vJammingTask, vDoorStatusCheckTask and vProtcolTask
  * Also update in the EEPROm the current state */
  void suspendOperationalTasks()
  {
    xTaskNotify(protocol_handler,0x02, eSetValueWithOverwrite );

    /* Suspend optarional tasks */
    vTaskSuspend(door_handler);
    vTaskSuspend(jammer_handler);
    /* wait for ignition to tturn off */
    vTaskDelay((pdMS_TO_TICKS(10000)));
    vTaskSuspend(ignition_handler);
    vTaskSuspend(protocol_handler);
  }
  /* Restart by hardware */
  void restartHW()
  {
    do{
      wdt_enable(WDTO_15MS);
      for(;;){ }
    } while(0);
  }

  void suspendTasks()
  {
    digitalWrite(BuzzerPin, HIGH);
    vTaskDelay(pdMS_TO_TICKS(100));
    digitalWrite(BuzzerPin, LOW);
    /* nofitfy secure protocol off */
    xTaskNotify(protocol_handler,0x02, eSetValueWithOverwrite );
    /* send vCCTask CC ON notification */
    xTaskNotify(cc_handler,0x01, eSetValueWithOverwrite );
    vTaskDelay(pdMS_TO_TICKS(500));
    vTaskSuspend(ignition_handler);
    vTaskSuspend(protocol_handler);
    vTaskSuspend(door_handler);
    vTaskSuspend(jammer_handler);
    vTaskSuspend(cc_handler);

  }
