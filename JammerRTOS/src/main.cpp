

#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <avr/wdt.h>
#include <EEPROM.h>

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
//#define PASSWORD_FREE_BYTES
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
#define EnableDoorPin          6
#define EngineStatusPin        8    // Jamming alert ping HIGH =
#define JamDetectionAuxPin     7
#define LedPin                 9
#define CCPin                  10    // Corta corriente HIGH = shutdown / LOW = normal
#define BuzzerPin              14
#define DoorPositivePin        15
#define NotifyPin             16

#define TIME_AFTER_START       15
#define TIME_AFTER_STOP        10
#define TIME_TO_STOP_ENGINE    40
#define TIME_TO_OPEN_DOOR      30
#define DOOR_ENABLE_SECONDS    7
#define NOTIFICATION_TIME      30
#define TIME_AFTER_OPEN_DOOR   30 // secs after the door was opened
#define TIME_JAMMING_SECURE    120
#define TIME_JAMMING_HANG      30*SECS_PER_MIN
#define TIME_JAMMING_AFTER_SECURE 120

#define PASSLENGTH 6
#define DEVICE_ID              001
//define BAUDRATE               38400
#define BAUDRATE               9600

/** the current address in the EEPROM (i.e. which byte we're going to write to next) **/
int address = 0;


TaskHandle_t cc_handler;
TaskHandle_t jammer_handler;
TaskHandle_t disable_handler;
TaskHandle_t protocol_handler;
TaskHandle_t ignition_handler;

TaskHandle_t door_handler;
TaskHandle_t passwd_handler;

void time(long val);
void printDigits(byte digits);
int checkPinStatus(uint8_t _pin, uint8_t lastButtonState);
void suspendTasks();
void keyGenerator(char * _temp);
int8_t readSerialBluetooth(char *_reading);
int8_t bluetoothInit( );
int8_t sendATCmd(char _cmd[],char *_response);


static void vProtocolTask(void *pvParameters);
static void vDisableTask(void *pvParameters);
static void vIgnitionNotification(void *pvParameters);
static void vJammingTask(void *pvParameters);
static void vPasswordTask(void *pvParameters);
static void vCCTask(void *pvParameters);
static void vDoorStatusCheckTask(void *pvParameters);

void vApplicationMallocFailedHook( void );
void vApplicationStackOverflowHook( void );
//---------------------------------------------------------------------
unsigned long _time = 0;
unsigned long _last_time = 0;

void setup() {

  #ifdef DEBUG
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);

  while (!Serial) {
  ; // wait for serial port to connect. Needed for native USB
  }
  #endif



  wdt_enable(WDTO_4S);

  xTaskCreate(vDisableTask,"Disable",configMINIMAL_STACK_SIZE+50,NULL,tskIDLE_PRIORITY,&disable_handler);
  xTaskCreate(vCCTask,"CC",configMINIMAL_STACK_SIZE+62-32+10, NULL, tskIDLE_PRIORITY,&cc_handler);
  xTaskCreate(vIgnitionNotification,"Ignition",configMINIMAL_STACK_SIZE+31,NULL,tskIDLE_PRIORITY,&ignition_handler);
  xTaskCreate(vProtocolTask,"Protocol",configMINIMAL_STACK_SIZE+40,NULL,tskIDLE_PRIORITY,&protocol_handler);
  xTaskCreate(vJammingTask,"Jamming",configMINIMAL_STACK_SIZE+40, NULL, tskIDLE_PRIORITY , &jammer_handler);
  xTaskCreate(vPasswordTask,"Password",configMINIMAL_STACK_SIZE+80, NULL, tskIDLE_PRIORITY ,&passwd_handler);
  xTaskCreate(vDoorStatusCheckTask,"Doorpos",configMINIMAL_STACK_SIZE+40, NULL, tskIDLE_PRIORITY ,&door_handler);

  pinMode(NotifyPin,OUTPUT);
  pinMode(CCPin, OUTPUT);
  pinMode(LedPin, OUTPUT);
  pinMode(BuzzerPin, OUTPUT);
  pinMode(EnableDoorPin, INPUT_PULLUP);
  pinMode(DoorPin, INPUT_PULLUP);
  pinMode(JamDetectionPin, INPUT_PULLUP);
  pinMode(DoorPositivePin, INPUT_PULLUP);
  pinMode(IgnitionPin, INPUT_PULLUP);
  pinMode(CCDisable, INPUT_PULLUP);
  pinMode(NotifyPin,OUTPUT);

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

  uint32_t ulNotifiedValue = 0x00;            /* notified recieve value*/
  TickType_t xFrequency = pdMS_TO_TICKS(10);  /* frecuency for waiting notification */
  int8_t k=0;
  #ifdef PROTOCOL_FREE_BYTES
  UBaseType_t uxHighWaterMark;
  #endif                   /* counter for displaying one time */


  for(;;) {

    #ifdef PROTOCOL_FREE_BYTES
    Serial.print("ProtocolFreeWords ");
    uxHighWaterMark = uxTaskGetStackHighWaterMark(protocol_handler);
    Serial.println(uxHighWaterMark);
    vTaskDelay(pdMS_TO_TICKS(5000));
    #endif

    xTaskNotifyWait( 0x00,      /* Don't clear any notification bits on entry. */
    0xF0, /* Reset second word of the notification value to 0 on exit. */
    &ulNotifiedValue, /* Notified value pass out in ulNotifiedValue. */
    xFrequency  );  /* Block indefinitely. */

    /* recieve engine off  notification from vIgnitionNotification */
    if( ( ulNotifiedValue == 0x02  ) )
    {
      digitalWrite(LedPin, LOW);
      k = 0;
      xFrequency = portMAX_DELAY;
      #ifdef PRINT_PROTOCOL_STATUS
      Serial.print("Secure Protocol Stoped!\n");
      #endif
    }

    /* recieve engine on  notification from vIgnitionNotification */
    if( ( ulNotifiedValue == 0x01 ) )
    {
      digitalWrite(LedPin,HIGH);

      #ifdef PRINT_PROTOCOL_STATUS
      if(k == 0)
        Serial.print("Secure Protocol Started!\n");
      k = 1;
      #endif
      xFrequency = pdMS_TO_TICKS(10);
      /* if button is pressed */
      if(digitalRead(EnableDoorPin) == LOW){
        #ifdef PRINT_PROTOCOL_STATUS
        Serial.print("vProtcolTask Suspended  ");
        Serial.print(TIME_TO_OPEN_DOOR);
        Serial.print(" seconds ");
        #endif
        /* send disable door open detection notification*/
        xTaskNotify(door_handler,0x01, eSetValueWithOverwrite);
        int ecount = 0;      /* EnableDoorPin counter */
        int i = 0;           /* counter for print */
        /* suspend vProtcolTask for TIME_TO_OPEN_DOOR seconds */
        do
        {
          #ifdef PRINT_PROTOCOL_STATUS
          if(i % 10 == 0)
          {
           Serial.print((TIME_TO_OPEN_DOOR+ecount)-i);
           Serial.print(" ");
          }
          #endif

          digitalWrite(LedPin,LOW);
          vTaskDelay(pdMS_TO_TICKS(1000));
          i++;
          digitalWrite(LedPin,HIGH);
          vTaskDelay(pdMS_TO_TICKS(1000));
          i++;
          /* if button is pressed , add 20 seconds more to te suspended task*/
          if( (digitalRead(EnableDoorPin) == LOW) && (ecount <= 50 ) )
              ecount += 10;

          } while(i < (TIME_TO_OPEN_DOOR + ecount) );

          /* send disable door open detection notification*/
          xTaskNotify(door_handler,0x00, eSetValueWithOverwrite);

        //  xTaskNotify(dneg_handler,0x00, eSetValueWithOverwrite);
        //  xTaskNotify(dpos_handler,0x00, eSetValueWithOverwrite);
          #ifdef PRINT_PROTOCOL_STATUS
          Serial.println("vProtocolTask Resumed ");
          #endif
        } /* end EnableDoorPin */
      } /* end secure protocol */


     /* if door is open redunce */
     #ifndef DOORSENSOR
     if(digitalRead(DoorPin) == LOW || digitalRead(DoorPositivePin) == LOW)
     {
     #endif
     #ifdef DOORSENSOR
     if(digitalRead(DoorPin) == HIGH)
     {
     #endif
      /* if door notification is open */
      if(  ulNotifiedValue == 0x11 )
      {
       digitalWrite(LedPin, LOW);
       #ifdef PRINT_PROTOCOL_STATUS
       Serial.print("vProtocolTask Suspended ");
       #endif

       /* suspend task for TIME_AFTER_OPEN_DOOR seconds */
       for(int j = 0; j < TIME_AFTER_OPEN_DOOR ; j++)
       {
        #ifdef PRINT_PROTOCOL_STATUS
        if (j % 10 == 0)
        {
          Serial.print(j);
          Serial.print(" ");
        }
        #endif
        vTaskDelay(pdMS_TO_TICKS(500));
        digitalWrite(LedPin,HIGH);
        vTaskDelay(pdMS_TO_TICKS(500));
        digitalWrite(LedPin,LOW);
      }

        digitalWrite(LedPin,LOW);
        #ifdef PRINT_PROTOCOL_STATUS
        Serial.println("vCCTask notification");
        #endif
        /* send CC on notification */
        xTaskNotify(cc_handler,0x03, eSetValueWithOverwrite );
        vTaskDelay(pdMS_TO_TICKS(TIME_AFTER_STOP*1000));
        ulNotifiedValue = 0x02;
      } /* end door open and protocol notification */
    } /* end door open redundace detection*/
  } /* end for(;;) */
}

//-----------------------------------------------------------------------------
/* This task activate send notitfication to activate CC ON using
 * door notification and jamming detection */
static void vDisableTask(void *pvParameters)
 {

  bool _last_state     = HIGH;              /* CCDisable pin last state*/
  bool _reading_state  = HIGH;              /* CCDisable pin reading state */
  bool _current_state  = HIGH;              /* CCDisable pin current state */

  bool suspendState    = false;             /* button suspended state */
  bool checkState      = false;



  int i = 0;
  TickType_t xTimeLow;
  int16_t lastDebounceTime = 0;                 /* the last time the EnableDoorPin pin was toggled */
  int16_t debounceDelay = pdMS_TO_TICKS(500);   /* the debounce time */

  int16_t ecount = 0;                           /*counter for EnableDoorPin on*/
  int16_t elimit  = 0;                          /* limit for vCCTask notfiication*/

  #ifdef DISABLE_FREE_BYTES
  UBaseType_t uxHighWaterMark;
  #endif

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
    while(digitalRead(EnableDoorPin) == LOW)
    {
    #ifdef PRINT_DISABLE_STATUS
    if((elimit-ecount) % 20  == 0){
      Serial.print(elimit-ecount);
      Serial.print(" ");
    }
    #endif
    /* when reach the limit */
    if(ecount == elimit)
    {
      /* if already suspended */
      if (suspendState)
      {

        #ifdef PRINT_DISABLE_STATUS
        Serial.println("Resumed");
        #endif
        digitalWrite(BuzzerPin, HIGH);
        vTaskDelay(pdMS_TO_TICKS(50));
        digitalWrite(BuzzerPin, LOW);
        vTaskDelay(pdMS_TO_TICKS(100));
        digitalWrite(BuzzerPin, HIGH);
        vTaskDelay(pdMS_TO_TICKS(50));
        digitalWrite(BuzzerPin, LOW);
        /* resume vCCTask if suspedend */
        vTaskResume(cc_handler);
        vTaskDelay(pdMS_TO_TICKS(500));
        /* send vCCTask  System restart notification */
        xTaskNotify(cc_handler,0x10, eSetValueWithOverwrite );
      }
      /* if not suspended */
      else
      {
        #ifdef PRINT_DISABLE_STATUS
        Serial.println("Suspended");
        #endif
        digitalWrite(BuzzerPin, HIGH);
        suspendTasks();
        suspendState = true;
        ecount = 0;
      }
    }

    /* count every 1 second */
    vTaskDelay(pdMS_TO_TICKS(100));
    ecount++;
  } /* end while EnableDoorPin */


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
        xTaskNotify(cc_handler,0x01, eSetValueWithOverwrite );
        #ifdef PRINT_DISABLE_STATUS
        Serial.println("CC OFF by SAFECAR");
        #endif
      }
      /* if CCDisable is LOW then CC ON */
      if(_current_state == LOW)
      {
        /* send CC ON notification */
        xTaskNotify(cc_handler,0x03, eSetValueWithOverwrite );
        #ifdef PRINT_DISABLE_STATUS
        Serial.println("CC ON by SAFECAR");
        #endif
      }
    } /* end if any change */
  } /* end if change after debounceDelay */
  _last_state = _reading_state;

  } /* end for (;;) */
}

//-----------------------------------------------------------------------------
/* This task check the igntion status and  send notitfication to
 * activate/deactivate  vProtocolTask */
static void vIgnitionNotification(void *pvParameters) {

  long c_engine_on = 0;                            /* engine on counter */
  long c_engine_off = 0;                           /* engine off counter */
  TickType_t xLastWakeTime = xTaskGetTickCount();  /* counter initialize */
  TickType_t xFrequency = pdMS_TO_TICKS(1000);     /* frecuency of the task */
  #ifdef IGNITION_FREE_BYTES
  UBaseType_t uxHighWaterMark;
  #endif
  for  (;;) {

    #ifdef IGNITION_FREE_BYTES
    Serial.print("IgntionFreeWords ");
    uxHighWaterMark = uxTaskGetStackHighWaterMark(ignition_handler);
    Serial.println(uxHighWaterMark);
    vTaskDelay(pdMS_TO_TICKS(5000));
    #endif
    /* wait 1 second */
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    /* if ignition is activated */

    if(digitalRead(IgnitionPin) == LOW)
    {
      /* check if the door is open */
      #ifndef DOORSENSOR /* for automotive sensor and positive/negative sensor*/
      if(digitalRead(DoorPin) == LOW || digitalRead(DoorPositivePin) == LOW){
      #endif
      #ifdef DOORSENSOR /* for magnetic sensor */
      if(digitalRead(DoorPin) == HIGH){
      #endif
      /* activate buzzer */
        digitalWrite(BuzzerPin, HIGH);
        vTaskDelay(pdMS_TO_TICKS(50));
        digitalWrite(BuzzerPin, LOW);
    }
      #ifdef PRINT_IGNITION_STATUS
      if(c_engine_on <= TIME_AFTER_START)
      {
        if(c_engine_on % 5 == 0)
        {
        Serial.print("Ignition ON ");
        time(c_engine_on);
        }
      }
      else
        if( c_engine_on % 60*5 == 0 )
        {
          Serial.print("Ignition ON ");
          time(c_engine_on);
        }
      #endif

      if(c_engine_on == TIME_AFTER_START)
      {
        /* Notify vProtocolTask ON because engine is supose to be on */
          xTaskNotify(protocol_handler,0x01, eSetValueWithOverwrite );
          #ifdef PRINT_IGNITION_STATUS
          Serial.println("Protocol Task notification");
          #endif
      }
        c_engine_on++;
        c_engine_off = 0;
    }  /* end igtion on */
    else
    { /* if ignition off */

      /* if ingtion is on by some time */
      if(c_engine_off == TIME_AFTER_STOP)
      {
        xTaskNotify(protocol_handler,0x02, eSetValueWithOverwrite );
        #ifdef PRINT_IGNITION_STATUS
        Serial.println("Protocol Task notification");
        #endif
      }

      #ifdef PRINT_IGNITION_STATUS
      if(c_engine_off <= TIME_AFTER_STOP)
      {
        Serial.print("Ignition OFF ");
        time(c_engine_off);
      }
      else
        if(c_engine_off % 60 == 0)
        {
          Serial.print("Ignition OFF ");
          time(c_engine_off);
        }
      #endif

        /* counter ingnition off */
        c_engine_on = 0;
        c_engine_off++;
      } /* end engine off*/

    } /* end for (;;)*/
  }

//-----------------------------------------------------------------------------
/* This task activate send notitfication to activate CC ON using
 * door notification and jamming detection */
static void vJammingTask(void *pvParameters) {
  uint32_t c_jammer = 0;                          /* jamming on counter */
  TickType_t xFrequency = pdMS_TO_TICKS(1000);    /* frecuency for waiting notification */
  uint32_t ulNotifiedValue = 0x00;                /* notified recieve value*/
  #ifdef JAMMING_FREE_BYTES
  UBaseType_t uxHighWaterMark;
  #endif

  for(;;) {

    #ifdef JAMMING_FREE_BYTES
    Serial.print("JammingFreeWords ");
    uxHighWaterMark = uxTaskGetStackHighWaterMark(jammer_handler);
    Serial.println(uxHighWaterMark);
    vTaskDelay(pdMS_TO_TICKS(5000));
    #endif
    // Sleep for one second.
    xTaskNotifyWait( 0x00,      /* Don't clear any notification bits on entry. */
    0xFF, /* Reset the notification value to 0 on exit. */
    &ulNotifiedValue, /* Notified value pass out in ulNotifiedValue. */
    xFrequency  );  /* Block indefinitely. */
    /* if jamming is detected */
    if(digitalRead(JamDetectionPin) == LOW)
    {
      /* if door is open (door status rendundance) */
      #ifndef DOORSENSOR /* for automotive sensor and positive/negative sensor*/
      if(digitalRead(DoorPin) == LOW || digitalRead(DoorPositivePin) == LOW)
      {
      #endif
      #ifdef DOORSENSOR /* for magnetic sensor */
      if(digitalRead(DoorPin) == HIGH)
      {
      #endif
        /* if notification from door is open*/
        if( ulNotifiedValue == 0x01 )
        {
          /*  Notify vCCTask ON because dooropen to vCCTask */
          xTaskNotify(cc_handler,0x03, eSetValueWithOverwrite );
          #ifdef PRINT_JAMMING_STATUS
          Serial.println("vCCTask notification");
          #endif
        }
      } // end rendundance door open detector

      /* if jammer is detected for several minutes time*/
      if (c_jammer == TIME_JAMMING_SECURE )
      {
        /* Notify vCCTask ON because Jamming elapsed time to vCCTask  */
        xTaskNotify(cc_handler,0x02, eSetValueWithOverwrite );
        #ifdef PRINT_JAMMING_STATUS
        Serial.println("vCCTask notification ");
        #endif
      }
      /* if jammer is hanged for too much time*/
      if (c_jammer == TIME_JAMMING_HANG)
      {
        /* Notify vCCTask OFF because jammer is hanged */
        xTaskNotify(cc_handler,0x01, eSetValueWithOverwrite );

//        #ifdef PRINT_JAMMING_STATUS
//        Serial.println("vJammingTask Suspended 10min ");
//        #endif
//        /* Suspend jamming detection for 10 minutes */
//        for (int i ; i < (10*60) ; i++)
//          vTaskDelay(pdMS_TO_TICKS(1000));
//          #ifdef PRINT_JAMMING_STATUS
//          Serial.println("vJammingTask Resumed");
//          #endif

      }
      #ifdef PRINT_JAMMING_STATUS
        Serial.print("Jammer detected ");
        time(c_jammer);
      if( (c_jammer <= TIME_JAMMING_SECURE)  && (c_jammer % 30 == 0)  )
      {
        Serial.print("Jammer Secure ON ");
        time(c_jammer);
      }
      if ( c_jammer > TIME_JAMMING_SECURE )
        if( (c_jammer % (60*5) == 0 ))
        {
          Serial.print("Jammer Hang ON ");
          time(c_jammer);
        }
      #endif
      c_jammer++;
    } /* end jamming detection */

    else
    {
      c_jammer = 0;
    }
  } /*/ end for(;;) */
}

//-----------------------------------------------------------------------------
/* This task receive notifications from door tasks and evaluate
 * if send CC notification to turn ON the CC */

static void vPasswordTask(void *pvParameters) {

  bool suspend         = false;          /* bluetooth suspended state */
  char readingString[10];              /* reading string from bluetooth */
  char _id = 20;                      /* local store for ID device */

  #ifdef PASSWORD_FREE_BYTES
  UBaseType_t uxHighWaterMark;
  #endif

  if(BAUDRATE == 38400)
    bluetoothInit();
  else
    Serial1.begin(9600);

  //Serial1.begin(9600);

  for (;;) {

    #ifdef PASSWORD_FREE_BYTES
    Serial.print("PasswordFreeWors ");
    uxHighWaterMark = uxTaskGetStackHighWaterMark(passwd_handler);
    Serial.println(uxHighWaterMark);
    vTaskDelay(pdMS_TO_TICKS(5000));
    #endif

    vTaskDelay(configTICK_RATE_HZ);

    if (readSerialBluetooth(readingString) > 0)
    {
      Serial1.println(readingString);

      if (strcmp("suspend",readingString) == 0)
        {
            suspendTasks();
            Serial1.println("AJ - Suspended");
        }
      else
        if(strcmp("auth", readingString) == 0)
        {
          _id = 'a';
          address = 0;
          Serial1.println("ID: ");
          while (_id != NULL)
          {
            _id = EEPROM.read(address++);
            Serial1.print(_id);
          }
          Serial1.println();
        }
        else
          if(strcmp("restart", readingString) == 0)
          {
            digitalWrite(BuzzerPin, HIGH);
            vTaskDelay(pdMS_TO_TICKS(50));
            digitalWrite(BuzzerPin, LOW);
            vTaskDelay(pdMS_TO_TICKS(100));
            digitalWrite(BuzzerPin, HIGH);
            vTaskDelay(pdMS_TO_TICKS(50));
            digitalWrite(BuzzerPin, LOW);
            /* resume vCCTask if suspedend */
            vTaskResume(cc_handler);
            vTaskDelay(pdMS_TO_TICKS(500));
            /* send vCCTask  System restart notification */
            Serial1.println("System will be restarted");
            xTaskNotify(cc_handler,0x10, eSetValueWithOverwrite );
          }
          else
          {
            Serial1.println("No command available");
          }
    }
  }
}

static void vDoorStatusCheckTask(void *pvParameters)
{
  /* HIGH means door closed */
  #ifndef DOORSENSOR
  bool negativeState     = HIGH;                      /* notified door state */
  bool lastNegativeState = HIGH;                      /* last door notified value */
  bool readingNegative   = HIGH;                      /* current reading */
  #endif
  bool positiveState     = HIGH;
  bool lastPositiveState = HIGH;
  bool readingPositive   = HIGH;

  /* LOW means door closed */
  #ifdef DOORSENSOR
  bool negativeState     = LOW;                       /* notified door state */
  bool lastNegativeState = LOW;                       /* last door notified value */
  bool readingNegative   = LOW;                       /* current reading */
  #endif



  int16_t lastDebounceTimePositive = 0;
  int16_t lastDebounceTimeNegative = 0;                   /* the last time the output pin was toggled */
  int16_t debounceDelay = pdMS_TO_TICKS(500);     /* the debounce time */
  uint32_t ulNotifiedValue = 0x00;                /* notified recieve value*/
  BaseType_t xReturned;                           /* sended notification return */
  TickType_t xFrequency = pdMS_TO_TICKS(100);     /* frecuency for waiting notification */

  #ifdef DOOR_FREE_BYTES
  UBaseType_t uxHighWaterMark;
  #endif

  for(;;)
  {
    #ifdef DOOR_FREE_BYTES
    Serial.print("DoorFreeWords ");
    uxHighWaterMark = uxTaskGetStackHighWaterMark(door_handler);
    Serial.println(uxHighWaterMark);
    vTaskDelay(pdMS_TO_TICKS(5000));
    #endif

    xTaskNotifyWait( 0x00, 0x00, &ulNotifiedValue, xFrequency  );

        readingPositive = digitalRead(DoorPositivePin);

        #ifdef PRINT_DOORSTATUS
        if( readingPositive == HIGH )
          Serial.println("DoorPositive Closed");
        else
          Serial.println("DoorPositive Open");
        #endif

      if (readingPositive != lastPositiveState)
        lastDebounceTimePositive = xTaskGetTickCount();

      /* if there is no change after debounceDelay */
      if( ( xTaskGetTickCount() - lastDebounceTimePositive ) > debounceDelay )
      {
        /* if the current reading is different from lastnotification value */
        if ( (readingPositive != positiveState) )
        {
          positiveState = readingPositive;
          /* if door is open */
          if( positiveState == LOW)
          {
            /* if recieve notification , suspend any door status check */
            if(ulNotifiedValue != 0x01)
            {
              /* Notify dooropen to vProtcolTask */
              xReturned =  xTaskNotify(protocol_handler,( 1UL << 4UL ), eSetBits );
              #ifdef PRINT_DOORSTATUS
              if(xReturned == pdPASS)
              Serial.println("DoorPOSopen vProtocolTask notification");
              #endif
            }
            /* Notify dooropen to vJammingTask */
            xReturned = xTaskNotify(jammer_handler,0x01, eSetValueWithOverwrite);
            #ifdef PRINT_DOORSTATUS
            if(xReturned == pdPASS)
              Serial.println("DooroPOSpen vJammingTask notification");
              #endif
          } /* end if door is open */
          else /* if door is close do nothing */
          {
            #ifdef PRINT_DOORSTATUS
            Serial.print("DoorPOS closed\n");
            #endif
          }
        } /* end if any change */
      } /* end if change after debounceDelay */
      lastPositiveState = readingPositive;

    /* readings every 100 ms */
    readingNegative = digitalRead(DoorPin);

    #ifdef PRINT_DOORSTATUS
    if( readingNegative == HIGH )
      Serial.println("DoorNegative Closed");
    else
      Serial.println("DoorNegative Open");
    #endif
    /* if any change reset the timer count */
    if (readingNegative != lastNegativeState )
          lastDebounceTimeNegative = xTaskGetTickCount();

    /* if there is no change after debounceDelay */
    if( ( xTaskGetTickCount() - lastDebounceTimeNegative ) > debounceDelay )
    {
      if ( (readingNegative != negativeState) )
      {
        /* if the current reading is different from lastnotification value */
        negativeState = readingNegative;
        /* if door is open */
        #ifndef DOORSENSOR /* automotive sensor */
        if(negativeState == LOW)
        {
        #endif
        #ifdef DOORSENSOR /* magenitc sensor */
        if(negativeState == HIGH )
        {
        #endif
        /* if recieve notification , suspend any door status check */
          if(ulNotifiedValue != 0x01)
          {
            /* Notify dooropen to vProtcolTask */
            xReturned =  xTaskNotify(protocol_handler,( 1UL << 4UL ), eSetBits );
            #ifdef PRINT_DOORSTATUS
            if(xReturned == pdPASS)
              Serial.println("DoorNEGopen vProtocolTask notification");
            #endif
          }
          /* Notify dooropen to vJammingTask */
          xReturned = xTaskNotify(jammer_handler,0x01, eSetValueWithOverwrite);
          #ifdef PRINT_DOORSTATUS
          if(xReturned == pdPASS)
            Serial.println("DooroNEGpen vJammingTask notification");
          #endif
        } /* end if door is open */
        else /* if door is close do nothing */
        {
          #ifdef PRINT_DOORSTATUS
          Serial.print("DoorNEG closed\n");
          #endif
        }
      } /* end if any change */
    } /* end if change after debounceDelay */
      lastNegativeState = readingNegative;
  }
}

//------------------------------------------------------------------------------
/* This task recieve notifications from other task to activate / deactivate the CC
*  and restart the MCU by hardware */
static void vCCTask(void *pvParameters) {
  uint32_t ulNotifiedValue = 0x00;
  TickType_t xFrequency = pdMS_TO_TICKS(10);
  #ifdef CC_FREE_BYTES
  UBaseType_t uxHighWaterMark;
  #endif

  for (;;) {

    #ifdef CC_FREE_BYTES
    Serial.print("CCFreeWords ");
    uxHighWaterMark = uxTaskGetStackHighWaterMark(cc_handler);
    Serial.println(uxHighWaterMark);
    vTaskDelay(pdMS_TO_TICKS(5000));
    #endif


      xTaskNotifyWait( 0x00,      /* Don't clear any notification bits on entry. */
      0xFF, /* Reset the notification value to 0 on exit. */
      &ulNotifiedValue, /* Notified value pass out in
      ulNotifiedValue. */
      xFrequency  );  /* Block indefinitely. */


      /* Deactivate CC with CCPin */
      if( ( ulNotifiedValue == 0x01 ) )
      {
          digitalWrite(CCPin, LOW);
          #ifdef PRINT_CC_STATUS
          Serial.println("CC = OFF blocked");
          #endif
      }

      /* Activate CC with digital CCPin in a On/Off secuence */
      if( ( ulNotifiedValue == 0x02 ))
      {
        TickType_t xFrequency = pdMS_TO_TICKS(1000);
        for(int i = 0 ; i < 3 ; i++)
        {
          digitalWrite(CCPin, HIGH);
          vTaskDelay(xFrequency);
        }
        for(int i = 0 ; i < 20 ; i++)
        {
          digitalWrite(CCPin, LOW);
          vTaskDelay(xFrequency);
        }
        for(int i = 0 ; i < 3 ; i++)
        {
          digitalWrite(CCPin, HIGH);
          vTaskDelay(xFrequency);
        }
        for(int i = 0 ; i < 20 ; i++)
        {
          digitalWrite(CCPin, LOW);
          vTaskDelay(xFrequency);
        }
        digitalWrite(CCPin, HIGH);
        #ifdef PRINT_CC_STATUS
        Serial.println("CC = ON");
        #endif
      } /* end deactivation */

      /* Activate CC with digital CCPin */
      if( ( ulNotifiedValue == 0x03) )
      {
        #ifdef PRINT_CC_STATUS
        Serial.println("CC = ON blocked");
        #endif
        digitalWrite(CCPin, HIGH);
      }

      /* Watchdog System-Reset */
      if( ( ulNotifiedValue == 0x10 ) )
      {
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
/* Set the initial baudrate for sendig an AT command, then restore dthe baudrate
 * for normal bluetooth operation */
int8_t bluetoothInit( )
{
  int16_t check;
  char response[20];
  Serial1.begin(BAUDRATE);
  int8_t i = 0;
  char val;
  char * id;
  bool _done = false;
  TickType_t xTime1 = xTaskGetTickCount();


  #ifdef PRINT_PASSWORD_STATUS
  Serial.println("AT+ADDR?");
  #endif

  Serial1.println("AT+ADDR?");

    while(!_done )
    {
    if(Serial1.available())
    {

      /* read the avaliavable characters */
      val = Serial1.read();
      if (val != '\r' && val != '\n')
      {
        #ifdef PRINT_PASSWORD_STATUS
        Serial.print(val);
        #endif
        response[i] = val;
        i++;
      }
      vTaskDelay(pdMS_TO_TICKS(10));

      if( val == '\n'  )
      {
        /* save a string */
        response[i] = NULL;
        i = 0;
        val = '\r';

        #ifdef PRINT_PASSWORD_STATUS
        Serial.print("\nNew String: ");
        Serial.print(response);
        #endif

        /* check the string saved */
        if(strcmp(response, "OK")==0)
        {
          #ifdef PRINT_PASSWORD_STATUS
          Serial.println("\tfinish response");
          #endif
          _done = true;
        }
        else
        {
          if(strcmp(response,"ERROR:(0)")==0)
          {
            #ifdef PRINT_PASSWORD_STATUS
            Serial.println("\tSend Again");
            #endif
            Serial1.println("AT+ADDR?");
          }
          else
          {
            #ifdef PRINT_PASSWORD_STATUS
            Serial.print("\tNew ID: Safecar:\n");
            #endif
            id = strrchr(response,':');

            for(int i = 1; i<=strlen(id) ; i ++)
            {

              EEPROM.update(address+i-1, id[i]);
              #ifdef PRINT_PASSWORD_STATUS
              Serial.print(address+i-1);
              Serial.print("     ");
              Serial.println(id[i]);
              #endif
            }
          }
        }

      } /* end if */
    }
  }

  _done = false;
  val = '0';
  char sbuff[40];

  vTaskDelay(pdMS_TO_TICKS(500));

  sprintf(sbuff,"AT+NAME=SAFECAR_%s",id+1);
  Serial1.println(sbuff);

  #ifdef PRINT_PASSWORD_STATUS
  Serial.println(sbuff);
  #endif

  xTime1 = xTaskGetTickCount();

  while(!_done)
  {

  if(Serial1.available())
  {
    /* read the avaliavable characters */
    val = Serial1.read();
    if (val != '\r' && val != '\n')
    {
      #ifdef PRINT_PASSWORD_STATUS
      Serial.print(val);
      #endif
      response[i] = val;
      i++;
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    if( val == '\n'  )
    {
      /* save a string */
      response[i] = NULL;
      i = 0;
      val = '\r';

      #ifdef PRINT_PASSWORD_STATUS
      Serial.print("\nNew String: ");
      Serial.print(response);
      #endif

      /* check the string saved */
      if(strcmp(response, "OK")==0)
      {
        #ifdef PRINT_PASSWORD_STATUS
        Serial.println("\tfinish response");
        #endif
        _done = true;
      }
      else
      {
        if(strcmp(response,"ERROR:(0)")==0)
        {
          #ifdef PRINT_PASSWORD_STATUS
          Serial.println("\tSend Again");
          #endif
          Serial1.println(sbuff);
        }
        else
        {
          #ifdef PRINT_PASSWORD_STATUS
          Serial.print("\tNew NAME: ");
          #endif
          Serial.println(response);
          }
      }
    }
  }
  }
  for(int k = 0; k<4 ; k++)
  {
    digitalWrite(BuzzerPin, HIGH);
    vTaskDelay(pdMS_TO_TICKS(50));
    digitalWrite(BuzzerPin, LOW);
    vTaskDelay(pdMS_TO_TICKS(50));

  }
  #ifdef PRINT_PASSWORD_STATUS
  Serial.println("ready for use");
  #endif
  Serial1.begin(9600);
  return 0;
}

//------------------------------------------------------------------------------
/* Read and string from bluetooth , store it in _readings and return the number
 * of characters readed */
int8_t readSerialBluetooth(char * _reading)
{
  int i = 0;
  char val2;
  while( Serial1.available() )
  {
    val2 = Serial1.read();
    if (val2 != '\r' && val2 != '\n')
    {
      _reading[i] = val2;
      i++;
    }
    vTaskDelay(configTICK_RATE_HZ/10);
  }
  _reading[i] = NULL;
  return i;
}

//------------------------------------------------------------------------------

void suspendTasks()
{
  digitalWrite(BuzzerPin, HIGH);
  vTaskDelay(pdMS_TO_TICKS(100));
  digitalWrite(BuzzerPin, LOW);

  /* nofitfy secure protocol off */
  xTaskNotify(protocol_handler,0x02, eSetValueWithOverwrite );
  #ifdef PRINT_PASSWORD_STATUS
  Serial.println("Protocol Task notification");
  #endif
  /* send vCCTask CC ON notification */
  xTaskNotify(cc_handler,0x01, eSetValueWithOverwrite );
  vTaskDelay(pdMS_TO_TICKS(500));
  vTaskSuspend(ignition_handler);
  vTaskSuspend(protocol_handler);
  vTaskSuspend(door_handler);

  //vTaskSuspend(dneg_handler);
  //vTaskSuspend(dpos_handler);
  vTaskSuspend(jammer_handler);
  vTaskSuspend(cc_handler);

}

void printDigits(byte digits){
   // utility function for digital clock display: prints colon and leading 0

   Serial.print(":");
   if(digits < 10)
     Serial.print('0');
   Serial.print(digits,DEC);

}

void time(long val){

  int minutes = numberOfMinutes(val);
  int seconds = numberOfSeconds(val);

   // digital clock display of current time

   printDigits(minutes);
   printDigits(seconds);
   Serial.println();

}
