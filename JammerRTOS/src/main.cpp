
#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <avr/wdt.h>
#include <EEPROM.h>
#include "timers.h"

const char VERSION[] = "3.0.2";

/* Useful Constants */
#define SECS_PER_MIN  (60UL)
#define SECS_PER_HOUR (3600UL)
#define SECS_PER_DAY  (SECS_PER_HOUR * 24L)

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
#define DOOROPEN       LOW
#define DOORCLOSE      HIGH
#define NORMAL_STATE       1        /* cirtcuit normal state */
#define PROTOCOL_STATE     2        /* CC avtivated sate */
#define SUSPEND_STATE      3        /* CC deactivated and all the task suspended */
#define JAMMED_STATE       4
#define GPS_STATE          5
#define BLUETOOTH_STATE    6
#define UNKNOWN_STATE 69
#define ID 5
#define BUFF_SIZE     20

// inputs
#define CCDisable              2      // Enable/disable jamming detection HIGH = on & LOW = off
#define IgnitionPin            3  // (with pull down resistor) Engine power HIGH = on & LOW = off
#define JamDetectionPin        4
#define DoorNegativePin        5     // Door sensor  HIGH = open / LOW = close
#define DisarmPin              6
#define LedPin                 9
#define CCPin                  10    // Corta corriente HIGH = shutdown / LOW = normal
#define BlueEnablePin          14
#define DoorPositivePin        15
#define BlueDisablePin         8   // 10 Protocol block - 11 suspended
#define TestPin                7


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

void vTimerCallback( TimerHandle_t xTimer );
void vTimerMemoryCheckCallback( TimerHandle_t xTimer );
void vTimerRestart( TimerHandle_t xTimer);

void restartHW();
void updateState(int8_t _state);

void setup() {
  Serial1.begin(9600);
  //Serial.begin(9600);

    wdt_enable(WDTO_2S);

   xTaskCreate(vProtocolTask
    ,"Protocol"
    ,configMINIMAL_STACK_SIZE
    ,NULL,tskIDLE_PRIORITY
    ,&protocol_handler);

    xTaskCreate(vJammingTask
     ,"Jamming"
     ,configMINIMAL_STACK_SIZE
     ,NULL
     ,tskIDLE_PRIORITY
     ,&jammer_handler);


   xTaskCreate(vBlueTask
    ,"Blue"
    ,configMINIMAL_STACK_SIZE+20
    ,NULL
    ,tskIDLE_PRIORITY
    ,&blue_handler);

   xTaskCreate(vCCTask
    ,"CC"
    ,configMINIMAL_STACK_SIZE
    ,NULL
    ,tskIDLE_PRIORITY
    ,&cc_handler);

    xTaskCreate(vIOTask
     ,"IO"
     ,configMINIMAL_STACK_SIZE
     ,NULL
     ,tskIDLE_PRIORITY
     ,&io_handler);

     xTaskCreate(vTestTask
      ,"Test"
      ,configMINIMAL_STACK_SIZE
      ,NULL
      ,tskIDLE_PRIORITY
      ,&test_handler);

  /* restart notification callback */
  xTimerNotification = xTimerCreate(
    "Timer",
    configTICK_RATE_HZ*5,
    pdTRUE,
    (void *) 0,
    vTimerCallback
  );

  xTimerRestart = xTimerCreate(
    "Timer",
    configTICK_RATE_HZ*10,
    pdTRUE,
    (void *) 0,
    vTimerRestart
  );
  /*  xTimerMemoryChecknotification callback */
//  xTimerMemoryCheck = xTimerCreate(
//    "Timer",
//    configTICK_RATE_HZ*5,
//    pdTRUE,
//    (void *) 0,
//     vTimerMemoryCheckCallback
//  );

  pinMode(CCPin, OUTPUT);
  pinMode(LedPin, OUTPUT);
  pinMode(BlueEnablePin, OUTPUT);
  pinMode(DisarmPin, INPUT_PULLUP);
  pinMode(DoorNegativePin, INPUT_PULLUP);
  pinMode(JamDetectionPin, INPUT_PULLUP);
  pinMode(DoorPositivePin, INPUT_PULLUP);
  pinMode(IgnitionPin, INPUT_PULLUP);
  pinMode(CCDisable, INPUT_PULLUP);


  digitalWrite(BlueEnablePin,HIGH);

//    EEPROM.update(stateAddress, NORMAL_STATE);
int8_t savedState = EEPROM.read(stateAddress);
if(savedState == NORMAL_STATE)
  xTaskNotify(cc_handler,0x01, eSetValueWithOverwrite );
else
if(savedState == PROTOCOL_STATE)
  xTaskNotify(cc_handler,0x05, eSetValueWithOverwrite );
else
if(savedState == SUSPEND_STATE)
  xTaskNotify(cc_handler,0x03, eSetValueWithOverwrite );
else
if(savedState == JAMMED_STATE)
  xTaskNotify(cc_handler,0x05, eSetValueWithOverwrite );
else
if(savedState   == GPS_STATE)
  xTaskNotify(cc_handler,0x05, eSetValueWithOverwrite );
else
if(savedState == BLUETOOTH_STATE)
  xTaskNotify(cc_handler,0x05, eSetValueWithOverwrite );

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

  uint16_t c_ign_on = 0;          /* engine on counter */
  uint16_t c_ign_off = 0;         /* engine off counter */
  uint8_t armed = false;
  uint8_t ignition = false;
  uint8_t disarm = false;

  uint8_t dPosS     = DOORCLOSE;    // door Positive State
  uint8_t lastDPosS = DOORCLOSE;    // last door Positive State
  uint8_t dPosR     = DOORCLOSE;    // door Positive reading
  uint32_t lastDPosDebounceTime = 0;  // the last time the output pin was toggled
  uint32_t debounceDDelay = pdMS_TO_TICKS(1000)*2;    // the debounce time; increase if the output flickers


  uint8_t dNegS     = DOORCLOSE;    // door negative state
  uint8_t lastDNegS = DOORCLOSE;    // last door negative state
  uint8_t dNegR     = DOORCLOSE;    // door negative reading
  TickType_t lastDNegDebounceTime = 0;  // the last time the output pin was toggled

  uint8_t disarmState = HIGH;
  uint8_t lastDisarmState = HIGH;

  TickType_t lastDebounceTime = 0;  // the last time the output pin was toggled
  TickType_t debounceDelay = pdMS_TO_TICKS(100);    // the debounce time; increase if the output flickers

  // to aviod blocked at the first time
  dPosS = digitalRead(DoorPositivePin);
  lastDPosS = dPosS;
  dPosR = dPosS;
  dNegS = digitalRead(DoorNegativePin);
  lastDNegS = dNegS;
  dNegR = dNegS;

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
        dPosR = digitalRead(DoorPositivePin);

        if(dPosR != lastDPosS)
        {
          lastDPosDebounceTime = xTaskGetTickCount();
          Serial1.println("PosDoor Change");
        }

        if((xTaskGetTickCount()-lastDPosDebounceTime) > debounceDDelay)
        {
          if(dPosR != dPosS)
          {
            dPosS = dPosR;
            if(dPosS == DOOROPEN)
            {
              updateState(PROTOCOL_STATE);
              xTaskNotify(cc_handler,0x02, eSetValueWithOverwrite );
              armed = false;
            }
          }
        }
        lastDPosS = dPosR;


        // check door status
        dNegR = digitalRead(DoorNegativePin);

        if(dNegR != lastDNegS)
        {
          lastDNegDebounceTime = xTaskGetTickCount();
          Serial1.println("NegDoor Change");
        }
        if((xTaskGetTickCount()-lastDNegDebounceTime) > debounceDDelay)
        {
          if(dNegR != dNegS)
          {
            dNegS = dNegR;
            if(dNegS == DOOROPEN)
            {
              updateState(PROTOCOL_STATE);
              xTaskNotify(cc_handler,0x02, eSetValueWithOverwrite );
              armed = false;
            }
          }
        }
        lastDNegS = dNegR;

          int8_t reading  = !digitalRead(DisarmPin);

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
            int8_t c_disarm = 0;
            int8_t toggle = false;
            int8_t c_add = 0;
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
            dPosS = digitalRead(DoorPositivePin);
            lastDPosS = dPosS;
            dNegS = digitalRead(DoorNegativePin);
            lastDNegS = dNegS;
          }

        }
    } /* end for(;;) */
  }
//-----------------------------------------------------------------------------
static void vJammingTask(void *pvParameters)
{
    uint8_t jammed = false;
    uint16_t c_jammed = 0;

    uint8_t dPosS     = DOORCLOSE;    // door Positive State
    uint8_t lastDPosS = DOORCLOSE;    // last door Positive State
    uint8_t dPosR     = DOORCLOSE;    // door Positive reading
    TickType_t long lastDPosDebounceTime = 0;  // the last time the output pin was toggled
    TickType_t debounceDDelay = pdMS_TO_TICKS(1000)*2;    // the debounce time; increase if the output flickers


    uint8_t dNegS     = DOORCLOSE;    // door negative state
    uint8_t lastDNegS = DOORCLOSE;    // last door negative state
    uint8_t dNegR     = DOORCLOSE;    // door negative reading
    TickType_t lastDNegDebounceTime = 0;  // the last time the output pin was toggled


    uint8_t jammedState = HIGH;
    uint8_t lastJammedState = HIGH;
    TickType_t lastDebounceTime = 0;  // the last time the output pin was toggled
    TickType_t debounceDelay = pdMS_TO_TICKS(500);    // the debounce time; increase if the output flickers

    // to aviod blocked at the first time
    dPosS = digitalRead(DoorPositivePin);
    lastDPosS = dPosS;
    dPosR = dPosS;
    dNegS = digitalRead(DoorNegativePin);
    lastDNegS = dNegS;
    dNegR = dNegS;

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

          // check door status
          dPosR = digitalRead(DoorPositivePin);

          if(dPosR != lastDPosS)
            lastDPosDebounceTime = xTaskGetTickCount();

          if((xTaskGetTickCount()-lastDPosDebounceTime) > debounceDDelay)
          {
            if(dPosR != dPosS)
            {
              dPosS = dPosR;
              if(dPosS == DOOROPEN)
              {
                xTaskNotify(cc_handler,0x04, eSetValueWithOverwrite );
                jammed = false;
              }
            }
          }
          lastDPosS = dPosR;

          // check door status
          dNegR = digitalRead(DoorNegativePin);

          if(dNegR != lastDNegS)
            lastDNegDebounceTime = xTaskGetTickCount();

          if((xTaskGetTickCount()-lastDNegDebounceTime) > debounceDDelay)
          {
            if(dNegR != dNegS)
            {
              dNegS = dNegR;
              if(dNegS == DOOROPEN)
              {
                updateState(JAMMED_STATE);
                xTaskNotify(cc_handler,0x04, eSetValueWithOverwrite );
                jammed = false;
              }
            }
          }
          lastDNegS = dNegR;

          c_jammed++;
          vTaskDelay(pdMS_TO_TICKS(1000));
          if(c_jammed >= TIME_JAMMING_SECURE){
            updateState(JAMMED_STATE);
            xTaskNotify(cc_handler,0x04, eSetValueWithOverwrite );
            jammed = false;
          }
        }
    } /* end for (;;)*/
}
//-----------------------------------------------------------------------------
static void vBlueTask(void *pvParameters)
{
    uint8_t c_data = 0;
    char readingString[BUFF_SIZE];              /* reading string from bluetooth */
    TickType_t xLastDataReceived;
    uint8_t config = pdTRUE;

    readingString[0] = '\0';
    xLastDataReceived = xTaskGetTickCount();

    for  (;;)
    {
       if(config)
       {
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

      if(Serial1.available() > 0)
      {
          readingString[c_data] = Serial1.read();
          readingString[c_data+1] = '\0';
          xLastDataReceived = xTaskGetTickCount();
          c_data++;
          // for overflow
          if(c_data >= BUFF_SIZE)
          {
            readingString[0] = '\0';
            c_data = 0;
          }
      }
      if((xTaskGetTickCount() - xLastDataReceived) > 30)
      {
        c_data = 0;
        vTaskDelay(pdMS_TO_TICKS(300));
      }
      if (strcmp("suspender",readingString) == 0)
      {
        updateState(SUSPEND_STATE);
        xTaskNotify(cc_handler,0x03, eSetValueWithOverwrite );
        readingString[0] = '\0';
      }
      if(strcmp("normal", readingString) == 0)
      {
        updateState(NORMAL_STATE);
        xTaskNotify(cc_handler,0x10, eSetValueWithOverwrite );
        readingString[0] = '\0';
      }
      if (strcmp("bloquear",readingString) == 0)
      {
        updateState(BLUETOOTH_STATE);
        xTaskNotify(cc_handler,0x05, eSetValueWithOverwrite );
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
        int8_t savedState = EEPROM.read(stateAddress);
        Serial1.print("s");
        Serial1.println(savedState);
        readingString[0] = '\0';
      }
      if (strcmp("version",readingString) == 0)
      {
        Serial1.print("v");
        Serial1.println(VERSION);
        readingString[0] = '\0';
      }
      if (strcmp("test",readingString) == 0)
      {
        Serial1.print("Starting Test..\n");
      }
      if(strcmp("OK+CONN", readingString) == 0)
      {
        xTimerStart(xTimerNotification,0);
        xTimerStop(xTimerRestart,0);
        vTaskDelay(configTICK_RATE_HZ*1);
        int8_t savedState = EEPROM.read(stateAddress);
        for(int i = 1; i<=2 ; i++)
        {
          vTaskDelay(configTICK_RATE_HZ);
          Serial1.println();
          Serial1.print("s");
          Serial1.println(savedState);
          Serial1.print("v");
          Serial1.println(VERSION);
          readingString[0] = '\0';
        }
      }
      if(strcmp("OK+LOST", readingString) == 0)
      {
        xTimerStop(xTimerNotification,0);
        xTimerStart(xTimerRestart,0);
        // restart the bluetooth
        digitalWrite(BlueEnablePin,LOW);
        vTaskDelay(configTICK_RATE_HZ);
        digitalWrite(BlueEnablePin,HIGH);
        readingString[0] = '\0';
      }
    } /* end for (;;) */
  }
//------------------------------------------------------------------------------
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
      if( ( ulNotifiedValue == 0x01 ))
      {
        digitalWrite(CCPin, CCOFF);
      }

      /* Activate CC after 1 minute */
      if( ( ulNotifiedValue == 0x02) )
      {
        vTaskSuspend(jammer_handler);
        vTaskSuspend(protocol_handler);
        vTaskSuspend(io_handler);
        int8_t c_open = 0;
        int8_t toggle = false;
        Serial1.println("Door Open");
        do
        {
          digitalWrite(LedPin,toggle);
          c_open++;
          toggle = !toggle;
          Serial1.print((TIME_TO_OPEN_DOOR*2)-c_open);
          Serial1.print(" ");
          vTaskDelay(pdMS_TO_TICKS(1000/2));
        } while(c_open <= (TIME_AFTER_OPEN_DOOR*2));

        digitalWrite(LedPin,LOW);
        digitalWrite(CCPin, CCON);
        vTaskDelay(pdMS_TO_TICKS(500));
        vTaskResume(io_handler);
      }

      /* Deactivate CC and suspend tasks*/
      if( ( ulNotifiedValue == 0x03) )
      {
        vTaskSuspend(jammer_handler);
        vTaskSuspend(protocol_handler);
        digitalWrite(CCPin, CCOFF);
        digitalWrite(LedPin,LOW);
        vTaskDelay(pdMS_TO_TICKS(500));
      }

      /* Activate CC with digital CCPin in a On/Off secuence */
      if( ( ulNotifiedValue == 0x04 ))
      {
        for (int j = 0; j  < 2 ; j++ )
        {
          vTaskSuspend(jammer_handler);
          vTaskSuspend(protocol_handler);
          vTaskSuspend(io_handler);
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
        vTaskResume(io_handler);
      }

      /* Activate CC with and suspend tasks */
      if( ( ulNotifiedValue == 0x05))
      {
        vTaskSuspend(jammer_handler);
        vTaskSuspend(protocol_handler);
        digitalWrite(CCPin, CCON);
        digitalWrite(LedPin,LOW);
        vTaskDelay(pdMS_TO_TICKS(500));
      }
      /* Watchdog System-Reset */
      if( ( ulNotifiedValue == 0x10 ) )
      {
        Serial1.println("System Restart!");
        digitalWrite(CCPin, CCOFF);
        vTaskDelay(pdMS_TO_TICKS(500));
        do{
          wdt_enable(WDTO_15MS);
          for(;;){ }
        } while(0);
      }
    } /* end for(;;) */
  }
//------------------------------------------------------------------------------
static void vIOTask(void *pvParameters)
{
  uint8_t blocked = false;
  uint8_t normal = false;
  uint8_t _last_state ;              /* CCDisable pin last state*/
  uint8_t _reading_state;               /* CCDisable pin reading state */
  uint8_t _current_state;              /* CCDisable pin current state */

  TickType_t lastDebounceTime = 0;
  TickType_t debounceDelay = pdMS_TO_TICKS(500);

    _reading_state = digitalRead(CCDisable);
   _last_state   = _reading_state;

   int8_t savedState = EEPROM.read(stateAddress);

   if(savedState == JAMMED_STATE || savedState == PROTOCOL_STATE || savedState == GPS_STATE || savedState == BLUETOOTH_STATE)
   {
      _last_state     = HIGH;              /* CCDisable pin last state*/
      _reading_state  = HIGH;              /* CCDisable pin reading state */
      _current_state  = HIGH;              /* CCDisable pin current state */
   }
   if(savedState == SUSPEND_STATE || savedState == NORMAL_STATE)
   {
     _last_state     = LOW;              /* CCDisable pin last state*/
     _reading_state  = LOW;              /* CCDisable pin reading state */
     _current_state  = LOW;              /* CCDisable pin current state */
   }

   for(;;)
   {

     int8_t savedState = EEPROM.read(stateAddress);
     if(savedState == JAMMED_STATE || savedState == PROTOCOL_STATE || savedState == GPS_STATE || savedState == BLUETOOTH_STATE)
     {
       blocked = pdTRUE;
     }
     if(savedState == SUSPEND_STATE || savedState == NORMAL_STATE)
     {
       blocked = !pdTRUE;
     }

     _reading_state = digitalRead(CCDisable);
     if(_reading_state != _last_state )
     {
       lastDebounceTime = xTaskGetTickCount();
     }

     if((xTaskGetTickCount() - lastDebounceTime) > debounceDelay)
     {
       if(_reading_state != _current_state)
       {
         _current_state = _reading_state;
       if(_current_state == LOW && blocked){
         updateState(NORMAL_STATE);
         Serial1.println("CC OFF");
         xTaskNotify(cc_handler,0x10, eSetValueWithOverwrite );

       }
       // if CCDisable is LOW then CC ON
       if(_current_state == HIGH && !blocked){
         updateState(GPS_STATE);
         Serial1.println("CC ON");
         xTaskNotify(cc_handler,0x05, eSetValueWithOverwrite );

       }
     }
     }
     _last_state = _reading_state;
   }
}

static void vTestTask(void *pvParameters)
{

  int  reading;         // door positive reading
  bool open;
  bool close;         // door positive closed
  bool ready = false;
  open = 0;
  close = 0;

  int pin[6];

Serial1.println("Test init");
Serial1.println("DoorPositivePin");
Serial1.println("DoorNegativePin");
Serial1.println("DoorIgnitionPin");
Serial1.println("DoorJamDetectionPin");
Serial1.println("DoorDisarmPin");
Serial1.println("DoorCCDisablePin");

pin[0] = DisarmPin;
pin[1] = CCDisable;
pin[2] = DoorPositivePin;
pin[3] = DoorNegativePin;
pin[4] = JamDetectionPin;
pin[5] = IgnitionPin;
int pinNumber = 0;

 for(;;)
   {
     reading = digitalRead(pin[pinNumber]);
     if(reading == HIGH) // door positive closed
      open =  pdTRUE;
     if(reading == LOW)  // door positive opened
      close =  pdTRUE;

      if(open && close)
      {
        Serial1.print("Pin ");
        Serial1.print(pinNumber);
        Serial1.print("/");
        Serial1.print(sizeof(pin)/sizeof(pin[0]));
        Serial1.println(" OK");
        open = false;
        close = false;
        pinNumber++;

        if(pinNumber == sizeof(pin)/sizeof(pin[0]))
          pinNumber = 0;
      }
   }
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
  Serial1.print("s");
  Serial1.println(savedState);
  Serial1.print("v");
  Serial1.println(VERSION);
}

void vTimerRestart( TimerHandle_t xTimer )
{
  // restart the bluetooth
  digitalWrite(BlueEnablePin,LOW);
  vTaskDelay(configTICK_RATE_HZ);
  vTaskDelay(configTICK_RATE_HZ);
  digitalWrite(BlueEnablePin,HIGH);

  int16_t savedState = EEPROM.read(stateAddress);
  configASSERT( pxTimer );

  if(digitalRead(IgnitionPin) || savedState == PROTOCOL_STATE || savedState == JAMMED_STATE || savedState == GPS_STATE || savedState == BLUETOOTH_STATE) // if ignition off
    c_off++;
  else
    c_off = 0;

  if(c_off >= TIME_RESET_HW)
    restartHW();
}

void updateState(int8_t _state)
{
  EEPROM.update(stateAddress, _state);
  Serial1.print("s");
  Serial1.println(_state);
}
void vTimerMemoryCheckCallback( TimerHandle_t xTimer )
{
  /* Inspect our own high water mark on entering the task. */
  UBaseType_t uxHighWaterMark;
  uxHighWaterMark = uxTaskGetStackHighWaterMark( protocol_handler );
  Serial.println("xStack Remain  ");
  Serial.print("xProtocolTask ");
  Serial.println(uxHighWaterMark);
  uxHighWaterMark = uxTaskGetStackHighWaterMark( jammer_handler );
  Serial.print("xJammingTask ");
  Serial.println(uxHighWaterMark);
  uxHighWaterMark = uxTaskGetStackHighWaterMark( cc_handler );
  Serial.print("xCCTask ");
  Serial.println(uxHighWaterMark);
  uxHighWaterMark = uxTaskGetStackHighWaterMark( blue_handler );
  Serial.print("xBlueTask ");
  Serial.println(uxHighWaterMark);
}
