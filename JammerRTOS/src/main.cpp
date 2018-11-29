// Simple demo of three threads
// LED blink thread, print thread, and idle loop
#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
//#include <semphr.h>  // add the FreeRTOS functions for Semaphores (or Flags).
//#include <FreeRTOS_AVR.h>





//#define DEBUG 1
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
#define SpeakerPin             16

#define TIME_AFTER_START       15
#define TIME_AFTER_STOP        10
#define TIME_TO_STOP_ENGINE    40
#define TIME_TO_OPEN_DOOR      30
#define DOOR_ENABLE_SECONDS    7
#define NOTIFICATION_TIME      30
#define TIME_AFTER_OPEN_DOOR   60 // secs after the door was opened
#define TIME_JAMMING_SECURE    120
#define TIME_JAMMING_HANG      30*SECS_PER_MIN
#define TIME_JAMMING_AFTER_SECURE 120

TaskHandle_t cc_handler;
TaskHandle_t jammer_handler;
TaskHandle_t disable_handler;
TaskHandle_t protocol_handler;
TaskHandle_t ignition_handler;
TaskHandle_t dpos_handler;
TaskHandle_t dneg_handler;
TaskHandle_t passwd_handler;


void time(long val);
void printDigits(byte digits);
int checkPinStatus(uint8_t _pin, uint8_t lastButtonState);


static void vDoorPosTask(void *pvParameters);
static void vDoorNegTask(void *pvParameters);
static void vProtocolTask(void *pvParameters);
static void vDisableTask(void *pvParameters);
static void vIgnitionNotification(void *pvParameters);
static void vJammingTask(void *pvParameters);
//static void vPasswordTask(void *pvParameters);
static void vCCTask(void *pvParameters);


void vApplicationMallocFailedHook( void ){

  Serial.println("faliedhook Memory");
}
void vApplicationStackOverflowHook( TaskHandle_t xTask, signed char *pcTaskName ){

  Serial.println(*pcTaskName);
}

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
  //Serial1.begin(9600);

  xTaskCreate(vDisableTask,"Disable",configMINIMAL_STACK_SIZE+120-25-10-10-10 ,NULL,tskIDLE_PRIORITY,&disable_handler);
  xTaskCreate(vCCTask,"CC",configMINIMAL_STACK_SIZE+62-32+10, NULL, tskIDLE_PRIORITY,&cc_handler);
  xTaskCreate(vIgnitionNotification,"Ignition",configMINIMAL_STACK_SIZE+31+10+10,NULL,tskIDLE_PRIORITY,&ignition_handler);
  xTaskCreate(vProtocolTask,"Protocol",configMINIMAL_STACK_SIZE+62+20,NULL,tskIDLE_PRIORITY,&protocol_handler);
  xTaskCreate(vJammingTask,"Jamming",configMINIMAL_STACK_SIZE+31+20, NULL, tskIDLE_PRIORITY , &jammer_handler);
//  xTaskCreate(vPasswordTask,"Password",configMINIMAL_STACK_SIZE+25+10+10, NULL, tskIDLE_PRIORITY ,&passwd_handler);
  xTaskCreate(vDoorNegTask,"Doorneg",configMINIMAL_STACK_SIZE+16+11, NULL, tskIDLE_PRIORITY ,&dneg_handler);
  xTaskCreate(vDoorPosTask,"Doorpos",configMINIMAL_STACK_SIZE+16+11, NULL, tskIDLE_PRIORITY ,&dpos_handler);


  pinMode(CCPin, OUTPUT);
  pinMode(LedPin, OUTPUT);
  pinMode(BuzzerPin, OUTPUT);
  pinMode(EnableDoorPin, INPUT_PULLUP);
  pinMode(DoorPin, INPUT_PULLUP);
  pinMode(JamDetectionPin, INPUT_PULLUP);
  pinMode(DoorPositivePin, INPUT_PULLUP);
  pinMode(IgnitionPin, INPUT_PULLUP);
  pinMode(CCDisable, INPUT_PULLUP);

  //vTaskStartScheduler();

  // should never return
  //Serial.println(F("Die"));
  //while(1);
  }

  //------------------------------------------------------------------------------
  // WARNING idle loop has a very small stack (configMINIMAL_STACK_SIZE)
  // loop must never block
  void loop() {
  }

//-------------------------------------------------------------------------------------
static void vProtocolTask(void *pvParameters){

    uint32_t ulNotifiedValue = 0x00;
    TickType_t xFrequency = pdMS_TO_TICKS(10);
    int8_t k=0;

    /* Inspect our own high water mark on entering the task. */


      for(;;) {

      xTaskNotifyWait( 0x00,      /* Don't clear any notification bits on entry. */
      0xF0, /* Reset the notification value to 0 on exit. */
      &ulNotifiedValue, /* Notified value pass out in
      ulNotifiedValue. */
      xFrequency  );  /* Block indefinitely. */
      /* Inspect our own high water mark on entering the task. */



      // block until notification
      if( ( ulNotifiedValue == 0x02  ) )
      {

        Serial.print("Secure Protocol Stoped!\n");
        digitalWrite(LedPin, LOW);
        k = 0;
        xFrequency = portMAX_DELAY;

      }

      //// secure protocol enable
      if( ( ulNotifiedValue == 0x01 ) )
      {

        #ifdef DEBUG
        if(k == 0)
          Serial.print("Secure Protocol Started!\n");
        k = 1;
        #endif


        digitalWrite(LedPin,HIGH);
        xFrequency = pdMS_TO_TICKS(10);

        // if the button is pressed
        if(digitalRead(EnableDoorPin) == LOW){

          #ifdef DEBUG
          Serial.print("vProtcolTask Suspended  ");
          Serial.print(TIME_TO_OPEN_DOOR);
          Serial.print(" seconds ");
          #endif

          xTaskNotify(dneg_handler,0x01, eSetValueWithOverwrite);
          xTaskNotify(dpos_handler,0x01, eSetValueWithOverwrite);

          int count = 0;
          int i = 0;
          do{

            #ifdef DEBUG
              if(i % 10 == 0){
               Serial.print((TIME_TO_OPEN_DOOR+count)-i);
               Serial.print(" ");
            }
            #endif

            digitalWrite(LedPin,LOW);
            vTaskDelay(pdMS_TO_TICKS(1000));
            i++;
            digitalWrite(LedPin,HIGH);
            vTaskDelay(pdMS_TO_TICKS(1000));
            i++;
            if(digitalRead(EnableDoorPin) == LOW && count <=50){
              count += 10;
            }

          } while(i < TIME_TO_OPEN_DOOR+count);

          xTaskNotify(dneg_handler,0x00, eSetValueWithOverwrite);
          xTaskNotify(dpos_handler,0x00, eSetValueWithOverwrite);
          #ifdef DEBUG
            Serial.println("vProtocolTask Resumed ");
          #endif

        } // end EnableDoor
      } // end secure protocol

      #ifndef DOORSENSOR
     if(digitalRead(DoorPin) == LOW || digitalRead(DoorPositivePin) == LOW){
     #endif
     #ifdef DOORSENSOR
     if(digitalRead(DoorPin) == HIGH){
     #endif

      if(  ulNotifiedValue == 0x11 ){ // if door y s open and protocol is up

          digitalWrite(LedPin, LOW);
          #ifdef DEBUG
          Serial.print("vProtocolTask Suspended ");
          #endif

          for(int j = 0; j < TIME_AFTER_OPEN_DOOR ; j++){
            #ifdef DEBUG
             if (j % 10 == 0){
              Serial.print(j);
              Serial.print(" ");
            }
            #endif
            vTaskDelay(pdMS_TO_TICKS(1000));
            digitalWrite(LedPin,HIGH);
            vTaskDelay(pdMS_TO_TICKS(1000));
            digitalWrite(LedPin,LOW);
          }
          digitalWrite(LedPin,LOW);

          #ifdef DEBUG
          Serial.println("vCCTask notification");
          #endif
          xTaskNotify(cc_handler,0x03, eSetValueWithOverwrite );
          vTaskDelay(pdMS_TO_TICKS(TIME_AFTER_STOP*1000));
          ulNotifiedValue = 0x02;
        }
      }
    }
  }

  //------------------------------------------------------------------------------
static void vDisableTask(void *pvParameters) {

  uint32_t ulNotifiedValue = 0x01;
  boolean _last_state = HIGH;
  boolean _reading_state = HIGH;
  boolean _current_state = HIGH;
  boolean _current_statei = LOW;
  boolean suspend = false;

  uint32_t randomNumber;

  boolean suspendState = false;
  boolean checkState = false;


  char val[10];
  char val2;
  char password[10] = "jaime";

  int i = 0;
  TickType_t xTimeLow;
  int16_t lastDebounceTime = 0;  // the last time the output pin was toggled
  int16_t debounceDelay = pdMS_TO_TICKS(500);    // the debounce time; increase if the output flickers

  int16_t count =0;
  int16_t limit = 0;



  for  (;;) {


          count = 0;
          limit = random(45,120);
    while(digitalRead(EnableDoorPin) == LOW){

    #ifdef DEBUG
    if((limit-count) % 20  == 0){
      Serial.print(limit-count);
      Serial.print(" ");
    }
    #endif

    /* when reach limit, suspend */
    if(count == limit){
      if (suspendState)
      {
        digitalWrite(BuzzerPin, HIGH);
        vTaskDelay(pdMS_TO_TICKS(50));
        digitalWrite(BuzzerPin, LOW);
        vTaskDelay(pdMS_TO_TICKS(100));
        digitalWrite(BuzzerPin, HIGH);
        vTaskDelay(pdMS_TO_TICKS(50));
        digitalWrite(BuzzerPin, LOW);
        vTaskResume(cc_handler);
        vTaskDelay(pdMS_TO_TICKS(500));
        xTaskNotify(cc_handler,0x01, eSetValueWithOverwrite );
        suspendState = false;
        count = 0;
      }
      else
      {
        digitalWrite(BuzzerPin, HIGH);
        vTaskDelay(pdMS_TO_TICKS(100));
        digitalWrite(BuzzerPin, LOW);
        xTaskNotify(cc_handler,0x01, eSetValueWithOverwrite );
        vTaskDelay(pdMS_TO_TICKS(500));
        vTaskSuspend(cc_handler);


        suspendState = true;
        count = 0;
      }
  }

  count++;
  vTaskDelay(pdMS_TO_TICKS(80));
}



    vTaskDelay(pdMS_TO_TICKS(10));

        _reading_state = digitalRead(CCDisable);

        if(_reading_state != _last_state ){
          lastDebounceTime = xTaskGetTickCount();
      }

      if (( xTaskGetTickCount() - lastDebounceTime) > debounceDelay){

        if (_reading_state != _current_state ){
          _current_state = _reading_state;


      if(_current_state == HIGH){ // desactiva el cc
        #ifdef DEBUG
          Serial.println("CC OFF by SAFECAR");
        #endif
        xTaskNotify(cc_handler,0x01, eSetValueWithOverwrite );
      }

      if(_current_state == LOW){ // se activa el cc

        #ifdef DEBUG
          Serial.println("CC ON by SAFECAR");
        #endif
        xTaskNotify(cc_handler,0x03, eSetValueWithOverwrite );
      }
    }
  }

  _last_state = _reading_state;


/*
      while(Serial1.available()){
        val2 = Serial1.read();
        if (val2 != '\r' && val2 != '\n'){
          val[i] = val2;
          i++;
        }
        //erial1.println(val2);
        vTaskDelay(configTICK_RATE_HZ/10);
      }
      val[i] = NULL;

     if(i > 0){
       Serial.println(password);
       Serial.println(val);

       if (strcmp (password,val) == 0){

         Serial.println("Correct answer!");

         if(!suspend){

           Serial.print("Suspend\n\r");
           xTaskNotify(cc_handler,( 1UL << 3UL ), eSetBits );
           vTaskSuspend( cc_handler );
           vTaskSuspend( ignition_handler );
           vTaskSuspend( protocol_handler );
           xTaskNotify(ignition_handler,( 1UL << 0UL ), eSetBits );
           suspend = true;
         }
        else{
          Serial.print("Resume\n\r");
          vTaskResume( cc_handler );
      //  vTaskResume( jammer_handler );
          vTaskResume( ignition_handler );
          vTaskResume( protocol_handler );
          suspend = false;
       }
     }

      else {
        vTaskDelay(configTICK_RATE_HZ/5);
        Serial.println("wrong!");
        }
        i = 0;
      }

    */}
  }

  //------------------------------------------------------------------------------
  static void vIgnitionNotification(void *pvParameters) {

    long c_engine_on = 0;
    long c_engine_off = 0;
    TickType_t xLastWakeTime;
    // Initialise the xLastWakeTime variable with the current time.
    xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000);




    for  (;;) {


      vTaskDelayUntil(&xLastWakeTime, xFrequency);

      if(digitalRead(IgnitionPin) == LOW){

        #ifndef DOORSENSOR
        // sensor de puerta de auto
        if(digitalRead(DoorPin) == LOW || digitalRead(DoorPositivePin) == LOW){
        #endif
        #ifdef DOORSENSOR
        // sensor de puerta magnetico
        if(digitalRead(DoorPin) == HIGH){
        #endif


        digitalWrite(BuzzerPin, HIGH);
        vTaskDelay(pdMS_TO_TICKS(50));
        digitalWrite(BuzzerPin, LOW);
      }

      #ifdef DEBUG

      if(c_engine_on <= TIME_AFTER_START){
        if(c_engine_on % 5 == 0){
        Serial.print("Ignition ON ");
        time(c_engine_on);
      }
      }
      else
        if( c_engine_on % 60*5 == 0 ){
          Serial.print("Ignition ON ");
          time(c_engine_on);
        }

        if(c_engine_on == TIME_AFTER_START){
          Serial.println("Protocol Task notification");
        }
        #endif


        if(c_engine_on == TIME_AFTER_START){
          xTaskNotify(protocol_handler,0x01, eSetValueWithOverwrite );
        }
        c_engine_on++;
        c_engine_off = 0;

      }

      else { // ENGINE OFF

        #ifdef DEBUG
        if(c_engine_off <= TIME_AFTER_STOP){
          Serial.print("Ignition OFF ");
          time(c_engine_off);
        }
        else
          if(c_engine_off % 60 == 0){
            Serial.print("Ignition OFF ");
            time(c_engine_off);
          }

          if(c_engine_off == TIME_AFTER_STOP){
            Serial.println("Protocol Task notification");
          }
          #endif

        if(c_engine_off == TIME_AFTER_STOP){
          xTaskNotify(protocol_handler,0x02, eSetValueWithOverwrite );
        }

        c_engine_on = 0;
        c_engine_off++;
      }

    }

  }


  //------------------------------------------------------------------------------
  static void vJammingTask(void *pvParameters) {

    uint32_t c_jammer = 0;
    TickType_t xFrequency;
    uint32_t ulNotifiedValue;


    xFrequency = pdMS_TO_TICKS(1000);

    /* Inspect our own high water mark on entering the task. */

    for(;;) {
      // Sleep for one second.
      xTaskNotifyWait( 0x00,      /* Don't clear any notification bits on entry. */
      0xFF, /* Reset the notification value to 0 on exit. */
      &ulNotifiedValue, /* Notified value pass out in
      ulNotifiedValue. */
      xFrequency  );  /* Block indefinitely. */




      /* jamming detection */
      if(digitalRead(JamDetectionPin) == LOW)
      {

      // redundance door detector
      #ifndef DOORSENSOR // for automotive sensor
        if(digitalRead(DoorPin) == LOW || digitalRead(DoorPositivePin) == LOW){
     #endif
      #ifdef DOORSENSOR // for magnetico sensor
        if(digitalRead(DoorPin) == HIGH){
      #endif

      /* notification from door debounce task*/
        if( ulNotifiedValue == 0x01 )
        {
          #ifdef DEBUG
            Serial.println("vCCTask notification");
          #endif

          xTaskNotify(cc_handler,0x03, eSetValueWithOverwrite );
        }

      } // rendundance door detector

      if (c_jammer == TIME_JAMMING_SECURE ){
          #ifdef DEBUG
            Serial.println("vCCTask notification ");
          #endif
            xTaskNotify(cc_handler,0x02, eSetValueWithOverwrite );
        }

        if (c_jammer == TIME_JAMMING_HANG){
            xTaskNotify(cc_handler,0x01, eSetValueWithOverwrite );
            #ifdef DEBUG
              Serial.println("vJammingTask Suspended 10min ");
            #endif
            for (int i ; i<10*60; i++)
            vTaskDelay(pdMS_TO_TICKS(1000));

            #ifdef DEBUG
              Serial.println("vJammingTask Resumed");
            #endif
        }


        #ifdef DEBUG
          if( (c_jammer <= TIME_JAMMING_SECURE)  && (c_jammer % 30 == 0)  ){
            Serial.print("Jammer Secure ON ");
            time(c_jammer);
          }
          if ( c_jammer > TIME_JAMMING_SECURE )
            if( (c_jammer % (60*5) == 0 )){
              Serial.print("Jammer Hang ON ");
              time(c_jammer);
            }
        #endif
      c_jammer++;
      }
      else{
      c_jammer = 0;
      }
    }
  }

//-----------------------------------------------------------------------------
static void vPasswordTask(void *pvParameters) {

  boolean suspendState = false;
  int16_t count,limit;

  for (;;) {

        count = 0;
        limit = random(45,120);
        while(digitalRead(EnableDoorPin) == LOW){

        #ifdef DEBUG
        if((limit-count) % 20  == 0){
          Serial.print(limit-count);
          Serial.print(" ");
        }
        #endif

          /* when reach limit, suspend */
          if(count == limit){
            if (suspendState)
            {
              digitalWrite(BuzzerPin, HIGH);
              vTaskDelay(pdMS_TO_TICKS(20));
              digitalWrite(BuzzerPin, LOW);
              vTaskDelay(pdMS_TO_TICKS(100));
              digitalWrite(BuzzerPin, HIGH);
              vTaskDelay(pdMS_TO_TICKS(20));
              digitalWrite(BuzzerPin, LOW);
              vTaskResume(cc_handler);
              xTaskNotify(cc_handler,0x04, eSetValueWithOverwrite );

              suspendState = false;
              count = 0;
            }
            else
            {
              digitalWrite(BuzzerPin, HIGH);
              vTaskDelay(pdMS_TO_TICKS(100));
              digitalWrite(BuzzerPin, LOW);
              xTaskNotify(cc_handler,0x01, eSetValueWithOverwrite );
              vTaskSuspend(cc_handler);


              suspendState = true;
              count = 0;
            }
        }

        count++;
        vTaskDelay(pdMS_TO_TICKS(80));
      }
      vTaskDelay(pdMS_TO_TICKS(1000));

    }
  }
  //------------------------------------------------------------------------------
  static void vDoorNegTask(void *pvParameters) {

  #ifndef DOORSENSOR
  bool doorState = HIGH;
  bool lastDoorState = HIGH;   // the previous reading from the input pin
  bool reading = HIGH;
  #endif
  #ifdef DOORSENSOR
  bool doorState = LOW;
  bool lastDoorState = LOW;   // the previous reading from the input pin
  bool reading = LOW;

  #endif
  int16_t lastDebounceTime = 0;  // the last time the output pin was toggled
  int16_t debounceDelay = pdMS_TO_TICKS(1000);    // the debounce time; increase if the output flickers
  BaseType_t xReturned;
  TickType_t xFrequency = pdMS_TO_TICKS(100);
  uint32_t ulNotifiedValue = 0x00;



      for (;;) {

        xTaskNotifyWait( 0x00,      /* Don't clear any notification bits on entry. */
        0x00, /* Reset the notification value to 0 on exit. */
        &ulNotifiedValue, /* Notified value pass out in
        ulNotifiedValue. */
        xFrequency  );  /* Block indefinitely. */

        reading = digitalRead(DoorPin);

        if (reading != lastDoorState) {
          // reset the debouncing timer
          lastDebounceTime = xTaskGetTickCount();
        }

        if (( xTaskGetTickCount() - lastDebounceTime) > debounceDelay){

          if (reading != doorState ) {
              doorState = reading;
              #ifndef DOORSENSOR // automotive sensor
              // if the door is open
                if(doorState == LOW){
              #endif

              #ifdef DOORSENSOR // magenitc sensor
              // if the door is open
                if(doorState == HIGH){
              #endif

              /*
              #ifdef DEBUG
              Serial.print("Door open\n");
              #endif
              */if(ulNotifiedValue != 0x01){
                xReturned =  xTaskNotify(protocol_handler,( 1UL << 4UL ), eSetBits );
                #ifdef DEBUG
                  if(xReturned == pdPASS){
                    Serial.println("Dooropen vProtocolTask notification");
                  }
                #endif
              }
                xReturned = xTaskNotify(jammer_handler,0x01, eSetValueWithOverwrite);
                #ifdef DEBUG
                  if(xReturned == pdPASS){
                    Serial.println("Dooropen vJammingTask notification");
                  }
                #endif

                } // end if door open

                else { // if the door is close
                  #ifdef DEBUG
                  Serial.print("Door closed\n");
                  #endif
                }
            }
          }

        lastDoorState = reading;
      }
  }
  //------------------------------------------------------------------------------
  //------------------------------------------------------------------------------
  static void vDoorPosTask(void *pvParameters) {

  bool doorState = HIGH;
  bool lastDoorState = HIGH;   // the previous reading from the input pin
  bool reading = HIGH;
  bool doorStatePos = HIGH;

 uint32_t ulNotifiedValue = 0x00;

 int16_t lastDebounceTime = 0;  // the last time the output pin was toggled
 int16_t debounceDelay = pdMS_TO_TICKS(1000);    // the debounce time; increase if the output flickers
 BaseType_t xReturned;
 TickType_t xFrequency = pdMS_TO_TICKS(100);

      for (;;) {

        xTaskNotifyWait( 0x00,      /* Don't clear any notification bits on entry. */
        0x00, /* Reset the notification value to 0 on exit. */
        &ulNotifiedValue, /* Notified value pass out in
        ulNotifiedValue. */
        xFrequency  );  /* Block indefinitely. */

        reading = digitalRead(DoorPositivePin);

        if (reading != lastDoorState) {
          // reset the debouncing timer
          lastDebounceTime = xTaskGetTickCount();
        }

        if (( xTaskGetTickCount() - lastDebounceTime) > debounceDelay){

          if (reading != doorState ) {
              doorState = reading;
              #ifndef DOORSENSOR // automotive sensor
              // if the door is open
                if(doorState == LOW){
              #endif

              #ifdef DOORSENSOR // magenitc sensor
              // if the door is open
                if(doorState == HIGH){
              #endif

              #ifdef DEBUG
                Serial.print("Door open\n");

              #endif
              if(ulNotifiedValue != 0x01){
              xReturned =  xTaskNotify(protocol_handler,( 1UL << 4UL ), eSetBits );
              #ifdef DEBUG
                if(xReturned == pdPASS){
                  Serial.println("Dooropen vProtocolTask notification");
                }
              #endif
            }
              xReturned = xTaskNotify(jammer_handler,0x01, eSetValueWithOverwrite);
              #ifdef DEBUG
                if(xReturned == pdPASS){
                  Serial.println("Dooropen vJammingTask notification");
                }
              #endif

                } // end if door open

                else { // if the door is close
                  #ifdef DEBUG
                  Serial.print("Door closed\n");
                  #endif
                }
            }
          }

        lastDoorState = reading;
      }
  }
  //------------------------------------------------------------------------------


  static void vCCTask(void *pvParameters) {
  uint32_t ulNotifiedValue = 0x00;
  TickType_t xFrequency = pdMS_TO_TICKS(500);
  UBaseType_t uxHighWaterMark;


      for (;;) {
      #ifdef DEBUG
        uxHighWaterMark = uxTaskGetStackHighWaterMark(ignition_handler  );
        Serial.print("ingnition word free ");
        Serial.println(uxHighWaterMark);
      #endif

      xTaskNotifyWait( 0x00,      /* Don't clear any notification bits on entry. */
      0xFF, /* Reset the notification value to 0 on exit. */
      &ulNotifiedValue, /* Notified value pass out in
      ulNotifiedValue. */
      xFrequency  );  /* Block indefinitely. */


      // unblock forever
      if( ( ulNotifiedValue == 0x01 ) )
      {

          digitalWrite(CCPin, LOW);
          #ifdef DEBUG
          Serial.println("CC = OFF blocked");
          #endif
      }

      if( ( ulNotifiedValue == 0x02 ))
      {

        TickType_t xFrequency = configTICK_RATE_HZ/10;

        for(int i=0; i<2;i++){
          digitalWrite(CCPin, HIGH);
          vTaskDelay(xFrequency);
        }


        for(int i=0; i<20;i++){
          digitalWrite(CCPin, LOW);
          vTaskDelay(xFrequency);
        }

        for(int i=0; i<2;i++){
          digitalWrite(CCPin, HIGH);
          vTaskDelay(xFrequency);
        }

        for(int i=0; i<20;i++){
          digitalWrite(CCPin, LOW);
          vTaskDelay(xFrequency);
        }
        #ifdef DEBUG
        Serial.println("CC = ON");
        #endif
        digitalWrite(CCPin, HIGH);

      }


      if( ( ulNotifiedValue == 0x03) )
      {
        #ifdef DEBUG
        Serial.println("CC = ON blocked");
        #endif
        digitalWrite(CCPin, HIGH);
      }

    }
  }
//---------------------------------------------------------------------------


//---------------------------------------------------------------------------


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
