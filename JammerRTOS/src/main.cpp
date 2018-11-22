// Simple demo of three threads
// LED blink thread, print thread, and idle loop
#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
//#include <semphr.h>  // add the FreeRTOS functions for Semaphores (or Flags).
//#include <FreeRTOS_AVR.h>
#include <queue.h>


#define DEBUG 1
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

#define TIME_AFTER_START       30
#define TIME_AFTER_STOP        30
#define TIME_TO_STOP_ENGINE    40
#define TIME_TO_OPEN_DOOR      30
#define DOOR_ENABLE_SECONDS    7
#define NOTIFICATION_TIME      30
#define TIME_AFTER_OPEN_DOOR   60 // secs after the door was opened


TaskHandle_t cc_handler;
TaskHandle_t jammer_handler;
TaskHandle_t disable_handler;
TaskHandle_t protocol_handler;
TaskHandle_t ignition_handler;
TaskHandle_t iputs_handler;

QueueHandle_t xDoorQueue;

void time(long val);
void printDigits(byte digits);
int checkPinStatus(uint8_t _pin, uint8_t lastButtonState);



static void vProtocolTask(void *pvParameters) {


  uint32_t ulNotifiedValue = 0x00;
  TickType_t xFrequency = pdMS_TO_TICKS(10);
  int8_t k=0;

    for(;;) {

    xTaskNotifyWait( 0x00,      /* Don't clear any notification bits on entry. */
    0x00, /* Reset the notification value to 0 on exit. */
    &ulNotifiedValue, /* Notified value pass out in
    ulNotifiedValue. */
    xFrequency  );  /* Block indefinitely. */


    // block until notification
    if( ( ulNotifiedValue == 0x02  ) ){
      Serial.print("Secure Protocol Stoped!\n");
      digitalWrite(LedPin, LOW);
      k = 0;
      xFrequency = portMAX_DELAY;
      }

    //
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
        Serial.print("Protcol Disable during ");
        Serial.print(TIME_TO_OPEN_DOOR);
        Serial.println(" seconds");
        #endif

        for(int i = 0; i < TIME_TO_OPEN_DOOR; i++ ){
          digitalWrite(LedPin,HIGH);
          vTaskDelay(pdMS_TO_TICKS(30));
          digitalWrite(LedPin,LOW);
          vTaskDelay(pdMS_TO_TICKS(1000));
          digitalWrite(LedPin,HIGH);
        } // end for


        #ifdef DEBUG
        Serial.print("Protcol Disable during ");
        Serial.print(TIME_TO_OPEN_DOOR*4);
        Serial.println(" seconds");
        #endif

          for(int i = 0; i < TIME_TO_OPEN_DOOR*4; i++ ){
          digitalWrite(LedPin,HIGH);
          vTaskDelay(pdMS_TO_TICKS(30));
          digitalWrite(LedPin,LOW);
          vTaskDelay(pdMS_TO_TICKS(500));
          digitalWrite(LedPin,HIGH);
        } // end for
        }

      }

      if( ( ulNotifiedValue & 0x10 ) != 0){ // if door y s open and protocol is up

        digitalWrite(LedPin, LOW);
        Serial.print("Secure Protocol in course!\n");
        Serial.print("CC on in ");

        for(int j = 0; j < TIME_AFTER_OPEN_DOOR ; j++){
          Serial.print(j);
          Serial.print(" ");
          vTaskDelay(configTICK_RATE_HZ/10);
          digitalWrite(LedPin,HIGH);
          vTaskDelay(configTICK_RATE_HZ/20);
          digitalWrite(LedPin,LOW);
        }
            xTaskNotify(cc_handler,( 1UL << 4UL ), eSetBits );
            vTaskDelay(configTICK_RATE_HZ*TIME_AFTER_START);
      }
  }
}

//------------------------------------------------------------------------------
static void vDisableTask(void *pvParameters) {
//attachInterrupt(digitalPinToInterrupt(CCDisable), CCInterrupt, CHANGE );
  uint32_t ulNotifiedValue = 0x01;
  boolean _last_state = HIGH;
  boolean _current_state = HIGH;
  boolean _current_statei = LOW;
  boolean suspend = false;


  char val[10];
  char val2;
  char password[10] = "jaime";

  int i = 0;
  TickType_t xTimeLow;

  xTimeLow = xTaskGetTickCount();
  for  (;;) {

      _current_state = digitalRead(CCDisable);

      if(_current_state != _last_state ){


      if(_current_state == HIGH){ // desactiva el cc
        #ifdef DEBUG
          Serial.println("CC OFF by SAFECAR");
        #endif
        xTaskNotify(cc_handler,0x08, eSetBits );
      }

      if(_current_state == LOW){ // se activa el cc


        #ifdef DEBUG
          Serial.println("CC ON by SAFECAR");
        #endif

        xTaskNotify(cc_handler,0x10, eSetBits );


      }

      _last_state = _current_state;
    }

    if( ( ulNotifiedValue & 0x02 ) != 0 ){
      _current_statei = digitalRead(CCDisable);
      if(_current_statei != _last_state){
        #ifdef DEBUG
        Serial.println("Inverted CC Disable by SAFECAR");
        #endif
        if(digitalRead(CCDisable) == LOW){ // desactiva el cc
        xTaskNotify(cc_handler,( 1UL << 3UL ), eSetBits );
        ulNotifiedValue = 0x01;
        }
      if(digitalRead(CCDisable) == HIGH){ // se activa el cc
        #ifdef DEBUG
        Serial.print(" CC on  by SAFECAR ");
        #endif
        xTaskNotify(cc_handler,( 1UL << 4UL ), eSetBits );
        }
      }
    }

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
  }
}

//------------------------------------------------------------------------------
static void vIgnitionNotification(void *pvParameters) {

  long c_engine_on = 0;
  long c_engine_off = 0;
  TickType_t xLastWakeTime;


  // Initialise the xLastWakeTime variable with the current time.
  xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = configTICK_RATE_HZ/10;

  for  (;;) {
    // Wait for the next cycle.

    vTaskDelayUntil( &xLastWakeTime, xFrequency );

    if(digitalRead(IgnitionPin) == LOW){

      #ifdef DEBUG

    if(c_engine_on <= TIME_AFTER_START){
      Serial.print("Ignition ON ");
      time(c_engine_on);
    }
    else
      if(c_engine_on%60==0){
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

  int16_t c_jammer = 0;

  for(;;) {
    // Sleep for one second.
    vTaskDelay(configTICK_RATE_HZ);

    if(digitalRead(JamDetectionPin) == LOW){
      c_jammer++;
      #ifndef DOORSENSOR
      if(digitalRead(DoorPin) == LOW || digitalRead(DoorPositivePin) == LOW){
      #endif
      #ifdef DOORSENSOR
      if(digitalRead(DoorPin) == HIGH){
      #endif
        // se puede esperar un tiempo antes apagar el motor
        xTaskNotify(cc_handler,( 1UL << 0UL ), eSetBits );
        vTaskSuspend( NULL );
      }
      if (c_jammer >= 120){
        c_jammer = 0;

        xTaskNotify(cc_handler,( 1UL << 1UL ), eSetBits );
       vTaskSuspend( NULL );

      }
      Serial.print("Jammer Alert secs");
      Serial.println(c_jammer);
    }
    else{
    c_jammer = 0;
    }
  }
}
//-----------------------------------------------------------------------------
static void vPasswordTask(void *pvParameters) {

    for (;;) {
      Serial.println("password");
  }
}
//------------------------------------------------------------------------------
static void vDoorTask(void *pvParameters) {
uint32_t ulNotifiedValue = 0x00;
#ifndef DOORSENSOR
bool doorStateNeg = HIGH;
bool lastDoorNegState = HIGH;   // the previous reading from the input pin
bool readingNeg = HIGH;
bool doorStatePos = HIGH;
bool lastDoorPosState = HIGH;   // the previous reading from the input pin
bool readingPos = HIGH;
#endif
#ifdef DOORSENSOR
bool doorStateNeg = LOW;
bool lastDoorNegState = LOW;   // the previous reading from the input pin
bool readingNeg = LOW;
bool doorStatePos = LOW;
bool lastDoorPosState = LOW;   // the previous reading from the input pin
bool readingPos = LOW;

#endif
int i = 0;
int j = 0;



xDoorQueue = xQueueCreate( 10, sizeof( int8_t ) );


    for (;;) {

      readingNeg = digitalRead(DoorPin);
      readingPos = digitalRead(DoorPositivePin);

      if (readingNeg != lastDoorNegState) {
            doorStateNeg = checkPinStatus(DoorPin,lastDoorNegState);
      }
      else {
      if (readingPos != lastDoorPosState) {
            doorStatePos = checkPinStatus(DoorPositivePin,lastDoorPosState);
      }
    }

      lastDoorNegState = doorStateNeg;
      lastDoorPosState = doorStatePos;

      #ifndef DOORSENSOR
      // if the door is open
        if(doorStateNeg == LOW ||doorStatePos == LOW){
      #endif

      if( xQueue1 != 0 )
{
    /* Send an unsigned long.  Wait for 10 ticks for space to become
    available if necessary. */
    xQueueSend( xQueue1,1 ,0);
}


      #ifdef DOORSENSOR

        if(doorStateNeg == HIGH){
      #endif
      #ifdef DEBUG
        if(i == 0){Serial.print("Door open\n"); i++; j=0;}
      #endif

      // if the door is open here send notification
      xTaskNotify(ignition_handler, 0x04, eSetBits);



        }


        else { // if the door is close
          #ifdef DEBUG
          if(j == 0){Serial.print("Door closed\n");j++; i=0;}
          #endif

          if( xQueue1 != 0 )
    {
        /* Send an unsigned long.  Wait for 10 ticks for space to become
        available if necessary. */
        xQueueSend( xQueue1,0 ,0);
    }
          xTaskNotify(ignition_handler, 0x0, eSetBits);


        }


    }
}
//------------------------------------------------------------------------------
static void vCCTask(void *pvParameters) {
uint32_t ulNotifiedValue = 0x00;
    for (;;) {

    xTaskNotifyWait( 0x00,      /* Don't clear any notification bits on entry. */
    0xFF, /* Reset the notification value to 0 on exit. */
    &ulNotifiedValue, /* Notified value pass out in
    ulNotifiedValue. */
    portMAX_DELAY  );  /* Block indefinitely. */


    if( ( ulNotifiedValue == 0x02 ))
    {

      Serial.print("2 minutes Jammer detection ... so you have 2 minutes slow down\n\r");

      Serial.print("CC on by jammer! \n\r");
      for(int i=0; i<3;i++){
        digitalWrite(CCPin, HIGH);
        vTaskDelay(configTICK_RATE_HZ);
      }

      Serial.print("CC off by jammer! \n\r");
      for(int i=0; i<10;i++){
        digitalWrite(CCPin, LOW);
        vTaskDelay(configTICK_RATE_HZ);
      }
      Serial.print("CC on by jammer! \n\r");
      for(int i=0; i<3;i++){
        digitalWrite(CCPin, HIGH);
        vTaskDelay(configTICK_RATE_HZ);
      }

      Serial.print("CC off by jammer! \n\r");
      for(int i=0; i<10;i++){
        digitalWrite(CCPin, LOW);
        vTaskDelay(configTICK_RATE_HZ);
      }

      Serial.print("CC on by jammer! \n\r");
      for(int i=0; i<1;i++){
        digitalWrite(CCPin, HIGH);
        vTaskDelay(configTICK_RATE_HZ);
      }

    }
    if( ( ulNotifiedValue == 0x08 ) )
    {
      vTaskResume(ignition_handler);
      vTaskResume(protocol_handler);
      digitalWrite(CCPin, LOW);
    }

    if( ( ulNotifiedValue == 0x10 ) )
    {
  //    xTaskNotify(protocol_handler,0x02, eSetValueWithOverwrite);
  //    xTaskNotify(ignition_handler, 0x11, eSetBits);
      digitalWrite(CCPin, HIGH);
    }
  }
}
//---------------------------------------------------------------------------
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
  Serial1.begin(9600);

  //xTaskCreate(vDisableTask,"Disable",configMINIMAL_STACK_SIZE ,NULL,tskIDLE_PRIORITY,&disable_handler);
  xTaskCreate(vCCTask,"CC",configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY,&cc_handler);
  xTaskCreate(vIgnitionNotification,"Ignition",configMINIMAL_STACK_SIZE,NULL,tskIDLE_PRIORITY,&ignition_handler);
  xTaskCreate(vProtocolTask,"Protocol",configMINIMAL_STACK_SIZE,NULL,tskIDLE_PRIORITY,&protocol_handler);
  //xTaskCreate(vJammingTask,"Jamming",configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY , &jammer_handler);
  //xTaskCreate(vPasswordTask,"Password",configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY ,NULL);
  xTaskCreate(vDoorTask,"Door",configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY ,NULL);

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

// return 0 => LOW ; return 1 => HIGH
 int checkPinStatus(uint8_t _pin, uint8_t lastButtonState){


int reading;
// Initialise the xLastWakeTime variable with the current time.

// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = pdMS_TO_TICKS(1000);    // the debounce time; increase if the output flickers

lastDebounceTime = xTaskGetTickCount();

do{


    reading =  digitalRead(_pin);

    if (reading != lastButtonState) {
      // reset the debouncing timer
      lastDebounceTime = xTaskGetTickCount();
      lastButtonState = reading;
    }

  } while (( xTaskGetTickCount() - lastDebounceTime) < debounceDelay) ;

  return lastButtonState;
}

  void printDigits(byte digits){
   // utility function for digital clock display: prints colon and leading 0
   Serial.print(":");
   if(digits < 10)
     Serial.print('0');
   Serial.print(digits,DEC);
  }

  void time(long val){
  int days = elapsedDays(val);
  int hours = numberOfHours(val);
  int minutes = numberOfMinutes(val);
  int seconds = numberOfSeconds(val);

   // digital clock display of current time
   Serial.print(days,DEC);
   printDigits(hours);
   printDigits(minutes);
   printDigits(seconds);
   Serial.println();

  }
