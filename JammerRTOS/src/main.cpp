// Simple demo of three threads
// LED blink thread, print thread, and idle loop
//#include <Arduino_FreeRTOS.h>
#include <FreeRTOS_AVR.h>
#include <Arduino.h>

//#define DEBUG 1
//#define DOORSENSOR

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
#define TIME_TO_STOP_ENGINE   40
#define TIME_TO_OPEN_DOOR      10
#define DOOR_ENABLE_SECONDS    7
#define NOTIFICATION_TIME      30
#define TIME_AFTER_OPEN_DOOR   60 // secs after the door was opened



TaskHandle_t cc_handler;
TaskHandle_t jammer_handler;
TaskHandle_t disable_handler;
TaskHandle_t protocol_hdlr;
TaskHandle_t ignition_hdlr;


static void vProtocolTask(void *pvParameters) {

  bool protocol = false;
  uint32_t ulNotifiedValue = 0x00;
  Serial.println("ProtocolTask initialize!");

  for(;;) {
    xTaskNotifyWait( 0x00,      /* Don't clear any notification bits on entry. */
    0xFF, /* Reset the notification value to 0 on exit. */
    &ulNotifiedValue, /* Notified value pass out in
    ulNotifiedValue. */
    configTICK_RATE_HZ  );  /* Block indefinitely. */

    #ifndef DOORSENSOR
    if(digitalRead(DoorPin) == LOW || digitalRead(DoorPositivePin) == LOW){
    #endif
    #ifdef DOORSENSOR
    if(digitalRead(DoorPin) == HIGH){
    #endif
      Serial.print("Door open\n");
    } else {
      Serial.print("Door closed\n");
    }

    if( ( ulNotifiedValue & 0x01 ) != 0 )
    {
      Serial.print("Secure Protocol Started!\n");
      digitalWrite(LedPin,HIGH);
      protocol = true;
    }
    if( ( ulNotifiedValue & 0x02 ) != 0 )
    {
     Serial.print("Secure Protocol Stoped!\n");
     digitalWrite(LedPin, LOW);
     protocol = false;
    }


    if(protocol && digitalRead(EnableDoorPin) == LOW){
      Serial.print("Enable Door!\n\r");
      for(int i = 0; i < TIME_TO_OPEN_DOOR; i++ ){
        digitalWrite(LedPin,HIGH);
        digitalWrite(BuzzerPin,HIGH);
        vTaskDelay(configTICK_RATE_HZ/100);
        digitalWrite(BuzzerPin,LOW);

        digitalWrite(LedPin,LOW);
        vTaskDelay(configTICK_RATE_HZ);
        digitalWrite(LedPin,HIGH);
      } // end for

      #ifndef DOORSENSOR
        if(digitalRead(DoorPin) == LOW || digitalRead(DoorPositivePin) == LOW){ // door open
      #endif
      #ifdef DOORSENSOR
        if(digitalRead(DoorPin) == HIGH){  // door open
      #endif

          Serial.print("Shutdown the engine or  ");
            for(int j = 0; j < TIME_TO_STOP_ENGINE; j++){
              Serial.print(j);
              Serial.print(" ");
              digitalWrite(LedPin,HIGH);
              digitalWrite(BuzzerPin,HIGH);
              vTaskDelay(configTICK_RATE_HZ/100);
              digitalWrite(BuzzerPin,LOW);

              digitalWrite(LedPin,LOW);
              vTaskDelay(configTICK_RATE_HZ/2);
              digitalWrite(LedPin,HIGH);
              #ifndef DOORSENSOR
                if(digitalRead(DoorPin) == HIGH && digitalRead(DoorPositivePin) == HIGH){
              #endif
              #ifdef DOORSENSOR
                if(digitalRead(DoorPin) == LOW){
              #endif
                j = TIME_TO_STOP_ENGINE;
                Serial.print("Secure Protocol Started!\n");
                digitalWrite(LedPin,HIGH);

                }
            } // end for
        } // end if door open
    } // end if button pressed

    #ifndef DOORSENSOR
    if(protocol && (digitalRead(DoorPin) == LOW || digitalRead(DoorPositivePin) == LOW)){
    #endif
    #ifdef DOORSENSOR
     if(protocol && digitalRead(DoorPin) == HIGH){
    #endif
      digitalWrite(LedPin, LOW);
      protocol = false;
      Serial.print("Secure Protocol Stoped!\n");
      Serial.print("CC on in ");
      for(int j = 0; j < TIME_AFTER_OPEN_DOOR ; j++){
      Serial.print(j);
      Serial.print(" ");
      vTaskDelay(configTICK_RATE_HZ);
      digitalWrite(LedPin,HIGH);
      vTaskDelay(configTICK_RATE_HZ/20);
      digitalWrite(LedPin,LOW);
      }
      Serial.println();
      xTaskNotify(cc_handler,( 1UL << 2UL ), eSetBits );
      xTaskNotify(disable_handler,( 1UL << 1UL ), eSetBits );
    //  xTaskNotify(ignition_handler,( 1UL << 1UL ), eSetBits );

  //    protocol = false;
    }
  //  if(protocol == false){
  //    Serial.print("Secure Protocol Stoped!\n");
  //    digitalWrite(LedPin,LOW);
  //  }
  }
}

//------------------------------------------------------------------------------
static void vDisableTask(void *pvParameters) {
//attachInterrupt(digitalPinToInterrupt(CCDisable), CCInterrupt, CHANGE );
  uint32_t ulNotifiedValue = 0x01;
  boolean _last_state = HIGH;
  boolean _current_state = HIGH;
  boolean _current_statei = LOW;
  boolean _last_statei = LOW;

  for  (;;) {
  //  xTaskNotifyWait( 0x00,      /* Don't clear any notification bits on entry. */
  //  0x00, /* Reset the notification value to 0 on exit. */
  //  &ulNotifiedValue, /* Notified value pass out in
  //  ulNotifiedValue. */
  //  configTICK_RATE_HZ);  /* Block indefinitely. */

    //if( ( ulNotifiedValue & 0x01 ) != 0 ){
      vTaskDelay(configTICK_RATE_HZ);
      _current_state = digitalRead(CCDisable);
      if(_current_state != _last_state ){
      Serial.println("Normal Disable");
      if(_current_state == HIGH){ // desactiva el cc
        xTaskNotify(cc_handler,( 1UL << 3UL ), eSetBits );
      }
      if(_current_state == LOW){ // se activa el cc
        xTaskNotify(cc_handler,( 1UL << 4UL ), eSetBits );
      }
      _last_state = _current_state;
    }
//  }
    if( ( ulNotifiedValue & 0x02 ) != 0 ){
      _current_statei = digitalRead(CCDisable);
      if(_current_statei != _last_state){
        Serial.println("Inverted Disable");
        if(digitalRead(CCDisable) == LOW){ // desactiva el cc
        xTaskNotify(cc_handler,( 1UL << 3UL ), eSetBits );
        ulNotifiedValue = 0x01;
        }
      if(digitalRead(CCDisable) == HIGH){ // se activa el cc
        xTaskNotify(cc_handler,( 1UL << 4UL ), eSetBits );
        }
      }
    }
  }
}

//------------------------------------------------------------------------------
static void vIgnitionNotification(void *pvParameters) {

  int32_t c_engine_on = 0;
  int32_t c_engine_off = 0;

//  digitalWrite(EngineStatusPin,HIGH);

  for  (;;) {
    vTaskDelay(configTICK_RATE_HZ);
    //if(digitalRead(IgnitionPin) == LOW || EngineDelay){
      if(digitalRead(IgnitionPin) == LOW){
      if(c_engine_on >= 0 && c_engine_on <= TIME_AFTER_START){
    //  digitalWrite(EngineStatusPin,LOW);
      Serial.print("Ignition on ");
      Serial.print(c_engine_on);
      Serial.print("(s)\n\r");
      }
      if(c_engine_on == TIME_AFTER_START){
        xTaskNotify(protocol_hdlr,( 1UL << 0UL ), eSetBits );
      }
      if(c_engine_on != 0 && c_engine_on % 60 == 0){
        Serial.print("Engine  min on ");
        Serial.println(c_engine_on/60);
      }
      c_engine_on++;
      c_engine_off = 0;
    }
    else { // ENGINE OFF
      if(c_engine_off >= 0 && c_engine_off <= TIME_AFTER_STOP){
    //  digitalWrite(EngineStatusPin, HIGH);
      Serial.print("Ignition off ");
      Serial.print(c_engine_off);
      Serial.print("(s)\n\r");
      }
      if(c_engine_off == TIME_AFTER_STOP){
        xTaskNotify(protocol_hdlr,( 1UL << 1UL ), eSetBits );
      }
      if(c_engine_off !=0 && c_engine_off % 60 == 0){
      Serial.print("Engine min off ");
      Serial.println(c_engine_off/60);
      }
      c_engine_on = 0;
      c_engine_off++;
  }
/*
    xTaskNotifyWait( 0x00, 0xFF,&ulNotifiedValue,ulNotifiedValue,configTICK_RATE_HZ );

     if( ( ulNotifiedValue & 0x01 ) != 0 ){
       Serial.print("Engine delay on\n\r");
       EngineDelay = true;
     }
     if( ( ulNotifiedValue & 0x02 ) != 0 ){
       Serial.print("Engine delay off\n\r");
       EngineDelay = false;
       c_notification = 0;
    }
*/
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
//------------------------------------------------------------------------------
static void vCCTask(void *pvParameters) {
uint32_t ulNotifiedValue = 0x00;
    for (;;) {

    xTaskNotifyWait( 0x00,      /* Don't clear any notification bits on entry. */
    0xFF, /* Reset the notification value to 0 on exit. */
    &ulNotifiedValue, /* Notified value pass out in
    ulNotifiedValue. */
    portMAX_DELAY  );  /* Block indefinitely. */



    if( ( ulNotifiedValue & 0x01 ) != 0 )
    {
      Serial.print("JAMMER AND DOOR ... so CC on !\n\r");
      digitalWrite(CCPin, HIGH);
    //  digitalWrite(SpeakerPin, HIGH);
      xTaskNotify(ignition_hdlr,( 1UL << 0UL ), eSetBits );
    }

    if( ( ulNotifiedValue & 0x02 ) != 0 )
    {

//      digitalWrite(SpeakerPin, HIGH);
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

      xTaskNotify(ignition_hdlr,( 1UL << 0UL ), eSetBits );
    }

    if( ( ulNotifiedValue & 0x04 ) != 0 )
    {
      Serial.print("You did not push the button ... so CC ON\n\r");
      digitalWrite(CCPin, HIGH);
      xTaskNotify(ignition_hdlr,( 1UL << 0UL ), eSetBits );
    }

    if( ( ulNotifiedValue & 0x08 ) != 0 )
    {
      Serial.print("No Danger .. so CC OFF\n\r");
      digitalWrite(CCPin, LOW);
  //    digitalWrite(SpeakerPin, LOW);
      vTaskResume(jammer_handler);
      vTaskResume(protocol_hdlr);
      xTaskNotify(ignition_hdlr,( 1UL << 1UL ), eSetBits );
    }
    if( ( ulNotifiedValue & 0x10 ) != 0 )
    {
      Serial.print("CC on ! by Safecar\n\r");
      digitalWrite(CCPin, HIGH);
    }
  }
}
//---------------------------------------------------------------------------
//---------------------------------------------------------------------
unsigned long _time = 0;
unsigned long _last_time = 0;

/*
void CCInterrupt() {
   BaseType_t taskYieldRequired = 0;
  taskYieldRequired = xTaskResumeFromISR( disable_handler );
if(taskYieldRequired == pdTRUE)
  taskYIELD();
}
  //if( xYieldRequired == pdTRUE )
  //taskYIELD();
//------------------------------------------------------------------------------
*/
void setup() {

  #ifdef DEBUG
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
//  Serial1.begin(4800);
  while (!Serial) {
//    ; // wait for serial port to connect. Needed for native USB, on LEONARDO, MICRO, YUN, and other 32u4 based boards.
  }
  #endif

  xTaskCreate(vDisableTask,"Disable",configMINIMAL_STACK_SIZE + 5,NULL,tskIDLE_PRIORITY + 2,&disable_handler);
  xTaskCreate(vCCTask,"CC",configMINIMAL_STACK_SIZE + 30, NULL, tskIDLE_PRIORITY + 2,&cc_handler);
  xTaskCreate(vJammingTask,"Jamming",configMINIMAL_STACK_SIZE + 20, NULL, tskIDLE_PRIORITY + 1, &jammer_handler);
  xTaskCreate(vIgnitionNotification,"Ignition",configMINIMAL_STACK_SIZE + 50,NULL,tskIDLE_PRIORITY + 3,&ignition_hdlr);
  xTaskCreate(vProtocolTask,"Protocol",configMINIMAL_STACK_SIZE + 20,NULL,tskIDLE_PRIORITY + 3,&protocol_hdlr);

  pinMode(CCPin, OUTPUT);
  //pinMode(EngineStatusPin,OUTPUT);
  //Serial.print("EngineStatusPin out\n");
//  pinMode(SpeakerPin,OUTPUT);
//  Serial.print("SpeakerPin out\n");
  pinMode(LedPin, OUTPUT);
  pinMode(BuzzerPin, OUTPUT);
  pinMode(EnableDoorPin, INPUT_PULLUP);
  pinMode(DoorPin, INPUT_PULLUP);
  pinMode(JamDetectionPin, INPUT_PULLUP);
  pinMode((DoorPositivePin), INPUT_PULLUP);
  pinMode(IgnitionPin, INPUT_PULLUP);
  pinMode(CCDisable, INPUT_PULLUP);
  xTaskNotify(disable_handler,( 1UL << 0UL ), eSetBits );

  vTaskStartScheduler();

  // should never return
  Serial.println(F("Die"));
  while(1);
  }

  //------------------------------------------------------------------------------
  // WARNING idle loop has a very small stack (configMINIMAL_STACK_SIZE)
  // loop must never block
  void loop() {
  }
