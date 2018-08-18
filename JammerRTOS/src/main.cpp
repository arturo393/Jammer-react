// Simple demo of three threads
// LED blink thread, print thread, and idle loop
#include <FreeRTOS_AVR.h>
#include <Arduino.h>
#include "basic_io_avr.h"

#define DEBUG 1
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
#define GeneralPin             15
#define SpeakerPin             16

#define TIME_AFTER_START       30
#define TIME_AFTER_STOP        30
#define DOOR_ENABLE_SECONDS    7
#define NOTIFICATION_TIME      30

TaskHandle_t cc_handler;
TaskHandle_t jammer_handler;
TaskHandle_t disable_handler;
//TaskHandle_t door_handler;
TaskHandle_t protocol_hdlr;
TaskHandle_t ignition_hdlr;

void CCInterrupt();

//------------------------------------------------------------------------------
static void vProtocolTask(void *pvParameters) {


  bool protocol = false;
  uint32_t ulNotifiedValue = 0x00;

  for(;;) {
    xTaskNotifyWait( 0x00,      /* Don't clear any notification bits on entry. */
    0xFF, /* Reset the notification value to 0 on exit. */
    &ulNotifiedValue, /* Notified value pass out in
    ulNotifiedValue. */
    configTICK_RATE_HZ/3  );  /* Block indefinitely. */

    if( ( ulNotifiedValue & 0x01 ) != 0 )
    {
      vPrintString("Secure Protocol Started!\n");
      digitalWrite(LedPin,HIGH);
      protocol = true;
    }
    if( ( ulNotifiedValue & 0x02 ) != 0 )
    {
     vPrintString("Secure Protocol Stoped!\n");
     digitalWrite(LedPin, LOW);
     protocol = false;
    }
    if(protocol && digitalRead(EnableDoorPin) == LOW){
      vPrintString("Enable Door!\n\r");
      for(int i = 0; i < 9; i++ ){
        digitalWrite(LedPin,LOW);
        digitalWrite(BuzzerPin,HIGH);
        vTaskDelay(configTICK_RATE_HZ);
        digitalWrite(LedPin,HIGH);
        digitalWrite(BuzzerPin,LOW);
        vTaskDelay(configTICK_RATE_HZ);
      }
      if(digitalRead(DoorPin) == LOW){
  //    if(digitalRead(DoorPin) == HIGH){
        protocol = false;
      }
    }
    if(protocol && digitalRead(DoorPin) == LOW){
    // if(protocol && digitalRead(DoorPin) == HIGH){
      xTaskNotify(cc_handler,( 1UL << 2UL ), eSetBits );
      protocol = false;
    }
    if(protocol == false){
      digitalWrite(LedPin,LOW);
    }
  }
}

//------------------------------------------------------------------------------
static void vDisableTask(void *pvParameters) {

noInterrupts();
attachInterrupt(digitalPinToInterrupt(CCDisable), CCInterrupt, CHANGE );
interrupts();

  for  (;;) {

    vTaskSuspend( NULL );
    if(digitalRead(CCDisable) == HIGH){ // desactiva el cc
      xTaskNotify(cc_handler,( 1UL << 3UL ), eSetBits );
    }
    if(digitalRead(CCDisable) == LOW){ // se activa el cc
      xTaskNotify(cc_handler,( 1UL << 4UL ), eSetBits );
    }
  }
}

//------------------------------------------------------------------------------
static void vIgnitionNotification(void *pvParameters) {
  bool EngineDelay = false;
  uint32_t ulNotifiedValue = 0x00;
  int32_t c_engine_on = 0;
  int32_t c_engine_off = 0;
  int16_t c_notification = 0;

taskENTER_CRITICAL();
  digitalWrite(EngineStatusPin,HIGH);
taskEXIT_CRITICAL();

  for  (;;) {

    // Sleep for one second.
//    vTaskDelay(configTICK_RATE_HZ);
    if(digitalRead(IgnitionPin) == LOW || EngineDelay){
      if(c_engine_on == 0){
      digitalWrite(EngineStatusPin,LOW);
      vPrintString("Ignition on\n");
      }
      if(c_engine_on == TIME_AFTER_START){
        xTaskNotify(protocol_hdlr,( 1UL << 0UL ), eSetBits );
      }

      #ifdef DEBUG
      if((c_engine_on % 60) == 0){
        vPrintStringAndNumber("Engine  min on ",c_engine_on/60);
      }
      #endif // DEBUG

      c_engine_on++;
      c_engine_off = 0;
    }
    else { // ENGINE OFF
      if(c_engine_off == 0){
      taskDISABLE_INTERRUPTS();
      vTaskDelay(configTICK_RATE_HZ*4);
      digitalWrite(EngineStatusPin, HIGH);
      vTaskDelay(configTICK_RATE_HZ*4);
  //    taskENABLE_INTERRUPTS();
      vPrintString("Ignition off\n");
      }
      if(c_engine_off == TIME_AFTER_STOP){
        xTaskNotify(protocol_hdlr,( 1UL << 1UL ), eSetBits );
      }
      #ifdef DEBUG
      if((c_engine_off % 60) == 0)
        vPrintStringAndNumber("Engine min off ",c_engine_off/60);
      #endif // DEBUG

  c_engine_on = 0;
  c_engine_off++;
}

    xTaskNotifyWait( 0x00,      /* Don't clear any notification bits on entry. */
    0xFF, /* Reset the notification value to 0 on exit. */
    &ulNotifiedValue, /* Notified value pass out in
    ulNotifiedValue. */
    configTICK_RATE_HZ  );  /* Block indefinitely. */

     if( ( ulNotifiedValue & 0x01 ) != 0 ){
       vPrintString("Engine delay on\n\r");
       EngineDelay = true;
     }
     if( ( ulNotifiedValue & 0x02 ) != 0 ){
       vPrintString("Engine delay off\n\r");
       EngineDelay = false;
       c_notification = 0;

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

      #ifdef DEBUG
      vPrintStringAndNumber("Jammer Alert secs",c_jammer);
      #endif // DEBUG

      if(digitalRead(DoorPin) == LOW){
        // se puede esperar un tiempo antes apagar el motor
        xTaskNotify(cc_handler,( 1UL << 0UL ), eSetBits );
        vTaskSuspend( NULL );
      }

      if (c_jammer >= 120){
        c_jammer = 0;
        xTaskNotify(cc_handler,( 1UL << 1UL ), eSetBits );
        vTaskSuspend( NULL );
      }
    }
    else{
    c_jammer = 0;
  }
  }
}

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
      vPrintString("JAMMER AND DOOR ... so CC on !\n\r");
      digitalWrite(CCPin, HIGH);
      digitalWrite(SpeakerPin, HIGH);
      xTaskNotify(ignition_hdlr,( 1UL << 0UL ), eSetBits );
    }

    if( ( ulNotifiedValue & 0x02 ) != 0 )
    {

      digitalWrite(SpeakerPin, HIGH);
      vPrintString("2 minutes Jammer detection ... so you have 2 minutes slow down\n\r");

      vPrintString("CC on by jammer! \n\r");
      for(int i=0; i<3;i++){
        digitalWrite(CCPin, HIGH);
        vTaskDelay(configTICK_RATE_HZ);
      }

      vPrintString("CC off by jammer! \n\r");
      for(int i=0; i<10;i++){
        digitalWrite(CCPin, LOW);
        vTaskDelay(configTICK_RATE_HZ);
      }
      vPrintString("CC on by jammer! \n\r");
      for(int i=0; i<3;i++){
        digitalWrite(CCPin, HIGH);
        vTaskDelay(configTICK_RATE_HZ);
      }

      vPrintString("CC off by jammer! \n\r");
      for(int i=0; i<10;i++){
        digitalWrite(CCPin, LOW);
        vTaskDelay(configTICK_RATE_HZ);
      }

      vPrintString("CC on by jammer! \n\r");
      for(int i=0; i<1;i++){
        digitalWrite(CCPin, HIGH);
        vTaskDelay(configTICK_RATE_HZ);
      }

      xTaskNotify(ignition_hdlr,( 1UL << 0UL ), eSetBits );
    }

    if( ( ulNotifiedValue & 0x04 ) != 0 )
    {
      vPrintString("You did not push the button ... so CC ON\n\r");
      digitalWrite(CCPin, HIGH);
      xTaskNotify(ignition_hdlr,( 1UL << 0UL ), eSetBits );
    }

    if( ( ulNotifiedValue & 0x08 ) != 0 )
    {
      vPrintString("No Danger .. so CC OFF\n\r");
      digitalWrite(CCPin, LOW);
      digitalWrite(SpeakerPin, LOW);
      vTaskResume(jammer_handler);
      vTaskResume(protocol_hdlr);
      xTaskNotify(ignition_hdlr,( 1UL << 1UL ), eSetBits );
    }
    if( ( ulNotifiedValue & 0x10 ) != 0 )
    {
      vPrintString("CC on ! by Safecar\n\r");
      digitalWrite(CCPin, HIGH);
    }
  }
}
//------------------------------------------------------------------------------
unsigned long _time = 0;
unsigned long _last_time = 0;


void CCInterrupt() {
   BaseType_t taskYieldRequired = 0;
  taskYieldRequired = xTaskResumeFromISR( disable_handler );
if(taskYieldRequired == pdTRUE)
  taskYIELD();
}
  //if( xYieldRequired == pdTRUE )
  //taskYIELD();
//------------------------------------------------------------------------------
void setup() {
  Serial.begin(9600);
  // wait for Leonardo
  while(!Serial) {}

 vPrintString("Inicio del programa\n");


  vPrintString("CCDisable interrupt enable\n");
  xTaskCreate(vDisableTask,"Disable",configMINIMAL_STACK_SIZE + 5,NULL,tskIDLE_PRIORITY + 2,&disable_handler);
  vPrintString("Disable task created\n");
  xTaskCreate(vCCTask,"CC",configMINIMAL_STACK_SIZE + 30, NULL, tskIDLE_PRIORITY + 2,&cc_handler);
  vPrintString("CC task created\n");
  xTaskCreate(vJammingTask,"Jamming",configMINIMAL_STACK_SIZE + 20, NULL, tskIDLE_PRIORITY + 1, &jammer_handler);
  vPrintString("Jamming task created\n");

  xTaskCreate(vIgnitionNotification,"Ignition",configMINIMAL_STACK_SIZE + 50,NULL,tskIDLE_PRIORITY + 3,&ignition_hdlr);
  vPrintString("vIgnitionNotification task created\n");

  xTaskCreate(vProtocolTask,"Protocol",configMINIMAL_STACK_SIZE + 20,NULL,tskIDLE_PRIORITY + 3,&protocol_hdlr);
  vPrintString("Protocol task created\n");
  //xTaskCreate(vDoorTask,"Door",configMINIMAL_STACK_SIZE + 10,NULL,tskIDLE_PRIORITY + 3,&door_handler);

  vPrintString("Se crean las tareas\n");


  pinMode(CCPin, OUTPUT);
  vPrintString("CCPin output\n");
  pinMode(EngineStatusPin,OUTPUT);
  vPrintString("EngineStatusPin out\n");
  pinMode(SpeakerPin,OUTPUT);
  vPrintString("SpeakerPin out\n");
  pinMode(LedPin, OUTPUT);
  vPrintString("LedPin out\n");
  pinMode(BuzzerPin, OUTPUT);
  vPrintString("BuzzerPin out\n");
  pinMode(EnableDoorPin, INPUT_PULLUP);
  vPrintString("EnableDoorPin input pullup\n");
  pinMode(DoorPin, INPUT_PULLUP);
  vPrintString("DoorPin input pullup\n");
  pinMode(JamDetectionPin, INPUT_PULLUP);
  vPrintString("JamDetectionPin input pullup\n");
  pinMode(IgnitionPin, INPUT_PULLUP);
  vPrintString("IgnitionPin input pullup\n");
  pinMode(CCDisable, INPUT_PULLUP);
  vPrintString("CCDisable input pullup\n");
  vPrintString("All task ara created!\n\n\n\r");
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
