
// Simple demo of three threads
// LED blink thread, print thread, and idle loop
#include <FreeRTOS_AVR.h>
#include <Arduino.h>

#define DEBUG 1

// inputs
#define JamDetectionPin        4
     // Jamming detection 2 HIGH = off & LOW = on
#define JamDetectionAuxPin     7
#define EnginePin              3  // (with pull down resistor) Engine power HIGH = on & LOW = off
#define DoorPin                5     // Door sensor  HIGH = open / LOW = close
#define EnableDoorPin          6
#define DisablePin             2
     // Enable/disable jamming detection HIGH = on & LOW = off
#define GeneralPin             15
// output|
#define CCPin                  10    // Corta corriente HIGH = shutdown / LOW = normal
#define LedPin                 9
/* for arduino mini pro
#define SpeakerPin             12
#define BuzzerPin              11
#define EngineOn         13    // Jamming alert ping HIGH =
*/
#define SpeakerPin             16
#define BuzzerPin              14
#define EngineOn         8    // Jamming alert ping HIGH =

#define TIME_AFTER_START      15
#define DOOR_ENABLE_SECONDS    7
#define NOTIFICATION_TIME     60*60

volatile uint32_t count = 0;

// handle for blink task
TaskHandle_t cc_handler;
TaskHandle_t jammer_handler;
TaskHandle_t disable_handler;
TaskHandle_t door_handler;
TaskHandle_t protocol_hdlr;
TaskHandle_t ignition_hdlr;
//------------------------------------------------------------------------------
static void vDoorTask(void *pvParameters) {

  int16_t c_engine = 0;

  for(;;) {
    // Sleep for one second.
    vTaskDelay(configTICK_RATE_HZ);
    if(digitalRead(EnginePin) == LOW){

      if(c_engine == TIME_AFTER_START){
        xTaskNotify(protocol_hdlr,( 1UL << 0UL ), eSetBits );
      }

      #ifdef DEBUG
      if(c_engine % 60 == 0){
        Serial.print(c_engine / 60 );
        Serial.print("(min)");
        Serial.println(" Engine ON ");
      }
      #endif DEBUG

      c_engine++;
    }
    else { // ENGINE OFF
      c_engine = 0;
      Serial.println("Engine is power off");
    }
  }
}

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
      Serial.println("Secure Protocol Started!");
      digitalWrite(LedPin,HIGH);
      protocol = true;
    }
    if(protocol && digitalRead(EnableDoorPin) == LOW){
      Serial.println("Enable Door!");
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

  for
  (;;) {
    if(digitalRead(DisablePin) == HIGH){ // desactiva el cc
      xTaskNotify(cc_handler,( 1UL << 3UL ), eSetBits );
    }
    if(digitalRead(DisablePin) == LOW){ // se activa el cc
      xTaskNotify(cc_handler,( 1UL << 4UL ), eSetBits );
    }
    vTaskSuspend( NULL );
  }
}

//------------------------------------------------------------------------------
static void vIgnitionTask(void *pvParameters) {
  bool EngineDelay = false;
  uint32_t ulNotifiedValue = 0x00;
  digitalWrite(EngineOn,HIGH);


  for  (;;) {
    xTaskNotifyWait( 0x00,      /* Don't clear any notification bits on entry. */
    0xFF, /* Reset the notification value to 0 on exit. */
    &ulNotifiedValue, /* Notified value pass out in
    ulNotifiedValue. */
    configTICK_RATE_HZ  );  /* Block indefinitely. */

    if(digitalRead(EnginePin) == LOW){
      digitalWrite(EngineOn,LOW);
    }
    else{

     digitalWrite(EngineOn, HIGH);
    }
     if( ( ulNotifiedValue & 0x01 ) != 0 ){
       Serial.println("Engine delay on");
       EngineDelay = true;
     }
     if( ( ulNotifiedValue & 0x02 ) != 0 ){
       EngineDelay = false;
     }
     if(EngineDelay == true){
       vTaskDelay(configTICK_RATE_HZ*NOTIFICATION_TIME);
       digitalWrite(EngineOn,LOW);
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
      Serial.print("Jammer Alert ");
      Serial.print(c_jammer);
      Serial.println("(s)");
      #endif DEBUG

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
  int16_t c_cc = 0;

  for (;;) {

    xTaskNotifyWait( 0x00,      /* Don't clear any notification bits on entry. */
    0xFF, /* Reset the notification value to 0 on exit. */
    &ulNotifiedValue, /* Notified value pass out in
    ulNotifiedValue. */
    portMAX_DELAY  );  /* Block indefinitely. */

    if( ( ulNotifiedValue & 0x01 ) != 0 )
    {
      Serial.print("JAMMER AND DOOR ... so ");
      digitalWrite(CCPin, HIGH);
      digitalWrite(SpeakerPin, HIGH);
      Serial.print("CC on! ");
      c_cc++;
      Serial.print(c_cc);
      Serial.println("(s)");
      xTaskNotify(ignition_hdlr,( 1UL << 0UL ), eSetBits );

    }

    if( ( ulNotifiedValue & 0x02 ) != 0 )
    {
      digitalWrite(SpeakerPin, HIGH);
      Serial.print("2 minutes Jammer detection ... so you have 2 minutes slow down");
      digitalWrite(CCPin, HIGH);
      Serial.print("CC on by jammer! ");
      vTaskDelay(configTICK_RATE_HZ*3);

      digitalWrite(CCPin, LOW);
      Serial.print("CC off by jammer! ");
      vTaskDelay(configTICK_RATE_HZ*10);

      digitalWrite(CCPin, HIGH);
      Serial.print("CC on by jammer!  ");
      vTaskDelay(configTICK_RATE_HZ*3);

      digitalWrite(CCPin, LOW);
      Serial.print("CC off  by jammer!! ");
      vTaskDelay(configTICK_RATE_HZ*10);

      digitalWrite(CCPin, HIGH);
      Serial.println("CC on by jammer! ! ");

      xTaskNotify(ignition_hdlr,( 1UL << 0UL ), eSetBits );
    }

    if( ( ulNotifiedValue & 0x04 ) != 0 )
    {
      Serial.println("You did not push the button ... so CC ON");
      digitalWrite(CCPin, HIGH);
      xTaskNotify(ignition_hdlr,( 1UL << 0UL ), eSetBits );
    }

    if( ( ulNotifiedValue & 0x08 ) != 0 )
    {
      Serial.println("No Danger .. so CC OFF");
      digitalWrite(CCPin, LOW);
      digitalWrite(SpeakerPin, LOW);
      c_cc = 0;
      vTaskResume(jammer_handler);
      vTaskResume(protocol_hdlr);
      xTaskNotify(ignition_hdlr,( 1UL << 1UL ), eSetBits );
    }
    if( ( ulNotifiedValue & 0x10 ) != 0 )
    {
      Serial.println("CC on ! by Safecar");
      digitalWrite(CCPin, HIGH);
    }
  }
}
//------------------------------------------------------------------------------
static void ExternalInterrupt()
{
  BaseType_t xYieldRequired;
  xYieldRequired = xTaskResumeFromISR( disable_handler );
  Serial.println("Interrupcion");
  if( xYieldRequired == pdTRUE )
  taskYIELD();
}
//------------------------------------------------------------------------------
void setup() {
  Serial.begin(9600);
  // wait for Leonardo
  //while(!Serial) {}
  pinMode(CCPin, OUTPUT);
  pinMode(EngineOn,OUTPUT);
  pinMode(SpeakerPin,OUTPUT);
  pinMode(LedPin, OUTPUT);
  pinMode(BuzzerPin, OUTPUT);

  pinMode(EnableDoorPin, INPUT_PULLUP);
  pinMode(DoorPin, INPUT_PULLUP);
  pinMode(JamDetectionPin, INPUT_PULLUP);
  pinMode(EnginePin, INPUT_PULLUP);
  pinMode(DisablePin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(DisablePin), ExternalInterrupt, CHANGE );

  xTaskCreate(vIgnitionTask,
    "Ignition",
    configMINIMAL_STACK_SIZE + 10,
    NULL,
    tskIDLE_PRIORITY + 5,
    &ignition_hdlr);

  xTaskCreate(vProtocolTask,
    "Protocol",
    configMINIMAL_STACK_SIZE + 20,
    NULL,
    tskIDLE_PRIORITY + 4,
    &protocol_hdlr);

  xTaskCreate(vDoorTask,
    "Door",
    configMINIMAL_STACK_SIZE + 10,
    NULL,
    tskIDLE_PRIORITY + 3,
    &door_handler);
/*
  xTaskCreate(vDisableTask,
    "Disable",
    configMINIMAL_STACK_SIZE + 5,
    NULL,
    tskIDLE_PRIORITY + 2,
    &disable_handler);
*/
  xTaskCreate(vCCTask,
    "CC",
    configMINIMAL_STACK_SIZE + 30,
    NULL,
    tskIDLE_PRIORITY + 2,
    &cc_handler);

  xTaskCreate(vJammingTask,
    "Jamming",
    configMINIMAL_STACK_SIZE + 20,
    NULL,
    tskIDLE_PRIORITY + 1,
    &jammer_handler);

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
