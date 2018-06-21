// Simple demo of three threads
// LED blink thread, print thread, and idle loop
#include <FreeRTOS_AVR.h>
#include <Arduino.h>

// inputs
#define JamDetectionPin        3     // Jamming detection 2 HIGH = off & LOW = on
#define JamDetectionAuxPin     7
#define EnginePin              4  // (with pull down resistor) Engine power HIGH = on & LOW = off
#define DoorPin                5     // Door sensor  HIGH = open / LOW = close
#define EnableDoorPin          6
#define DisablePin             2     // Enable/disable jamming detection HIGH = on & LOW = off
#define GeneralPin             8


// output
#define CCPin                  10    // Corta corriente HIGH = shutdown / LOW = normal
#define LedPin                 9
#define SpeakerPin             12
#define BuzzerPin              11
#define JammerAlertPin         13    // Jamming alert ping HIGH =

#define TIME_AFTER_START      15
#define DOOR_ENABLE_SECONDS    7

volatile uint32_t count = 0;

// handle for blink task
TaskHandle_t cc_handler;
TaskHandle_t jammer_handler;
TaskHandle_t disable_handler;
TaskHandle_t door_handler;
TaskHandle_t protocol_hdlr;

//------------------------------------------------------------------------------
static void vDoorTask(void *pvParameters) {

  pinMode(DoorPin, INPUT_PULLUP);
  pinMode(EnginePin, INPUT_PULLUP);
  pinMode(EnableDoorPin, INPUT_PULLUP);
  pinMode(LedPin, OUTPUT);
  int16_t c_engine = 0;

  for(;;) {
    // Sleep for one second.
    vTaskDelay(configTICK_RATE_HZ);
    if(digitalRead(EnginePin) == LOW){
      if(c_engine == TIME_AFTER_START){
        xTaskNotify(protocol_hdlr,( 1UL << 0UL ), eSetBits );
      }
      // for debbugin
      if(c_engine % 60 == 0){
        Serial.print(c_engine / 60 );
        Serial.print("(min)");
        Serial.println(" Engine ON ");
      }
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

  pinMode(DoorPin, INPUT_PULLUP);
  pinMode(EnableDoorPin, INPUT_PULLUP);
  pinMode(LedPin, OUTPUT);
  pinMode(BuzzerPin, OUTPUT);
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
        vTaskDelay(configTICK_RATE_HZ/3);
        digitalWrite(LedPin,HIGH);
        digitalWrite(BuzzerPin,LOW);
        vTaskDelay(configTICK_RATE_HZ/3);
      }
      if(digitalRead(DoorPin) == HIGH){
        protocol = false;
      }
    }
    if(protocol && digitalRead(DoorPin) == HIGH){
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
  pinMode(DisablePin, INPUT_PULLUP);
  for(;;) {
    if(digitalRead(DisablePin) == HIGH){
      xTaskNotify(cc_handler,( 1UL << 3UL ), eSetBits );
    }
    if(digitalRead(DisablePin) == LOW){
      xTaskNotify(cc_handler,( 1UL << 4UL ), eSetBits );
    }
    vTaskSuspend( NULL );
  }
}

//------------------------------------------------------------------------------
static void vJammingTask(void *pvParameters) {

  pinMode(JammerAlertPin, OUTPUT);
  pinMode(DoorPin, INPUT_PULLUP);
  pinMode(JamDetectionPin, INPUT_PULLUP);
  pinMode(SpeakerPin,OUTPUT);
  int16_t c_jammer = 0;
  for(;;) {
    // Sleep for one second.

    vTaskDelay(configTICK_RATE_HZ);

    if(digitalRead(JamDetectionPin) == LOW){
      Serial.print("Jammer Alert ");
      c_jammer++;
      Serial.print(c_jammer);
      Serial.println("(s)");
      digitalWrite(JammerAlertPin,LOW);

      if(digitalRead(DoorPin) == HIGH){
        xTaskNotify(cc_handler,( 1UL << 0UL ), eSetBits );
        vTaskSuspend( NULL );
      }

      if (c_jammer >= 120){
        xTaskNotify(cc_handler,( 1UL << 1UL ), eSetBits );
        c_jammer = 0;
        vTaskSuspend( NULL );
      }
    }
    else{
    c_jammer = 0;
    digitalWrite(JammerAlertPin,HIGH);
  }
  }
}

static void vCCTask(void *pvParameters) {
  pinMode(CCPin, OUTPUT);
  pinMode(JammerAlertPin,OUTPUT);
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
      digitalWrite(JammerAlertPin,HIGH);
      Serial.print("CC on! ");
      c_cc++;
      Serial.print(c_cc);
      Serial.println("(s)");

    }

    if( ( ulNotifiedValue & 0x02 ) != 0 )
    {
      digitalWrite(SpeakerPin, HIGH);
      Serial.print("2 minutes Jammer detection ... so you have 2 minutes slow down");
      digitalWrite(CCPin, HIGH);
      digitalWrite(JammerAlertPin, HIGH);
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
    }

    if( ( ulNotifiedValue & 0x04 ) != 0 )
    {
      Serial.println("You did not push the button ... so CC ON");
      digitalWrite(CCPin, HIGH);
      digitalWrite(JammerAlertPin,HIGH);
    }

    if( ( ulNotifiedValue & 0x08 ) != 0 )
    {
      Serial.println("No Danger .. so CC OFF");
      digitalWrite(CCPin, LOW);
      digitalWrite(SpeakerPin, LOW);
      digitalWrite(JammerAlertPin,LOW);
      c_cc = 0;
      vTaskResume(jammer_handler);
      vTaskResume(protocol_hdlr);
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
  if( xYieldRequired == pdTRUE )
  taskYIELD();
}
//------------------------------------------------------------------------------
void setup() {
  Serial.begin(9600);
  // wait for Leonardo
  //while(!Serial) {}

  attachInterrupt(digitalPinToInterrupt(DisablePin), ExternalInterrupt, CHANGE);

  xTaskCreate(vProtocolTask,
    "CC",
    configMINIMAL_STACK_SIZE + 50,
    NULL,
    tskIDLE_PRIORITY + 2,
    &protocol_hdlr);

  xTaskCreate(vDoorTask,
    "CC",
    configMINIMAL_STACK_SIZE + 50,
    NULL,
    tskIDLE_PRIORITY + 2,
    &door_handler);

  xTaskCreate(vDisableTask,
    "CC",
    configMINIMAL_STACK_SIZE + 50,
    NULL,
    tskIDLE_PRIORITY + 2,
    &disable_handler);

  xTaskCreate(vCCTask,
    "CC",
          configMINIMAL_STACK_SIZE + 50,
          NULL,
          tskIDLE_PRIORITY + 2,
          &cc_handler);

          xTaskCreate(vJammingTask,
            "Jamming",
            configMINIMAL_STACK_SIZE + 50,
            NULL,
            tskIDLE_PRIORITY + 2,
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
