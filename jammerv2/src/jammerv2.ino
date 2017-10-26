#include <Arduino_FreeRTOS.h>

#include<CountUpDownTimer.h>


CountUpDownTimer Jam(UP, LOW); // Default precision is HIGH, but you can change
CountUpDownTimer JAMAlert(UP, LOW); // Default precision is HIGH, but you can change it to also be LOW
CountUpDownTimer Door1(UP, LOW); // Default precision is HIGH, but you can change it to also be LOW
CountUpDownTimer Libre(UP, LOW); // Default precision is HIGH, but you can change it to also be LOW
int t_jam = 30;     // Jamming time in minutres
int t_door = 30;    // door open time in minutres

#define CCPIN          3    // Corta corriente
#define LEDPIN         4    // Relay pin 1 on/off
#define JAMPIN         5    // JAmming detector open/gnd
#define DOOR1PIN       7    // Door switch open/gnd
#define DOOR2PIN       12    // Door switch Vcc/open
#define STARTPIN       14   // restart engine

#define JAM1TOUT       10    // time to shotdowown the the engine in secs
#define J_TIMEOUT_SECS 30    // time to shutdown the engine in min
#define J_TIMEOUT_MIN   1
#define M_TIMEOUT_SECS 30    // tiempo para activar el motor después del jamming
#define M_TIMEOUT_MIN   6
// the setup function runs once when you press reset or power the board
boolean toggle = false;
//Variables de estado
bool JAMMER;
bool CC_ESTADO;
int j_counter = 0;


#include <OneWire.h>
#define ILEDPIN 10
#define READPIN 15
#define WRITEPIN 16
#define LED 10
byte key_to_write[] = { 0x01, 0x00, 0x3E, 0x5C, 0x03, 0x00, 0x00, 0x35 };
OneWire  ds(READPIN);
OneWire  dw(WRITEPIN);

bool WRITE = true;
// define two tasks for Blink & JammerRead
void TaskBlink( void *pvParameters );
void TaskJammerRead( void *pvParameters );
void TaskIbuttonRead( void *pvParameters);
// the setup function runs once when you press reset or power the board
void setup() {

  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);

  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB, on LEONARDO, MICRO, YUN, and other 32u4 based boards.
  }

  // Now set up two tasks to run independently.
  xTaskCreate(
    TaskBlink
    ,  (const portCHAR *)"Blink"   // A name just for humans
    ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  0  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL );

  xTaskCreate(
    TaskJammerRead
    ,  (const portCHAR *) "JammerRead"
    ,  128  // Stack size
    ,  NULL
    ,  2  // Priority
    ,  NULL );

    xTaskCreate(
      TaskIbuttonRead
      ,  (const portCHAR *) "IbuttonRead"
      ,  128  // Stack size
      ,  NULL
      , 3  // Priority
      ,  NULL );

  // Now the task scheduler, which takes over control of scheduling individual tasks, is automatically started.
}

void loop()
{
  // Empty. Things are done in Tasks.
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/
void TaskIbuttonRead(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

  pinMode(ILEDPIN,OUTPUT);
  digitalWrite(ILEDPIN,HIGH);

  for(;;){

  byte i;
  byte data[8];

  digitalWrite(ILEDPIN,LOW);
  vTaskDelay(2000/portTICK_PERIOD_MS);
  digitalWrite(ILEDPIN,HIGH);

  if (ds.reset() !=1)
  {
    Serial.print("Esperando por ibutton\n");
    digitalWrite(ILEDPIN,HIGH);
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    digitalWrite(ILEDPIN,LOW);
  }
  else {

  printStatus("Reading iButton Key.");
  readKey(data);
  Serial.print("CRC:\t");  Serial.println(OneWire::crc8(data, 7), HEX);

  if (data[0] & data[1] & data[2] & data[3] & data[4] & data[5] & data[6] & data[7] == 0xFF)
  {
    printStatus("iButton is clear!");
  }
  else {

    if(WRITE == true){
      // Check wether the key has been written
      for (i = 0; i < 8; i++)
        if (data[i] != key_to_write[i])  break;
        else
          if (i == 7){
            printStatus("...iButton is already written!");

          }

      // Froce writing
      Serial.print("\nWritting new iButton Key:");
      if (! writeKey(key_to_write))  printStatus("iButton writing has been done!");
      else  printStatus("Failed to write the iButton!");
    }

  }
}


}

}

void TaskBlink(void *pvParameters)  // This is a task.
{
  (void) pvParameters;


/*
  Blink
  Turns on an LED on for one second, then off for one second, repeatedly.


  by Arturo Guadalupi
*/

  // initialize digital LEDPIN on pin 13 as an output.
  for (;;) // A Task shall never return or exit.
  {

    Serial.print(" Status: ");
    Serial.print(HIGH);
    Serial.print('\t');
    vTaskDelay( 5000 / portTICK_PERIOD_MS ); // wait for one second
    digitalWrite(ILEDPIN, LOW);    // turn the LED off by making the voltage LOW    Serial.print("\t Status: ");
    Serial.print(LOW);
    Serial.print('\t');
    vTaskDelay( 5000 / portTICK_PERIOD_MS ); // wait for one second
  }
}


void TaskJammerRead(void *pvParameters)  // This is a task.
{
  (void) pvParameters;
  pinMode(CCPIN, OUTPUT);
  pinMode(LEDPIN, OUTPUT);


  pinMode(JAMPIN, INPUT_PULLUP);
  pinMode(DOOR1PIN, INPUT_PULLUP);
  pinMode(DOOR2PIN, INPUT_PULLUP);
  pinMode(STARTPIN, INPUT_PULLUP);

  digitalWrite(CCPIN, HIGH);
  digitalWrite(LEDPIN, LOW);

  Jam.StartTimer();
  JAMAlert.StartTimer();
  Door1.StartTimer();
  Libre.StartTimer();

  JAMMER = false;
/*
  JammerReadSerial
  Reads an analog input on pin 0, prints the result to the serial monitor.
  Graphical representation is available using serial plotter (Tools > Serial Plotter menu)
  Attach the center pin of a potentiometer to pin A0, and the outside pins to +5V and ground.

  This example code is in the public domain.
*/

  for (;;)
  {
    JAMAlert.Timer();
    Door1.Timer(); // run the timer


    if (digitalRead(JAMPIN) == LOW ) {
      // configuración de timer cuando hay jamming
      Jam.Timer();             // comienza el jamming
      Libre.ResetTimer();    // para el Libre del tiempo

      // muestra el tiempo

      if (Jam.TimeHasChanged()){
        Serial.print ("Jamming detectado durante ");
        Serial.print(Jam.ShowHours());
        Serial.print(":");
        Serial.print(Jam.ShowMinutes());
        Serial.print(":");
        Serial.println(Jam.ShowSeconds());
      }

      // Actua
      digitalWrite(LEDPIN,HIGH);
      JAMMER = true;
    }

    else { // JAMPIN == HIGH (no jamming)

      // configura timers cuando no hay jamming
      Jam.ResetTimer();
      Libre.Timer();

      if(Libre.ShowMinutes() >= 10 ){
        Libre.ResetTimer();
      }
      // muestra el tiempo

      if (Libre.TimeHasChanged() ) {
        Serial.print ("Sin Jammer durante ");
        Serial.print(Libre.ShowHours());
        Serial.print(":");
        Serial.print(Libre.ShowMinutes());
        Serial.print(":");
        Serial.println(Libre.ShowSeconds());
        toggle = !toggle;
      }

      // Actua
      digitalWrite(LEDPIN,LOW);
      if(JAMMER == true){
        digitalWrite(LEDPIN,toggle);
      }
      if(Libre.ShowSeconds() >= J_TIMEOUT_SECS && Libre.ShowMinutes() >= J_TIMEOUT_MIN ){
        JAMMER = false;
      }
    }// fin de la detección del jamming


    if (JAMMER == true){// Si se detectó el jammer
      JAMAlert.Timer();

      if (JAMAlert.TimeHasChanged() ) {
        Serial.print ("Alerta Jammer ");
        Serial.print(JAMAlert.ShowHours());
        Serial.print(":");
        Serial.print(JAMAlert.ShowMinutes());
        Serial.print(":");
        Serial.println(JAMAlert.ShowSeconds());
      }

      if(digitalRead(DOOR1PIN) == HIGH){
        digitalWrite(CCPIN, LOW);
        CC_ESTADO = false;
      }

    } // no hay alerta de jammer
    else {
      JAMAlert.ResetTimer();

      if (digitalRead(STARTPIN) == LOW){
        Serial.println("STARTBUTTON");
        digitalWrite(CCPIN,HIGH); // revisar si se enciende al cerrar la púerta
      }

      if (Libre.ShowMinutes() >= M_TIMEOUT_MIN && Libre.ShowSeconds() >= M_TIMEOUT_SECS )
        digitalWrite(CCPIN,HIGH);
    }

}
}

#ifndef RW1990_2
void writeByte(byte data)
{
  for(int data_bit=0; data_bit<8; data_bit++){
    if (data & 1){
      digitalWrite(WRITEPIN, LOW);  pinMode(WRITEPIN, OUTPUT);  vTaskDelay(0.06/portTICK_PERIOD_MS);
      pinMode(WRITEPIN, INPUT); digitalWrite(WRITEPIN, HIGH);
      vTaskDelay(10/portTICK_PERIOD_MS);
    } else {
      digitalWrite(WRITEPIN, LOW); pinMode(WRITEPIN, OUTPUT);  //delayMicroseconds(5);
      pinMode(WRITEPIN, INPUT); digitalWrite(WRITEPIN, HIGH);
      vTaskDelay(10/portTICK_PERIOD_MS);
    }
    data = data >> 1;
  }
}

#else
//Use for RW1990.2
void writeByte(byte data)
{
  for(int data_bit=0; data_bit<8; data_bit++){
    if (data & 1){
      digitalWrite(WRITEPIN, LOW); pinMode(WRITEPIN, OUTPUT);
      pinMode(WRITEPIN, INPUT); digitalWrite(WRITEPIN, HIGH);
      vTaskDelay(10/portTICK_PERIOD_MS);
    } else {
      digitalWrite(WRITEPIN, LOW); pinMode(WRITEPIN, OUTPUT);
      vTaskDelay(0.040/portTICK_PERIOD_MS);
      pinMode(WRITEPIN, INPUT); digitalWrite(WRITEPIN, HIGH);
      vTaskDelay(10/portTICK_PERIOD_MS);
    }
    data = data >> 1;
  }
}
#endif

void readKey(byte *data){
//ds.skip();
  ds.reset();
vTaskDelay(50/portTICK_PERIOD_MS);
  ds.write(0x33);  //RD cmd
  ds.read_bytes(data, 8);
  Serial.print("Key: \t");

    for(byte i = 0; i < 8; i++) {
      Serial.print(data[i], HEX);
      if (i != 7) Serial.print(":");
    }
  Serial.println();
}

byte writeKey(byte *data)
{
  ds.reset();  ds.write(0xD1);
  //send logical 0
  digitalWrite(WRITEPIN, LOW); pinMode(WRITEPIN, OUTPUT); vTaskDelay(0.060/portTICK_PERIOD_MS);
  pinMode(WRITEPIN, INPUT); digitalWrite(WRITEPIN, HIGH); vTaskDelay(10/portTICK_PERIOD_MS);

  ds.reset();  ds.write(0xD5);
  for(byte i=0; i<8; i++)
  {
    writeByte(key_to_write[i]);
    Serial.print('*');
  }

  ds.reset();  ds.write(0xD1);
  //send logical 1
  digitalWrite(WRITEPIN, LOW); pinMode(WRITEPIN, OUTPUT); vTaskDelay(0.010/portTICK_PERIOD_MS);
  pinMode(WRITEPIN, INPUT); digitalWrite(WRITEPIN, HIGH); vTaskDelay(10/portTICK_PERIOD_MS);

  //Validate write operation.
  byte data_r[8];
  ds.skip();  ds.reset();  ds.write(0x33);  //RD cmd
  ds.read_bytes(data_r, 8);

  for (byte i = 0; i < 8; i++)
    if (data_r[i] != data[i])  return -1;

  return 0;
}

void printStatus(char *state)
{
  Serial.print("\nStatus:\t"); Serial.println(state);
}
