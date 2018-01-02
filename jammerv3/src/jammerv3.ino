#include <CountUpDownTimer.h>

//#include "OneWireHub.h"
//#include "DS2401.h"  // Serial Number

//constexpr uint8_t pin_led       { 4 };
//constexpr uint8_t pin_onewire   { 5 };
//auto hub     = OneWireHub(pin_onewire);
//auto ds1990A = DS2401( DS2401::family_code, 0x88, 0xC4, 0x07, 0x18, 0x00, 0x00 );

CountUpDownTimer Jam(UP, LOW); // Default precision is HIGH, but you can change
CountUpDownTimer JAMAlert(UP, LOW); // Default precision is HIGH, but you can change it to also be LOW
CountUpDownTimer Door1(UP, LOW); // Default precision is HIGH, but you can change it to also be LOW
CountUpDownTimer Libre(UP, LOW); // Default precision is HIGH, but you can change it to also be LOW
int t_jam = 30;     // Jamming time in minutres
int t_door = 30;    // door open time in minutres

#define CCPIN          10    // Corta corriente
#define LEDPIN         16    // Relay pin 1 on/off
#define JAMPIN         8    // JAmming detector open/gnd
#define DOOR1PIN       9    // Door switch open/gnd
#define STARTPIN       6   // restart engine

#define JAM1TOUT       10    // time to shotdowown the the engine in secs
#define J_TIMEOUT_SECS 30    // time to shutdown the engine in min
#define J_TIMEOUT_MIN   1
#define M_TIMEOUT_SECS 30    // tiempo para activar el motor después del jamming
#define M_TIMEOUT_MIN   6

// the setup function runs once when you press reset or power the board
boolean toggle = false;


//Variables de estado
bool JAMMERALERT;
bool CC_ESTADO;

int j_counter = 0;
void setup() {

  Serial.begin(9600);

  pinMode(CCPIN, OUTPUT);
  pinMode(LEDPIN, OUTPUT);
  pinMode(JAMPIN, INPUT_PULLUP);
  pinMode(DOOR1PIN, INPUT_PULLUP);
  pinMode(STARTPIN, INPUT_PULLUP);

  digitalWrite(CCPIN, LOW);
  digitalWrite(LEDPIN, LOW);

  Jam.StartTimer();
  JAMAlert.StartTimer();
  Door1.StartTimer();
  Libre.StartTimer();

  JAMMERALERT = false;

//  hub.attach(ds1990A); // always online
}

// the loop function runs over and over again forever
void loop() {
//  hub.poll();


  if (digitalRead(JAMPIN) == LOW ) {
    // configuración de timer cuando hay jamming
    Jam.Timer();             // comienza el jamming
    Libre.ResetTimer();    // para el Libre del tiempo
    JAMAlert.ResetTimer();

    // Actua
    digitalWrite(LEDPIN,HIGH);
    JAMMERALERT = true;

    // muestra el tiempo
    if (Jam.TimeHasChanged()){
      Serial.print (" Jamming detectado durante ");
      Serial.print(Jam.ShowHours());
      Serial.print(":");
      Serial.print(Jam.ShowMinutes());
      Serial.print(":");
      Serial.println(Jam.ShowSeconds());
    }

  }

  else { // JAMPIN == HIGH (no jamming)

    // configura timers cuando no hay jamming
    Jam.ResetTimer();
    Libre.Timer();

    // reinicia el timer despues de 10 minutos
    if(Libre.ShowMinutes() >= 10 ){
      Libre.ResetTimer();
    }

    // Actua
    digitalWrite(LEDPIN,LOW);

   // muestra el tiempo

    if (Libre.TimeHasChanged() ) {
      Serial.print ("  Sin Jammer durante ");
      Serial.print(Libre.ShowHours());
      Serial.print(":");
      Serial.print(Libre.ShowMinutes());
      Serial.print(":");
      Serial.println(Libre.ShowSeconds());
    }

    if(Libre.ShowSeconds() >= J_TIMEOUT_SECS && Libre.ShowMinutes() >= J_TIMEOUT_MIN ){
      JAMMERALERT = false;
    }
  }// fin de la detección del jamming



  if (JAMMERALERT == true){// Si se detectó el jammer
    JAMAlert.Timer();



    if(digitalRead(DOOR1PIN) == HIGH){
      Door1.Timer();
      digitalWrite(CCPIN, HIGH);
    }
    else {
      Door1.ResetTimer();
    }

    digitalWrite(LEDPIN,toggle);


    if (JAMAlert.TimeHasChanged() ) {
      Serial.print ("Alerta Jammer ");
      Serial.print(JAMAlert.ShowHours());
      Serial.print(":");
      Serial.print(JAMAlert.ShowMinutes());
      Serial.print(":");
      Serial.print(JAMAlert.ShowSeconds());
      toggle = !toggle;
    }

    if (Door1.TimeHasChanged() ) {
      Serial.print ("  Puerta Abierta durante ");
      Serial.print(Door1.ShowHours());
      Serial.print(":");
      Serial.print(Door1.ShowMinutes());
      Serial.print(":");
      Serial.print(Door1.ShowSeconds());
    }

  } // no hay alerta de jammer

  else {
    JAMAlert.ResetTimer();

    if (digitalRead(STARTPIN) == LOW){
      Serial.println("STARTBUTTON");
      digitalWrite(CCPIN,LOW); // revisar si se enciende al cerrar la púerta
    }

    if (Libre.ShowMinutes() >= M_TIMEOUT_MIN && Libre.ShowSeconds() >= M_TIMEOUT_SECS )
      digitalWrite(CCPIN,LOW);
  }



}
