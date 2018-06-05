#include <CountUpDownTimer.h>
#include <Arduino.h>
CountUpDownTimer Jam(UP, LOW); // Default precision is HIGH, but you can change
CountUpDownTimer JAMAlert(UP, LOW); // Default precision is HIGH, but you can change it to also be LOW
CountUpDownTimer Door1(UP, LOW); // Default precision is HIGH, but you can change it to also be LOW
CountUpDownTimer Libre(UP, LOW); // Default precision is HIGH, but you can change it to also be LOW
int t_jam = 30;     // Jamming time in minutres
int t_door = 30;    // door open time in minutres

#define CCPIN                 10    // Corta corriente HIGH = shutdown / LOW = normal
//#define LEDPIN       14    // Relay pin 1 on/off
#define ENGINE_PIN             8  // (with pull down resistor) Engine power HIGH = on & LOW = off
#define JAM_DETECTION2_PIN     6     // Jamming detection 2 HIGH = off & LOW = on
#define DOOR1PIN               9     // Door sensor  HIGH = open / LOW = close
#define JAM_DISABLE_PIN        2     // Enable/disable jamming detection HIGH = on & LOW = off
#define JAM_ALERT_PIN         16    // Jamming alert ping HIGH =

#define ENABLE_DOOR_PIN        3          //


#define JAM1TOUT       10    // time to shotdowown the the engine in secs
#define J_TIMEOUT_SECS 30    // time to shutdown the engine in min
#define J_TIMEOUT_MIN   1
#define M_TIMEOUT_SECS 30    // tiempo para activar el motor después del jamming
#define M_TIMEOUT_MIN   6

#define T_JAMMERALERT  1000*60*90
#define T_CC_ON        1000*3600*6

bool SENSOR_INVERT = false; // sensor invertido con rele = true;
bool RELE_ON = true;      // true si se usa rele ; false si no se usa.
// the setup function runs once when you press reset or power the board
boolean toggle = false;
boolean status= false;

/* INTERRUPT DEBOUNCING VARIABLES */
long lastDebounceTime = 0;  // the last time the output pin was toggled
long debounceDelay = 50;    // the debounce time; increase if the output flickers

//Variables de estado
bool JAMMERALERT;
bool CC_ON;
bool JAMMERDISABLE;
bool JAMMER1;
bool JAMMER2;
bool DOOR;

unsigned long c_jammer;
unsigned long c_cc;

int j_counter = 0;
void setup() {

  noInterrupts(); // Critical
/* jamming alert timming */
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;

  OCR1A = 15625;
  TCCR1B |= (1 << WGM12);   // CTC mode
  TCCR1B |= (1 << CS12);
  TCCR1B |= (1 << CS10);
  //TIMSK1 |= (1 << OCIE1A);

/* CC timer */
  TCCR3A = 0;
  TCCR3B = 0;
  TCNT3 = 0;

  OCR3A = 15625;
  TCCR3B |= (1 << WGM32);   // CTC mode
  TCCR3B |= (1 << CS32);
  TCCR3B |= (1 << CS30);


  interrupts();

  Serial.begin(9600);

  pinMode(CCPIN, OUTPUT);
  pinMode(JAM_ALERT_PIN, OUTPUT);

//  pinMode(JAM_DETECTION1_PIN, INPUT_PULLUP);
  pinMode(JAM_DETECTION2_PIN, INPUT_PULLUP);
  pinMode(DOOR1PIN, INPUT_PULLUP);
  pinMode(JAM_DISABLE_PIN, INPUT_PULLUP);
  pinMode(ENGINE_PIN,INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(JAM_DISABLE_PIN), jammerState, LOW );
  attachInterrupt(digitalPinToInterrupt(ENGINE_PIN), engineState, RISING );

/* variables initilization*/

  digitalWrite(CCPIN,LOW);
  digitalWrite(JAM_ALERT_PIN, LOW);
  CC_ON = false;
  JAMMERDISABLE = false;
  JAMMER1 = false;
  JAMMER2 = false;
  JAMMERALERT = false;

  c_jammer = 0;
  c_cc = 0;
  delay(3000);

}

ISR(TIMER1_COMPA_vect)          // timer compare interrupt service routine
{

  if(c_jammer < 180){
  JAMMERALERT = true;
  Serial.print("Jammer Alert ");
  c_jammer++;
  Serial.print(c_jammer);
  Serial.println("(s)");
}
  else{
    JAMMERALERT = false;
    TIMSK1 = 0 ;
  }

}

ISR(TIMER3_COMPA_vect)          // timer compare interrupt service routine
{

  if(c_cc < 1200){
  CC_ON = true;
  Serial.print("CC on! ");
  c_cc++;
  Serial.print(c_cc);
  Serial.println("(s)");
}
  else{
    CC_ON = false;
    TIMSK3 = 0 ;
    c_cc = 0;
  }

}

void jammerState(){

  Serial.println("Disable");
  JAMMERALERT = false;
  CC_ON = false;
  TIMSK1 = 0;
  TIMSK3 = 0 ;
  c_cc = 0;
  Serial.println();

  }

void engineState(){
if((long)(micros()-lastDebounceTime) >= debounceDelay){



}

lastDebounceTime = micros();

}

void loop() {


  JAMMER1 = !digitalRead(JAM_DETECTION1_PIN);
  JAMMER2 = !digitalRead(JAM_DETECTION2_PIN);
  DOOR = !digitalRead(DOOR1PIN);



// JAMMER DETECTION
  if (JAMMER1 || JAMMER2){
    c_jammer = 0;
    TIMSK1 |= (1 << OCIE1A);
    digitalWrite(JAM_ALERT_PIN,HIGH);
}
else{
  digitalWrite(JAM_ALERT_PIN,LOW);
}

if(JAMMERALERT){

if (digitalRead(DOOR1PIN) == HIGH){
   TIMSK1 = 0;
   TIMSK3 |= (1 << OCIE3A);
}
}




/*

if (MOTOR > 15 (s) && DOOR){
  ALERT = true;
}

if(Dbutton ){
  if(!DOOR > 5 (s))
  ALERT = false;
}

if(DISABLE){
  ALERT = false;
}

else {
  ALERT = false;
}

*/


if(CC_ON)
digitalWrite(CCPIN,HIGH);
else
digitalWrite(CCPIN,LOW);

  }
