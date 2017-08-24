#include<CountUpDownTimer.h>
#include <OneWire.h>

OneWire  ds(15);  // on pin 10 (a 4.7K resistor is necessary)
byte addr[8];

CountUpDownTimer Jam(UP, LOW); // Default precision is HIGH, but you can change
CountUpDownTimer Jam2(UP, LOW); // Default precision is HIGH, but you can change it to also be LOW
CountUpDownTimer Door1(UP, LOW); // Default precision is HIGH, but you can change it to also be LOW
CountUpDownTimer Display(UP, LOW); // Default precision is HIGH, but you can change it to also be LOW
int t_jam = 30;     // Jamming time in minutres
int t_door = 30;    // door open time in minutres

#define ENGPIN         3    // Relay pin 0 on/of
#define LEDPIN        4    // Relay pin 1 on/off
#define JAMPIN         7    // JAmming detector open/gnd
#define DOOR1PIN       5    // Door switch open/gnd
#define DOOR2PIN       12    // Door switch Vcc/open
#define STARTPIN       14   // restart engine



#define JAM1TOUT       10    // time to shotdowown the the engine in secs
#define JAM2TOUT       1     // time to shutdown the engine in min

// the setup function runs once when you press reset or power the board
boolean toggle = false;

int j_counter = 0;
void setup() {


  Serial.begin(9600);

  pinMode(ENGPIN, OUTPUT);
  pinMode(LEDPIN, OUTPUT);


  pinMode(JAMPIN, INPUT_PULLUP);
  pinMode(DOOR1PIN, INPUT_PULLUP);
  pinMode(DOOR2PIN, INPUT_PULLUP);
  pinMode(STARTPIN, INPUT_PULLUP);

  digitalWrite(ENGPIN, HIGH);
  digitalWrite(LEDPIN, LOW);

  Jam.StartTimer();
  Jam2.StartTimer();
  Door1.StartTimer();
  Display.StartTimer();

}

// the loop function runs over and over again forever
void loop() {

  byte result;

  // search looks through all devices on the bus
  ds.reset_search();

  if(result = !ds.search(addr)) {
    // Serial.println("Scanning...");
  } else if(OneWire::crc8(addr, 7) != addr[7]) {
    Serial.println("Invalid CRC");
    delay(del);
    return;
  } else {
    for(byte i=0; i<8; i++) {
      Serial.print(addr[i], HEX);
      Serial.print(" ");
    }
    Serial.print("\n");
    digitalWrite(ledpin, HIGH);
    delay(1000);
    digitalWrite(ledpin, LOW);
  }

Jam.Timer(); // run the timer
Jam2.Timer();
Door1.Timer(); // run the timer
Display.Timer();

if (digitalRead(JAMPIN) == LOW ) {

  if(Jam2.ShowMinutes() >= JAM2TOUT){
    digitalWrite(ENGPIN,LOW);
  }

  // check door open
  if (digitalRead(DOOR1PIN) == HIGH) { // se para el auto por 10 minutos

    if (Door1.TimeHasChanged() ) {
      Serial.print ("JAm and Door open detected for ...");
      Serial.print(Door1.ShowHours());
      Serial.print(":");
      Serial.print(Door1.ShowMinutes());
      Serial.print(":");
      Serial.println(Door1.ShowSeconds());
      toggle = !toggle;
      digitalWrite(LEDPIN,toggle);

    }

    //check door open time
    if(Door1.ShowSeconds() >= JAM1TOUT){
      digitalWrite(ENGPIN,LOW);
    }
  } // end door open

  else { // DOOR1PIN == LOW (door close)

    Door1.ResetTimer();


    if (Jam.TimeHasChanged()){
      Serial.print ("Jamming detected for ...");
      Serial.print(Jam.ShowHours());
      Serial.print(":");
      Serial.print(Jam.ShowMinutes());
      Serial.print(":");
      Serial.println(Jam.ShowSeconds());
      toggle = !toggle;
      digitalWrite(LEDPIN,toggle);
    }
  }

}

else { // JAMPIN == HIGH (no jamming)
  Jam.ResetTimer();
  Jam2.ResetTimer();

  // for display
  digitalWrite(LEDPIN,HIGH);
  if (Display.TimeHasChanged() ) {
    Serial.print ("Waiting for jammer for");
    Serial.print(Display.ShowHours());
    Serial.print(":");
    Serial.print(Display.ShowMinutes());
    Serial.print(":");
    Serial.println(Display.ShowSeconds());

    if (digitalRead(DOOR1PIN) == HIGH) {
      Serial.println("Door Open");
    }
  }

}

if (digitalRead(STARTPIN) == LOW && digitalRead(JAMPIN)== HIGH){
  Serial.println("STARTBUTTON");
  digitalWrite(ENGPIN,HIGH); // revisar si se enciende al cerrar la púerta
}
}
