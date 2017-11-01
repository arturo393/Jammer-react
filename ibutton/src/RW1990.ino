/*The sketch is for reading RW1990, Rw2004 compatible iButton; writing RW1990, RW1990.2 only.*/
/*Connect iButton to the pin with number PIN*/

#include <OneWire.h>
#define READPIN 15
#define WRITEPIN 16
#define LED 10
#define WRITE      /*Uncomment if need to write a key.*/
//#define RW1990_2   /*Uncomment if you deal with RW1990.2*/

//byte key_to_write[] = { 0x01, 0xE4, 0x6D, 0x7B, 0x00, 0x00, 0x00, 0x09 };  /*Home01*/
byte key_to_write[] = { 0x01, 0x00, 0x3E, 0x5C, 0x03, 0x00, 0x00, 0x35 };  /*Home02*/
//byte key_to_write[] = { 0x01, 0x5F, 0x0C, 0x84, 0x00, 0x00, 0x00, 0xC1 };  /*Home03*/

//byte key_to_write[]= { 0x01, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0x9B };/*Universal for KS-2002,Metakom,Cyfral,Visit*/
//byte key_to_write[]= { 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x9B };/*~-_*/
//byte key_to_write[]= { 0x01, 0x00, 0x36, 0x5A, 0x11, 0x40, 0xBE, 0xE1 };/*Universal for KS-2002,Metakom,Visit*/
//byte key_to_write[]= { 0x01, 0xBE, 0x40, 0x11, 0x5A, 0x36, 0x00, 0xE1 };/*~-_Universal key for VIZIT*/
//byte key_to_write[]= { 0x01, 0x00, 0x00, 0x0A, 0x11, 0x40, 0xBE, 0x1D };/*Universal key for Cyfral*/

OneWire  ds(READPIN);
OneWire  dw(WRITEPIN);
void setup(void) {
  Serial.begin(9600);
  pinMode(LED,OUTPUT);
  digitalWrite(LED,HIGH);
}

void loop(void) {
  byte i;
  byte data[8];

  digitalWrite(LED,LOW);
  delay(2000);
  digitalWrite(LED,HIGH);

  if (ds.reset() !=1)
  {
    Serial.print("Esperando por ibutton\n");
    digitalWrite(LED,HIGH);
    delay(3000);
    digitalWrite(LED,LOW);
    return;
  }


  printStatus("Reading iButton Key.");
  readKey(data);
  Serial.print("CRC:\t");  Serial.println(OneWire::crc8(data, 7), HEX);

  if (data[0] & data[1] & data[2] & data[3] & data[4] & data[5] & data[6] & data[7] == 0xFF)
  {
    printStatus("iButton is clear!");
    return;
  }

  #ifndef WRITE
  return;
  #endif

  // Check wether the key has been written
  for (i = 0; i < 8; i++)
    if (data[i] != key_to_write[i])  break;
    else
      if (i == 7){
        printStatus("...iButton is already written!");
        return;
      }

  // Froce writing
  Serial.print("\nWritting new iButton Key:");
  if (! writeKey(data))  printStatus("iButton writing has been done!");
  else  printStatus("Failed to write the iButton!");

}
//END of main loop

#ifndef RW1990_2
void writeByte(byte data)
{
  for(int data_bit=0; data_bit<8; data_bit++){
    if (data & 1){
      digitalWrite(WRITEPIN, LOW);  pinMode(WRITEPIN, OUTPUT);  delayMicroseconds(60);
      pinMode(WRITEPIN, INPUT); digitalWrite(WRITEPIN, HIGH);
      delay(10);
    } else {
      digitalWrite(WRITEPIN, LOW); pinMode(WRITEPIN, OUTPUT);  //delayMicroseconds(5);
      pinMode(WRITEPIN, INPUT); digitalWrite(WRITEPIN, HIGH);
      delay(10);
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
      delay(10);
    } else {
      digitalWrite(WRITEPIN, LOW); pinMode(WRITEPIN, OUTPUT);
      delayMicroseconds(40);
      pinMode(WRITEPIN, INPUT); digitalWrite(WRITEPIN, HIGH);
      delay(10);
    }
    data = data >> 1;
  }
}
#endif

void readKey(byte *data){
//ds.skip();
  ds.reset();
delay(50);
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
  dw.reset();  dw.write(0xD1);
  //send logical 0
  digitalWrite(WRITEPIN, LOW); pinMode(WRITEPIN, OUTPUT); delayMicroseconds(60);
  pinMode(WRITEPIN, INPUT); digitalWrite(WRITEPIN, HIGH); delay(10);

  dw.reset();  dw.write(0xD5);
  dw.write_bytes(data, 8);
  for(byte i=0; i<8; i++)
  {
    writeByte(data[i]);
    Serial.print(data[i], HEX);
    if (i != 7) Serial.print(":");
  }

  dw.reset();  dw.write(0xD1);
  //send logical 1
  digitalWrite(WRITEPIN, LOW); pinMode(WRITEPIN, OUTPUT); delayMicroseconds(10);
  pinMode(WRITEPIN, INPUT); digitalWrite(WRITEPIN, HIGH); delay(10);

  //Validate write operation.
  byte data_r[8];
  dw.skip();  dw.reset();  dw.write(0x33);  //RD cmd
  dw.read_bytes(data_r, 8);

  Serial.print("Validate: ");
  for(byte i=0; i<8; i++)
  {
    Serial.print(data_r[i], HEX);
    if (i != 7) Serial.print(":");
  }
  Serial.println();
  for (byte i = 0; i < 8; i++)
    if (data_r[i] != data[i])  return -1;

  return 0;
}

void printStatus(char *state)
{
  Serial.print("\nStatus:\t"); Serial.println(state);
}

/*
Information Sources:
1) http://arduino.ru/forum/programmirovanie/onewire-zapis-na-bolvanku-rw1990
2) http://blog.danman.eu/cloning-ibutton-using-rw1990-and-avr/
3) http://sun-student.ru/hard/rw1990/finale.html
4) http://electromost.com/news/protokol_dlja_ehlektronnykh_kljuchej_rw1990/2011-04-24-35
*/
