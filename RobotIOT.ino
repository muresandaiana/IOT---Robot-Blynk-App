
#include <NewPing.h>             //librarie pentru senzor ultrasonic
#define BLYNK_PRINT Serial      //librarie pentru aplicatia mobila
#include <ESP8266_Lib.h>        //librarie pentru modul wifi
#include <BlynkSimpleShieldEsp8266.h>
#include <DHT.h>                //librarie pentru senzor de temperatura
#define EspSerial Serial1      //port serial1 pentru a putea conecta TX,RX la 18 si 19
#define ESP8266_BAUD 115200    //setare baudrate specific modulului nostru wifi
ESP8266 wifi(&EspSerial);     //conectarea placii la modul
#define DHTPIN 8          
#define DHTTYPE DHT11                         // DHT 11
DHT dht(DHTPIN, DHTTYPE);
BlynkTimer timer;

WidgetLED led4(V4);                             // pin virtual Led rosu din aplicatie

char auth[] = "UzlOqYxjzRxXjbwM-AMZ_7ilbVJakY8c";     //token de autorizare 
char ssid[] = "Ispas";                               //denumire wifi
char pass[] = "08728833";                             //parola wifi

const int LeftMotorForward = 6;    //pin miscare robot stanga-inainte
const int LeftMotorBackward = 7;   //pin miscare robot stanga-inapoi
const int RightMotorForward = 5;   //pin miscare robot dreapta-inainte
const int RightMotorBackward = 4;  //pin miscare robot dreapta-inapoi

//conectarea senzorului ultrasonic la pinii analogici
#define trig_pin A1 //pin trig
#define echo_pin A2 //pin echo

#define maximum_distance 200   //initializare distanta maxima la care poate detecta senzorul obiecte
boolean goesForward = false;
int distance = 100;             //initializare distanta
NewPing sonar(trig_pin, echo_pin, maximum_distance); //functionarea senzorului folosind sonar

void sendSensor()       //trimiterea temperaturii pe tot parcursul functionarii           
{
  float t = dht.readTemperature();  // temp in grade celsius
  Serial.println("Temp: ");
  Serial.println(t);
  if (isnan(t)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  Blynk.virtualWrite(V5, t);  //pe pin virtual v5 trimitem temperatura
}


int readPing(){  //citire distanta in cm
  delay(70);
  int cm = sonar.ping_cm();
  if (cm==0){
    cm=250;
  }

  return cm;
}


void sendLed(){      //semnalizare led rosu pentru distanta <=20
   distance  = readPing();
   delay(100);
   distance  = readPing();
   delay(100);
   distance  = readPing();
   if(distance<=20){
     led4.on();
    }
    else {
      led4.off();
      }
  }


void setup()
{
  Serial.begin(9600);
  EspSerial.begin(ESP8266_BAUD);
  delay(10);

  Blynk.begin(auth, wifi, ssid, pass);
  dht.begin();
  timer.setInterval(1000L, sendSensor);

  pinMode(RightMotorForward, OUTPUT);
  pinMode(LeftMotorForward, OUTPUT);
  pinMode(LeftMotorBackward, OUTPUT);
  pinMode(RightMotorBackward, OUTPUT);
 

}

void loop()
{
  
  Blynk.run();    //pornire aplicare
  timer.run();
  
}


void moveStop() {    //oprire robot

  digitalWrite(RightMotorForward, LOW);
  digitalWrite(LeftMotorForward,  LOW);
  digitalWrite(RightMotorBackward, LOW);
  digitalWrite(LeftMotorBackward, LOW);
  
}

void moveForward() {    //miscare inainte

  if (!goesForward) {


    sendLed();
    sendSensor();

    goesForward = true;

    digitalWrite(LeftMotorForward, HIGH);
    digitalWrite(RightMotorForward, HIGH);

    digitalWrite(LeftMotorBackward, LOW);
    digitalWrite(RightMotorBackward, LOW);
  }


}

void moveBackward() {     //miscare inapoi

  goesForward = false;

  digitalWrite(LeftMotorBackward, HIGH);
  digitalWrite(RightMotorBackward, HIGH);

  digitalWrite(LeftMotorForward, LOW);
  digitalWrite(RightMotorForward, LOW);

}

void turnRight() {        //intoarcere la dreapta

  digitalWrite(LeftMotorForward, HIGH);
  digitalWrite(RightMotorBackward, HIGH);

  digitalWrite(LeftMotorBackward, LOW);
  digitalWrite(RightMotorForward, LOW);

  delay(500);

  digitalWrite(LeftMotorForward, HIGH);
  digitalWrite(RightMotorForward, HIGH);

  digitalWrite(LeftMotorBackward, LOW);
  digitalWrite(RightMotorBackward, LOW);


}

void turnLeft() {        //intoarcere la stanga

  digitalWrite(LeftMotorBackward, HIGH);
  digitalWrite(RightMotorForward, HIGH);

  digitalWrite(LeftMotorForward, LOW);
  digitalWrite(RightMotorBackward, LOW);

  delay(500);

  digitalWrite(LeftMotorForward, HIGH);
  digitalWrite(RightMotorForward, HIGH);

  digitalWrite(LeftMotorBackward, LOW);
  digitalWrite(RightMotorBackward, LOW);
}


//atribuirea fiecarui buton din aplicatie la miscarea corespunzatoare
BLYNK_WRITE(V0)
{
 
  if (param[0]){
    moveForward();
 
    }
  else{
     moveStop();
  }
   
}

BLYNK_WRITE(V1)
{
   
  if (param[0]){
       moveBackward();
       }
  else{
  
    moveStop();
    }
     
}

BLYNK_WRITE(V2)
{
  
  if (param[0]){
    
    turnRight();}
  else{
    
    moveStop();}
     
}

BLYNK_WRITE(V3)
{

  if (param[0]){
    
    turnLeft();}
  else{
    
    moveStop();  }
}
