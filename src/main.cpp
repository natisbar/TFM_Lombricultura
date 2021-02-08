#include <Wire.h>
#include "SparkFunBME280.h"
#include <ArduinoJson.h>
// #include "Adafruit_VEML7700.h"
#include <OneWire.h>
#include <DallasTemperature.h>

BME280 mySensor;
StaticJsonDocument<200> doc;
StaticJsonDocument<200> doc2;
// Adafruit_VEML7700 veml = Adafruit_VEML7700();

const int pinDatosDQ = 9;
const int Trigger = 11;   //Pin digital 2 para el Trigger del sensor
const int Echo = 12;   //Pin digital 3 para el Echo del sensor

OneWire oneWireObjeto(pinDatosDQ);
DallasTemperature tempLom(&oneWireObjeto);
DallasTemperature tempPla(&oneWireObjeto);

int humPinLombriz = A0; 
float humLombriz = 0; 

int humPinPlanta = A1; 
float humPlanta = 0; 

int aguaPPin = 5;
int aguaLPin = 4;
int humusPin = 8;
int humusPin_agua = 6;
int humusPin_humus = 7;
int flujoPin = 10;
int NumVal=0;
bool modo, humus, agua, aux, auxh, auxh2, auxh3=false, evento=false;
bool humus_humus, humus_agua;
bool flot1state, flot2state, flujostate;
int min=18, max=6, total=27;
int d=0;
int repetir = 0;

int flot1Pin = 3;
int flot2Pin = 2;
// String Data = "{\"modo\":true,\"valva\":true,\"valvh\":true}";
// String Data = "";

int inicio=0;

float fmap(float x, float in_min, float in_max, float out_min, float out_max);
bool validar(bool val, int pin);
void TakeData(int fin);
void InitSensorTH();
void InitSensorLux();
void InitSensorUltras();
void TakeUltra();

const int T_GPS_SEGUNDOS=2;
const int T_GPS_MILIS =T_GPS_SEGUNDOS*1000;

void setup()
{
  Serial.begin(115200);
  // Serial.begin(9600);
  Wire.begin();
  tempLom.begin();
  pinMode(aguaLPin, OUTPUT);
  pinMode(aguaPPin, OUTPUT);
  pinMode(humusPin, OUTPUT);
  pinMode(humusPin_agua, OUTPUT);
  pinMode(humusPin_humus, OUTPUT);
  pinMode(flot1Pin, INPUT);
  pinMode(flot2Pin, INPUT);
  pinMode(flujoPin, INPUT);
  InitSensorUltras();
  InitSensorTH();
  InitSensorLux();
}

void loop()
{   
    int fin = millis() - inicio;
    TakeUltra();
    TakeData(fin);
    

    if(evento){
      aux = doc2["valval"].as<bool>(); 
      modo = validar(aux, aguaLPin);   //Valvula agua lombriz
      aux = doc2["valvap"].as<bool>(); 
      agua = validar(aux, aguaPPin);   //Valvula agua planta
      aux = doc2["valvh"].as<bool>(); 
      humus = validar(aux, humusPin);  //bomba humus
      aux = doc2["valvh_agua"].as<bool>();
      humus = validar(aux, humusPin_agua);  //bomba mezcla agua
      aux = doc2["valvh_humus"].as<bool>();
      humus = validar(aux, humusPin_humus); //bomba mezcla humus
      // auxh = doc2["valvh"].as<bool>();
      
      // serializeJson(doc2, Serial);
      // if(auxh){
      //  auxh2=true;
      // }
      // else auxh2=false;
      // evento = false;
    }
   
}

//Asignamos pines para sensor de ultrasonido
void InitSensorUltras(){
  pinMode(Trigger, OUTPUT); //pin como salida
  pinMode(Echo, INPUT);  //pin como entrada
  digitalWrite(Trigger, LOW);//Inicializamos el pin con 0
}

//Inicializar sensor de temperatura y humedad ambiente
void InitSensorTH(){
  if (mySensor.beginI2C() == false) //Begin communication over I2C
  {
    Serial.println("No se pudo establecer comunicación con el sensor de Hum y Temp.");
  }
  else Serial.println("Sensor de Hum y Temp OK");
}

//Inicializar sensor de lux
void InitSensorLux(){
  // if (!veml.begin()) {
  //   Serial.println("Sensor not found");
  // }
  // Serial.println("Sensor found");

  // veml.setGain(VEML7700_GAIN_1);
  // veml.setIntegrationTime(VEML7700_IT_800MS);
}


bool validar(bool val, int pin){
    if(val){
      digitalWrite(pin, HIGH);
      // Serial.println("salida HIGH"); 
      return true;
    }
    else {
      digitalWrite(pin, LOW);
      // Serial.println("salida LOW");
      return false;
    }
}


float fmap(float x, float in_min, float in_max, float out_min, float out_max)
{
   return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


//Evaluar datos procedentes del serial, convierto el string recibido en un JSON
void serialEvent() {
  evento = true;
  String Data = Serial.readString();
  deserializeJson(doc2, Data);
  // if(doc2["valvh"].as<bool>()==true){auxh = false;}
  Data="";
}


void TakeUltra(){
  long t; //timepo que demora en llegar el eco
  digitalWrite(Trigger, HIGH);
  delayMicroseconds(10);          //Enviamos un pulso de 10us
  digitalWrite(Trigger, LOW);
  t = pulseIn(Echo, HIGH); //obtenemos el ancho del pulso
  d = t/59;             //escalamos el tiempo a una distancia en cm
}

//Tomar datos de acuerdo con T_GPS_MILIS
void TakeData(int fin){
  if(fin >= T_GPS_MILIS){
      float BMEhumid = mySensor.readFloatHumidity();  
      float BMEtempC = mySensor.readTempC();
      // float lux = veml.readLux();
      tempLom.requestTemperatures();
      tempPla.requestTemperatures();
      humLombriz = analogRead(humPinLombriz);
      humLombriz = fmap(humLombriz, 0, 1023, 100.0, 0.0);
      humPlanta = analogRead(humPinPlanta);
      humPlanta = fmap(humPlanta, 0, 1023, 100.0, 0.0);
      float templombriz = tempLom.getTempCByIndex(0);
      float tempplanta = tempPla.getTempCByIndex(1);
      // flot1state = digitalRead(flot1Pin);
      // flot2state = digitalRead(flot2Pin);
      // flujostate = digitalRead(flujoPin);

      doc["Hum"] = ((int) (BMEhumid*100))/100.0;
      doc["Temp"] = BMEtempC;
      // doc["Lux"] = ((int) (lux*100))/100.0;
      doc["Templombriz"] = ((int) (templombriz*100))/100.0;
      doc["Tempplanta"] = ((int) (tempplanta*100))/100.0;
      doc["Humlombriz"] = ((int) (humLombriz*100))/100.0;
      doc["Humplanta"] = ((int) (humPlanta*100))/100.0;
      doc["Ultras"] = d;

      // String Data2 = "{\"modo\":false,\"valva\":false,\"valvh\":true}";
      // deserializeJson(doc2, Data2);
      // serializeJson(doc2, Serial);
      // Serial.println();
      serializeJson(doc, Serial);
      Serial.println(); 
      inicio = millis();
    }
}