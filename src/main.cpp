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

OneWire oneWireObjeto(pinDatosDQ);
DallasTemperature tempLom(&oneWireObjeto);
DallasTemperature tempPla(&oneWireObjeto);

int humPinLombriz = A0; 
float humLombriz = 0; 

int humPinPlanta = A1; 
float humPlanta = 0; 

int modoPin = 6;
int humusPin = 7;
int humusPin_agua = 5;
int humusPin_humus = 4;
int aguaPin = 8;
int flujoPin = 10;
int NumVal=0;
bool modo, humus, agua, aux, auxh, auxh2, auxh3=false, evento=false;
bool humus_humus, humus_agua;
bool flot1state, flot2state, flujostate;

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

const int T_GPS_SEGUNDOS=2;
const int T_GPS_MILIS =T_GPS_SEGUNDOS*1000;

void setup()
{
  // Serial.begin(115200);
  Serial.begin(9600);
  Wire.begin();
  tempLom.begin();
  pinMode(modoPin, OUTPUT);
  pinMode(humusPin, OUTPUT);
  pinMode(humusPin_agua, OUTPUT);
  pinMode(humusPin_humus, OUTPUT);
  pinMode(aguaPin, OUTPUT);
  pinMode(flot1Pin, INPUT);
  pinMode(flot2Pin, INPUT);
  pinMode(flujoPin, INPUT);
  InitSensorTH();
  InitSensorLux();
}

void loop()
{   
    int fin = millis() - inicio;
    TakeData(fin);

    // if(evento){
      aux = doc2["modo"].as<bool>();
      modo = validar(aux, modoPin);
      aux = doc2["valva"].as<bool>();
      agua = validar(aux, aguaPin);
      auxh = doc2["valvh"].as<bool>();
      // humus = validar(aux, humusPin);
      // serializeJson(doc2, Serial);
      if(auxh){
       auxh2=true;
      }
      else auxh2=false;
    //   evento = false;
    // }


    if(auxh2==true && auxh==true && NumVal<3){
      // Serial.println("entre a validación");
      if(!auxh3){
        if(flot1state==false && flot2state==false && NumVal==0){
          humus_humus = validar(true, humusPin_humus);
          Serial.println("ambos flotadores false");
          NumVal=1;
        }
        if(flot1state==true && flot2state==false && NumVal==1){
          humus_humus = validar(false, humusPin_humus);
          humus_agua = validar(true, humusPin_agua);
          Serial.println("flotador 1 true");
          NumVal=2;
        }
        if(flot1state==true && flot2state==true && NumVal==2){
          humus_humus = validar(false, humusPin_humus);
          humus_agua = validar(false, humusPin_agua);
          humus = validar(true, humusPin);
          NumVal=3;
          Serial.println("flotador 1 y 2 true");
          auxh3=true;
        }
      }
    }
    else if (auxh3==true && NumVal>2){
        if(flujostate==false && flot1state==false && flot2state==false){
          Serial.println("no hay flujo");
          auxh3=false;
          NumVal=0;
          humus = validar(false, humusPin);
        }

        else if (flujostate==true && flot1state==false && flot2state==false){
          humus = validar(true, humusPin);
          Serial.println("aun hay flujo");
        }
    }
    else if (auxh2==false){
      humus_humus = validar(false, humusPin_humus);
      humus_agua = validar(false, humusPin_agua);
      humus = validar(false, humusPin);
    }
   
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
      flot1state = digitalRead(flot1Pin);
      flot2state = digitalRead(flot2Pin);
      flujostate = digitalRead(flujoPin);

      doc["Hum"] = ((int) (BMEhumid*100))/100.0;
      doc["Temp"] = BMEtempC;
      // doc["Lux"] = ((int) (lux*100))/100.0;
      doc["Templombriz"] = ((int) (templombriz*100))/100.0;
      doc["Tempplanta"] = ((int) (tempplanta*100))/100.0;
      doc["Humlombriz"] = ((int) (humLombriz*100))/100.0;
      doc["Humplanta"] = ((int) (humPlanta*100))/100.0;
      doc["flot1state"] = flot1state;
      doc["flot2state"] = flot2state;

      String Data2 = "{\"modo\":false,\"valva\":false,\"valvh\":true}";
      deserializeJson(doc2, Data2);
      serializeJson(doc2, Serial);
      Serial.println();
      serializeJson(doc, Serial);
      Serial.println(); 

      Serial.println(NumVal);
      Serial.print("Humus: ");
      Serial.println(humus);
      Serial.print("estado flujo: ");
      Serial.println(flujostate);
      inicio = millis();
    }
}