//***********************************//
//Modulo des temperatura y humedad   //
//                                   //
//Matias Pierdominici                //
//mpierdominici@itba.edu.ar          //
//***********************************//
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <SimpleDHT.h>
#define DEBUGG


#define PIN_SENSOR D2

#define PIR_SENSOR D1

class myTimer
{
  public:
  myTimer(unsigned int seconds=0);
  bool timeOver(void);
  void setNewTime(unsigned long seconds_);
  void showInfo();
  
  unsigned long seconds;
  unsigned long startTime;
  void resetTimer(void);
    
};





myTimer rateData(2);
SimpleDHT11 sensor;

char * ssid ="WIFI Pier";
char * pass ="pagle736pagle";
unsigned int mqttPort=1883;

const char MqttUser[]="humedadBox";
const char MqttPassword[]="1234";
const char MqttClientID[]="humBox";

bool pirSensorState=false;

IPAddress mqttServer(192,168,0,116);

WiFiClient wclient;
PubSubClient mqtt_client(wclient);





void callback(char* topic, byte* payload, unsigned int length);
void  debug_message (char * string, bool newLine)
{
#ifdef DEBUGG
  if(string !=NULL)
  {
    if (!newLine)
    {
      Serial.print(string);
    }else
    {
      Serial.println(string);
    }
  }
  #endif
}

void setUpWifi(char * ssid, char * pass)
{
  String ip;
  debug_message(" ",true);
  debug_message(" ",true);
  debug_message("Conectandose a: ",false);
  debug_message(ssid,true);

  WiFi.begin(ssid,pass);

  while(WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    debug_message(".",false);
  }
  debug_message(" ",true);
  debug_message("Coneccion realizada",true);
  debug_message("La ip es: ",false);
  ip=WiFi.localIP().toString();
  debug_message((char *)ip.c_str(),true);
}

void setUpMqtt(void)
{
  mqtt_client.setServer(mqttServer,mqttPort);
  mqtt_client.setCallback(callback);
}


void callback(char* topic, byte* payload, unsigned int length)
{
  int tiempo=0;
  payload[length]='\n';
  String message((char *)payload);
  debug_message("Llego un mensage, topic:",false);
  debug_message(topic,false);
  debug_message(", payload : ",false);
  debug_message((char *)payload,true);

  if(!strcmp(topic,"boxSense/pirState"))
  {
    if (length>0)//solo un byte
    {
      //Serial.println(payload[0]);
      if(payload[0]==0 || payload[0]=='0')
      {
        debug_message("se desactivo la alarma",true);
        pirSensorState=false;
      }
       else
    {
      debug_message("se activo la alarma",true);
      pirSensorState=true;
    }
    }
   
  }

}

void reconnect()
{
  while(!mqtt_client.connected())
  {
    debug_message("Intentando conectar al servidor MQTT",true);
    if (mqtt_client.connect(MqttClientID,MqttUser,MqttPassword))
      {
            debug_message("conectado",true);
  
  
            // ...suscrivirse a topicos
            mqtt_client.subscribe("boxSense/setDataFrecuence");
            mqtt_client.subscribe("boxSense/pirState");
            


      }
      else
      {
        debug_message("intentando conetarse al broker",true);
        delay(3000);
      }
  }
}

void setup() {
  Serial.begin(9600);
  setUpWifi(ssid,pass);
  setUpMqtt();
  pinMode(PIR_SENSOR,OUTPUT);

}



void loop() {
  byte temp=0;
  byte hum=0;
  if (!mqtt_client.connected()) 
  {
      reconnect();
      
 }
 mqtt_client.loop();

 if(rateData.timeOver())
 {
    rateData.resetTimer();
    if(sensor.read(PIN_SENSOR,&temp,&hum,NULL)==SimpleDHTErrSuccess)
    {
      
      mqtt_client.publish("boxSense/temp",(String(temp)).c_str());
      mqtt_client.publish("boxSense/hum",(String(hum)).c_str()); 
        
    }
     //if detecte
      if(pirSensorState==true)
      {
        if(digitalRead(PIR_SENSOR)==true)
        {
          debug_message("se detecto movimiento",true);
          mqtt_client.publish("boxSense/motionDetection","1");
        }
        else
        {
          debug_message("no se detecto movimiento",true);
          mqtt_client.publish("boxSense/motionDetection","0");
        }
        
         
      }
    
 }




 

}





//***********************TIMER**********************************



myTimer::myTimer(unsigned int seconds)
{
  setNewTime(seconds);
}

//timeOver
//devuelve true si ya paso el tiempo seteado,
//caso contrario devuelve false
//
bool myTimer::timeOver(void)
{
  if((millis())>startTime)
  {
    resetTimer();
    return true;
  }
  else
  {
    return false;
  }
}

void myTimer::resetTimer(void)
{
  unsigned long temp=seconds+millis();
 
  startTime=temp;
  //Serial.print("se llamo a rest timer con: ");
  //Serial.println(startTime);
}

void  myTimer::setNewTime(unsigned long seconds_)
{
  unsigned long temp=1000*seconds_;
  //Serial.println(temp);
  seconds=temp;
 
  //Serial.print("s seteo un timer cada: ");
  //Serial.print(seconds_);
  //Serial.print(" se registro un tirmpo de: ");
  //Serial.println(seconds/1000);
  resetTimer();

}

void myTimer::showInfo()
{
  //Serial.println(startTime);
  unsigned long dif=startTime-millis();
  //Serial.print("Remaining time (seconds):");
  //Serial.println(dif/1000);
  //Serial.println(startTime);
  //Serial.println(millis());
  //Serial.println(seconds/1000);
}



