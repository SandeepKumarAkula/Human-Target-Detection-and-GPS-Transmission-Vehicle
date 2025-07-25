#include <WiFi.h>
#include <PubSubClient.h>

int IN1 = 2;
int IN2 = 4;
int IN3 = 26;
int IN4 = 27;

const char* ssid = "Sandeep";
const char* password =  "123456789";
const char* mqtt_server = "192.168.0.103";
const int mqtt_port = 1883;
WiFiClient espClient;
PubSubClient client(espClient);
void setup() 
{
  
 
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT); 
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT); 
  analogWrite(IN1, 0);
  analogWrite(IN2, 0);
  analogWrite(IN3, 0);
  analogWrite(IN4, 0);
  
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) 
  {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }
  Serial.print("Connected to WiFi :");
  Serial.println(WiFi.SSID());
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(MQTTcallback);
  while (!client.connected()) 
  {
    Serial.println("Connecting to MQTT...");
    if (client.connect("ESP8266"))
    {
      Serial.println("connected");
    }
    else
    {
      Serial.print("failed with state ");
      Serial.println(client.state());
      delay(2000);
    }
  }
  client.subscribe("test2");
}
void MQTTcallback(char* topic, byte* payload, unsigned int length) 
{
  Serial.print("Message received in topic: ");
  Serial.println(topic);
  Serial.print("Message:");
  String message;
  for (int i = 0; i < length; i++) 
  {
    message = message + (char)payload[i];
  }
  Serial.print(message);
  if (message == "forward") 
  {
    
    analogWrite(IN1, 0);
    analogWrite(IN2, 150);
    analogWrite(IN3, 0);
    analogWrite(IN4, 150);
    
  }
   else if (message == "turn") 
  { 
    
  
   
    analogWrite(IN1, 0);
    analogWrite(IN2, 150);
    
    
  
   
 
 
   
    
    
   
  }
  else if (message == "turn1")
  { 
    
   
  
   
    
    analogWrite(IN1, 150);
    analogWrite(IN2, 0);
  }  
  
   
   
   
   
   
 

  
    
  
  else if (message == "stop") 
  {
    
    analogWrite(IN1, 0);
    analogWrite(IN2, 0);
    analogWrite(IN3, 0);
    analogWrite(IN4, 0);
     
    
  }
  Serial.println();
  Serial.println("-----------------------");
}
void loop() 
{
  
  client.loop();
}
