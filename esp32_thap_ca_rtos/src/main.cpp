#include <Arduino.h>
#include <WiFi.h>
#include "DallasTemperature.h"
#include "Sensor.h"
#include <PubSubClient.h>
#include "WiFiClient.h"
#include <OneWire.h>
#include "topic.h"
#include <Ticker.h>
#include "analogWrite.h"
#include <TinyGsmClient.h>

#define motor_can_left_pin (12)
#define motor_can_right_pin (13)
#define motor_tray_left_pin (18)
#define motor_tray_right_pin (19) 
#define motor_fan_left_pin (15)
#define motor_fan_right_pin (2)
#define motor_cylinder_left_pin (27)
#define motor_cylinder_right_pin (14)
#define publish_environment_freq 50000

#define ph_sensor_pin (36)
#define do_sensor_pin (34)
#define temp_sensor_pin (4)
const char* ssid = "KHKT";
const char* password = "12345678";
const char* mqtt_server = "test.mosquitto.org";

WiFiClient espClient;
PubSubClient client(espClient);
Sensors Environment(ph_sensor_pin, do_sensor_pin);
OneWire onewire(temp_sensor_pin);
DallasTemperature ds18b20(&onewire);
TinyGsm sim800l(Serial2);
SemaphoreHandle_t sem_measure;
TimerHandle_t tim_measure;

double pH = 0;
double DO = 0;
double Temperature = 0;
bool is_message_sent_error = 0;//check if sim800l sent threshold message successfully
bool is_o2_over_threshold = false;//check if o2 is over threshold
bool is_pH_over_threshold = false;//check if pH is over threshold
bool is_temp_over_threshold = false;//check if temp is over threshold
bool is_enough_message = false; //check if the number of threshold message is enough
bool is_get_o2_threshold = false;
bool is_get_pH_threshold = false;
bool is_get_temp_threshold = false;

void setup_wifi (void);
void callback (char* topic, byte* payload, unsigned int length);
void reconnect (void);
void measureTask(void* parameter);
void sim800l_send_message(int O2_vuot_nguong, int PH_vuot_nguong, int nhietdo_vuot_nguong);
void motor_control(int pin1, int pin2, int speed, bool direction);
void MQTTTask(void* parameter);
void measureTimerCallback(TimerHandle_t xTimer);

void setup()
{
  Serial.begin(9600);
  Serial2.begin(9600);
  ds18b20.begin(); 

  analogWrite(motor_fan_left_pin, 0);
  analogWrite(motor_fan_right_pin, 0);

  analogWrite(motor_cylinder_left_pin, 0);
  analogWrite(motor_cylinder_right_pin, 0);

  analogWrite(motor_can_left_pin, 0);
  analogWrite(motor_can_right_pin, 0);

  analogWrite(motor_tray_left_pin, 0);
  analogWrite(motor_tray_right_pin, 0);

  //setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  xTaskCreatePinnedToCore(MQTTTask,
                          "MQTT Task",
                          1024 * 4,
                          NULL,
                          1,
                          NULL,
                          1);
  xTaskCreatePinnedToCore(measureTask,
                          "Measure Task",
                          1024 * 2,
                          NULL,
                          1,
                          NULL,
                          1);
  Serial.println("Task OK");
  sem_measure = xSemaphoreCreateBinary();
  tim_measure =  xTimerCreate("Measure Timer",
                              publish_environment_freq,
                              true,
                              0,
                              measureTimerCallback);
  xTimerStart(tim_measure, 10);
  vTaskDelete(NULL);
}

void loop()
{
}

void MQTTTask(void* parameter)
{
  while(1)
  {
    if (!client.connected())
    {
      reconnect();
    }
    else
    {
      client.loop();
    }
  }
}

void measureTask(void* parameter)
{
  while(1)
  {
    if (xSemaphoreTake(sem_measure, 10) == pdTRUE)
    {
      ds18b20.requestTemperatures();
      delay(500);
      Temperature = ds18b20.getTempCByIndex(0);
      DO = Environment.Get_DO_ug(Temperature);
      pH = Environment.Get_pH_Value();
      Serial.printf("Temp: %2.1f \n", Temperature);
      Serial.printf("DO: %1.3f \n", DO);
      Serial.printf("pH: %1.2f \n", pH);
      if ((DO > 0) && (Temperature > 0) && (pH > 0))
      {
        client.publish(node_environment_properties_DO_payload, String(DO).c_str(), true);
        client.publish(node_environment_properties_Temperature_payload, String(Temperature).c_str(), true);
        client.publish(node_environment_properties_pH_payload, String(pH).c_str(), true);
      }
    }
    if ((is_get_o2_threshold) && (is_get_pH_threshold) && (is_get_temp_threshold))
    {
      if (is_o2_over_threshold || is_pH_over_threshold || is_temp_over_threshold)
      {
        sim800l_send_message(is_o2_over_threshold, is_pH_over_threshold, is_temp_over_threshold);
      }
      is_get_temp_threshold = false;
      is_get_o2_threshold = false;
      is_get_pH_threshold = false;
    }
  }
}

void measureTimerCallback(TimerHandle_t xTimer)
{
  xSemaphoreGive(sem_measure);
}
void setup_wifi (void)
{
  Serial.print("Connecting to ");
  Serial.print(ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while(WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(1000);
  }
  Serial.println();
  Serial.println("Wifi Connected!");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback (char* topic, byte* payload, unsigned int length)
{
  String payload_content;
  Serial.print ("Message arrived [");
  Serial.print (topic);
  Serial.print ("]");
  for (unsigned int i = 0; i < length; i++)
  {
    Serial.print ((char)payload[i]);
    payload_content += (char)payload[i];
  }
  Serial.println ();
  if (!strcmp(topic, node_device_properties_foot_can_cmdset))
  {
    //client.publish(node_device_properties_foot_can_payload, payload, true);
    int dutyCycle = payload_content.toInt() * 255 / 100; 
    bool direction = true;
    if (dutyCycle > 0)
      direction = true;
    else
      direction = false;
    motor_control(motor_can_right_pin, motor_can_left_pin, dutyCycle, direction);  
  }
  else
  if (!strcmp(topic, node_device_properties_foot_tray_cmdset))
  {
    //.publish(node_device_properties_foot_tray_payload, payload, true);
    int dutyCycle = payload_content.toInt() * 255 / 100; 
    Serial.println(dutyCycle);
    bool direction = true;
    if (dutyCycle > 0)
      direction = true;
    else
      direction = false;
    motor_control(motor_tray_right_pin, motor_tray_left_pin, dutyCycle, direction);  
  }
  else
  if (!strcmp(topic, node_device_properties_fan_cmdset))
  {
    //client.publish(node_device_properties_fan_payload, payload, true);
    int dutyCycle = payload_content.toInt() * 255 / 100; 
    bool direction = true;
    if (dutyCycle > 0)
      direction = true;
    else
      direction = false;
    motor_control(motor_fan_right_pin, motor_fan_left_pin, dutyCycle, direction);  
  }
  else
  if (!strcmp(topic, node_device_properties_cylinder_cmdset))
  {
    bool direction = true;
    int speed = 0;
    //client.publish(node_device_properties_cylinder_payload, payload, true);
    if (!payload_content.compareTo("up"))
    {
      direction = true;
      speed = 255;
    }
    else
    if (!payload_content.compareTo("down"))
    {    
      direction = false;
      speed = 255;
    }
    else
    if (!payload_content.compareTo("stop"))
    {
      speed = 0;
    }
    motor_control(motor_cylinder_right_pin, motor_cylinder_left_pin, speed, direction);  
  }
  else
  if (!strcmp(topic, node_environment_properties_DO_threshold_set))
  {
    is_get_o2_threshold = true;
    if (!payload_content.compareTo("false"))
    {
      is_o2_over_threshold = true;
      client.publish(node_environment_properties_DO_threshold, "true", true);
    }
    else
    if (!payload_content.compareTo("true"))
    {
      is_o2_over_threshold = false;
      client.publish(node_environment_properties_DO_threshold, "false", true);
    }
  }
  else
  if (!strcmp(topic, node_environment_properties_Temperature_threshold_set))
  {
    is_get_temp_threshold = true;
    if (!payload_content.compareTo("false"))
    {
      is_temp_over_threshold = true;
      client.publish(node_environment_properties_Temperature_threshold, "true", true);
    }
    else
    if (!payload_content.compareTo("true"))
    {
      is_temp_over_threshold = false;
      client.publish(node_environment_properties_Temperature_threshold, "false", true);
    }
  }
  else
  if (!strcmp(topic, node_environment_properties_pH_threshold_set))
  {
    is_get_pH_threshold = true;
    if (!payload_content.compareTo("false"))
    {
      is_pH_over_threshold = true;
      client.publish(node_environment_properties_pH_threshold, "true", true);
    }
    else
    if (!payload_content.compareTo("true"))
    {
      is_pH_over_threshold = false;
      client.publish(node_environment_properties_pH_threshold, "false", true);
    }
  }
}

void reconnect (void)
{
  setup_wifi();
  // if (WiFi.status() != WL_CONNECTED)
  // {
  //   while ((WiFi.status() != WL_CONNECTED))
  //   {
  //     Serial.print(".");
  //     delay(1000);
  //   }
  //   Serial.println ();
  //   Serial.println ("Wifi Connected");
  //   Serial.println ("IP address: ");
  //   Serial.println (WiFi.localIP ());
  //   Serial.println(WiFi.macAddress ());
  // }

  while (!client.connected())
  {
    Serial.print ("Attemping MQTT connection...");
    //Create a client ID
    String clientID = "ESP32Client-";
    clientID += String (random(0xffff), HEX);
    //Attempt to connect
    if (client.connect(clientID.c_str(), "", "")) 
    {
      Serial.println("Connected");
                        /***********Subcribe***********/
      client.subscribe(node_device_properties_foot_can_cmdset);
      client.subscribe(node_device_properties_foot_tray_cmdset);
      client.subscribe(node_device_properties_fan_cmdset);
      client.subscribe(node_device_properties_cylinder_cmdset);
      client.subscribe(node_environment_properties_DO_threshold_set);
      client.subscribe(node_environment_properties_pH_threshold_set);
      client.subscribe(node_environment_properties_Temperature_threshold_set);
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state ());
      Serial.println(" try again in 5 seconds");
      delay (5000);
    }
  }
}


void sim800l_send_message(int O2_vuot_nguong, int PH_vuot_nguong, int nhietdo_vuot_nguong)
{
  Serial.println("Send message");
  int i=0;
  String warning1, warning2, warning3;
  String text1 = "VUOT NGUONG! ";
  String text2 = "O2 : ";
  String text3 = "mg/L, PH: ";
  String text4 = ", Nhiet do: ";
  String text5 = "*C ";
  String comma1,comma2;

  if(O2_vuot_nguong)
  {
    warning1 = "O2 ";
  }
  else
  {
    warning1 = "";
  }
  if(PH_vuot_nguong)
  {
    warning2 = "PH ";
  }
  else
  {
    warning2 = "";
  }
  if(nhietdo_vuot_nguong)
  {
    warning3 = "Nhiet Do ";
  }
  else
  {
    warning3 = "";
  }
  if(O2_vuot_nguong && PH_vuot_nguong)
  {
    comma1 = ", ";
  }
  else
  {
    comma1 = "";
  }
  if(PH_vuot_nguong && nhietdo_vuot_nguong)
  {
    comma2 = ", ";
  }
  else
  {
    comma2 = "";
  }
    
    
  String message = warning1 + comma1 + warning2 + comma2 + warning3 + text1 + text2 
                 + DO + text3 + pH + text4 + int(round(Temperature)) + text5 ;
  sim800l.sendSMS("0869186397", message);
  // String data_received ="";
  // unsigned long current_time;
  // do
  // {  
  //   sim800l.println("AT"); //Once the handshake test is successful, it will back to OK
  //   delay(500);
  //   sim800l.println("AT+CMGF=1"); // Configuring TEXT mode
  //   delay(500);
  //   sim800l.println("AT+CMGS=\"0869186397\"");//change ZZ with country code and xxxxxxxxxxx with phone number to sms
  //   delay(500);
  //   sim800l.print(message); //text content
  //   delay(500);
  //   sim800l.write(26);
  //   current_time= millis();
  //   do
  //   {
  //     if(sim800l.available() > 0)
  //     {
  //       int h = sim800l.available();
  //       h--;
  //       for(int i = 0; i < h; i++)
  //       {
  //         data_received += (char)sim800l.read();
  //       }
  //       sim800l.read();
  //   } 
  //     delay(50); 
  //   }
  //   while((data_received.indexOf("CMGS:")==-1) && ((millis()-current_time)<8000) );
  //   i++;
  // }
  // while(i < 3 && (data_received.indexOf("CMGS:") == -1));
  // if(data_received.indexOf("CMGS:") == -1)
  // {
  //   Serial.println("Error Sending Message!");
  //   is_message_sent_error=1;
  // }
  // else
  // {
  //   Serial.println("Message Sent Successfully"); 
  //   is_message_sent_error=0;
  // }
  //   Serial.println(millis() - current_time);
}

void motor_control(int pin1, int pin2, int speed, bool direction)
{
  analogWriteFrequency(10000);
  
  if(direction == 1)
  {
    analogWrite(pin1,speed);
    analogWrite(pin2,0);
  }
   else
  {
    analogWrite(pin1,0);
    analogWrite(pin2,speed);
  }
}