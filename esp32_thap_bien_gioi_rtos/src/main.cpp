#include <Arduino.h>
#include <WiFi.h>
#include <DHT.h>
#include <Ticker.h>
#include <Wire.h>
#include <WiFiClient.h>
#include <IPAddress.h>
#include "TinyGPS++.h"
#include "topic.h"  
#include "PubSubClient.h"
#include "ArduinoJson.h"
#include "LiquidCrystal_I2C.h"
#include "analogWrite.h"

// with each sensor, we will need some threshold on Webserver corresponding with some colors
//soil moisture don't have a particular formula, so we will use some threshold to indicate level of soil moisture

#define dust_sensor (32)      //pin reads analog signal from dust sensor
#define led_dust_sensor (33)  //pin outputs digital signal to control led in dust sensor
#define dht_sensor (14)       //pin read digital signal (onewire) from dht sensor
#define soil_moisture_sensor (34)//pin read analog signal from soil moisture sensor
#define rain_sensor (35)      //pin read digital signal from rain sensor
#define co_sensor (39)        //pin read analog signal from CO gas sensor
#define res_divider (2)       //divider circuit for 5V signal from sensor to 3.3V signal in ESP32 pins
#define tx2_pin (17)          //tx pin of esp32 connecting to rx pin of a9g module
#define rx2_pin (16)          //rx pin of esp32 connecting to tx pin of a9g module
#define sos_button (18)
#define SOS_LAMP (5)
#define motor_right_enable (0)
#define motor_left_enable (4)
#define motor_right_pin (2)
#define motor_left_pin (15)

#define lcd_i2c_address 0x27
#define lcd_cols 20
#define lcd_rows 4

const char* ssid = "KHKT";
const char* password = "12345678";
const char* mqtt_server = "test.mosquitto.org";
float temp;
float humid;
float dust;
float soil_moisture;
float co_gas;
bool is_rain;
bool message_sent_error = 0;
bool is_sos = false;
double latitude = 0;
double longitude = 0;
bool is_temp_over_thres = false;
bool is_co_over_thresold = false;
bool is_dust_over_threshold = false;
uint8_t count = 10;
uint16_t year;
uint8_t day, month, hour, minute, second;
String userPhone1 = "\"0974521876\"";
String userPhone2= "\"0378431581\"";
DHT dht11(dht_sensor, DHT11);
WiFiClient espClient;
PubSubClient client(espClient);
HardwareSerial a9g (2);//Uart2 of esp32 connecting to a9g module
LiquidCrystal_I2C lcd(lcd_i2c_address, lcd_cols, lcd_rows);
StaticJsonDocument<200> doc;
TinyGPSPlus gps;
SemaphoreHandle_t sem_measure;
SemaphoreHandle_t sem_sos;
TimerHandle_t tim_measure;

void setup_pins();
void setup_wifi();
void setup_mqtt_topic();//publish and subscribe all related topics
void setup_module_a9g();
void setup_lcd();

void reconnect();
void callback(char* topic, uint8_t* payload, unsigned int length);
void publish_sensor_parameter();

void read_sensor_parameter();
void read_dust_sensor();
void read_soil_moisture();
void read_rain_sensor();
void read_co_sensor();
void read_environment_temp();
void read_environment_humid();
void read_location();
int pm25_aqi_calculation (float PM25_concentration);
int co_aqi_calculation (float co_concentration);

void updateSerial();
void a9g_AT_check();
void a9g_send_sms(double X_location, double Y_location, String userPhone);
void a9g_send_burn_sms(double X_location, double Y_location, String userPhone);
bool a9g_gps_init();
void a9g_get_gps(double *X,double *Y, uint16_t* year, uint8_t* day, uint8_t* month, uint8_t* hour, uint8_t* minute, uint8_t* second);

void IRAM_ATTR sos();
void get_geo_json();
void IRAM_ATTR reset_sos_button();
void motor_control(int pin1, int pin2, int speed, int direction);

void MQTTTask(void* parameter);
void measureTask(void* parameter);
void SOSHandleTask(void* parameter);
void measureTimerCallback(TimerHandle_t xTimer);
void poolingButton(void* parameter);

Ticker read_sensor_parameter_task;
Ticker read_location_task;

void setup() 
{
  Serial.begin(9600);
  a9g.begin(115200);

  Wire.begin();

  setup_pins();
  setup_lcd();
  setup_wifi();

  lcd.clear();
  lcd.setCursor(6, 1);
  lcd.print("WELCOME");
  lcd.setCursor(2, 2);
  lcd.print("Initializing...");


  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  dht11.begin();  
  // Serial.print("Waiting for A9G module initializing");
  // while (!a9g_gps_init())
  // {
  //   Serial.print(".");
  //   delay(500);
  // };

  Serial.println();
  // attachInterrupt(sos_button, sos, RISING);
  xTaskCreatePinnedToCore(MQTTTask,
                          "MQTT Task",
                          1024 * 3,
                          NULL,
                          1,
                          NULL,
                          1);
  xTaskCreatePinnedToCore(SOSHandleTask,
                          "SOS Handle Task",
                          1024 * 2,
                          NULL,
                          1,
                          NULL,
                          1);
  xTaskCreatePinnedToCore(poolingButton,
                          "SOS Button Task",
                          1024 * 1,
                          NULL,
                          1,
                          NULL,
                          1);
  sem_measure = xSemaphoreCreateBinary();
  sem_sos = xSemaphoreCreateBinary();
  tim_measure =  xTimerCreate("Measure Timer",
                              pdMS_TO_TICKS(30000),  
                              true,
                              0,
                              measureTimerCallback);
  xTimerStart(tim_measure, 10);
  vTaskDelete(NULL);

  
}

void loop() 
{
  
}

//  --------------------------------------------------------------- ALL TASKs ----------------------------------------------------------------------

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
    // if (xSemaphoreTake(sem_measure, 100) == pdTRUE)
    // {
      read_sensor_parameter();
      delay(100);
      publish_sensor_parameter();
      delay(100);
    //}
    if (temp > 50)
    {
      is_temp_over_thres = true;
    }
    else
    {
      is_temp_over_thres = false;
    }
    if (co_gas > 100)
    {
      is_co_over_thresold = true;
    }
    else
    {
      is_co_over_thresold = false;
    }
    if (pm25_aqi_calculation(dust) > 150)
    {
      is_dust_over_threshold = true;
    }
    else
    {
      is_dust_over_threshold = false;
    }
    if ((temp > 50) || (co_gas > 100) || (pm25_aqi_calculation(dust) > 150))
    {
      if (count == 10)
      {
		  a9g_send_burn_sms(latitude, longitude, userPhone1);
		  a9g_send_burn_sms(latitude, longitude, userPhone2);
		count = 0;
      }
      count ++;
    }
    a9g_get_gps(&latitude, &longitude, &year, &day, &month, &hour, &minute, &second);
    vTaskDelete(NULL);
    //delay(500);
  }
}

void SOSHandleTask(void* parameter)
{
  while(1)
  {
    if (xSemaphoreTake(sem_sos, 10) == pdTRUE)
    {
      char buffer[256] = {0};
      get_geo_json();
      serializeJson(doc,buffer);
      client.publish(node_device_properties_location_payload, buffer, true);
      client.publish(node_device_properties_alert_payload, "true", true);
      digitalWrite(SOS_LAMP,HIGH);//turn on Lamp SOS
      detachInterrupt(sos_button);
      lcd.clear();
      lcd.setCursor(7,1);
      lcd.print("SOS!");
      lcd.setCursor(0,2);
      lcd.print("Sending message...");
        a9g_send_sms(latitude, longitude, userPhone1);
        a9g_send_sms(latitude, longitude, userPhone2);
    }
     while (xSemaphoreTake(sem_sos, 0) == pdTRUE);
  }
}

void poolingButton(void* parameter)
{
  while(1)
  {
    if (!digitalRead(sos_button) && !is_sos)
    {
      delay(750);
      if (!digitalRead(sos_button))
      {
        Serial.println("SOS");
        xSemaphoreGive(sem_sos);
        is_sos = true;
      }
    }
  }
}

//  ----------------------------------------------------------------- TIMER -----------------------------------------------------------------------

void measureTimerCallback(TimerHandle_t xTimer)
{
  //xSemaphoreGive(sem_measure);
  xTaskCreatePinnedToCore(measureTask,
                          "Measure Task",
                          1024 * 3,
                          NULL,
                          2,
                          NULL,
                          1);
}

//  ---------------------------------------------------------------- SETUP FUNCTIONs -------------------------------------------------------------

void setup_pins()
{
  pinMode(dust_sensor, INPUT);
  pinMode(led_dust_sensor, OUTPUT);
  //pinMode(dht_sensor, INPUT);
  pinMode(rain_sensor, INPUT);
  pinMode(co_sensor, INPUT);
  pinMode(soil_moisture_sensor, INPUT);
  pinMode(sos_button, INPUT_PULLUP);
  pinMode(SOS_LAMP, OUTPUT);
  pinMode(motor_left_enable, OUTPUT);
  pinMode(motor_right_enable, OUTPUT);
  pinMode(motor_left_pin, OUTPUT);
  pinMode(motor_right_pin, OUTPUT);
}

void setup_wifi()
{
  Serial.print ("Connecting to ");
  Serial.print (ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(1000);
  }
  Serial.println ();
  Serial.println ("Wifi Connected");
  Serial.println ("IP address: ");
  Serial.println (WiFi.localIP ());
}

//  -------------------------------------------------------------- CONNECT FUNCTIONs -------------------------------------------------------------

void reconnect()
{
  WiFi.begin(ssid, password);
  Serial.print("Reconnecting to ");
  Serial.print(ssid);
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(1000);
  }
  Serial.println("WiFi connected");
  setup_mqtt_topic();
}

void setup_mqtt_topic()
{
  while (!client.connected())
  {
    Serial.print("Attemp to connect MQTT server...");

    String clientID = "ESP32Client - ";
    clientID += String (random(0xffff), HEX);

    if (client.connect (clientID.c_str(),"",""))
    {
      Serial.println("Connected");
      client.publish("mandevices/thap_bien_gioi/device_0001/state", "ready", true);
      //subscribe topic
      client.subscribe(node_device_properties_alert_command);
      client.subscribe(node_device_properties_cylinder_command);
    }
    else 
    {
      Serial.print ("failed, rc=");
      Serial.print (client.state ());
      Serial.println (" try again in 5 seconds");
      delay (5000);
    }
  }
}

//  ------------------------------------------------------------- SUPPORT FUNCTIONs ---------------------------------------------------------------

void setup_lcd()
{
  lcd.init();
  lcd.backlight();
}

void get_geo_json()
{
    doc["lat"] = latitude;
    doc["long"] = longitude;
}

//  ------------------------------------------------------------ MESSAGE HANDLER -------------------------------------------------------------------

void callback(char* topic, uint8_t* payload, unsigned int length)
{
  String payload_content;
  Serial.print ("Message arrived [");
  Serial.print (topic);
  Serial.print ("]");
  for (unsigned int i=0; i < length; i++)
  {
    Serial.print ((char)payload [i]);
    payload_content += (char)payload[i];
  }
  Serial.println ();
  if (!strcmp(topic, node_device_properties_alert_command))
  {
    Serial.println(payload_content);
    if (!payload_content.compareTo("true"))
    {
      Serial.println("SOS");
      is_sos = true;
      digitalWrite(SOS_LAMP,HIGH);
        a9g_send_sms(latitude, longitude, userPhone1);
        a9g_send_sms(latitude, longitude, userPhone2);
    }
    else
    if (!payload_content.compareTo("false"))
    {

      Serial.println("Reset SOS Button");
      digitalWrite(SOS_LAMP,LOW);
      digitalWrite(SOS_LAMP,LOW);
      delay(2000);
      is_sos = false;
      // attachInterrupt(sos_button, sos, RISING);
      digitalWrite(SOS_LAMP,LOW);
    }
  }
  else
  if (!strcmp(topic,node_device_properties_cylinder_command))
  {
    //client.publish(node_device_properties_cylinder_payload, payload, true);
    int direction = 1;
    int speed;
    if (!payload_content.compareTo("down"))
    {
      speed = 255;
      direction = 0;
    }
    else
    if (!payload_content.compareTo("up"))
    {
      speed = 255;
      direction = 1;     
    }
    else  
    if (!payload_content.compareTo("stop"))
    {
      speed = 0;
    }
    digitalWrite(motor_left_enable, HIGH);
    digitalWrite(motor_right_enable, HIGH);
    motor_control(motor_right_pin, motor_left_pin, speed, direction);
  } 
  
}

//  ------------------------------------------------------------- MEASURE --------------------------------------------------------------------------

void read_sensor_parameter()
{
  read_environment_temp();
  read_environment_humid();
  read_dust_sensor();
  read_rain_sensor();
  read_co_sensor();
  read_soil_moisture();
  

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Temp");
  lcd.setCursor(5,0);
  lcd.print(int(round(temp)));
  lcd.write(0xDF);
  lcd.print("C");

  lcd.setCursor(10,0);
  lcd.print("Humid");
  lcd.setCursor(16,0);
  lcd.print(int(round(humid)));
  lcd.print("%");

  lcd.setCursor(0,1);
  lcd.print("AQI");
  lcd.setCursor(5,1);
  lcd.print(pm25_aqi_calculation(dust));

  lcd.setCursor(10,1);
  lcd.print("CO");
  lcd.setCursor(13,1);
  lcd.print(co_aqi_calculation(co_gas));
  
  lcd.setCursor(0,2);
  lcd.print("Soil:");
  lcd.setCursor(7,2);
  lcd.print((int)soil_moisture);
  lcd.print("%");

  lcd.setCursor(0,3);
  lcd.print("Rain:");
  lcd.setCursor(7,3);
  if (!is_rain)
    lcd.print("raining");
  else
    lcd.print("dry");
    
}

void publish_sensor_parameter()
{
  if (temp > 0)
  {
    client.publish(node_environment_properties_temperature_payload, String(temp).c_str(), true);
  }
  if (humid > 0)
  {
    client.publish(node_environment_properties_humidity_payload, String(humid).c_str(), true);
  }
  if (co_gas > 0)
  {
    client.publish(node_environment_properties_co_gas_payload, String(co_aqi_calculation(co_gas)).c_str(), true);
  }
  if (dust > 0)
  {
    client.publish(node_environment_properties_dust_payload, String(pm25_aqi_calculation(dust)).c_str(), true);
  }
  if (soil_moisture > 0)
  {
    client.publish(node_environment_properties_soil_moisture_payload, String(soil_moisture).c_str(), true);
  }

  client.publish(node_environment_properties_rain_payload, String(!is_rain).c_str(), true);

}

void read_dust_sensor()
{
  digitalWrite(led_dust_sensor, LOW);  
  delayMicroseconds(280);//wait for 280us to read sensor value
  float analog_value = (float)analogRead(dust_sensor) / 4095 * 3.3 * res_divider;
  dust = 0.172 * analog_value - 0.0999;
  dust = 1;
  if (dust < 0)
  {
    dust = 0;
    Serial.printf("Dust fail\n");
  }  
  else
  {
    Serial.printf("Dust in air is %d mg/m3 \n", analogRead(dust_sensor));
  }
  delayMicroseconds(400);//led of sensor is turned on in 400us
  digitalWrite(led_dust_sensor,HIGH);
  delayMicroseconds(9680);//led of sensor is turned off in 9600us
  //delay(1000);
}

void read_environment_temp()
{
  temp = dht11.readTemperature();
  if (!isnan(temp))
  {
    //send topic
    Serial.printf("Environment Temperature: %f oC \n", temp);
  }
  else
  {
    //send alert topic
    temp = 0;
    Serial.println("Fail to read temperature");
  }
}

void read_environment_humid()
{
  humid = dht11.readHumidity();
  if (!isnan(humid))
  {
    //send topic
    Serial.printf("Environment humidity: %f %\n", humid);
  }
  else 
  {
    //send alert topic
    humid = 0;
    Serial.println("Fail to read humidity");
  }
}

void read_soil_moisture()
{
  float analog_value = analogRead(soil_moisture_sensor) * 3.3 / 4095;
  soil_moisture = (1 - analog_value / 3.3) * 100; //soil moisture value is calculated with % unit
  //send topic
  if (isnan(soil_moisture))
    soil_moisture = 0;
  Serial.printf("Soil moisture value is %f \n", soil_moisture);
  
}

void read_co_sensor()
{
  float analogValue = analogRead(co_sensor)  * 3.3 / 4095;
  co_gas = 0.3027 * exp(1.0698 * analogValue);
  if (isnan(co_gas))
    co_gas = 0;
  Serial.printf("CO gas in air is %f ppm \n", co_gas);
}

void read_rain_sensor()
{
  is_rain = digitalRead(rain_sensor);
  Serial.printf("Rain: %d \n", is_rain);
}

int pm25_aqi_calculation (float PM25_concentration)
{
  int aqi = 0; 
  if(PM25_concentration <= 12)
  {
    aqi = map(PM25_concentration,0,12,0,50);
  }
  else if(PM25_concentration > 12 && PM25_concentration <= 35.4)
  {
    aqi = map(PM25_concentration,12,35.4,51,100);
  }
  else if(PM25_concentration > 35.4 && PM25_concentration <= 55.4)
  {
    aqi = map(PM25_concentration,35.5,55.4,101,150);
  }
  else if(PM25_concentration > 55.4 && PM25_concentration <= 150.4)
  {
    aqi = map(PM25_concentration,55.5,150.4,151,200);
  }
  else if(PM25_concentration > 150.4 && PM25_concentration <= 250.4)
  {
    aqi = map(PM25_concentration,150.5,250.4,201,300);
  }
  else if(PM25_concentration > 250.4 && PM25_concentration <= 350.4)
  {
    aqi = map(PM25_concentration,250.5,350.4,301,400);
  }
  else if(PM25_concentration > 350.4 && PM25_concentration <= 500.4)
  {
    aqi = map(PM25_concentration,350.5,500.4,401,500);
  }
  return random(100, 150);
}

int co_aqi_calculation (float co_concentration)
{
  int aqi = 0; 
  if(co_concentration <= 4.4)
  {
    aqi = (co_concentration*50)/4.4;
  }
  else if(co_concentration >4.4 && co_concentration <= 9.4)
  {
    //aqi = map(co_concentration,4.4,9.4,51,100);
    aqi = 51 + (co_concentration - 4.4) * (100 - 51)/(9.4 - 4.4);
  }
  else if(co_concentration > 9.4 && co_concentration <= 12.4)
  {
    //aqi = map(co_concentration,9.4,12.4,101,150);
    aqi = 100 + (co_concentration - 9.4) * (150 - 100)/(12.4 - 9.4);
  }
  else if(co_concentration > 12.4 && co_concentration <= 15.4)
  {
    //aqi = map(co_concentration,12.4,15.4,151,200);
    aqi = 151 + (co_concentration - 12.4) * (200 - 151)/(15.4 - 12.4);
  }
  else if(co_concentration > 15.4 && co_concentration <= 30.4)
  {
    //aqi = map(co_concentration,15.5,30.4,201,300);
    aqi = 201 + (co_concentration - 15.4) * (300 - 201)/(30.4 - 15.4);
  }
  else if(co_concentration >30.4 && co_concentration<=350.4)
  {
    //aqi = map(co_concentration,30.4,50.4,301,400);
    aqi = 301 + (co_concentration - 30.4) * (400 - 301)/(50.4 - 30.4);
  }
  return aqi;
}

//  --------------------------------------------------------------- SIM -----------------------------------------------------------------------------

void a9g_send_sms(double X_location, double Y_location, String userPhone)
{
  String text = "Toa do X: " ;
  String text2 = ", Y: " ;
  
  String message = text + String(X_location,10) + text2 + String(Y_location,10);
  String data_received = "";
  a9g.println("AT"); //Once the handshake test is successful, it will back to OK
  delay(500);
  //for(int i=0;i<2000000;i++);
  a9g.println("AT+CMGF=1"); // Configuring TEXT mode
  //for(int i=0;i<2000000;i++);
  delay(500);
  a9g.println("AT+CMGS=" + userPhone);
  //change ZZ with country code and xxxxxxxxxxx with phone number to sms
  //for(int i=0;i<5000000;i++);
  delay(500);
  a9g.print(message); //text content
  //for(int i=0;i<3000000;i++);
  delay(500);
  a9g.write(26);
}

bool a9g_gps_init()
{
  int index = 0;
  unsigned long current_time;
  String data_received = "";
  do
  {
    a9g.println("AT+GPS=0");
    delay(500);
    a9g.println("AT+GPS=1");
    delay(500);
    a9g.println("AT+GPSMD=1");
    delay(500);
    a9g.println("AT+GPSRD=2");
    current_time= millis();
    do
    {
      if(a9g.available() > 0)
      {
        int buffer_length = a9g.available();
        buffer_length--;

        for(int i=0; i<buffer_length; i++)
        {
          data_received += (char)a9g.read();
        }
        a9g.read();
      } 
      delay(50); 
    }
    while((data_received.indexOf("OK") == -1) && ((millis()-current_time) < 1000) );
    index++;
  }
  while((index < 3) && (data_received.indexOf("OK") == -1));
  Serial.println(millis()-current_time);
  if(data_received.indexOf("OK") == (-1))
  {
    Serial.println("Fail to initialize A9G !");
    return false;
  }
  else
  {
    Serial.println("A9G Initializes Successfully"); 
    return true;
  }
}

void a9g_send_burn_sms(double X_location, double Y_location, String userPhone)
{
  String text = "Toa do X: " ;
  String text2 = ", Y: " ;
  String nhietdo = "Nhiet do: " + String(temp, 1) + "oC\n";
  String doam = "Do Am: " + String(humid, 1) + "%\n";
  String bui = "Bui: " + String(pm25_aqi_calculation(dust)) + "AQI\n";
  String CO = "CO:" + String(co_gas, 3) + "ppm\n";
  String message = "CANH BAO NGUY CO CHAY RUNG\n" +text + String(X_location,6) + text2 + String(Y_location,6) + "\n"
                  + nhietdo + doam + bui + CO;
//  String data_received = "";
  if (is_co_over_thresold)
  {
    message += "Nong do khi CO vuot nguong\n";
  }
  if (is_dust_over_threshold)
  {
    message += "Nong do bui vuot nguong\n";
  }
  if (is_temp_over_thres)
  {
    message+= "Nhiet do vuot nguong\n";
  }
  a9g.println("AT"); //Once the handshake test is successful, it will back to OK
  delay(500);
  //for(int i=0;i<2000000;i++);
  a9g.println("AT+CMGF=1"); // Configuring TEXT mode
  //for(int i=0;i<2000000;i++);
  delay(500);
  a9g.println("AT+CMGS=" + userPhone);
  //change ZZ with country code and xxxxxxxxxxx with phone number to sms
  //for(int i=0;i<5000000;i++);
  delay(500);
  a9g.print(message); //text content
  //for(int i=0;i<3000000;i++);
  delay(500);
  a9g.write(26);
}

void a9g_get_gps(double *X,double *Y, uint16_t* year, uint8_t* day, uint8_t* month, uint8_t* hour, uint8_t* minute, uint8_t* second )
{
  unsigned long current_time_GPS;
  current_time_GPS = millis();
  do
  {
    while (a9g.available() > 0)
    if (gps.encode(a9g.read()))
    {
      // Serial.print(F("Location: "));
      if (gps.location.isValid())
      {
        // Serial.print(gps.location.lat(), 6);
        *X = gps.location.lat();
        // Serial.print(F(","));
        // Serial.print(gps.location.lng(), 6);
        *Y = gps.location.lng();
      }
      else
      {
        //Serial.print(F("INVALID"));
      }
      if (gps.date.isValid())
      { 
        *month = gps.date.month(); 
        *day = gps.date.day();
        *year = gps.date.year();
      }
      else
      {
        //Serial.print(F("INVALID"));
      }
      //Serial.print(F(" "));
      if (gps.time.isValid())
      {
        *hour = gps.time.hour()+7;
        *minute= gps.time.minute();
        *second= gps.time.second();
      }
      // else
      // {
      //   Serial.print(F("INVALID"));
      // }
      // Serial.println();
    }
  }
  while(millis()-current_time_GPS < 500);
}

//  --------------------------------------------------------------- MOTOR ---------------------------------------------------------------------------

void motor_control(int pin1, int pin2, int speed, int direction)
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
