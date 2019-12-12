/**
 * @file IoT_esp32.ino
 * @brief Temperatura e umidade IoT - Main
 * @author Cássio F. Braga e Nathan Engelmann Nogara
 * @date 12-09-17
 */

#include <PubSubClient.h>
#include <WiFiClientSecure.h>
#include <WiFi.h>
#include <DHT.h>
#include "esp_deep_sleep.h"
#include <NTPClient.h>
#include <WiFiUdp.h>

#define US_TO_S 1000000   //microssegundos para segundos
#define SEGUNDOS 60
#define LOCAL_GMT -3

#define ESP_ON 33.9
#define ESP_OFF 0.15
#define SENSOR_ON 2.5
#define SENSOR_OFF 0.1

/*
 * Definição do DHT11
 */
#define DHTPIN 23 // pino de dados do DHT11
#define DHTTYPE DHT11 // define o tipo de sensor, no caso DHT11

/*
 * Plataforma para conectar o dispositivo ligado ao wifi - Tago.io
 */

const char* ssid = "CASSIO";
const char* password = "34321256";
const char* mqttServer = "mqtt.tago.io";
const int mqttPort = 8883;
const char* mqttUser = "esp32";
const char* mqttPassword = "159e1ccb-9277-473b-b701-6ae3e50cedcc";

int LED_BUILTIN = 2;
int humidity = 0;
int temperature = 0;
int ligado;
int desligado;
int t_total;

float tON, tOFF;
float corrente;
 
DHT dht(DHTPIN, DHTTYPE);

WiFiClientSecure espClient;
PubSubClient client(espClient);

// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

// Variables to save date and time
String formattedDate;
String dayStamp;
String timeStamp;
String DataFinal;

void sensorDHT();
void initSerial();
void initWiFi();
void initTempo();
void Tempo();
void CalculaConsumo();
void reconnectMQTT();
void envia();
void dorme();

//! @brief Configura o esp
//! @param nada
void setup() {
    pinMode (LED_BUILTIN, OUTPUT);

    initSerial();

    initWiFi();

    client.setServer(mqttServer, mqttPort);
    //client.setCallback(callback);

    reconnectMQTT();

    dht.begin();

    initTempo();

    Serial.print("Tentando enviar a mensagem");
}

//! @brief Coleta e salva os dados do sensor DHT
//! @param nada
void sensorDHT()
{
    humidity = dht.readHumidity();
    temperature = dht.readTemperature();
    // testa se retorno é valido, caso contrário algo está errado.
    if (isnan(temperature) || isnan(humidity)) 
    {
      Serial.println("Falha na leitura do DHT");
    }
    else
    {
      Serial.print("Umidade: ");
      Serial.print(humidity);
      Serial.print("%\t");
      Serial.print("Temperatura: ");
      Serial.print(temperature);
      Serial.println("°C");
    }
}

//! @brief Inicia a comunicação serial
//! @param nada
void initSerial() 
{
    Serial.begin(115200);
}

//! @brief Inicia a comunicação WiFi
//! @param nada
void initWiFi()
{
    //Loga no WiFi
    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print("Connecting to WiFi:");
        Serial.println(ssid);
    }

    Serial.println("Connected to the WiFi network");
    Serial.println("");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
}

//! @brief Seta parâmetros do tempo
//! @param nada
void initTempo()
{
  // Initialize a NTPClient to get time
  timeClient.begin();
  // Set offset time in seconds to adjust for your timezone, for example:
  // GMT +1 = 3600
  // GMT +8 = 28800
  // GMT -1 = -3600
  // GMT 0 = 0
  timeClient.setTimeOffset(3600*LOCAL_GMT);
}

//! @brief Converte valores do tempo
//! @param nada
void Tempo()
{
    while(!timeClient.update()) {
    timeClient.forceUpdate();
  }
  // The formattedDate comes with the following format:
  // 2018-05-28T16:00:13Z
  // We need to extract date and time
  formattedDate = timeClient.getFormattedDate();
  Serial.println(formattedDate);

  // Extract date
  int splitT = formattedDate.indexOf("T");
  dayStamp = formattedDate.substring(0, splitT);
  Serial.print("DATE: ");
  Serial.println(dayStamp);
  // Extract time
  timeStamp = formattedDate.substring(splitT+1, formattedDate.length()-1);
  Serial.print("HOUR: ");
  Serial.println(timeStamp);

  DataFinal = dayStamp + String(" ") + timeStamp;

  Serial.println(DataFinal);
  
}

//! @brief Cálculo do consumo do esp
//! @param nada
void CalculaConsumo()
{
  desligado = US_TO_S*SEGUNDOS;
  ligado = micros();
  t_total = desligado + ligado;

  tON = (float)ligado/t_total;
  tOFF = (float)desligado/t_total;

  //Enquanto ligado, de acordo com o datasheet do esp32, a potência do consumo é de 20,5dBm, totalizando 112mW,
  //tendo em vista que a alimentação é de 3v3, temos que o consumo é de 33,9mA
  //Desligado, o consumo é de 150uA
  //O sensor consome 2,5mA enquanto mede, e 150uA em standby
  //OBS: valores em casos extremos
  //tON == Tempo que o esp e o sensor ficaram ligados (em %)
  //tOFF == Tempo que o esp e o sensor ficaram desligados (em %)
  /*!
   * ESP_ON == 33.9mA
     ESP_OFF == 0.15mA
     SENSOR_ON == 2.5mA
     SENSOR_OFF == 0.1mA

     Se a média de consumo for de 90% ON e 10% OFF, temos que:
     I = 3,86mA

     Se a bateria for 1200mAh:
     1200mAh/3,86mA = 310h = 13 dias    <- Valores aproximados

     
   */
  corrente = tON*(ESP_ON + SENSOR_ON) + tOFF*(ESP_OFF + SENSOR_OFF);
}

//! @brief Faz a conexão MQTT
//! @param nada
void reconnectMQTT() 
{
     while (!client.connected()) {
        Serial.println("Connecting to MQTT…");
        String clientId = "ESP32Client-";
        clientId += String(random(0xffff), HEX);
        if (client.connect(clientId.c_str(), mqttUser, mqttPassword )) {
            Serial.println("connected");
        } else {
            Serial.print("failed with state ");
            Serial.print(client.state());
            delay(2000);
        }
    }
}

//! @brief Envia os dados para o broker
//! @param nada
void envia()
{
    sensorDHT();
    Tempo();

    Serial.println("Mensagem enviada com sucesso...\n");

    //String MSG = String("{ \"variable": "temperature", "unit"    : "F", "value\"   : 55, \"time\"    : \"2015-11-03 13:44:33\", \"location\": {\"lat\": 42.2974279, \"lng\": -85.628292} }");
    
    String TMP = String("{\"variable\":\"Temperatura\", \"value\":") + String(temperature)+ String(",\"unit\":\"ºC\"}");
    client.publish("tago/data/post", TMP.c_str());

    String UMI = String("{\"variable\":\"Umidade\", \"value\":") + String(humidity)+ String(",\"unit\":\"%\"}");
    client.publish("tago/data/post", UMI.c_str());

    CalculaConsumo();
    
    String CUR = String("{\"variable\":\"Consumo\", \"value\":") + String(corrente)+ String(",\"unit\":\"mA\"}");
    client.publish("tago/data/post", CUR.c_str());
}

//! @brief Coloca o esp para dormir
//! @param nada
void dorme()
{
    //Dorme por 1 minuto
    esp_deep_sleep_enable_timer_wakeup(US_TO_S*SEGUNDOS);
    esp_deep_sleep_start();
}

//! @brief Código que roda em loop
//! @param nada
void loop() {
    reconnectMQTT();
    envia();
    dorme();
}
