/*
 Examples to show how to publish data to Konker IoT hub using the MQTT protocol.

Device Requirements:
 * LG01
 * Arduino 1.5.8 IDE or above

Modified : 2018-08-10 helderbetiol@gmail.com

 */

//#include <Process.h>
#include <MemoryFree.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <SPI.h>
#include <RH_RF95.h>
#include <Console.h>
#include "YunClient.h"

YunClient yun;
PubSubClient client(yun);
RH_RF95 rf95;

#define BAUDRATE 115200
uint16_t crcdata = 0;
uint16_t recCRCData = 0;
float frequency = 868.0;

const char* mqtt_server = "mqtt.demo.konkerlabs.net";
const char* USER = "9rj1t6r46668";
const char* PWD = "mrqPCDHxffRt";
const char* PUB = "data/9rj1t6r46668/pub/temperature";
//const char* SUB = "";

uint16_t calcByte(uint16_t crc, uint8_t b)
{
    uint32_t i;
    crc = crc ^ (uint32_t)b << 8;
  
    for ( i = 0; i < 8; i++)
    {
      if ((crc & 0x8000) == 0x8000)
        crc = crc << 1 ^ 0x1021;
      else
        crc = crc << 1;
    }
    return crc & 0xffff;
}

uint16_t CRC16(uint8_t *pBuffer, uint32_t length)
{
    uint16_t wCRC16 = 0;
    uint32_t i;
    if (( pBuffer == 0 ) || ( length == 0 ))
    {
        return 0;
    }
    for ( i = 0; i < length; i++)
    {
        wCRC16 = calcByte(wCRC16, pBuffer[i]);
    }
    return wCRC16;
}

uint16_t recdata( unsigned char* recbuf, int Length)
{
    crcdata = CRC16(recbuf, Length - 2); //Get CRC code
    recCRCData = recbuf[Length - 1]; //Calculate CRC Data
    recCRCData = recCRCData << 8; //
    recCRCData |= recbuf[Length - 2];
}

void reconnect() {
  // Entra no Loop ate estar conectado
  while (!client.connected()) {
    Console.print("Attempting MQTT connection...");
    // Usando um ID unico (Nota: IDs iguais causam desconexao no Mosquito)
    // Tentando conectar
    if (client.connect(USER, USER, PWD)) {
      Console.println("connected");
      // Subscrevendo no topico esperado
      //client.subscribe(SUB);
    } else {
      Console.print("Falhou! Codigo rc=");
      Console.print(client.state());
      Console.println(" Tentando novamente em 5 segundos");
      // Esperando 5 segundos para tentar novamente
      delay(5000);
    }
  }
}

void setup() 
{
    // Initialize Bridge
    Bridge.begin(BAUDRATE);
    // Initialize Serial
    Console.begin();
    if (!rf95.init())
        Console.println("init failed");
    // Setup ISM frequency
    rf95.setFrequency(frequency);
    // Setup Power,dBm
    rf95.setTxPower(13);
    client.setServer(mqtt_server, 1883);
    Console.println("Start Listening ");
}

void loop()
{
  //Console.println("Start loop ");

  if (!client.connected()) {
    reconnect();
  }
  
  //mqtt_publish(10, 10);
  //delay(5000);
  
    if (rf95.waitAvailableTimeout(1000))// Listen Data from LoRa Node
    {
        uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];//receive data buffer
        uint8_t len = sizeof(buf);//data buffer length
        if (rf95.recv(buf, &len))//Check if there is incoming data
        {
            recdata( buf, len);
            if(crcdata == recCRCData) //Check if CRC is correct
            {
                Console.println("Get Data from LoRa Node");
                if(buf[0] == 1||buf[1] == 1||buf[2] ==1) //Check if the ID match the LoRa Node ID
                {
                    uint8_t data[] = "   Server ACK";//Reply 
                    data[0] = buf[0];
                    data[1] = buf[1];
                    data[2] = buf[2];
                    rf95.send(data, sizeof(data));// Send Reply to LoRa Node
                    rf95.waitPacketSent();
                    //Console.println("Replied LoRa Node");
                    //int newData[4] = {0, 0, 0, 0}; //Store Sensor Data here
                    int newData[2] = {0, 0};
                    for (int i = 0; i < 2; i++)
                    {
                        newData[i] = buf[i + 3];
                    }
                    
                    mqtt_publish(newData[0], newData[1]);
                }
            } else {
              Console.println("CRC data wrong ");      
            }
         }
         else
         {
              Console.println("recv failed");
          }
     }
}

void mqtt_publish(int h, int t)
{

  StaticJsonBuffer<60> JSONbuffer;
  JsonObject& JSONencoder = JSONbuffer.createObject();

  JSONencoder["temperature"] = t;
  JSONencoder["humidity"] = h;
  //JsonArray& values = JSONencoder.createNestedArray("values");
  //values.add(20);
  //values.add(21);
  //values.add(23);
 
  char JSONmessageBuffer[60];
  JSONencoder.printTo(JSONmessageBuffer, sizeof(JSONmessageBuffer));

  Console.println(JSONmessageBuffer);
  Console.println("Sending to Konker...");
  client.publish(PUB, JSONmessageBuffer); 
  client.loop(); 
  Console.println("Konker done");

  Console.print("freeMemory()=");
  Console.println(freeMemory());
}

