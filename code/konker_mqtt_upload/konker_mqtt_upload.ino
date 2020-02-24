/*
 Examples to show how to publish data to Konker IoT hub using the MQTT protocol.

Device Requirements:
 * LG01
 * Arduino 1.5.8 IDE or above

Modified : 2018-08-10 helderbetiol@gmail.com

 */

//#include <stdio.h>
#include <Process.h>
#include <MemoryFree.h>
//#include <ArduinoJson.h>
//unsigned long lastMillis = 0;
#include <SPI.h>
#include <RH_RF95.h>
#include <Console.h>
//#include "YunClient.h"
//YunClient client;
RH_RF95 rf95;

//If you use Dragino IoT Mesh Firmware, uncomment below lines.
//For product: LG01. 
#define BAUDRATE 115200

//If you use Dragino Yun Firmware, uncomment below lines.
//For product: Yun Shield 
//#define BAUDRATE 250000
//unsigned long myChannelNumber = 555383;
//const char * myWriteAPIKey = "87BPGHF5V2LGP4RZ";
uint16_t crcdata = 0;
uint16_t recCRCData = 0;
float frequency = 868.0;

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
    Console.println("Start Listening ");
}

void loop()
{
  //Console.println("Start loop ");
  //mqtt_publish(10, 10);
  
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
                    //int h = newData[0];
                    //int t = newData[1];
                    
                    //Console.println("Sending to Konker...");
                    mqtt_publish(newData[0], newData[1]);
                    //Console.println("Konker done");
                }
            } else {
              Console.println("CRC data wrong ");      
            }
         }
         else
         {
              Console.println("recv failed");
              //;
          }
     }
//    if(millis() - lastMillis > 20000) // Upload Per 20 seconds
//    {
//        lastMillis = millis();
//        Console.println("Publish a data to Konker");
//        mqtt_publish(28, 27);
//    }
}

//String generate_data() // Generate a random data
//{
//    String count = "" ;
//    count = random(100, 500);
//    return count;
//}

void mqtt_publish(int h, int t)
{
    Process p;    // Create a process and call it "p"
    p.begin("mosquitto_pub"); // Process that launch the "mosquitto_pub" command
    //p.addParameter("-d");
    p.addParameter("-h");
    p.addParameter(F("mqtt.demo.konkerlabs.net"));
    //p.addParameter("-i");
    //p.addParameter("deviceId-wY8HTBUTtu");  // Add Device ID
    p.addParameter("-p");
    p.addParameter("1883");
    //p.addParameter("-q");
    //p.addParameter("0");
    p.addParameter("-m");
    String value = String(t);
    String str1 = "{\"temperature\": "; 
    String str2 = "";
    str2 = str1 + value; 
    str1 = str2 + ", \"humidity\": ";
    value = String(h);
    str2 = str1 + value;
    str1 = str2 + "}";

//StaticJsonBuffer<60> JSONbuffer;
//  JsonObject& JSONencoder = JSONbuffer.createObject();
//
//  JSONencoder["temperature"] = t;
//  JSONencoder["humidity"] = h;
  //JsonArray& values = JSONencoder.createNestedArray("values");
  //values.add(20);
  //values.add(21);
  //values.add(23);
 
  //char JSONmessageBuffer[60];
//  String JSONmessageBuffer = "";
//  JSONencoder.printTo(JSONmessageBuffer);//, sizeof(JSONmessageBuffer));
//    String str1 = "";
//    String str2 = "";
//    str2 = "\"" + JSONmessageBuffer;
//    str1 = str2 + "\"";
    //sprintf(str, "{\"temperature\": %d, \"unit\": \"celsius\", \"humidity\": %d}", t, h);
//    JSONmessageBuffer[0] = "\"";
//    JSONmessageBuffer[1] = "{";
//    JSONmessageBuffer[50] = "\"";
//    Console.println(JSONmessageBuffer);
//    p.addParameter(JSONmessageBuffer);
    Console.println(str1);
    p.addParameter(str1);
    //p.addParameter("{\"temperature\": 29, \"unit\": \"celsius\", \"humidity\": 70}");
    p.addParameter("-u");
    p.addParameter(F("9rj1t6r46668"));      // User Name for IoT hub
    p.addParameter("-P");
    p.addParameter(F("mrqPCDHxffRt"));   // Password for IoT Hub
    p.addParameter("-t");
    p.addParameter(F("data/9rj1t6r46668/pub/temperature"));    // Publish to this topic
    p.run();    // Run the process and wait for its termination
    //Console.println("roda shell");

    Console.print("freeMemory()=");
    Console.println(freeMemory());

    // A process output can be read with the stream methods
    //while (p.available() > 0) {
      //char c = p.read();
      //Console.print(c);
    //}
  //   Ensure the last bit of data is sent.
    //Console.flush();
    //Console.println(str);
}

