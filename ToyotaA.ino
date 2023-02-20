#include <SPI.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

#include "main.h"
#include "mcp2515_can.h"

// настройки точки доступа Wi-Fi
const char *ssid = "CAR";            // название беспроводной сети
const char *password = "1357908642"; // пароль к сети
const uint16_t udpPort = 0xAA55;     // номер порта для обмена данными
WiFiUDP UDP;                         // UDP-сокет

#define UDP_BUFFER_SIZE 1000                  // размер буфера пакетов
uint8_t udpOutBuffer[UDP_BUFFER_SIZE] = {0};  // буфер накопления пакетов на отправку на компьютер
uint8_t *udpOutBufferPosition = udpOutBuffer; // указатель на этот буфер
uint8_t udpInBuffer[UDP_BUFFER_SIZE] = {0};   // буфер приёма пакета от компьютера

OutCANFrame outCANFrame;   // буфер отправки данных в буфер передачи через Wi-Fi
CANFrame inCANFrame = {0}; // принятый из последовательного порта или через Wi-Fi пакет для отправки в CAN-шину

// переменные для замера различных скоростных параметров
uint16_t counterFPS = 0; // CAN-пакеты в секунду
uint16_t counterBPS = 0; // байты в секунду
uint32_t counterTime = millis();

// флаг для обработки прерывания поступления данных из CAN-шины и объявление CAN-шины
volatile bool canDataReceived = false;
mcp2515_can CAN(CAN_CS);

void sendPacketToPC(void);
bool parseBuffer(uint8_t *receivedData, size_t length);

// ###################################################################################################################################
//  Обработчик прерывания поступивших данных
// ###################################################################################################################################
IRAM_ATTR void CANInterrupt(void)
{
  canDataReceived = true; // установить флаг наличия данных в буфере
}

// ###################################################################################################################################
//  Инициализация
// ###################################################################################################################################
void setup(void)
{
  Serial.begin(115200);

  pinMode(RST_MCP, OUTPUT);
  digitalWrite(RST_MCP, HIGH);

  // подключить обработчик прерывания о поступлении данных по спадающему уровню сигнала
  pinMode(CAN_INT, INPUT);
  attachInterrupt(digitalPinToInterrupt(CAN_INT), CANInterrupt, FALLING);

  // настройка скорости обмена и частоты кварца
  while (CAN.begin(CAN_500KBPS, MCP_8MHz) != CAN_OK)
  {
    Serial.println("\nCAN BUS Shield init fail");
    Serial.println(" Init CAN BUS Shield again");
    delay(100);
  }
  Serial.println("\nCAN BUS Shield init ok!");

  // // типы фильтров
  // #define STD 0
  // #define EXT 1

  // // сброс фильтров
  // CAN.init_Filt(0, STD, 0x0);
  // CAN.init_Filt(1, STD, 0x0);
  // CAN.init_Filt(2, STD, 0x0);
  // CAN.init_Filt(3, STD, 0x0);
  // CAN.init_Filt(4, STD, 0x0);
  // CAN.init_Filt(5, STD, 0x0);

  // // очистка масок
  // CAN.init_Mask(0, STD, 0x0);
  // CAN.init_Mask(1, STD, 0x0);

  // включение точки доступа
  WiFi.softAPConfig(IPAddress(192, 168, 1, 1),
                    IPAddress(192, 168, 1, 1),
                    IPAddress(255, 255, 255, 0));

  WiFi.mode(WIFI_AP);
  while (!WiFi.softAP(ssid, password))
  {
    delay(100);
  }
  UDP.begin(udpPort);

  IPAddress ip = WiFi.softAPIP(); // получить IP-адрес
  Serial.println(ip);             // распечатать IP-адрес
}

// ###################################################################################################################################
//  Основной цикл программы
// ###################################################################################################################################
void loop(void)
{
  uint32_t currentTime = millis();

  // если в CAN-буфере есть данные...
  // if (canDataReceived)
  // {
  // сбросить флаг данных в буфере
  canDataReceived = false;

  while (CAN.checkReceive() == CAN_MSGAVAIL)
  {
    Serial.println("Resive");
    // чтение данных в буфер
    if (CAN.readMsgBuf(&outCANFrame.Frame.Length, outCANFrame.Frame.Data) == CAN_OK)
    {
      // получение идентификатора пакета (достоверный только после чтения сообщения в буфер)
      outCANFrame.Frame.ID = CAN.getCanId();
      sendPacketToPC();

      // замеры скоростных параметров
      if (currentTime - counterTime >= 1000)
      {
        counterTime = currentTime;
        outCANFrame.Frame.ID = 0;
        outCANFrame.Frame.Length = 4;
        // при копировании little-endian, вместо big-endian, поэтому надо побайтово делать
        // количество пакетов в секунду, примерная скорость около 745 пакетов в секунду
        outCANFrame.Frame.Data[0] = highByte(counterFPS); // counterFPS >> 8 & 0xFF;
        outCANFrame.Frame.Data[1] = lowByte(counterFPS);  // counterFPS & 0xFF;
        // количество байтов в секунду, примерная скорость около 9200 байтов в секунду
        outCANFrame.Frame.Data[2] = highByte(counterBPS); // counterBPS >> 8 & 0xFF;
        outCANFrame.Frame.Data[3] = lowByte(counterBPS);  // counterBPS & 0xFF;

        sendPacketToPC();

        counterFPS = 0;
        counterBPS = 0;
      }

      counterFPS++;
      counterBPS += (5 + outCANFrame.Frame.Length);
    }
  }
  // }

  {
    // Проверка и приём данных из Wi-Fi
    int packetSize = UDP.parsePacket();
    if (packetSize)
    {
      int length = UDP.read(udpInBuffer, UDP_BUFFER_SIZE);
      if (length > 0)
      {
        // если принятые данные распарсились, то в inCANFrame будет принятый пакет
        if (parseBuffer(udpInBuffer, length))
        {
          Serial.println("Send");
          // if (CAN.sendMsgBuf(inCANFrame.ID, 0, inCANFrame.Length, inCANFrame.Data) == CAN_OK)
          // {
          //     // отправилось без ошибок
          // }
        }
      }
    }
  }
}

// ###################################################################################################################################
//  Отправка полученного CAN-пакета в Wi-Fi
// ###################################################################################################################################
void sendPacketToPC(void)
{
  size_t dataLength = 9 + outCANFrame.Frame.Length;
  // накопление данных в буфере, это необходимо для увеличения пропускной способности
  if (UDP_BUFFER_SIZE - (udpOutBufferPosition - udpOutBuffer) >= 17)
  {
    // добавить данные в буфер
    memcpy(udpOutBufferPosition, &outCANFrame, dataLength);
    udpOutBufferPosition += dataLength;
  }
  else
  {
    // отправить данные
    UDP.beginPacket(IPAddress(192, 168, 1, 255), udpPort);
    UDP.write(udpOutBuffer, udpOutBufferPosition - udpOutBuffer);
    UDP.endPacket();

    udpOutBufferPosition = udpOutBuffer;
  }
}

// ###################################################################################################################################
//  Парсер принятых данных
// ###################################################################################################################################
bool parseBuffer(uint8_t *receivedData, size_t length)
{
  if (length >= 10 && length <= 17)
  {
    // проверка сигнатуры
    if (*(uint32_t *)receivedData == SIGNATURE)
    {
      receivedData += 4;
      inCANFrame.ID = *(uint32_t *)receivedData;
      receivedData += 4;
      inCANFrame.Length = *receivedData;
      receivedData++;
      for (size_t i = 0; i < inCANFrame.Length; i++)
      {
        inCANFrame.Data[i] = *receivedData++;
      }
      return true;
    }
  }
  return false;
}
