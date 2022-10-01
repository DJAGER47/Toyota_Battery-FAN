#include <SPI.h>
#include <mcp2515_can.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

#include "main.h"

// Для ESP8266 шилд подключен следующим образом:
// INT  - GPIO5  (D1)
// SCK  - GPIO14 (D5)
// MOSI - GPIO13 (D7)
// MISO - GPIO12 (D6)
// CS   - GPIO15 (D8)

#define CAN_INT 5   // сигнал INT на GPIO5 (D1)
#define CAN_CS  15  // сигнал CS на GPIO15 (D8)


// Если необходима отправка тестовых данных
#define TEST
#ifdef TEST
    #define TEST_INTERVAL 10
    uint32_t testCounter = millis();
    uint8_t  testInc = 0;
    uint8_t  testDec = 255;
#endif


// настройки точки доступа Wi-Fi
const char* ssid     = "CAR";                   // название беспроводной сети
const char* password = "1357908642";            // пароль к сети
const uint16_t udpPort = 0xAA55;                // номер порта для обмена данными
WiFiUDP UDP;                                    // UDP-сокет

#define  UDP_BUFFER_SIZE 1000                   // размер буфера пакетов
uint8_t  udpOutBuffer[UDP_BUFFER_SIZE] = { 0 }; // буфер накопления пакетов на отправку на компьютер
uint8_t* udpOutBufferPosition = udpOutBuffer;   // указатель на этот буфер
uint8_t  udpInBuffer[UDP_BUFFER_SIZE] = { 0 };  // буфер приёма пакета от компьютера

OutCANFrame outCANFrame;            // буфер отправки данных в буфер передачи через Wi-Fi
CANFrame    inCANFrame  = { 0 };    // принятый из последовательного порта или через Wi-Fi пакет для отправки в CAN-шину

// переменные для замера различных скоростных параметров
uint16_t counterFPS  = 0;           // CAN-пакеты в секунду
uint16_t counterBPS  = 0;           // байты в секунду
uint32_t counterTime = millis();

// флаг для обработки прерывания поступления данных из CAN-шины и объявление CAN-шины
volatile bool canDataReceived = false;
mcp2515_can CAN(CAN_CS);


void sendPacketToPC(void);
bool parseBuffer(uint8_t* receivedData, size_t length);
void serialEvent();


//###################################################################################################################################
// Обработчик прерывания поступивших данных
//###################################################################################################################################
IRAM_ATTR
void CANInterrupt(void)
{
    canDataReceived = true;     // установить флаг наличия данных в буфере
}

//###################################################################################################################################
// Инициализация
//###################################################################################################################################
void setup(void)
{
    // подключить обработчик прерывания о поступлении данных по спадающему уровню сигнала
    pinMode(CAN_INT, INPUT);
    attachInterrupt(digitalPinToInterrupt(CAN_INT), CANInterrupt, FALLING);

    // настройка скорости обмена и частоты кварца
    while (CAN.begin(CAN_500KBPS, MCP_16MHz) != CAN_OK)
    {
        delay(100);
    }
    // типы фильтров
    #define STD 0
    #define EXT 1
    
    // сброс фильтров
    CAN.init_Filt(0, STD, 0x0);
    CAN.init_Filt(1, STD, 0x0);
    CAN.init_Filt(2, STD, 0x0);
    CAN.init_Filt(3, STD, 0x0);
    CAN.init_Filt(4, STD, 0x0);
    CAN.init_Filt(5, STD, 0x0);
    
    // очистка масок
    CAN.init_Mask(0, STD, 0x0);
    CAN.init_Mask(1, STD, 0x0);

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

    Serial.begin(115200);
    IPAddress ip=WiFi.softAPIP();// получить IP-адрес
    Serial.println(ip);// распечатать IP-адрес
}

//###################################################################################################################################
// Основной цикл программы
//###################################################################################################################################
void loop(void)
{
    uint32_t currentTime = millis();

    #ifdef TEST
        if (currentTime - testCounter >= TEST_INTERVAL)
        {
            testCounter = currentTime;

            testInc++;
            testDec--;
            
            outCANFrame.Frame.ID = 0xABC;
            outCANFrame.Frame.Length = 8;
            outCANFrame.Frame.Data[0] = 0x11;
            outCANFrame.Frame.Data[1] = testInc;
            outCANFrame.Frame.Data[2] = 0x22;
            outCANFrame.Frame.Data[3] = testDec;
            outCANFrame.Frame.Data[4] = 0x33;
            outCANFrame.Frame.Data[5] = testInc;
            outCANFrame.Frame.Data[6] = 0x44;
            outCANFrame.Frame.Data[7] = testDec;
            
            canDataReceived = true;
        }
    #endif
    
    // если в CAN-буфере есть данные...
    if (canDataReceived)
    {
        // сбросить флаг данных в буфере
        canDataReceived = false;

    #ifndef TEST
        while (CAN.checkReceive() == CAN_MSGAVAIL)
    #endif
        {
            // чтение данных в буфер
        #ifndef TEST
            if (CAN.readMsgBuf(&outCANFrame.Frame.Length, outCANFrame.Frame.Data) == CAN_OK)
        #endif
            {
                // получение идентификатора пакета (достоверный только после чтения сообщения в буфер)
            #ifndef TEST
                outCANFrame.Frame.ID = CAN.getCanId();
            #endif
                sendPacketToPC();            
			
                // замеры скоростных параметров
                if (currentTime - counterTime >= 1000)
                {
                    counterTime = currentTime;
                    outCANFrame.Frame.ID = 0;
                    outCANFrame.Frame.Length = 4;
                    // при копировании little-endian, вместо big-endian, поэтому надо побайтово делать
                    // количество пакетов в секунду, примерная скорость около 745 пакетов в секунду
                    outCANFrame.Frame.Data[0] = highByte(counterFPS);  // counterFPS >> 8 & 0xFF;
                    outCANFrame.Frame.Data[1] = lowByte(counterFPS);   // counterFPS & 0xFF;
                    // количество байтов в секунду, примерная скорость около 9200 байтов в секунду
                    outCANFrame.Frame.Data[2] = highByte(counterBPS);  // counterBPS >> 8 & 0xFF;
                    outCANFrame.Frame.Data[3] = lowByte(counterBPS);   // counterBPS & 0xFF;

                    sendPacketToPC();
                    
                    counterFPS = 0;
                    counterBPS = 0;
                }

                counterFPS++;
                counterBPS += (5 + outCANFrame.Frame.Length);
            }
        }
    }

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
                    if (CAN.sendMsgBuf(inCANFrame.ID, 0, inCANFrame.Length, inCANFrame.Data) == CAN_OK)
                    {
                        // отправилось без ошибок
                    }
                }
            }
        }
    }
}

//###################################################################################################################################
// Отправка полученного CAN-пакета в Wi-Fi
//###################################################################################################################################
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

//###################################################################################################################################
// Парсер принятых данных
//###################################################################################################################################
bool parseBuffer(uint8_t* receivedData, size_t length)
{
    if (length >= 10 && length <= 17)
    {
        // проверка сигнатуры 
        if (*(uint32_t*)receivedData == SIGNATURE)
        {
            receivedData += 4;
            inCANFrame.ID = *(uint32_t*)receivedData;
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

