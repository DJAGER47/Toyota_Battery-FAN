#ifndef __MAIN_H__
#define __MAIN_H__

#ifdef __cplusplus
extern "C"
{
#endif

#define SIGNATURE 0x55AA55AA // сигнатура передаваемых пакетов

  // структура хранения данных CAN-пакета
  struct CANFrame
  {
    uint32_t ID;     // идентификатор пакета
    uint8_t Length;  // длина данных
    uint8_t Data[8]; // сами данные
  };

  // структура хранения данных отправляемых в последовательный порт или через Wi-Fi
  struct OutCANFrame
  {
    uint32_t Signature = SIGNATURE;
    CANFrame Frame;
  };

#define RST_ESP 16
#define CAN_CS 15
#define RST_MCP 4
#define CAN_INT 5

#ifdef __cplusplus
}
#endif

#endif
