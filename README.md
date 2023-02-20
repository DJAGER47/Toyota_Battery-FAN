// Для ESP8266 шилд подключен следующим образом:
// RST                     |  TX
// ADC                     |  RX
// CH_PD                   |  INT     - GPIO5  (D1)
// RST_ESP - GPIO16 (  )   |  RST_MCP - GPIO4
// SCK     - GPIO14 (D5)   |  RTS-DTR - GPIO0
// MISO    - GPIO12 (D6)   |  Pull Up - GPIO2
// MOSI    - GPIO13 (D7)   |  CS      - GPIO15 (D8)
// VCC                     |  GND