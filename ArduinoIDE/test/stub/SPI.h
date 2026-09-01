#ifndef SPI_STUB_H
#define SPI_STUB_H
#include "Arduino.h"
class SPISettings { public: SPISettings(uint32_t, uint8_t, uint8_t) {} };
class SPIClass {
public:
  void begin(); void beginTransaction(SPISettings); void endTransaction();
  uint8_t transfer(uint8_t);
};
extern SPIClass SPI;
#endif
