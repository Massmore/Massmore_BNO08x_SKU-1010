#ifndef WIRE_STUB_H
#define WIRE_STUB_H
#include "Arduino.h"
class TwoWire : public Stream {
public:
  bool begin(); bool begin(int,int);
  void beginTransmission(uint8_t);
  uint8_t endTransmission();
  size_t requestFrom(uint8_t, uint8_t);
  size_t write(uint8_t) override;
  size_t write(const uint8_t*, size_t) override;
  int available() override; int read() override; int peek() override;
  bool setBufferSize(size_t);
  void setTimeOut(uint16_t);
  void setClock(uint32_t);
};
extern TwoWire Wire;
#endif
