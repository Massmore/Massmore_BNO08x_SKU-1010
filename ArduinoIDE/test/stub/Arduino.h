#ifndef ARDUINO_STUB_H
#define ARDUINO_STUB_H
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define MSBFIRST 1
#define SPI_MODE3 3
#define SERIAL_8N1 0x800001c
typedef uint8_t byte;
typedef bool boolean;
unsigned long millis();
unsigned long micros();
void delay(unsigned long);
void delayMicroseconds(unsigned int);
void pinMode(int, int);
void digitalWrite(int, int);
int  digitalRead(int);
int  digitalPinToInterrupt(int);
class Print {
public:
  virtual size_t write(uint8_t) = 0;
  virtual size_t write(const uint8_t*, size_t);
  size_t print(const char*); size_t print(int); size_t print(unsigned); size_t print(float); size_t print(float,int); size_t print(double,int);
  size_t print(char); size_t print(unsigned long); size_t print(long);
  size_t print(int,int); size_t print(unsigned,int); size_t print(unsigned long,int);
  size_t print(uint8_t,int);
  size_t println(const char*); size_t println(int); size_t println(unsigned); size_t println(float); size_t println(float,int); size_t println(double,int);
  size_t println(char); size_t println(unsigned long); size_t println(long);
  size_t println(int,int); size_t println(unsigned,int); size_t println(unsigned long,int);
  size_t println();
  size_t printf(const char*, ...);
};
class Stream : public Print {
public:
  virtual int available() = 0;
  virtual int read() = 0;
  virtual int peek() = 0;
  size_t write(uint8_t) override { return 1; }
};


/* ---- extra stubs used by the examples ---- */
#ifndef F
#define F(x) (x)
#endif
#define HEX 16
#define DEC 10
#define BIN 2
class HardwareSerial : public Stream {
public:
  void begin(unsigned long);
  void begin(unsigned long, uint32_t, int, int);
  operator bool() const { return true; }
  int available() override; int read() override; int peek() override;
};
extern HardwareSerial Serial;
extern HardwareSerial Serial1;
#endif
