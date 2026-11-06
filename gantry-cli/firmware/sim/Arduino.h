#pragma once
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string>
#include <deque>
#include <cmath>
#include <ctype.h>
#define IRAM_ATTR
#define RISING 1
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define LOW 0
#define HIGH 1
#define SERIAL_8N1 0x800001c
#define ESP_ARDUINO_VERSION_MAJOR 3
#define ARDUINO_USB_CDC_ON_BOOT 1
struct String {
  std::string s;
  String() {} String(const char* c): s(c?c:"") {} String(int v): s(std::to_string(v)) {} String(unsigned v): s(std::to_string(v)) {} String(unsigned long v): s(std::to_string(v)) {}
  void reserve(size_t n){ s.reserve(n);} unsigned length() const {return s.size();} const char* c_str() const { return s.c_str(); }
  String& operator+=(const String& o){ s+=o.s; return *this;} String& operator+=(const char* o){ s+=o; return *this;}
};
uint32_t micros(); uint32_t millis(); void delay(uint32_t ms);
struct Stream {
  std::deque<int> in; std::string out; bool connected = true;
  virtual ~Stream() {}
  int available(){ return in.size(); } int read(){ if(in.empty()) return -1; int c=in.front(); in.pop_front(); return c; }
  size_t print(const char* x){ out += x; return 0; }
  void feed(const std::string& t){ for(char c: t) in.push_back((unsigned char)c); }
};
struct HWCDC : Stream { void begin(int){} void setTxTimeoutMs(uint32_t){} explicit operator bool() const { return connected; } };
struct HardwareSerial : Stream { void begin(unsigned long, uint32_t, int8_t, int8_t){} size_t setRxBufferSize(size_t n){return n;} size_t setTxBufferSize(size_t n){return n;} };
extern HWCDC Serial; extern HardwareSerial Serial1;
void pinMode(int,int); int digitalRead(int);
inline void noInterrupts(){} inline void interrupts(){}
inline int digitalPinToInterrupt(int p){ return p; }
void attachInterrupt(int, void(*)(), int);
void detachInterrupt(int);
void digitalWrite(int,int);
bool ledcAttach(int,uint32_t,uint8_t); bool ledcWrite(int,uint32_t);
