#ifndef RGB_LED_H_
#define RGB_LED_H_

#include "mbed.h"

namespace rgb_led
{
typedef enum
{
  OFF = 0,
  RED,
  GREEN,
  BLUE,
  YELLOW,
  CYAN,
  VIOLET,
  WHITE
} RgbColors;

class RgbLed
{
 public:
  RgbLed(PinName r, PinName g, PinName b);
  void setColor(RgbColors color);

 private:
  DigitalOut red_;
  DigitalOut green_;
  DigitalOut blue_;
};

}  // namespace rgb_led

#endif  // RGB_LED_H_
