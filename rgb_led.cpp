#include "rgb_led.h"

namespace rgb_led
{
RgbLed::RgbLed(PinName r, PinName g, PinName b) : red_(r), green_(g), blue_(b)
{
}

void RgbLed::setColor(RgbColors color)
{
  // LEDs are active low
  switch (color)
  {
    case RED:
      red_.write(0);
      green_.write(1);
      blue_.write(1);
      break;
    case GREEN:
      red_.write(1);
      green_.write(0);
      blue_.write(1);
      break;
    case BLUE:
      red_.write(1);
      green_.write(1);
      blue_.write(0);
      break;
    case YELLOW:
      red_.write(0);
      green_.write(0);
      blue_.write(1);
      break;
    case CYAN:
      red_.write(1);
      green_.write(0);
      blue_.write(0);
      break;
    case VIOLET:
      red_.write(0);
      green_.write(1);
      blue_.write(0);
      break;
    case WHITE:
      red_.write(0);
      green_.write(0);
      blue_.write(0);
      break;
    default:
      red_.write(1);
      green_.write(1);
      blue_.write(1);
      break;
  }
}

}  // namespace rgb_led
