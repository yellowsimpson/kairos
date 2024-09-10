#include "motor.h"

motor pin(9, 10);

void setup()
{
  pin.begin();
}

void loop()
{
  pin.turn_on();
  delay(1000);
  pin.turn_off();
  delay(1000);
}
