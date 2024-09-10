#include "Arduino.h"
#include "motor.h"

motor::motor(int pin, int pin1)
{
  pin_numA = pin;
  pin_numB = pin1;
}

void motor::begin()
{
  pinMode(pin_numA, OUTPUT);
  pinMode(pin_numB, OUTPUT);
}

void motor::turn_on()
{
  digitalWrite(pin_numA, HIGH);
  digitalWrite(pin_numB, LOW);
}

void motor::turn_off()
{
  digitalWrite(pin_numA, LOW);
  digitalWrite(pin_numB, HIGH);
}
