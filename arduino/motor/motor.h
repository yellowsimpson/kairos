#ifndef MOTOR_H
#define MOTOR_H

class motor
{
  public:
    motor(int pin, int pin1);
    void begin();
    void turn_on();
    void turn_off();

    int pin_numA;
    int pin_numB;
};

#endif
