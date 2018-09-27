#ifndef __MOTOR_DRIVER_H
#define __MOTOR_DRIVER_H

//define the pins -- use what you want, but be sure to get them correct
#define DIR_PIN_1 2 
#define DIR_PIN_2 3 
#define PWM_PIN 4 

#define ENC_PIN_A 10
#define ENC_PIN_B 11

#define FORWARD 1
#define BRAKING 0
#define REVERSE (-1)

//for managing the encoder interrupts
volatile int32_t encoderCount = 0;
void ISR_A(void)
{
  if(digitalRead(ENC_PIN_A) == digitalRead(ENC_PIN_B)) encoderCount++;
  else encoderCount--;
}
void ISR_B(void)
{
  if(digitalRead(ENC_PIN_A) == digitalRead(ENC_PIN_B)) encoderCount--;
  else encoderCount++;
}

class MotorDriver
{
protected:
  //direction pins
  int dir1, dir2;

  //speed pin
  int pwmPin;

public:
  //takes direction pins and pwm pin as inputs -- does NOT control the second pwm pin or the enable pin (I recommend you hard-wire those)
  MotorDriver(int d1, int d2, int pwm) : dir1(d1), dir2(d2), pwmPin(pwm)
  {
  }

  void Init(void)
  {
    pinMode(dir1, OUTPUT);
    pinMode(dir2, OUTPUT);
    pinMode(pwmPin, OUTPUT);

    FullStop();

    //attach the encoder interrupts
    attachInterrupt(digitalPinToInterrupt(ENC_PIN_A), ISR_A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_PIN_B), ISR_B, CHANGE);
  }

  void SetDirection(int direction)
  {
    if(direction == FORWARD)
    {
      digitalWrite(dir1, HIGH);
      digitalWrite(dir2, LOW);
    }
    else if(direction == REVERSE)
    {
      digitalWrite(dir1, LOW);
      digitalWrite(dir2, HIGH);
    }
    else //full stop
    {
      digitalWrite(dir1, HIGH);
      digitalWrite(dir2, HIGH);
    }
  }

  void SetPower(int power) 
  {
    power = constrain(power, -255, 255); //limit power to -255 <= power <= 255
    if(power > 0) SetDirection(FORWARD);
    else SetDirection(REVERSE);
    
    analogWrite(pwmPin, abs(power));
  }

  void FullStop(void)
  {
    SetPower(0);
    SetDirection(BRAKING);
  }
};

#endif

