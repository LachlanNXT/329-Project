#include "TimerThree.h"

#define M1 14
#define M2 10
#define M3 4
#define M4 12
#define encoder_left 5
#define encoder_right 8

double encoder_to_rev = 909.72;
boolean forward;

volatile int state_L = 0;
volatile int state_R = 0;
volatile int count_L;
volatile int count_R;
const double pi = 3.14159;
volatile double Rad_L;
volatile double Rad_R;

volatile int pwm_L = 0;
volatile int pwm_R = 0;

char input;
byte pos = 0;
double Left_Speed = 0;
double Right_Speed = 0;
char off_value = 50;
bool Left_direction = true;
bool Right_direction = true;
char data;

int ledPin = 11;

int turnOff = 0;
bool countOff = false;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  pinMode(encoder_left, INPUT);
  pinMode(encoder_right, INPUT);
  attachInterrupt(encoder_left, doEncoderL, RISING);
  attachInterrupt(encoder_right, doEncoderR, RISING);
  Timer3.initialize(10000); // 10000 micro seconds, 10 milliseconds, 0.01 seconds
  Timer3.attachInterrupt(reset_timer); // attach timer interrupt
  //Serial.println("L=Left motor speed");
  //Serial.println("R=Right motor speed");
  //Serial.println("l=left direction(1 forward, 0 backward):  ");
  //Serial.println("r=right direction(1 forward, 0 backward):  ");

  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);
}

void doEncoderL() {
  state_L++;
}

void doEncoderR() {
  state_R++;
}

void reset_timer() {
  if (countOff) {
    turnOff++;
    if (turnOff > 10) {
      digitalWrite(ledPin, LOW);
      countOff = false;
    }
  }
  count_L = state_L;
  count_R = state_R;
  Rad_L = ((2 * pi) / encoder_to_rev * count_L) / .01;
  Rad_R = ((2 * pi) / encoder_to_rev * count_R) / .01;
  state_L = 0;
  state_R = 0;

  int dpwmL = round((Left_Speed - Rad_L));

  int dpwmR = round((Right_Speed - Rad_R));

  pwm_L += dpwmL;

  pwm_R += dpwmR;

  if (pwm_L > 255)
  {
    pwm_L = 255;
  }

  if (pwm_L < 0)
  {
    pwm_L = 0;
  }
  if (pwm_R > 255)
  {
    pwm_R = 255;
  }

  if (pwm_R < 0)
  {
    pwm_R = 0;
  }
  /*
      if (Rad_L < Left_Speed)
      {
        pwm_L++;
        if (pwm_L > 255)
        {
          pwm_L = 255;
          //digitalWrite(ledPin, HIGH);
        }
      }
      else
      {
        pwm_L--;
        //digitalWrite(ledPin, LOW);
        if (pwm_L < 0)
        {
          pwm_L = 0;
        }

      }

      if (Rad_R < Right_Speed)
      {
        pwm_R++;
        if (pwm_R > 255)
        {
          pwm_R = 255;
          //digitalWrite(ledPin, HIGH);
        }
      }
      else
      {
        pwm_R--;
        //digitalWrite(ledPin, LOW);
        if (pwm_R < 0)
        {
          pwm_R = 0;
        }


      }
      */

}

void Motor_R(int pwm, bool forward)
{
  if (forward == true)
  {
    analogWrite(M1, pwm);
    analogWrite(M2, 0);
  }
  else
  {
    analogWrite(M2, pwm);
    analogWrite(M1, 0);

  }
}

void Motor_L(int pwm, bool forward)
{
  if (forward == true)
  {
    analogWrite( M3, pwm);
    analogWrite(M4, 0);
  }
  else
  {
    analogWrite(M4, pwm);
    analogWrite(M3, 0);
  }
}
void Stop_R()
{
  digitalWrite(M1, HIGH);
  digitalWrite(M2, HIGH);
}
void Stop_L()
{
  digitalWrite(M3, HIGH);
  digitalWrite(M4, HIGH);
}

void loop() {
  // put your main code here, to run repeatedly:

  Motor_L(pwm_L, true);
  Motor_R(pwm_R, true);
  // first char read
  if (Serial.available()) {
    digitalWrite(ledPin, HIGH);
    countOff = true;
    data = Serial.read();
    switch (data) {

      case 'L':
        while (!Serial.available()); //second char read
        //data = Serial.read();
        //Left_Speed = data - 48;
        // this can be anything or off_value can be anything. it apparently read ASCII values. let say L2 is pressed. once L is detected; Ascii value of 2 is 50. therefore, 2*50-50= 50, motor speed is sent as 50 pwm. find cooresponding ASCII values for higher speed.
        Left_Speed = getNumber();
        if (Left_Speed < 0)
          Left_Speed = 0;
        break;

      case 'R':
        while (!Serial.available());
        //Serial.println(Rad_L);
        data = Serial.read();
        Right_Speed = data - 48 ;
        if (Right_Speed < 0)
          Right_Speed = 0;
        //Serial.println(Right_Speed);

        break;

      case 'l':
        while (!Serial.available());
        data = Serial.read();
        //Serial.println(data);
        //Serial.print('\n');
        if (data == '1') {
          Left_direction = 1;
        }
        else {
          Left_direction = 0;
        }
        break;

      case 'r':
        while (!Serial.available());
        data = Serial.read();
        //Serial.println(data);
        if (data == '1') {
          Right_direction = 1;
        }
        else {
          Right_direction = 0;
        }
        break;
    }
  }
}

double getNumber() {

  char number[4];
  int intNum[4];
  double result = 0;
  for (int i = 0; i < 4; i++) {
    number[i] = Serial.read();
    intNum[i] = number[i] - 48;
  }
  result = intNum[0] * 10 + intNum[1] + intNum[2] / 10 + intNum[3] / 100;
  Serial.print(result);
  return result;

}
























