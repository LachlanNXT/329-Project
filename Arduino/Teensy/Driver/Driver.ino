#include "TimerThree.h"
#include "Math.h"

#define M1 14
#define M2 10
#define M3 4
#define M4 12
#define encoder_left 5
#define encoder_right 8

const double encoder_to_rev = 227.43;
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

double Left_Speed = 0;
double Right_Speed = 0;
bool Left_direction = true;
bool Right_direction = true;
char data;

int ledPin = 11;

//serial recieve light
int turnOff = 0;
volatile bool countOff = false;

//serial sending
volatile int sendCount = 0;
byte Lsend[4] = {0, 0, 0, 0};
byte Rsend[4] = {0, 0, 0, 0};

//PID
double errorL = 0;
double errorR = 0;
double sumL = 0;
double sumR = 0;
const double Kp = 3;
const double Ki = 0;
const double Kd = 3;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  pinMode(encoder_left, INPUT);
  pinMode(encoder_right, INPUT);
  attachInterrupt(encoder_left, doEncoderL, RISING);
  attachInterrupt(encoder_right, doEncoderR, RISING);
  Timer3.initialize(100000); // 100000 micro seconds, 100 milliseconds, 0.1 seconds
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
    //turnOff++;
    //if (turnOff > 10) {
      digitalWrite(ledPin, LOW);
      countOff = false;
    //}
  }
  count_L = state_L;
  count_R = state_R;
  Rad_L = ((2 * pi) / encoder_to_rev * count_L) / .1;
  Rad_R = ((2 * pi) / encoder_to_rev * count_R) / .1;
  state_L = 0;
  state_R = 0;

  sendCount++;
  if (sendCount >= 3) {
    makeNumber(Rad_L, Lsend);
    makeNumber(Rad_R, Rsend);
    Serial.write("SL");
    Serial.print(Lsend[0]);
    Serial.print(Lsend[1]);
    Serial.print(Lsend[2]);
    Serial.print(Lsend[3]);
    Serial.write("R");
    Serial.print(Rsend[0]);
    Serial.print(Rsend[1]);
    Serial.print(Rsend[2]);
    Serial.print(Rsend[3]);
    Serial.write("E");
    Serial.println();
    sendCount = 0;
    digitalWrite(ledPin, HIGH);
    countOff = true;
  }
  
  if (Left_Speed == 0) {
    pwm_L = 0;
    Stop_L();
  }
  double newErrorL = Left_Speed - Rad_L;
  double diffL = newErrorL - errorL;
  sumL = sumL + newErrorL;
  errorL = newErrorL;
  
  int dpwmL = round(Kp*errorL + Ki*sumL + Kd*diffL);
  
  if (Right_Speed == 0) {
    pwm_R = 0;
    Stop_R();
  }
  double newErrorR = Right_Speed - Rad_R;
  double diffR = newErrorR - errorR;
  sumR = sumR + newErrorR;
  errorR = newErrorR;

  int dpwmR = round(Kp*errorR + Ki*sumR + Kd*diffR);

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

  Motor_L(pwm_L, Left_direction);
  Motor_R(pwm_R, Right_direction);
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
        // it apparently read ASCII values. let say L2 is pressed. once L is detected; Ascii value of 2 is 50. therefore, 2*50-50= 50, motor speed is sent as 50 pwm. find cooresponding ASCII values for higher speed.
        Left_Speed = getNumber();
        if (Left_Speed < 0)
          Left_Speed = 0;
        break;

      case 'R':
        while (!Serial.available());
        Right_Speed = getNumber();
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
  double intNum[4];
  double result = 0;
  for (int i = 0; i < 4; i++) {
    number[i] = Serial.read();
    intNum[i] = number[i] - 48;
  }
  result = intNum[0] * 10 + intNum[1] + intNum[2] / 10 + intNum[3] / 100;
  //Serial.print(result);
  return result;

}

void makeNumber(double inNumber, byte outNumber[]) {
  double one = floor(inNumber / 10);
  double two = floor(inNumber - one * 10);
  double three = floor((inNumber - one * 10 - two) * 10);
  double four = floor((inNumber - one * 10 - two) * 100 - three * 10);
  outNumber[0] = (byte ((int) (one)));
  outNumber[1] = (byte ((int) (two)));
  outNumber[2] = (byte ((int) (three)));
  outNumber[3] = (byte ((int) (four)));

}

