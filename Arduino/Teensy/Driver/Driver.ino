
#include "TimerThree.h"
#include <PID_v1.h>

#define M1 14
#define M2 10
#define M3 4
#define M4 12
#define encoder_left 5
#define encoder_right 7

int encoder_to_rev = 210;
boolean forward1;
double Setpoint_L, Input_L, Output_L;
int kp=10;
int ki=0;
int kd=0;
double Setpoint_R, Input_R, Output_R;
PID myPID_R(&Input_R, &Output_R, &Setpoint_R, kp,ki,kd, DIRECT);
PID myPID_L(&Input_L, &Output_L, &Setpoint_L,kp,ki,kd, DIRECT);

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
int Left_Speed = 0;
int Right_Speed = 0;
char off_value = 50;
bool Left_direction = true;
bool Right_direction = true;
char data;
unsigned long lastMillis_L = 0;

int ledPin = 11;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  pinMode(encoder_left, INPUT);
  pinMode(encoder_right, INPUT);
  attachInterrupt(encoder_left, doEncoderL, RISING);
  attachInterrupt(encoder_right, doEncoderR, RISING);
  Timer3.initialize(10000);
  Timer3.attachInterrupt(reset_timer);
  //Serial.println("L=Left motor speed");
  //Serial.println("R=Right motor speed");
  //Serial.println("l=left direction(1 forward, 0 backward):  ");
  //Serial.println("r=right direction(1 forward, 0 backward):  ");

  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);
  myPID_L.SetMode(AUTOMATIC);
  myPID_R.SetMode(AUTOMATIC);


}

void doEncoderL() {

  state_L++;
  //if (state_L==100)
  // {
  // lastMillis_L=millis();
  // Rad_L= ((2*pi)/(210*100))/(millis()-lastMillis_L);
  //  state_L=0;

  // }
 

}

void doEncoderR() {
  state_R++;
   

}

void reset_timer() {
  Serial.print('9');
  count_L = state_L;
  count_R = state_R;
  Rad_L = ((2 * pi) / 2.1 * count_L);
  Rad_R = ((2 * pi) / 2.1 * count_R);
  state_L = 0;
  state_R = 0;
//  Setpoint_L=Left_Speed;
//  Setpoint_R=Right_Speed;
//  Input_L=Rad_L;
//  Input_R=Rad_R;
//  myPID_L.Compute();
//  myPID_R.Compute();
// if (Output_L>Setpoint_L)
//
//
//  if (Output_R<Setpoint_R)
//  {
//    pwm_R++;
//    if (pwm_R>255)
//    {
//      pwm_R=255;
//    }
//  }
//  else
//  {
//    pwm_R--;
//    if (pwm_R<0)
//    {
//      pwm_R=0;
//    }
//    
//  }
//   if (Output_L<Setpoint_L)
//  {
//    pwm_L++;
//    if (pwm_L>255)
//    {
//      pwm_L=255;
//    }
//  }
//  else
//  {
//    pwm_L--;
//    if (pwm_L<0)
//    {
//      pwm_L=0;
//    }
//    
//  }

//kp=kp+0.1;
PID_LR();



}

void PID_LR()
{   /*
 int dpwmL = round(10*(Left_Speed - Rad_L));
 
 int dpwmR = round(15*(Right_Speed - Rad_R));
 
 pwm_L += dpwmL;
 
 pwm_R += dpwmR;
   
   
    if (pwm_L>255)
    {
      pwm_L=255;
    }
  
    if (pwm_L<0)
    {
      pwm_L=0;
    }
       if (pwm_R>255)
    {
      pwm_R=255;
    }
  
    if (pwm_R<0)
    {
      pwm_R=0;
    }
    
*/

 Setpoint_L=Left_Speed;
 
 Input_L=Rad_L;
  
 myPID_L.Compute();
   
                   
                          
    if (Rad_L<Left_Speed)
  {
    pwm_L++;
    if (pwm_L>255)
    {
      pwm_L=255;
    }
  }
  else
  {
    pwm_L--;
    if (pwm_L<0)
    {
      pwm_L=0;
    }
    
  }
   Setpoint_R=Right_Speed;
  Input_R=Rad_R;
   myPID_R.Compute();

    if (Rad_R<Right_Speed)
  {
    pwm_R++;
    if (pwm_R>255)
    {
      pwm_R=255;
    }
  }
  else
  {
    pwm_R--;
    if (pwm_R<0)
    {
      pwm_R=0;
    }
    
   
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
  
  Motor_L(pwm_L, true);
  Motor_R(pwm_R, true);
  if (Serial.available()) {// first char read
    data = Serial.read();
    //Serial.println(data);
    switch (data) {
      case 'L':
        while (!Serial.available()); //second char read
        data = Serial.read();
        Left_Speed = data-48;
        // this can be anything or off_value can be anything. it apparently read ASCII values. let say L2 is pressed. once L is detected; Ascii value of 2 is 50. therefore, 2*50-50= 50, motor speed is sent as 50 pwm. find cooresponding ASCII values for higher speed.
        if (Left_Speed < 0)
          Left_Speed = 0;

        //Serial.println(Left_Speed);

        break;
      case 'R':
        while (!Serial.available());
        //Serial.println(Rad_L);
        data = Serial.read();
        Right_Speed = data-48 ;
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
    //Serial.print('\n');
 
   
  }





}

























