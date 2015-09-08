/*
 Fading

 This example shows how to fade an LED using the analogWrite() function.

 The circuit:
 * LED attached from digital pin 9 to ground.

 Created 1 Nov 2008
 By David A. Mellis
 modified 30 Aug 2011
 By Tom Igoe

 http://arduino.cc/en/Tutorial/Fading

 This example code is in the public domain.

 */


int zeroPin = 8;
int ledPin = 9;    // LED connected to digital pin 9
int ledState = LOW; 
int zeroState = LOW;
int fadeValue1;

void setup() {
  pinMode(zeroPin, OUTPUT);
  digitalWrite(zeroPin, ledState);
  // nothing happens in setup
}

void loop() {
  
  while (1) {
    
    if (zeroState == LOW)
      zeroState = HIGH;
    else
      zeroState = LOW;
      
      digitalWrite(zeroPin, zeroState);
  // fade in from min to max in increments of 5 points:
  for (int fadeValue = 0 ; fadeValue <= 255; fadeValue += 5) {
    // sets the value (range from 0 to 255):
    
    if (zeroState == HIGH)
      fadeValue1 = 255-fadeValue;
      else
      fadeValue1 = fadeValue;
      
    analogWrite(ledPin, fadeValue1);
    // wait for 30 milliseconds to see the dimming effect
    delay(30);
  }

  // fade out from max to min in increments of 5 points:
  for (int fadeValue = 255 ; fadeValue >= 0; fadeValue -= 5) {
    // sets the value (range from 0 to 255):
    
        if (zeroState == HIGH)
      fadeValue1 = 255-fadeValue;
      else
      fadeValue1 = fadeValue;
      
    analogWrite(ledPin, fadeValue1);
    // wait for 30 milliseconds to see the dimming effect
    delay(30);
  }
  }
}


