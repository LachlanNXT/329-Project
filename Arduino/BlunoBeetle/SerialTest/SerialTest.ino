/* Simple Serial ECHO script : Written by ScottC 03/07/2012 */

/* Use a variable called byteRead to temporarily store
   the data coming from the computer */
byte byteRead;
int ledPin = 11;
int flashcount = 0;
String message = String(123) + "$" + String(123);//"This is a message";

void setup() {
  // Turn the Serial Protocol ON
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT);
}

void loop() {
  /*  check if data has been sent from the computer: */
  if (Serial.available()) {
    digitalWrite(ledPin, HIGH);
    /* read the most recent byte */
    if (Serial.peek() != -1) {
      flashcount = 0;
      do {
        byteRead = Serial.read();
        if (byteRead == 97) {
          flashcount++;
        }
        //Serial.println(byteRead);
      } while (byteRead != 0x0A);
    }
    delay(500);
    digitalWrite(ledPin, LOW);
    delay(500);
  }
  Serial.print(flashcount);
  digitalWrite(ledPin,LOW);
  delay(500);
  for (int i = 0; i<flashcount; i++) {

    Serial.print(flashcount);
    digitalWrite(ledPin,HIGH);
    delay(100);
    digitalWrite(ledPin, LOW);
    delay(100);

  }
  

}

