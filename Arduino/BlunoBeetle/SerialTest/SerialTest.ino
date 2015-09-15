/* Simple Serial ECHO script : Written by ScottC 03/07/2012 */

/* Use a variable called byteRead to temporarily store
   the data coming from the computer */
byte byteRead;

void setup() {                
// Turn the Serial Protocol ON
  Serial.begin(9600);
  pinMode(13, OUTPUT);
}

void loop() {
   /*  check if data has been sent from the computer: */
  if (Serial.available()) {
    /* read the most recent byte */
    byteRead = Serial.read();
    /*ECHO the value that was read, back to the serial port. */
    Serial.println(byteRead);
  }
  if (byteRead == 44) {
    digitalWrite(13, HIGH);
  }
  if (byteRead == 45) {
    digitalWrite(13, LOW);
  }
}



