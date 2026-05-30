#include <Servo.h>

Servo esc;
Servo servo1;
int number=0;
int escpower=0;
void setup() {
  Serial.begin(9600);
  esc.attach(9, 1000, 2000);
    esc.writeMicroseconds(1000);  // send minimum throttle
   delay(5000);   
    servo1.attach(10);
    servo1.write(90);
}

void loop() {

  if (Serial.available() > 0) {

    number = Serial.parseInt();
    number = constrain(number, 0, 100);

    escpower = map(number, 0, 100, 1000, 2000);

    esc.writeMicroseconds(escpower);

    Serial.print("Throttle: ");
    Serial.print(number);
    Serial.println("%");
  }

  // keep refreshing ESC signal
  esc.writeMicroseconds(escpower);

  delay(20);
}
