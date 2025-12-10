#include <Servo.h>

Servo myservo_x;
Servo myservo_y;  

String receivedString = "";
int x_val = 90;
int y_val = 90;

void setup() {
  Serial.begin(9600);
  myservo_x.attach(3);
  myservo_y.attach(5);
  pinMode(2,OUTPUT);

  myservo_x.write(x_val);
  myservo_y.write(y_val);
}

void loop() {
	digitalWrite(2,HIGH)
  if (Serial.available() > 0) {

    receivedString = Serial.readStringUntil('\n'); 
    receivedString.trim();// Removes any extra whitespace/newlines


    int separatorIndex = receivedString.indexOf('x'); 

 
    if (separatorIndex != -1) {
      
  
      String x_part = receivedString.substring(0, separatorIndex);
      String y_part = receivedString.substring(separatorIndex + 1);

      x_val = x_part.toInt();
      y_val = y_part.toInt();

      
      x_val = constrain(x_val, 0, 180);
      y_val = constrain(y_val, 0, 180);


      myservo_x.write(x_val);
      myservo_y.write(y_val);
      delay(50);
/*
      Serial.print("Moved X to: ");
      Serial.print(x_val);
      Serial.print(" | Y val: ");
      Serial.println(y_val);*/
    }
  }
}
