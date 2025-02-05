#include <Servo.h>

Servo meuServo;
int portaServo = 9;

void setup() {
  Serial.begin(9600); 
  meuServo.attach(portaServo); 
}

void loop() {

  //meuServo.write(0);

  if (Serial.available() > 0) { 
    String stringo = Serial.readString(); 

    stringo.trim();

    if (stringo.startsWith("LR_TURN_TO:")) {
      //String S_ang = substring(8);
      int I_ang = stringo.substring(11).toInt();
      Serial.println(I_ang);
      meuServo.write(I_ang);
    }
  }
}
  

