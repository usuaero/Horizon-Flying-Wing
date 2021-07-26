#include <PulsePosition.h>
#include <Servo.h>

PulsePositionInput DLRXinput(FALLING);
int SApin = 9;
int SEpin = 10;
int SRpin = 11;
Servo Sail;
Servo Selev;
Servo Srud;

void setup() {
  DLRXinput.begin(13); //signal from DL must be connected to pin 13

  Sail.attach(SApin);
  Selev.attach(SEpin);
  Srud.attach(SRpin);
  
  Serial.print("Setup Complete\n");
}

void loop() {
  // read in the PWM channels from the DL
  int chAil = DLRXinput.read(1);
  int chElev = DLRXinput.read(2);
  //int chThr = DLRXinput.read(3);
  int chRudd = DLRXinput.read(4);

  Serial.print(chAil);Serial.print("\t");
  Serial.print(chElev);Serial.print("\t");
  //Serial.print(chThr);Serial.print("\t");
  Serial.print(chRudd);Serial.print("\n");

  Sail.write(PWM2Deg(chAil));
  Selev.write(PWM2Deg(chElev));
  Srud.write(PWM2Deg(chRudd));
  delay(100);

}

int PWM2Deg (int pwm){
  int deg = round(pwm*0.133 - 110);
  
  return deg;
}
