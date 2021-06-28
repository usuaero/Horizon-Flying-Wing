/*
Code Outline
1) Read in PPM signal from reciever
2) Interpret PPM signal as separate PWM components 
  - Throttle [Thr] --> PASS THROUGH
  - Aileron  [Ail]
  - Elevator [Elev]
  - Rudder   [Rud] --> NOT USED
3) Map Ail & Elev inputs into the corresponding 11 servo outputs
  - see mode1 function 
  - [SL0] [SL1] [SL2] [SL3] [SL4] [SC] [SR0] [SR1] [SR2] [SR3] [SR4] 
5) Convert servo deflections (degrees) back into PWM signals
6) output the throttle & 11 servo commands as individual PWM signals
*/

/*
 * NOTES:
 * PWM values treated as integers
 * degree values treated as doubles
 * 
 * CHECK THE DEGREE TO PWM MAPPING FUNCTION
 */

// Servo Library -- to control each servo/ send out individual PWM signals
#include "Servo.h"
// PulsePosition Library -- to read in / interpret the PPM signal 
#include <PulsePosition.h>
// Math library may not be needed for Mode 1??
//#include <math.h>

/*
 * FUNCTIONS USED
 */
void ReadInPPM(int PPMinput[]);
void mode1(int dl_pwm, int dm_pwm, double *lr);
double left(double s, double a);
double right(double s, double a);
double bounds(double x);
double pwm2frac(int pwm);
double DegtoPWM(double deg,int i);
void SendPPM(int output[]);

/*
 * CONSTANTS USED
 */
// transmitter values
#define TRANS_PWM_MIN 900.0
#define TRANS_PWM_MAX 2096.0
#define TRANS_PWM_NOM 1495.0
#define TRANS_PWM_NOISE 150.0

#define D 20.0  // max deflection of any given control surface (deg)

/*
 * GLOBAL VARIABLES
 */
 
// Create the PPM input variable - read on FALLING edge
PulsePositionInput DLRXinput(FALLING);

// Define 12 Servo instances & their corresponding pin on the Teensy
// 11 servos & 1 motor (signal split to both motors)
Servo Thrust;
int pThr = 18;
Servo L0;
int pL0 = 7;
Servo L1S;
int pL1 = 8;
Servo L2S;
int pL2 = 9;
Servo L3S;
int pL3 = 10;
Servo L4S;
int pL4 = 11;
Servo Center;
int pC = 1;
Servo R4;
int pR4 = 3;
Servo R3;
int pR3 = 2;
Servo R2;
int pR2 = 4;
Servo R1;
int pR1 = 5;
Servo R0;
int pR0 = 6;

// Launch the serial port in setup
void setup() {
  
  DLRXinput.begin(13); //signal from RX must be connected to pin 13

  Thrust.attach(pThr);
  L0.attach(pL0);
  L1S.attach(pL1);
  L2S.attach(pL2);
  L3S.attach(pL3);
  L4S.attach(pL4);
  Center.attach(pC);
  R4.attach(pR4);
  R3.attach(pR3);
  R2.attach(pR2);
  R1.attach(pR1); 
  R0.attach(pR0); 
  
  Serial.print("Setup Complete\n");
}

void ReadInPPM(int PPMinput[]){
  // read in the PWM channels from the RX PPM signal
  int Ail = DLRXinput.read(1);  // roll - delta l
  int Elev = DLRXinput.read(2); // pitch - delta m
  int Thr = DLRXinput.read(3);  // motor - pass through directly 
  //int Rud = DLRXinput.read(4);  // yaw - ignore Mode 1

  // Write out the Thrust command immediately
  Thrust.write(Thr);

  // Write the Ail & Elev values to the PPMinput array
  PPMinput[0] = Ail;
  PPMinput[1] = Elev;

}

void mode1(int dl_pwm, int dm_pwm, double *lr) {
    double s[5], a[5], dl, dm;
    int i;
    
    dl = pwm2frac(dl_pwm);
    dm = pwm2frac(dm_pwm);
    
    for (i=0; i<5; i++) {
        s[i] = D * dm;
        a[i] = D * dl;
    }
    
    // set center control surface
    lr[5] = bounds(D * dm);
    // loop thru inboard to outboard control surfaces
    for (i=0; i<5; i++) {
        // set left control surface
        lr[4-i] = bounds( left(s[i], a[i]));
        // set right control surface
        lr[6+i] = bounds(right(s[i], a[i]));
    }
}

double left(double s, double a) {
    /*
     * s = symmetric value (deg)
     * a = asymmetric value (deg)
    */
    return s - a;
}

double right(double s, double a) {
    /*
     * s = symmetric value (deg)
     * a = asymmetric value (deg)
    */
    return s + a;
}

double bounds(double x) {
    /*
     * x = value in degrees
    */
    if (x < -D) x = -D;
    if (x >  D) x = D;
    return x;
}

double pwm2frac(int pwm) {
    if (pwm > int(TRANS_PWM_MAX + TRANS_PWM_NOISE) || pwm < int(TRANS_PWM_MIN - TRANS_PWM_NOISE)) return 0.0;
    if (pwm > int(TRANS_PWM_MAX)) return 1.0;
    if (pwm < int(TRANS_PWM_MIN)) return -1.0;
    return 2.0 * (double(pwm) - TRANS_PWM_MIN) / (TRANS_PWM_MAX - TRANS_PWM_MIN) - 1.0;
}

// NOTE: This function needs to be updated with actual degree mapping 
double DegtoPWM(double deg, int i){
  double C1 = 1.0;
  double C2 = 90.0;
  // degree mapping for different servo sections
  switch(i){
    // default returns the original degree value
    default: break;
    
    case 0: // L4
    {
      C1 = 1.6483;
    }
    break;
    
    case 1: // L3
    {
      C1 = 1.8466;
    }
    break;
    
    case 2: // L2
    {
      C1 = 1.746;
    }
    break;
    
    case 3: // L1
    {
      C1 = 2.193;
    }
    break;
    
    case 4: // L0
    {
      C1 = 2.2315;
    }
    break;
    
    case 5: // Center
    {
      C1 = -3.7146;
    }
    break;
    
    case 6: // R0 
    {
      C1 = -2.1149;
    }
    break;
    
    case 7: // R1
    {
      C1 = -2.246;
    }
    break;
    
    case 8: // R2
    {
      C1 = -2.1162;
    }
    break;
    
    case 9: // R3
    {
      C1 = -1.6606;
    }
    break;
    
    case 10: // R4
    {
      C1 = -1.5492;
    }
    break;
  }
  double pwm = C1*deg + C2;
  //pwm = int(pwm);

  return pwm;
}

void SendPPM(double output[]){
  // individually sends out servo commands - PWM signal
  
  L4S.write(output[0]);
  L3S.write(output[1]);
  L2S.write(output[2]);
  L1S.write(output[3]);
  L0.write(output[4]);
  Center.write(output[5]);
  R0.write(output[6]);
  R1.write(output[7]);
  R2.write(output[8]);
  R3.write(output[9]);
  R4.write(output[10]);
}

void loop() {
  int PPMinput[2];
    
  // 1) Read in the PPM signal & convert to 4 PWM signals
  ReadInPPM(PPMinput);
  
  double degOutput[11];
  // map the traditional degree inputs to the 11 servo inputs
  mode1(PPMinput[0],PPMinput[1],degOutput);
  
  // initialize the array to hold the PWM outputs
  double pwmOutput[11] = {0,0,0,0,0,0,0,0,0,0,0};

  int i;
  // convert the 11 servo deflections to pwm signals (skip throttle)
  for (i = 0; i < 11; i++){
    pwmOutput[i] = DegtoPWM(degOutput[i],i);
  }
  
  // send out the individual PWM signals
  SendPPM(pwmOutput);

}
