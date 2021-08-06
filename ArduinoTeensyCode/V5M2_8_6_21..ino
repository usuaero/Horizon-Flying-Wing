/* Code Outline
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

/* NOTES:
 * PWM values treated as integers
 * degree values treated as doubles
 * 
 * CHECK THE DEGREE TO PWM MAPPING FUNCTION
 */

// included the needed libraries
//##########################################################################
// Servo Library -- to control each servo/ send out individual PWM signals
#include "Servo.h"
// PulsePosition Library -- to read in / interpret the PPM signal 
#include <PulsePosition.h>
// Math library
//#include <math.h>
// library to read in pixhawk data
#include "c_library_v2-master/mavlink.h"
// include control mapping items
#include "controlMapping.h"

// FUNCTIONS USED
//##########################################################################
// Sabrina's functions
void ReadInPPM(int PPMinput[]);
void DegtoPWM(double *deg, double *pwm);
void SendPPM(int output[]);
void comm_receive();
/* Zach's functions
double left(double s, double a);
double right(double s, double a);
double bounds(double x);
double calc_dL(double V, double phi, double p, double climbRate);
double pwm2frac(int pwm);
void mode1(int dl_pwm, int dm_pwm, double *lr);
void mode2(double dL, double dl, double dm, double *lr);
*/

// define the global constants (they are all in uppercase for ease of identifying)
//##########################################################################
/* Zach's constants
#define PI 3.14159265               // pi
#define G 32.174                    // acceleration due to gravity (ft/sec^2)
#define RHO 0.0020482               // density of air (slugs/ft^3)
// control parameters
#define D 20.0                      // max deflection of any given control surface (deg)
#define DL_MAX 0.9                  // max acceptable value for dL
#define DL_MIN 0.1                  // mIN acceptable value for dL
#define PSCT_GAIN 4.0               // tunable gain on the SCT criteria for the calcCL2 function
// aircraft properties
#define W 16.0                      // weight of aircraft (lbf)                 ***THIS NEEDS TO BE UPDATED***
#define S 12.29923797666569         // planform area of main wing (ft^2)
#define B 9.91936999529998          // wingspan (ft)
// transmitter values
#define TRANS_PWM_MIN 900.0
#define TRANS_PWM_MAX 2096.0
#define TRANS_PWM_NOM 1495.0
#define TRANS_PWM_NOISE 150.0 */
// deg2servoDeg mapping values
#define INTERCEPT 90.
#define SLOPE_L4 1.5922
#define SLOPE_L3 1.7089
#define SLOPE_L2 1.7762
#define SLOPE_L1 1.9802
#define SLOPE_L0 1.9902
#define SLOPE_CE -4.7937
#define SLOPE_R0 -2.1036
#define SLOPE_R1 -1.6441
#define SLOPE_R2 -1.712
#define SLOPE_R3 -1.9267
#define SLOPE_R4 -1.6738

// GLOBAL VARIABLES
//##########################################################################
// Create the PPM input variable - read on FALLING edge
PulsePositionInput DLRXinput(FALLING);
// Define 12 Servo instances & their corresponding pin on the Teensy
// 11 servos & 1 motor (signal split to both motors)
Servo Thrust;
int pThr = 18;
Servo L0;
int pL0 = 7;
Servo L1;
int pL1 = 8;
Servo L2;
int pL2 = 9;
Servo L3;
int pL3 = 10;
Servo L4;
int pL4 = 11;
Servo Ce;
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

//int PPMinput[2];
double degOutput[11], pwmOutput[11];

struct telemetryData pix;
struct pilotCommands pilot;

//##########################################################################
//##########################################################################
//##########################################################################

void loop() {
  int i;
  
  // 1) Read in the PPM signal & convert to 4 PWM signals
  ReadInPPM(pilot);
  
  // read in pixhawk data
  comm_receive(pix);
  
  // check for the mode
  if (pilot.modeSwitch < ?) {
    mode1(PPMinput[0],PPMinput[1],degOutput);
  }
  //else if (pilot.modeSwitch < ?) {
     implement a third mode
  //}
  else {
    
    mode2(dL, dl, dm, degOutput);
  }
  
  // convert degrees control surface deflection to degrees servo arm deflection
  DegtoPWM(degOutput, pwmOutput);
  
  // send out the individual PWM signals
  SendPPM(pwmOutput);
}

// Launch the serial port in setup
void setup() {
  
  DLRXinput.begin(13); //signal from RX must be connected to pin 13

  Thrust.attach(pThr);
  L0.attach(pL0);
  L1.attach(pL1);
  L2.attach(pL2);
  L3.attach(pL3);
  L4.attach(pL4);
  Ce.attach(pC);
  R4.attach(pR4);
  R3.attach(pR3);
  R2.attach(pR2);
  R1.attach(pR1); 
  R0.attach(pR0); 
  
  Serial1.begin(57600);    // TELEM2 from Pixhawk
  
  Serial.print("Setup Complete\n");
}

void ReadInPPM(struct pilotCommands x){
  // read in the PWM channels from the RX PPM signal
  x.ail = DLRXinput.read(1);  // roll - delta l
  x.ele = DLRXinput.read(2);  // pitch - delta m
  Thrust.write( DLRXinput.read(3) );  // motor - pass through directly 
  x.rud = DLRXinput.read(4);  // yaw - ignore Mode 1
  // we will need to read in one more channel, likely a 2 or 3 way switch so that the pilot can change modes as desired
  x.modeSwitch = DLRXinput.read(?);
  
  // Write out the Thrust command immediately
  //Thrust.write(x.thr);
}

void DegtoPWM(double *deg, double *pwm){
  pwm[0]  = deg[0]  * SLOPE_L4 + INTERCEPT;
  pwm[1]  = deg[1]  * SLOPE_L3 + INTERCEPT;
  pwm[2]  = deg[2]  * SLOPE_L2 + INTERCEPT;
  pwm[3]  = deg[3]  * SLOPE_L1 + INTERCEPT;
  pwm[4]  = deg[4]  * SLOPE_L0 + INTERCEPT;
  pwm[5]  = deg[5]  * SLOPE_CE + INTERCEPT;
  pwm[6]  = deg[6]  * SLOPE_R0 + INTERCEPT;
  pwm[7]  = deg[7]  * SLOPE_R1 + INTERCEPT;
  pwm[8]  = deg[8]  * SLOPE_R2 + INTERCEPT;
  pwm[9]  = deg[9]  * SLOPE_R3 + INTERCEPT;
  pwm[10] = deg[10] * SLOPE_R4 + INTERCEPT;
}

void SendPPM(double output[]){
  // individually sends out servo commands - PWM signal
  L4.write(output[0]);
  L3.write(output[1]);
  L2.write(output[2]);
  L1.write(output[3]);
  L0.write(output[4]);
  Ce.write(output[5]);
  R0.write(output[6]);
  R1.write(output[7]);
  R2.write(output[8]);
  R3.write(output[9]);
  R4.write(output[10]);
}

void comm_receive(struct telemetryData p) {
  uint8_t c;
  mavlink_message_t msg;
  mavlink_status_t status;
  
  while(Serial1.available() > 0) { // as long as buffer size bigger than 0 -- pull out serial data

    c = Serial1.read(); // reads in the serial message
    
    // Try to get a new message
    if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
        
        switch(msg.msgid) {
        default: break;
        // IMU data -- roll & yaw rates
        case MAVLINK_MSG_ID_RAW_IMU:  // #105 highres IMU / Check scaled IMU for acutal value readings 
          {
            mavlink_raw_imu_t raw_imu;
            mavlink_msg_raw_imu_decode(&msg, &raw_imu);
            p.rollRate = raw_imu.xgyro;
          }
          break;
        // airspeed 
        case MAVLINK_MSG_ID_VFR_HUD:  // #74 VFR_HUD
          {
            mavlink_vfr_hud_t vfr_hud;
            mavlink_msg_vfr_hud_decode(&msg, &vfr_hud);
            p.airspeed = vfr_hud.airspeed;  // m/s
            p.climbRate = vfr_hud.climb;  // m/s
          }
          break;
        // attitude data -- roll angles (bank angle)
        case MAVLINK_MSG_ID_ATTITUDE:  // #30 ATTITUDE
          {
            mavlink_attitude_t attitude;
            mavlink_msg_attitude_decode(&msg, &attitude);
            p.bank = attitude.roll;  // rad (-pi..+pi)
          }
          break;      
      }
    }
  }
}

